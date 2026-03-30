#include "librmcs_hardware/driver/librmcs_robot_driver.hpp"

#include <algorithm>
#include <cmath>

#include <librmcs/client/cboard.hpp>

#include "librmcs_hardware/driver/grouped_can_command.hpp"
#include "librmcs_hardware/devices/motor_device_base.hpp"
#include "librmcs_hardware/devices/motor_device_factory.hpp"

namespace librmcs_hardware {

namespace {

constexpr auto kStartupSendRetryDelay = std::chrono::milliseconds(2);
constexpr auto kStartupSendStepDelay = std::chrono::milliseconds(5);
constexpr auto kCommandResendPeriod = std::chrono::milliseconds(10);
constexpr std::size_t kStartupSendMaxRetries = 10;

}  // namespace

class LibrmcsRobotDriver::RobotBoard : public librmcs::client::CBoard {
public:
  RobotBoard(LibrmcsRobotDriver & owner, int32_t usb_pid)
  : librmcs::client::CBoard(usb_pid), owner_(owner), transmit_buffer_(*this, 16) {}

  bool send_frame(const CanFrame & frame) {
    const bool queued = frame.can_bus == CanBus::Can1
                          ? transmit_buffer_.add_can1_transmission(frame.can_id, frame.can_data)
                          : transmit_buffer_.add_can2_transmission(frame.can_id, frame.can_data);
    return queued && transmit_buffer_.trigger_transmission();
  }

private:
  void can1_receive_callback(
    uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
    uint8_t can_data_length) override
  {
    (void)is_extended_can_id;
    (void)is_remote_transmission;
    (void)can_data_length;
    owner_.handle_can_frame(CanBus::Can1, can_id, can_data);
  }

  void can2_receive_callback(
    uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
    uint8_t can_data_length) override
  {
    (void)is_extended_can_id;
    (void)is_remote_transmission;
    (void)can_data_length;
    owner_.handle_can_frame(CanBus::Can2, can_id, can_data);
  }

  LibrmcsRobotDriver & owner_;
  TransmitBuffer transmit_buffer_;
};

LibrmcsRobotDriver::LibrmcsRobotDriver(
  int32_t usb_pid,
  std::chrono::milliseconds command_timeout,
  std::vector<JointConfig> joint_configs)
: usb_pid_(usb_pid),
  command_timeout_(command_timeout),
  feedback_timeout_(std::max(command_timeout * 5, std::chrono::milliseconds(500)))
{
  joints_.reserve(joint_configs.size());
  command_efforts_.assign(joint_configs.size(), 0.0);
  for (const auto & joint_config : joint_configs) {
    joints_.push_back(make_motor_device(joint_config));
  }
}

LibrmcsRobotDriver::~LibrmcsRobotDriver() { stop(); }

void LibrmcsRobotDriver::start(bool enable_on_start) {
  std::scoped_lock lock(mutex_);
  if (running_) {
    return;
  }

  board_ = std::make_unique<RobotBoard>(*this, usb_pid_);
  running_ = true;
  commands_zeroed_ = true;
  std::fill(command_efforts_.begin(), command_efforts_.end(), 0.0);
  last_command_update_ = std::chrono::steady_clock::now();
  last_command_send_ = last_command_update_;

  event_thread_ = std::thread([this]() {
    try {
      board_->handle_events();
    } catch (...) {
    }
  });

  watchdog_thread_ = std::thread([this]() { watchdog_loop(); });

  if (enable_on_start) {
    send_activation_frames_locked();
    send_startup_frames_locked();
  }
}

void LibrmcsRobotDriver::stop() {
  std::unique_lock lock(mutex_);
  if (!running_) {
    return;
  }

  send_stop_commands_locked();
  running_ = false;
  if (board_) {
    board_->stop_handling_events();
  }
  lock.unlock();

  if (watchdog_thread_.joinable()) {
    watchdog_thread_.join();
  }
  if (event_thread_.joinable()) {
    event_thread_.join();
  }

  lock.lock();
  board_.reset();
}

bool LibrmcsRobotDriver::enable_motors() {
  std::scoped_lock lock(mutex_);
  if (!running_ || !board_) {
    return false;
  }
  // 与 start() 行为保持一致：先激活序列，再发启动序列。
  const bool ok1 = send_activation_frames_locked();
  const bool ok2 = send_startup_frames_locked();
  return ok1 && ok2;
}

bool LibrmcsRobotDriver::disable_motors() {
  std::scoped_lock lock(mutex_);
  if (!running_ || !board_) {
    return false;
  }
  return send_stop_commands_locked();
}

bool LibrmcsRobotDriver::wait_for_feedback(std::chrono::milliseconds timeout) const {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::scoped_lock lock(mutex_);
      if (std::all_of(
            joints_.begin(), joints_.end(),
            [](const auto & joint) { return joint->feedback_received(); })) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::scoped_lock lock(mutex_);
  return std::all_of(
    joints_.begin(), joints_.end(),
    [](const auto & joint) { return joint->feedback_received(); });
}

bool LibrmcsRobotDriver::any_feedback_received() const {
  std::scoped_lock lock(mutex_);
  return std::any_of(
    joints_.begin(), joints_.end(),
    [](const auto & joint) { return joint->feedback_received(); });
}

std::vector<std::string> LibrmcsRobotDriver::missing_feedback_joint_names() const {
  std::scoped_lock lock(mutex_);
  std::vector<std::string> names;
  names.reserve(joints_.size());
  for (const auto & joint : joints_) {
    if (!joint->feedback_received()) {
      names.push_back(joint->config().name);
    }
  }
  return names;
}

std::vector<JointState> LibrmcsRobotDriver::read_joint_states() const {
  std::scoped_lock lock(mutex_);
  const auto now = std::chrono::steady_clock::now();
  std::vector<JointState> states;
  states.reserve(joints_.size());
  for (const auto & joint : joints_) {
    auto state = joint->state();
    state.online = joint->is_online(now, feedback_timeout_);
    states.push_back(state);
  }
  return states;
}

bool LibrmcsRobotDriver::write_joint_efforts(std::span<const double> efforts) {
  std::scoped_lock lock(mutex_);
  if (!running_ || !board_ || efforts.size() != joints_.size()) {
    return false;
  }

  for (std::size_t index = 0; index < efforts.size(); ++index) {
    command_efforts_[index] = std::isfinite(efforts[index]) ? efforts[index] : 0.0;
  }
  last_command_update_ = std::chrono::steady_clock::now();
  commands_zeroed_ = false;
  return send_effort_commands_locked();
}

void LibrmcsRobotDriver::handle_can_frame(CanBus can_bus, uint32_t can_id, uint64_t can_data) {
  std::scoped_lock lock(mutex_);
  for (auto & joint : joints_) {
    if (joint->accepts_feedback(can_bus, can_id)) {
      joint->parse_feedback(can_bus, can_id, can_data);
      break;
    }
  }
}

void LibrmcsRobotDriver::watchdog_loop() {
  while (true) {
    {
      std::scoped_lock lock(mutex_);
      if (!running_) {
        return;
      }

      const auto now = std::chrono::steady_clock::now();
      if (!commands_zeroed_ && now - last_command_update_ > command_timeout_) {
        std::fill(command_efforts_.begin(), command_efforts_.end(), 0.0);
        commands_zeroed_ = true;
      }

      if (now - last_command_send_ >= kCommandResendPeriod) {
        send_effort_commands_locked();
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

bool LibrmcsRobotDriver::send_activation_frames_locked() {
  if (!board_) {
    return false;
  }

  bool ok = true;
  for (const auto & joint : joints_) {
    for (const auto & frame : joint->build_activate_sequence()) {
      ok = board_->send_frame(frame) && ok;
    }
  }
  return ok;
}

bool LibrmcsRobotDriver::send_startup_frames_locked() {
  if (!board_) {
    return false;
  }

  const auto send_startup_frame = [this](const CanFrame & frame) {
    for (std::size_t attempt = 0; attempt < kStartupSendMaxRetries; ++attempt) {
      if (board_->send_frame(frame)) {
        return true;
      }
      std::this_thread::sleep_for(kStartupSendRetryDelay);
    }
    return false;
  };

  std::vector<std::vector<GroupedCanCommand>> startup_grouped_commands;
  startup_grouped_commands.reserve(joints_.size());
  std::size_t startup_grouped_steps = 0;

  bool ok = true;
  for (const auto & joint : joints_) {
    for (const auto & frame : joint->build_startup_sequence()) {
      ok = send_startup_frame(frame) && ok;
    }
    startup_grouped_commands.push_back(joint->build_startup_grouped_commands());
    startup_grouped_steps = std::max(startup_grouped_steps, startup_grouped_commands.back().size());
  }

  for (std::size_t step = 0; step < startup_grouped_steps; ++step) {
    GroupedCanCommandAggregator grouped_commands;
    for (const auto & command_sequence : startup_grouped_commands) {
      if (step < command_sequence.size()) {
        grouped_commands.ingest(command_sequence[step]);
      }
    }
    for (const auto & frame : grouped_commands.flush()) {
      ok = send_startup_frame(frame) && ok;
    }
    std::this_thread::sleep_for(kStartupSendStepDelay);
  }
  return ok;
}

bool LibrmcsRobotDriver::send_effort_commands_locked() {
  if (!board_) {
    return false;
  }

  GroupedCanCommandAggregator grouped_commands;
  bool ok = true;
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    const auto effort = command_efforts_[index];
    const auto & joint = joints_[index];

    grouped_commands.ingest(joint->build_grouped_command(effort));
    for (const auto & frame : joint->build_direct_command(effort)) {
      ok = board_->send_frame(frame) && ok;
    }
  }

  for (const auto & frame : grouped_commands.flush()) {
    ok = board_->send_frame(frame) && ok;
  }
  last_command_send_ = std::chrono::steady_clock::now();
  return ok;
}

bool LibrmcsRobotDriver::send_stop_commands_locked() {
  if (!board_) {
    return false;
  }

  GroupedCanCommandAggregator grouped_commands;
  bool ok = true;
  for (const auto & joint : joints_) {
    grouped_commands.ingest(joint->build_grouped_command(0.0));
    for (const auto & frame : joint->build_stop_sequence()) {
      ok = board_->send_frame(frame) && ok;
    }
  }

  for (const auto & frame : grouped_commands.flush()) {
    ok = board_->send_frame(frame) && ok;
  }
  commands_zeroed_ = true;
  last_command_send_ = std::chrono::steady_clock::now();
  return ok;
}

}  // namespace librmcs_hardware

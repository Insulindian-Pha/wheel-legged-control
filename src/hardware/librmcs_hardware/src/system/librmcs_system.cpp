#include "librmcs_hardware/system/librmcs_system.hpp"

#include <algorithm>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace librmcs_hardware {

namespace {

std::string join_names(const std::vector<std::string> & names) {
  std::ostringstream stream;
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (index != 0) {
      stream << ", ";
    }
    stream << names[index];
  }
  return stream.str();
}

}  // namespace

LibrmcsSystem::LibrmcsSystem()
: logger_(rclcpp::get_logger("librmcs_hardware")) {}

LibrmcsSystem::~LibrmcsSystem() {
  if (input_node_) {
    try {
      input_executor_.remove_node(input_node_);
    } catch (...) {
    }
  }
  if (input_spin_thread_.joinable()) {
    input_executor_.cancel();
    input_spin_thread_.join();
  }
  if (driver_) {
    driver_->stop();
  }
}

hardware_interface::CallbackReturn
#if LIBRMCS_HARDWARE_ROS_DISTRO_HUMBLE
LibrmcsSystem::on_init(const hardware_interface::HardwareInfo & info)
#else
LibrmcsSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
#endif
{
  if (hardware_interface::SystemInterface::on_init(
#if LIBRMCS_HARDWARE_ROS_DISTRO_HUMBLE
        info
#else
        params
#endif
      ) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    usb_pid_ = std::stoi(get_parameter(info_.hardware_parameters, "usb_pid", "-1"));
    startup_timeout_ = std::chrono::milliseconds(
      std::stoi(get_parameter(info_.hardware_parameters, "startup_timeout_ms", "1500")));
    command_timeout_ = std::chrono::milliseconds(
      std::stoi(get_parameter(info_.hardware_parameters, "command_timeout_ms", "100")));
    allow_partial_feedback_ =
      parse_bool(get_parameter(info_.hardware_parameters, "allow_partial_feedback", "false"));
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger_, "Failed to parse hardware parameters: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto joint_count = info_.joints.size();
  hw_positions_.assign(joint_count, 0.0);
  hw_velocities_.assign(joint_count, 0.0);
  hw_efforts_.assign(joint_count, 0.0);
  hw_commands_.assign(joint_count, 0.0);
  joint_configs_.clear();
  joint_configs_.reserve(joint_count);

  try {
    std::set<std::pair<CanBus, uint32_t>> used_can_ids;
    for (const auto & joint : info_.joints) {
      if (joint.command_interfaces.size() != 1 ||
          joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT) {
        RCLCPP_ERROR(
          logger_, "Joint '%s' must expose exactly one effort command interface.",
          joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      const bool has_position = std::any_of(
        joint.state_interfaces.begin(), joint.state_interfaces.end(),
        [](const auto & interface_info) {
          return interface_info.name == hardware_interface::HW_IF_POSITION;
        });
      const bool has_velocity = std::any_of(
        joint.state_interfaces.begin(), joint.state_interfaces.end(),
        [](const auto & interface_info) {
          return interface_info.name == hardware_interface::HW_IF_VELOCITY;
        });
      const bool has_effort = std::any_of(
        joint.state_interfaces.begin(), joint.state_interfaces.end(),
        [](const auto & interface_info) {
          return interface_info.name == hardware_interface::HW_IF_EFFORT;
        });

      if (!has_position || !has_velocity || !has_effort) {
        RCLCPP_ERROR(
          logger_, "Joint '%s' must expose position, velocity, and effort state interfaces.",
          joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      JointConfig config;
      config.name = joint.name;
      config.vendor = parse_vendor(get_parameter(joint.parameters, "vendor"));
      config.model = get_parameter(joint.parameters, "model");
      config.can_bus = parse_can_bus(get_parameter(joint.parameters, "can_bus", "can1"));
      config.can_id = static_cast<uint32_t>(std::stoul(get_parameter(joint.parameters, "can_id"), nullptr, 0));
      config.encoder_zero_point = std::stoi(get_parameter(joint.parameters, "encoder_zero_point", "0"));
      config.reduction_ratio = std::stod(get_parameter(joint.parameters, "reduction_ratio", "0.0"));
      config.reversed = parse_bool(get_parameter(joint.parameters, "reversed", "false"));
      config.multi_turn_angle = parse_bool(get_parameter(joint.parameters, "multi_turn_angle", "false"));

      const auto can_key = std::make_pair(config.can_bus, config.can_id);
      if (!used_can_ids.insert(can_key).second) {
        RCLCPP_ERROR(
          logger_,
          "Duplicate CAN id 0x%X on the same bus as another joint — check URDF for '%s'.",
          config.can_id, joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      joint_configs_.push_back(config);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger_, "Failed to parse joint parameters: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LibrmcsSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 3);

  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    state_interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_POSITION, &hw_positions_[index]);
    state_interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[index]);
    state_interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[index]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LibrmcsSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    command_interfaces.emplace_back(
      info_.joints[index].name, hardware_interface::HW_IF_EFFORT, &hw_commands_[index]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn LibrmcsSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    driver_ = std::make_unique<LibrmcsRobotDriver>(usb_pid_, command_timeout_, joint_configs_);
    // 只有收到 start 指令才发送 enable（默认不上电）。
    driver_->start(false);
    motors_enabled_.store(false);

    if (!input_node_) {
      input_node_ = std::make_shared<rclcpp::Node>("librmcs_hardware_input_gate");
      input_sub_ = input_node_->create_subscription<control_input_msgs::msg::Inputs>(
        "control_input", rclcpp::QoS(10),
        [this](const control_input_msgs::msg::Inputs::SharedPtr msg) {
          // start 由发布端锁存（按下 START 后保持 1，按下 STOP 置 0）。
          // 此处只在状态切换时发送一次使能/失能（stop 优先）。
          if (msg->stop) {
            if (driver_) {
              driver_->disable_motors();
            }
            motors_enabled_.store(false);
          } else if (msg->start && !motors_enabled_.load()) {
            if (driver_) {
              driver_->enable_motors();
            }
            motors_enabled_.store(true);
          } else if (!msg->start && motors_enabled_.load()) {
            if (driver_) {
              driver_->disable_motors();
            }
            motors_enabled_.store(false);
          }
        });

      input_executor_.add_node(input_node_);
      input_spin_thread_ = std::thread([this]() { input_executor_.spin(); });
    }

    // 注意：默认不使能电机，可能不会有反馈；因此这里不强制等待反馈。
    if (motors_enabled_.load() && !driver_->wait_for_feedback(startup_timeout_)) {
      const auto missing_joint_names = driver_->missing_feedback_joint_names();
      const auto missing_summary = join_names(missing_joint_names);
      if (!allow_partial_feedback_) {
        RCLCPP_ERROR(
          logger_,
          "Timed out waiting for initial motor feedback. Missing joints: %s",
          missing_summary.c_str());
        driver_->stop();
        driver_.reset();
        return hardware_interface::CallbackReturn::ERROR;
      }

      RCLCPP_WARN(
        logger_,
        "Timed out waiting for initial motor feedback, but allow_partial_feedback is enabled. "
        "Continuing startup without requiring any initial feedback. Missing joints: %s",
        missing_summary.c_str());
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger_, "Failed to activate librmcs hardware: %s", ex.what());
    driver_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  const auto states = driver_->read_joint_states();
  for (std::size_t index = 0; index < states.size(); ++index) {
    hw_positions_[index] = states[index].position;
    hw_velocities_[index] = states[index].velocity;
    hw_efforts_[index] = states[index].effort;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LibrmcsSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  motors_enabled_.store(false);
  if (driver_) {
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    driver_->write_joint_efforts(hw_commands_);
    driver_->stop();
    driver_.reset();
  }

  if (input_node_) {
    try {
      input_executor_.remove_node(input_node_);
    } catch (...) {
    }
    input_node_.reset();
    input_sub_.reset();
  }
  if (input_spin_thread_.joinable()) {
    input_executor_.cancel();
    input_spin_thread_.join();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type LibrmcsSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!driver_) {
    return hardware_interface::return_type::ERROR;
  }

  const auto states = driver_->read_joint_states();
  if (states.size() != info_.joints.size()) {
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t index = 0; index < states.size(); ++index) {
    hw_positions_[index] = states[index].position;
    hw_velocities_[index] = states[index].velocity;
    hw_efforts_[index] = states[index].effort;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LibrmcsSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!driver_) {
    return hardware_interface::return_type::ERROR;
  }

  if (!motors_enabled_.load()) {
    // 未使能时不下发力矩命令（disable 指令由 stop 上升沿触发发送）
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    return hardware_interface::return_type::OK;
  }

  return driver_->write_joint_efforts(hw_commands_)
           ? hardware_interface::return_type::OK
           : hardware_interface::return_type::ERROR;
}

std::string LibrmcsSystem::get_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & key,
  const std::string & default_value)
{
  const auto iter = parameters.find(key);
  if (iter == parameters.end()) {
    if (default_value.empty()) {
      throw std::runtime_error("Missing required hardware parameter: " + key);
    }
    return default_value;
  }
  return iter->second;
}

bool LibrmcsSystem::parse_bool(const std::string & value, bool default_value) {
  if (value == "true" || value == "True" || value == "1") {
    return true;
  }
  if (value == "false" || value == "False" || value == "0") {
    return false;
  }
  return default_value;
}

JointConfig::Vendor LibrmcsSystem::parse_vendor(const std::string & value) {
  if (value == "LK") {
    return JointConfig::Vendor::Lk;
  }
  if (value == "DM") {
    return JointConfig::Vendor::Dm;
  }
  if (value == "DJI") {
    return JointConfig::Vendor::Dji;
  }
  if (value == "BM") {
    return JointConfig::Vendor::Bm;
  }
  throw std::runtime_error("Unsupported motor vendor: " + value);
}

CanBus LibrmcsSystem::parse_can_bus(const std::string & value) {
  if (value == "can1" || value == "CAN1") {
    return CanBus::Can1;
  }
  if (value == "can2" || value == "CAN2") {
    return CanBus::Can2;
  }
  throw std::runtime_error("Unsupported CAN bus: " + value);
}

}  // namespace librmcs_hardware

PLUGINLIB_EXPORT_CLASS(librmcs_hardware::LibrmcsSystem, hardware_interface::SystemInterface)

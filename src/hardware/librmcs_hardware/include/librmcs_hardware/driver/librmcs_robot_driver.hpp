#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "librmcs_hardware/config/motor_config.hpp"

namespace librmcs_hardware {

class MotorDeviceBase;

class LibrmcsRobotDriver {
public:
  LibrmcsRobotDriver(
    int32_t usb_pid,
    std::chrono::milliseconds command_timeout,
    std::vector<JointConfig> joint_configs);
  ~LibrmcsRobotDriver();

  LibrmcsRobotDriver(const LibrmcsRobotDriver &) = delete;
  LibrmcsRobotDriver & operator=(const LibrmcsRobotDriver &) = delete;

  void start(bool enable_on_start = true);
  void stop();

  // 运行期间按需使能/失能电机（发送各电机的 enable/disable 序列）。
  // 这些方法是线程安全的；若驱动未运行则返回 false。
  bool enable_motors();
  bool disable_motors();

  bool wait_for_feedback(std::chrono::milliseconds timeout) const;
  bool any_feedback_received() const;
  std::vector<std::string> missing_feedback_joint_names() const;
  std::vector<JointState> read_joint_states() const;
  bool write_joint_efforts(std::span<const double> efforts);

private:
  class RobotBoard;

  void handle_can_frame(CanBus can_bus, uint32_t can_id, uint64_t can_data);
  void watchdog_loop();

  bool send_activation_frames_locked();
  bool send_startup_frames_locked();
  bool send_effort_commands_locked();
  bool send_stop_commands_locked();

  const int32_t usb_pid_;
  const std::chrono::milliseconds command_timeout_;
  const std::chrono::milliseconds feedback_timeout_;

  std::vector<std::unique_ptr<MotorDeviceBase>> joints_;
  std::vector<double> command_efforts_;

  mutable std::mutex mutex_;
  std::unique_ptr<RobotBoard> board_;
  std::thread event_thread_;
  std::thread watchdog_thread_;
  bool running_ = false;
  bool commands_zeroed_ = true;
  std::chrono::steady_clock::time_point last_command_update_{};
  std::chrono::steady_clock::time_point last_command_send_{};
};

}  // namespace librmcs_hardware

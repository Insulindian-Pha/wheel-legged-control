#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "librmcs_hardware/config/motor_config.hpp"

namespace librmcs_hardware {

struct CanFrame {
  uint32_t can_id = 0;
  uint64_t can_data = 0;
  CanBus can_bus = CanBus::Can1;
};

struct GroupedCanCommand {
  bool valid = false;
  CanBus can_bus = CanBus::Can1;
  uint32_t tx_can_id = 0;
  std::size_t slot_index = 0;
  std::size_t slot_width_bytes = 0;
  std::size_t slot_count = 0;
  uint64_t payload = 0;
};

class MotorDeviceBase {
public:
  explicit MotorDeviceBase(JointConfig config);
  virtual ~MotorDeviceBase() = default;

  const JointConfig & config() const { return config_; }
  const JointState & state() const { return state_; }
  bool feedback_received() const { return feedback_received_; }
  std::chrono::steady_clock::time_point last_feedback_time() const { return last_feedback_time_; }

  bool accepts_feedback(CanBus can_bus, uint32_t can_id) const;
  void parse_feedback(CanBus can_bus, uint32_t can_id, uint64_t can_data);
  bool is_online(std::chrono::steady_clock::time_point now, std::chrono::milliseconds timeout) const;

  virtual std::vector<CanFrame> build_activate_sequence() const = 0;
  virtual std::vector<CanFrame> build_startup_sequence() const;
  virtual std::vector<GroupedCanCommand> build_startup_grouped_commands() const;
  virtual std::vector<CanFrame> build_stop_sequence() const = 0;
  virtual std::vector<CanFrame> build_direct_command(double effort) const = 0;
  virtual GroupedCanCommand build_grouped_command(double effort) const;

protected:
  virtual void store_feedback(uint64_t can_data) = 0;
  virtual void update_from_feedback() = 0;
  virtual double angle() const = 0;
  virtual double velocity() const = 0;
  virtual double torque() const = 0;

  JointConfig config_;
  JointState state_;
  bool feedback_received_ = false;
  std::chrono::steady_clock::time_point last_feedback_time_{};
};

}  // namespace librmcs_hardware

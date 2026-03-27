#include "librmcs_hardware/devices/motor_device_base.hpp"

#include <cmath>

namespace librmcs_hardware {

MotorDeviceBase::MotorDeviceBase(JointConfig config)
: config_(std::move(config)) {}

bool MotorDeviceBase::accepts_feedback(CanBus can_bus, uint32_t can_id) const {
  return config_.can_bus == can_bus && config_.can_id == can_id;
}

void MotorDeviceBase::parse_feedback(CanBus can_bus, uint32_t can_id, uint64_t can_data) {
  if (!accepts_feedback(can_bus, can_id)) {
    return;
  }

  store_feedback(can_data);
  update_from_feedback();
  state_.position = angle();
  state_.velocity = velocity();
  state_.effort = torque();
  state_.online = true;
  feedback_received_ = true;
  last_feedback_time_ = std::chrono::steady_clock::now();
}

bool MotorDeviceBase::is_online(
  std::chrono::steady_clock::time_point now,
  std::chrono::milliseconds timeout) const
{
  return feedback_received_ && now - last_feedback_time_ <= timeout;
}

std::vector<CanFrame> MotorDeviceBase::build_startup_sequence() const {
  return {};
}

std::vector<GroupedCanCommand> MotorDeviceBase::build_startup_grouped_commands() const {
  return {};
}

GroupedCanCommand MotorDeviceBase::build_grouped_command(double /*effort*/) const {
  return {};
}

}  // namespace librmcs_hardware

#include "librmcs_hardware/devices/dm_motor_device.hpp"

#include <stdexcept>

namespace librmcs_hardware {

DmMotorDevice::DmMotorDevice(const JointConfig & config)
: MotorDeviceBase(config),
  motor_([&config]() {
    auto motor_config = librmcs::device::DmMotor::Config{to_motor_type(config.model)}
                          .set_encoder_zero_point(config.encoder_zero_point);
    if (config.reversed) {
      motor_config.set_reversed();
    }
    if (config.multi_turn_angle) {
      motor_config.enable_multi_turn_angle();
    }
    return motor_config;
  }()) {}

std::vector<CanFrame> DmMotorDevice::build_activate_sequence() const {
  return {
    CanFrame{config_.can_id, librmcs::device::DmMotor::generate_clear_error_command(), config_.can_bus},
    CanFrame{config_.can_id, librmcs::device::DmMotor::generate_enable_command(), config_.can_bus},
  };
}

std::vector<CanFrame> DmMotorDevice::build_stop_sequence() const {
  return {CanFrame{config_.can_id, motor_.generate_disable_command(), config_.can_bus}};
}

std::vector<CanFrame> DmMotorDevice::build_direct_command(double effort) const {
  return {CanFrame{config_.can_id, motor_.generate_torque_command(effort), config_.can_bus}};
}

void DmMotorDevice::store_feedback(uint64_t can_data) { motor_.store_status(can_data); }
void DmMotorDevice::update_from_feedback() { motor_.update_status(); }
double DmMotorDevice::angle() const { return motor_.angle(); }
double DmMotorDevice::velocity() const { return motor_.velocity(); }
double DmMotorDevice::torque() const { return motor_.torque(); }

librmcs::device::DmMotor::Type DmMotorDevice::to_motor_type(const std::string & model) {
  if (model == "DM8009") {
    return librmcs::device::DmMotor::Type::DM8009;
  }
  throw std::runtime_error("Unsupported DM model: " + model);
}

}  // namespace librmcs_hardware

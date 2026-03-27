#include "librmcs_hardware/devices/lk_motor_device.hpp"

#include <stdexcept>

namespace librmcs_hardware {

LkMotorDevice::LkMotorDevice(const JointConfig & config)
: MotorDeviceBase(config),
  motor_([&config]() {
    auto motor_config = librmcs::device::LkMotor::Config{to_motor_type(config.model)}
                          .set_encoder_zero_point(config.encoder_zero_point);
    if (config.reversed) {
      motor_config.set_reversed();
    }
    if (config.multi_turn_angle) {
      motor_config.enable_multi_turn_angle();
    }
    return motor_config;
  }()) {}

std::vector<CanFrame> LkMotorDevice::build_activate_sequence() const {
  return {
    CanFrame{config_.can_id, librmcs::device::LkMotor::generate_startup_command(), config_.can_bus},
    CanFrame{config_.can_id, librmcs::device::LkMotor::generate_status_request(), config_.can_bus},
  };
}

std::vector<CanFrame> LkMotorDevice::build_stop_sequence() const {
  return {
    CanFrame{config_.can_id, librmcs::device::LkMotor::generate_disable_command(), config_.can_bus},
  };
}

std::vector<CanFrame> LkMotorDevice::build_direct_command(double effort) const {
  return {CanFrame{config_.can_id, motor_.generate_torque_command(effort), config_.can_bus}};
}

void LkMotorDevice::store_feedback(uint64_t can_data) { motor_.store_status(can_data); }
void LkMotorDevice::update_from_feedback() { motor_.update_status(); }
double LkMotorDevice::angle() const { return motor_.angle(); }
double LkMotorDevice::velocity() const { return motor_.velocity(); }
double LkMotorDevice::torque() const { return motor_.torque(); }

librmcs::device::LkMotor::Type LkMotorDevice::to_motor_type(const std::string & model) {
  if (model == "MG5010E_I10") {
    return librmcs::device::LkMotor::Type::MG5010E_I10;
  }
  if (model == "MG4010E_I10") {
    return librmcs::device::LkMotor::Type::MG4010E_I10;
  }
  throw std::runtime_error("Unsupported LK model: " + model);
}

}  // namespace librmcs_hardware

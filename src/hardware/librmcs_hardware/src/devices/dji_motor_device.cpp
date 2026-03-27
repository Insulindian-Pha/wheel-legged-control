#include "librmcs_hardware/devices/dji_motor_device.hpp"

#include <stdexcept>

namespace librmcs_hardware {

DjiMotorDevice::DjiMotorDevice(const JointConfig & config)
: MotorDeviceBase(config),
  motor_([&config]() {
    auto motor_config = librmcs::device::DjiMotor::Config{to_motor_type(config.model)}
                          .set_encoder_zero_point(config.encoder_zero_point);
    if (config.reduction_ratio > 0.0) {
      motor_config.set_reduction_ratio(config.reduction_ratio);
    }
    if (config.reversed) {
      motor_config.set_reversed();
    }
    if (config.multi_turn_angle) {
      motor_config.enable_multi_turn_angle();
    }
    return motor_config;
  }()) {}

std::vector<CanFrame> DjiMotorDevice::build_activate_sequence() const { return {}; }
std::vector<CanFrame> DjiMotorDevice::build_stop_sequence() const { return {}; }
std::vector<CanFrame> DjiMotorDevice::build_direct_command(double /*effort*/) const { return {}; }

GroupedCanCommand DjiMotorDevice::build_grouped_command(double effort) const {
  const auto low_group = config_.can_id >= 0x201 && config_.can_id <= 0x204;
  const auto high_group = config_.can_id >= 0x205 && config_.can_id <= 0x208;
  if (!low_group && !high_group) {
    return {};
  }

  return GroupedCanCommand{
    .valid = true,
    .can_bus = config_.can_bus,
    .tx_can_id = high_group ? 0x1FFU : 0x200U,
    .slot_index = high_group ? config_.can_id - 0x205U : config_.can_id - 0x201U,
    .slot_width_bytes = sizeof(uint16_t),
    .slot_count = 4,
    .payload = motor_.generate_command(effort),
  };
}

void DjiMotorDevice::store_feedback(uint64_t can_data) { motor_.store_status(can_data); }
void DjiMotorDevice::update_from_feedback() { motor_.update_status(); }
double DjiMotorDevice::angle() const { return motor_.angle(); }
double DjiMotorDevice::velocity() const { return motor_.velocity(); }
double DjiMotorDevice::torque() const { return motor_.torque(); }

librmcs::device::DjiMotor::Type DjiMotorDevice::to_motor_type(const std::string & model) {
  if (model == "GM6020") {
    return librmcs::device::DjiMotor::Type::GM6020;
  }
  if (model == "GM6020_VOLTAGE") {
    return librmcs::device::DjiMotor::Type::GM6020_VOLTAGE;
  }
  if (model == "M2006") {
    return librmcs::device::DjiMotor::Type::M2006;
  }
  if (model == "M3508") {
    return librmcs::device::DjiMotor::Type::M3508;
  }
  throw std::runtime_error("Unsupported DJI model: " + model);
}

}  // namespace librmcs_hardware

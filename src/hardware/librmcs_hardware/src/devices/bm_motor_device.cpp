#include "librmcs_hardware/devices/bm_motor_device.hpp"

#include <stdexcept>

namespace librmcs_hardware {

namespace {

constexpr std::size_t kBmWarmupCount = 10;
constexpr uint32_t kP1010StateCommandId = 0x38;

}  // namespace

BmMotorDevice::BmMotorDevice(const JointConfig & config)
: MotorDeviceBase(config),
  motor_([&config]() {
    auto motor_config = librmcs::device::BmMotor::Config{to_motor_type(config.model)}
                          .set_encoder_zero_point(config.encoder_zero_point);
    if (config.reversed) {
      motor_config.set_reversed();
    }
    return motor_config;
  }())
{
  motor_.set_feedback_can_id(config.can_id);
}

std::vector<CanFrame> BmMotorDevice::build_activate_sequence() const {
  if (!motor_.has_state_command()) {
    return {};
  }
  if (motor_.command_slot() != 0) {
    return {};
  }

  return {CanFrame{kP1010StateCommandId, motor_.generate_enable_command(), config_.can_bus}};
}

std::vector<CanFrame> BmMotorDevice::build_startup_sequence() const {
  return {};
}

std::vector<GroupedCanCommand> BmMotorDevice::build_startup_grouped_commands() const {
  if (!motor_.needs_startup_warmup()) {
    return {};
  }

  return std::vector<GroupedCanCommand>(kBmWarmupCount, build_grouped_command(0.0));
}

std::vector<CanFrame> BmMotorDevice::build_stop_sequence() const {
  if (motor_.has_state_command()) {
    if (motor_.command_slot() != 0) {
      return {};
    }
    return {CanFrame{kP1010StateCommandId, motor_.generate_disable_command(), config_.can_bus}};
  }
  return {};
}

std::vector<CanFrame> BmMotorDevice::build_direct_command(double effort) const {
  (void)effort;
  return {};
}

GroupedCanCommand BmMotorDevice::build_grouped_command(double effort) const {
  return GroupedCanCommand{
    .valid = true,
    .can_bus = config_.can_bus,
    .tx_can_id = motor_.command_can_id(),
    .slot_index = motor_.command_slot(),
    .slot_width_bytes = sizeof(uint16_t),
    .slot_count = motor_.group_slot_count(),
    .payload = motor_.generate_group_command_word(effort),
  };
}

void BmMotorDevice::store_feedback(uint64_t can_data) { motor_.store_status(can_data); }
void BmMotorDevice::update_from_feedback() { motor_.update_status(); }
double BmMotorDevice::angle() const { return motor_.angle(); }
double BmMotorDevice::velocity() const { return motor_.velocity(); }
double BmMotorDevice::torque() const { return motor_.torque(); }

librmcs::device::BmMotor::Type BmMotorDevice::to_motor_type(const std::string & model) {
  if (model == "P1010B_111") {
    return librmcs::device::BmMotor::Type::P1010B_111;
  }
  if (model == "M1505B_111") {
    return librmcs::device::BmMotor::Type::M1505B_111;
  }
  throw std::runtime_error("Unsupported BM model: " + model);
}

}  // namespace librmcs_hardware

#pragma once

#include <librmcs/device/dm_motor.hpp>

#include "librmcs_hardware/devices/motor_device_base.hpp"

namespace librmcs_hardware {

class DmMotorDevice : public MotorDeviceBase {
public:
  explicit DmMotorDevice(const JointConfig & config);

  std::vector<CanFrame> build_activate_sequence() const override;
  std::vector<CanFrame> build_stop_sequence() const override;
  std::vector<CanFrame> build_direct_command(double effort) const override;

protected:
  void store_feedback(uint64_t can_data) override;
  void update_from_feedback() override;
  double angle() const override;
  double velocity() const override;
  double torque() const override;

private:
  static librmcs::device::DmMotor::Type to_motor_type(const std::string & model);

  librmcs::device::DmMotor motor_;
};

}  // namespace librmcs_hardware

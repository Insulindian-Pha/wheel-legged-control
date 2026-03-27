#pragma once

#include <librmcs/device/dji_motor.hpp>

#include "librmcs_hardware/devices/motor_device_base.hpp"

namespace librmcs_hardware {

class DjiMotorDevice : public MotorDeviceBase {
public:
  explicit DjiMotorDevice(const JointConfig & config);

  std::vector<CanFrame> build_activate_sequence() const override;
  std::vector<CanFrame> build_stop_sequence() const override;
  std::vector<CanFrame> build_direct_command(double effort) const override;
  GroupedCanCommand build_grouped_command(double effort) const override;

protected:
  void store_feedback(uint64_t can_data) override;
  void update_from_feedback() override;
  double angle() const override;
  double velocity() const override;
  double torque() const override;

private:
  static librmcs::device::DjiMotor::Type to_motor_type(const std::string & model);

  librmcs::device::DjiMotor motor_;
};

}  // namespace librmcs_hardware

#pragma once

#include <memory>

#include "librmcs_hardware/config/motor_config.hpp"

namespace librmcs_hardware {

class MotorDeviceBase;

std::unique_ptr<MotorDeviceBase> make_motor_device(const JointConfig & config);

}  // namespace librmcs_hardware

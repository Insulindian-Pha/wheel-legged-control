#include "librmcs_hardware/devices/motor_device_factory.hpp"

#include <memory>
#include <stdexcept>

#include "librmcs_hardware/devices/dji_motor_device.hpp"
#include "librmcs_hardware/devices/bm_motor_device.hpp"
#include "librmcs_hardware/devices/dm_motor_device.hpp"
#include "librmcs_hardware/devices/lk_motor_device.hpp"

namespace librmcs_hardware {

std::unique_ptr<MotorDeviceBase> make_motor_device(const JointConfig & config) {
  switch (config.vendor) {
    case JointConfig::Vendor::Dji:
      return std::make_unique<DjiMotorDevice>(config);
    case JointConfig::Vendor::Lk:
      return std::make_unique<LkMotorDevice>(config);
    case JointConfig::Vendor::Dm:
      return std::make_unique<DmMotorDevice>(config);
    case JointConfig::Vendor::Bm:
      return std::make_unique<BmMotorDevice>(config);
  }

  throw std::runtime_error("Unsupported motor vendor.");
}

}  // namespace librmcs_hardware

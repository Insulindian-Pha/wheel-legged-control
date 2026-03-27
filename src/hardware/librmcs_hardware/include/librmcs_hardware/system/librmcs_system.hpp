#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/version.h>
#include <rclcpp/logger.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "librmcs_hardware/driver/librmcs_robot_driver.hpp"

#define LIBRMCS_HARDWARE_ROS_DISTRO_HUMBLE (HARDWARE_INTERFACE_VERSION_MAJOR < 3)

namespace librmcs_hardware {

class LibrmcsSystem : public hardware_interface::SystemInterface {
public:
  LibrmcsSystem();
  ~LibrmcsSystem() override;

  hardware_interface::CallbackReturn
#if LIBRMCS_HARDWARE_ROS_DISTRO_HUMBLE
  on_init(const hardware_interface::HardwareInfo & info) override;
#else
  on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
#endif

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static std::string get_parameter(
    const std::unordered_map<std::string, std::string> & parameters,
    const std::string & key,
    const std::string & default_value = "");
  static bool parse_bool(const std::string & value, bool default_value = false);
  static JointConfig::Vendor parse_vendor(const std::string & value);
  static CanBus parse_can_bus(const std::string & value);

  int32_t usb_pid_ = -1;
  std::chrono::milliseconds startup_timeout_{1500};
  std::chrono::milliseconds command_timeout_{100};
  bool allow_partial_feedback_ = false;
  std::vector<JointConfig> joint_configs_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_commands_;
  std::unique_ptr<LibrmcsRobotDriver> driver_;
  rclcpp::Logger logger_;
};

}  // namespace librmcs_hardware

#pragma once

#include <chrono>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/version.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <control_input_msgs/msg/inputs.hpp>

#include "librmcs_hardware/devices/bmi088_device.hpp"

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
  struct ImuState
  {
    double orientation_x{0.0};
    double orientation_y{0.0};
    double orientation_z{0.0};
    double orientation_w{1.0};
    double angular_velocity_x{0.0};
    double angular_velocity_y{0.0};
    double angular_velocity_z{0.0};
    double linear_acceleration_x{0.0};
    double linear_acceleration_y{0.0};
    double linear_acceleration_z{0.0};
  };

  static std::string get_parameter(
    const std::unordered_map<std::string, std::string> & parameters,
    const std::string & key,
    const std::string & default_value = "");
  static bool parse_bool(const std::string & value, bool default_value = false);
  static double parse_double(const std::string & value, double default_value);
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
  ImuState imu_state_;
  std::unique_ptr<Bmi088Processor> bmi088_processor_;
  std::string imu_sensor_name_{"imu_sensor"};
  Bmi088Processor::Config imu_config_{};
  std::unique_ptr<LibrmcsRobotDriver> driver_;
  rclcpp::Logger logger_;

  std::atomic<bool> motors_enabled_{false};

  std::shared_ptr<rclcpp::Node> input_node_;
  rclcpp::executors::SingleThreadedExecutor input_executor_;
  std::thread input_spin_thread_;
  rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr input_sub_;
};

}  // namespace librmcs_hardware

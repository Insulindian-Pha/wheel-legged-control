//
// LQR Controller for ros2_control
// Implements Linear Quadratic Regulator for wheel-legged robot balance control
//

#ifndef LQRCONTROLLER_H
#define LQRCONTROLLER_H

#include <memory>
#include <string>
#include <vector>
#include <array>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "vmc_controller/msg/vmc_state.hpp"
#include "lqr_controller/msg/lqr_state.hpp"

namespace lqr_controller
{

class LQRController : public controller_interface::ControllerInterface
{
public:
  LQRController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  // Wheel joint handles
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> left_wheel_cmd_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> right_wheel_cmd_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> left_wheel_state_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> right_wheel_state_;

  // Joint names
  std::string left_wheel_joint_name_;
  std::string right_wheel_joint_name_;

  // VMC state subscription
  rclcpp::Subscription<vmc_controller::msg::VMCState>::SharedPtr vmc_state_subscription_;
  std::string vmc_state_topic_;

  // Force command publisher (for Tp updates)
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr force_command_publisher_;
  std::string force_command_topic_;

  // LQR state publisher
  rclcpp::Publisher<lqr_controller::msg::LQRState>::SharedPtr lqr_state_publisher_;
  std::string lqr_state_topic_;

  // VMC state data (from subscription)
  double left_theta_;
  double left_d_theta_;
  double right_theta_;
  double right_d_theta_;
  double pitch_;
  double pitch_gyro_;
  double left_F0_;  // Current F0 from VMC (to preserve when updating Tp)
  double right_F0_;
  bool received_vmc_state_;

  // LQR gain matrices (12 values for each leg)
  std::array<double, 12> left_lqr_gains_;
  std::array<double, 12> right_lqr_gains_;

  // LQR gain polarity (12 values for each leg)
  // 1.0: keep original polarity, -1.0: invert
  std::array<double, 12> left_lqr_gain_polarity_;
  std::array<double, 12> right_lqr_gain_polarity_;

  // State estimation (simple integration from wheel velocity)
  double x_position_;  // Current position (m)
  double x_velocity_;  // Current velocity (m/s)
  double x_set_;       // Desired position (m)
  double v_set_;       // Desired velocity (m/s)

  // Parameters
  double wheel_radius_;      // Wheel radius for velocity calculation (m)
  double max_wheel_torque_;  // Maximum wheel torque (Nm)

  // Callbacks
  void vmc_state_callback(const vmc_controller::msg::VMCState::SharedPtr msg);

  // Helper functions
  void calculate_lqr_control(double& wheel_torque_left, double& wheel_torque_right, double& tp_left, double& tp_right);
  void update_state_estimation(double dt);
  void publish_force_command(double left_F0, double left_Tp, double right_F0, double right_Tp);
  void publish_lqr_state(double wheel_torque_left, double wheel_torque_right, double tp_left, double tp_right);
};

}  // namespace lqr_controller

#endif  // LQRCONTROLLER_H

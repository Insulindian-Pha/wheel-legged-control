//
// LQR Controller for ros2_control
// Implements Linear Quadratic Regulator for wheel-legged robot balance control
//

#ifndef LQRCONTROLLER_H
#define LQRCONTROLLER_H

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "lqr_controller/odom/odom_publisher.hpp"

#include "control_input_msgs/msg/inputs.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lqr_controller/msg/lqr_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "vmc_controller/msg/vmc_state.hpp"

#include "td_quadratic.hpp"

namespace lqr_controller {

class LQRController : public controller_interface::ControllerInterface {
public:
  LQRController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

protected:
  // Wheel joint handles
  std::vector<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      left_wheel_cmd_;
  std::vector<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      right_wheel_cmd_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      left_wheel_state_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      right_wheel_state_;

  // Joint names
  std::string left_wheel_joint_name_;
  std::string right_wheel_joint_name_;

  // VMC state subscription
  rclcpp::Subscription<vmc_controller::msg::VMCState>::SharedPtr
      vmc_state_subscription_;
  std::string vmc_state_topic_;

  // IMU subscription (optional): when imu_topic is non-empty,
  // pitch/pitch_gyro/yaw/yaw_gyro come from IMU
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  std::string imu_topic_;
  bool use_imu_for_state_; // true when imu_topic_ is non-empty
  bool received_imu_;      // true after first IMU message received

  // Control input subscription: ly -> v_set, lx -> dot_fai_set
  rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr
      control_input_subscription_;
  std::string control_input_topic_;

  // Force command publisher (for Tp updates)
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      force_command_publisher_;
  std::string force_command_topic_;

  // LQR state publisher
  rclcpp::Publisher<lqr_controller::msg::LQRState>::SharedPtr
      lqr_state_publisher_;
  std::string lqr_state_topic_;

  // Odometry handler (IMU subscribe inside; wheel v_body passed in)
  std::shared_ptr<odom::OdomPublisher> odom_handler_ = nullptr;
  std::string odom_topic_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  bool enable_odom_tf_ = true;
  bool enable_odom_msg_ = true;
  bool require_imu_for_odom_ = true;
  bool odom_use_pitch_ = true;
  double odom_publish_rate_ = 50.0;  // Hz

  // VMC state data (from subscription)
  double left_theta_;
  double left_d_theta_;
  double right_theta_;
  double right_d_theta_;
  double pitch_;
  double pitch_gyro_;
  double roll_gyro_;
  double left_F0_; // Current F0 from VMC (to preserve when updating Tp)
  double right_F0_;
  bool received_vmc_state_;

  // LQR gain matrices (10 values for each output: 4 outputs total)
  // Left wheel torque gains (10 values)
  std::array<double, 10> left_wheel_lqr_gains_;
  // Right wheel torque gains (10 values)
  std::array<double, 10> right_wheel_lqr_gains_;
  // Left leg torque gains (10 values)
  std::array<double, 10> left_leg_lqr_gains_;
  // Right leg torque gains (10 values)
  std::array<double, 10> right_leg_lqr_gains_;

  // State variable polarity (10 values, one for each state)
  // 1.0: keep original polarity, -1.0: invert
  // Applied to state values before multiplying with gains
  std::array<double, 10> state_polarity_;

  // 增益极性：每个输出、每个状态分量单独配置 (1.0 保持, -1.0 取反)
  // 实际参与计算为 gain[i] * gain_polarity[i]，便于保留原始 K 值仅用极性调符号
  std::array<double, 10> left_wheel_gain_polarity_;
  std::array<double, 10> right_wheel_gain_polarity_;
  std::array<double, 10> left_leg_gain_polarity_;
  std::array<double, 10> right_leg_gain_polarity_;

  // State estimation (simple integration from wheel velocity)
  double x_position_; // Current position (m) - corresponds to 's' in LQR
  double x_velocity_; // Current velocity (m/s) - corresponds to 'dot_s' in LQR
  // Raw command from control_input (ly -> v_cmd_; lx -> yaw_cmd_ = lx * yaw_max_)
  double v_cmd_;       // Desired velocity command (m/s)
  double yaw_cmd_;     // Desired yaw rate command (rad/s) = lx * yaw_max_
  // Raw setpoints: position integrated from v_cmd_; yaw integrated from yaw_cmd_
  double x_set_raw_;   // Integrated position target (m)
  double yaw_set_raw_; // Integrated yaw target (rad), yaw_set_raw_ += yaw_cmd_ * dt
  // AddCaclu-style unwrap: last_yaw_ (rad), unwrapped_yaw_ (rad) for angle normalization
  double last_yaw_;      // Last wrapped yaw (rad) for delta computation
  double unwrapped_yaw_; // Cumulative unwrapped yaw (rad)
  // TD output: planned setpoints fed to LQR (velocity-planned)
  double x_set_;       // Desired position (m), from TD getX1
  double v_set_;       // Desired velocity (m/s), from TD getX2 (position TD)
  double fai_set_;     // Desired yaw (rad), from TD getX1 (yaw TD)
  double dot_fai_set_; // Desired yaw rate (rad/s), from TD getX2 (yaw TD)
  double yaw_;         // Yaw angle (rad) - corresponds to 'fai' in LQR
  double yaw_gyro_;    // Yaw angular velocity (rad/s) - corresponds to 'dot_fai'
                       // in LQR

  // TD tracking differentiators for velocity planning (position and yaw)
  common::TDquadratic td_position_;
  common::TDquadratic td_yaw_;

  // Parameters (from config; v_max, yaw_max for control_input)
  double wheel_radius_;     // Wheel radius for velocity calculation (m)
  double max_wheel_torque_; // Maximum wheel torque (Nm)
  double v_max_;    // Max linear velocity (m/s), ly in [-1,1] -> v_cmd
  double yaw_max_;  // Max yaw rate (rad/s), lx in [-1,1] -> yaw_cmd = lx * yaw_max_
  double td_r_;     // TD tracking factor r (larger = faster), configurable

  // Callbacks
  void vmc_state_callback(const vmc_controller::msg::VMCState::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void control_input_callback(const control_input_msgs::msg::Inputs::SharedPtr msg);

  // Extract pitch [rad] and yaw [rad] from IMU quaternion (x,y,z,w)
  static double quaternion_to_pitch(double x, double y, double z, double w);
  static double quaternion_to_yaw(double x, double y, double z, double w);

  // Structure to hold state output contributions
  struct StateOutputContributions {
    // Left wheel contributions
    double left_wheel_s_output;
    double left_wheel_fai_output;
    double left_wheel_theta_ll_output;
    double left_wheel_theta_lr_output;
    double left_wheel_theta_b_output;
    // Right wheel contributions
    double right_wheel_s_output;
    double right_wheel_fai_output;
    double right_wheel_theta_ll_output;
    double right_wheel_theta_lr_output;
    double right_wheel_theta_b_output;
    // Left leg contributions
    double left_leg_s_output;
    double left_leg_fai_output;
    double left_leg_theta_ll_output;
    double left_leg_theta_lr_output;
    double left_leg_theta_b_output;
    // Right leg contributions
    double right_leg_s_output;
    double right_leg_fai_output;
    double right_leg_theta_ll_output;
    double right_leg_theta_lr_output;
    double right_leg_theta_b_output;
  };

  // Helper functions
  void calculate_lqr_control(double &wheel_torque_left,
                             double &wheel_torque_right, double &tp_left,
                             double &tp_right,
                             StateOutputContributions &output_contributions);
  void update_state_estimation(double dt);
  void publish_force_command(double left_F0, double left_Tp, double right_F0,
                             double right_Tp);
  void publish_lqr_state(double wheel_torque_left, double wheel_torque_right,
                         double tp_left, double tp_right,
                         const StateOutputContributions &output_contributions);

  // Utility function to normalize angle to [-π, π]
  static double normalize_angle(double angle);
};

} // namespace lqr_controller

#endif // LQRCONTROLLER_H

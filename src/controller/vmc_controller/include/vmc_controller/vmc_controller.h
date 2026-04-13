//
// VMC Controller for ros2_control
// Implements Virtual Model Control for leg linkage control
//

#ifndef VMCCONTROLLER_H
#define VMCCONTROLLER_H

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "control_input_msgs/msg/inputs.hpp"
#include "vmc_controller/vmc_calc.h"
#include "vmc_controller/msg/vmc_state.hpp"
#include "pid_ros.hpp"

namespace vmc_controller
{

class VMCController : public controller_interface::ControllerInterface
{
public:
  VMCController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  // Joint handles
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> left_front_joint_cmd_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> left_rear_joint_cmd_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> right_front_joint_cmd_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> right_rear_joint_cmd_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> left_front_joint_state_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> left_rear_joint_state_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> right_front_joint_state_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> right_rear_joint_state_;

  // Joint names
  std::string left_front_joint_name_;
  std::string left_rear_joint_name_;
  std::string right_front_joint_name_;
  std::string right_rear_joint_name_;

  // VMC leg structures
  VMCLeg left_leg_;
  VMCLeg right_leg_;

  // IMU data (subscribed from topic)
  double pitch_;
  double pitch_gyro_;
  bool received_imu_;
  rclcpp::Time last_imu_time_;

  // F0 and Tp values
  double left_F0_;
  double left_Tp_;
  double right_F0_;
  double right_Tp_;
  bool received_force_command_;

  // PID controllers for L0 control (F0 calculation)
  std::shared_ptr<PID::PidROS> left_leg_l0_pid_;
  std::shared_ptr<PID::PidROS> right_leg_l0_pid_;

  // PID controllers for theta control (Tp calculation)
  std::shared_ptr<PID::PidROS> left_leg_theta_pid_;
  std::shared_ptr<PID::PidROS> right_leg_theta_pid_;

  // 防劈岔补偿：使左右腿 theta 之和趋于 0，避免两腿劈岔
  std::shared_ptr<PID::PidROS> anti_split_pid_;
  bool enable_anti_split_;

  // Desired theta values
  double left_desired_theta_;
  double right_desired_theta_;

  // L0：desired_* 为初值；ry 对累加目标积分 d(l0_sp)/dt = ry * l0_ry_rate，再限幅到 [l0_min, l0_max]
  double left_desired_l0_;
  double right_desired_l0_;
  double l0_ry_rate_;  // m/s per unit ry
  double l0_target_min_;
  double l0_target_max_;
  double left_l0_target_accum_;
  double right_l0_target_accum_;
  double control_input_ry_;
  std::string control_input_topic_;

  // Parameters
  std::string imu_topic_;
  std::string force_command_topic_;
  std::string vmc_state_topic_;
  double max_torque_;

  // VMC parameters
  double l1_, l2_, l3_, l4_, l5_;

  // Joint initial angle offsets (used to set initial angles as zero point)
  double left_front_joint_offset_;
  double left_rear_joint_offset_;
  double right_front_joint_offset_;
  double right_rear_joint_offset_;

  // Joint angle inversion signs (used to match VMC coordinate system)
  double left_front_joint_invert_sign_;
  double left_rear_joint_invert_sign_;
  double right_front_joint_invert_sign_;
  double right_rear_joint_invert_sign_;

  // Pitch axis polarity (used to invert pitch and pitch_gyro if needed)
  double pitch_invert_sign_;

  // Subscriptions (for IMU and force commands)
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr force_command_subscription_;
  rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr control_input_subscription_;

  // Publisher (for VMC state)
  rclcpp::Publisher<vmc_controller::msg::VMCState>::SharedPtr vmc_state_publisher_;

  // Callbacks
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void force_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void control_input_callback(const control_input_msgs::msg::Inputs::SharedPtr msg);
  double quaternion_to_pitch(double x, double y, double z, double w);
  void extract_pitch_from_imu(const sensor_msgs::msg::Imu::SharedPtr msg, double& pitch, double& pitch_gyro);

  // State
  rclcpp::Time last_update_time_;

  // Debug related members
  bool enable_debug_;
  double debug_print_frequency_;
  uint64_t debug_print_counter_;
  rclcpp::Time last_debug_print_time_;

  // Debug functions
  bool is_valid_number(double value, const std::string& name) const;

  // Publish VMC state
  void publish_vmc_state(double left_front_torque_raw, double left_rear_torque_raw, double right_front_torque_raw,
                         double right_rear_torque_raw);
};

}  // namespace vmc_controller

#endif  // VMCCONTROLLER_H
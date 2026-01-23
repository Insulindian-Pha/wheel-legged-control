//
// Joint Torque Controller Node
// Reads joint states (position, velocity, effort) and publishes torque commands
//

#ifndef JOINTTORQUECONTROLLER_H
#define JOINTTORQUECONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>
#include <string>
#include <map>

class JointTorqueController final : public rclcpp::Node
{
public:
  JointTorqueController();

  ~JointTorqueController() override = default;

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void torque_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void publish_torque_commands();
  void update_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torque_command_subscription_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_command_publisher_;

  // Timer for periodic publishing
  rclcpp::TimerBase::SharedPtr timer_;

  // Joint names to control
  std::vector<std::string> joint_names_;

  // Current joint states (position, velocity, effort)
  std::map<std::string, double> joint_positions_;
  std::map<std::string, double> joint_velocities_;
  std::map<std::string, double> joint_efforts_;

  // Torque commands to send
  std::map<std::string, double> torque_commands_;

  // Parameters
  std::string joint_state_topic_;
  std::string torque_command_topic_;
  std::string torque_input_topic_;
  std::string controller_name_;
  double publish_rate_;
  double max_torque_;

  // State
  bool received_joint_states_;
};

#endif  // JOINTTORQUECONTROLLER_H

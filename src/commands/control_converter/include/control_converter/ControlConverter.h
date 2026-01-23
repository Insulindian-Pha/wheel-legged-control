//
// Control Converter Node
// Converts control_input_msgs/Inputs to joint torque commands
//

#ifndef CONTROLCONVERTER_H
#define CONTROLCONVERTER_H

#include <rclcpp/rclcpp.hpp>
#include <control_input_msgs/msg/inputs.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ControlConverter final : public rclcpp::Node
{
public:
  ControlConverter();

  ~ControlConverter() override = default;

private:
  void control_input_callback(const control_input_msgs::msg::Inputs::SharedPtr msg);
  void publish_torque_commands();

  rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std_msgs::msg::Float64MultiArray current_torque_commands_;
  double max_torque_wheel_;
  double max_torque_front_;
  double max_torque_rear_;
  bool received_input_;
};

#endif  // CONTROLCONVERTER_H

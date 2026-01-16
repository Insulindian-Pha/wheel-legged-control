//
// Control Converter Node
// Converts control_input_msgs/Inputs to geometry_msgs/TwistStamped for diff_drive_controller
//

#ifndef CONTROLCONVERTER_H
#define CONTROLCONVERTER_H

#include <rclcpp/rclcpp.hpp>
#include <control_input_msgs/msg/inputs.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class ControlConverter final : public rclcpp::Node
{
public:
  ControlConverter();

  ~ControlConverter() override = default;

private:
  void control_input_callback(const control_input_msgs::msg::Inputs::SharedPtr msg);
  void publish_cmd_vel();

  rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::TwistStamped current_twist_stamped_;
  double max_linear_x_;
  double max_linear_y_;
  double max_angular_z_;
  bool received_input_;
};

#endif  // CONTROLCONVERTER_H

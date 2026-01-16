//
// Control Converter Node Implementation
// Converts control_input_msgs/Inputs to geometry_msgs/TwistStamped for diff_drive_controller
//

#include "control_converter/ControlConverter.h"
#include <algorithm>

using namespace std::chrono_literals;

ControlConverter::ControlConverter() : Node("control_converter_node")
{
  // Declare parameters
  this->declare_parameter<double>("max_linear_x", 10.0);
  this->declare_parameter<double>("max_linear_y", 10.0);
  this->declare_parameter<double>("max_angular_z", 10.0);
  this->declare_parameter<std::string>("control_input_topic", "control_input");
  this->declare_parameter<std::string>("cmd_vel_topic", "/diff_drive_controller/cmd_vel");
  this->declare_parameter<double>("publish_rate", 50.0);
  this->declare_parameter<std::string>("frame_id", "base_link");

  // Get parameters
  max_linear_x_ = this->get_parameter("max_linear_x").as_double();
  max_linear_y_ = this->get_parameter("max_linear_y").as_double();
  max_angular_z_ = this->get_parameter("max_angular_z").as_double();
  std::string control_input_topic = this->get_parameter("control_input_topic").as_string();
  std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
  double publish_rate = this->get_parameter("publish_rate").as_double();
  std::string frame_id = this->get_parameter("frame_id").as_string();

  // Initialize twist stamped message
  current_twist_stamped_.header.frame_id = frame_id;
  current_twist_stamped_.twist.linear.x = 0.0;
  current_twist_stamped_.twist.linear.y = 0.0;
  current_twist_stamped_.twist.linear.z = 0.0;
  current_twist_stamped_.twist.angular.x = 0.0;
  current_twist_stamped_.twist.angular.y = 0.0;
  current_twist_stamped_.twist.angular.z = 0.0;
  received_input_ = false;

  // Create subscription
  subscription_ = create_subscription<control_input_msgs::msg::Inputs>(
      control_input_topic, 10, std::bind(&ControlConverter::control_input_callback, this, std::placeholders::_1));

  // Create publisher
  publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic, 10);

  // Create timer to periodically publish cmd_vel
  auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
  timer_ = create_wall_timer(timer_period, std::bind(&ControlConverter::publish_cmd_vel, this));

  RCLCPP_INFO(get_logger(), "Control converter node started");
  RCLCPP_INFO(get_logger(), "Subscribing to: %s", control_input_topic.c_str());
  RCLCPP_INFO(get_logger(), "Publishing to: %s", cmd_vel_topic.c_str());
  RCLCPP_INFO(get_logger(), "Max velocities - linear.x: %.2f, linear.y: %.2f, angular.z: %.2f", max_linear_x_,
              max_linear_y_, max_angular_z_);
}

void ControlConverter::control_input_callback(const control_input_msgs::msg::Inputs::SharedPtr msg)
{
  received_input_ = true;

  // If command is not 0, stop the robot
  if (msg->command != 0)
  {
    current_twist_stamped_.twist.linear.x = 0.0;
    current_twist_stamped_.twist.linear.y = 0.0;
    current_twist_stamped_.twist.angular.z = 0.0;
    RCLCPP_DEBUG(get_logger(), "Command %d received, stopping robot", msg->command);
    return;
  }

  // Map control inputs to twist commands
  // lx -> linear.x, ly -> linear.y, rx -> angular.z
  double linear_x = msg->lx * max_linear_x_;
  double linear_y = msg->ly * max_linear_y_;
  double angular_z = msg->rx * max_angular_z_;

  // Clamp values to ensure they don't exceed limits
  linear_x = std::clamp(linear_x, -max_linear_x_, max_linear_x_);
  linear_y = std::clamp(linear_y, -max_linear_y_, max_linear_y_);
  angular_z = std::clamp(angular_z, -max_angular_z_, max_angular_z_);

  current_twist_stamped_.twist.linear.x = linear_x;
  current_twist_stamped_.twist.linear.y = linear_y;
  current_twist_stamped_.twist.angular.z = angular_z;

  //   RCLCPP_DEBUG(get_logger(),
  //                "Control input: lx=%.2f, ly=%.2f, rx=%.2f -> Twist: linear.x=%.2f, linear.y=%.2f, angular.z=%.2f",
  //                msg->lx, msg->ly, msg->rx, linear_x, linear_y, angular_z);
}

void ControlConverter::publish_cmd_vel()
{
  // Update timestamp
  current_twist_stamped_.header.stamp = this->now();

  // If no input received for a while, send zero velocity for safety
  // This is handled by the timer publishing the current_twist_stamped_ which is updated
  // by the callback. If no callback happens, current_twist_stamped_ remains at zero.
  publisher_->publish(current_twist_stamped_);

  // Log published values for debugging (only when non-zero to reduce log spam)
  if (std::abs(current_twist_stamped_.twist.linear.x) > 0.01 ||
      std::abs(current_twist_stamped_.twist.linear.y) > 0.01 || std::abs(current_twist_stamped_.twist.angular.z) > 0.01)
  {
    // RCLCPP_INFO(get_logger(), "Publishing cmd_vel: linear.x=%.3f, linear.y=%.3f, angular.z=%.3f",
    //             current_twist_stamped_.twist.linear.x, current_twist_stamped_.twist.linear.y,
    //             current_twist_stamped_.twist.angular.z);
  }
}

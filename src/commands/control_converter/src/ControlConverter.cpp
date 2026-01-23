//
// Control Converter Node Implementation
// Converts control_input_msgs/Inputs to joint torque commands
//

#include "control_converter/ControlConverter.h"
#include <algorithm>

using namespace std::chrono_literals;

ControlConverter::ControlConverter() : Node("control_converter_node")
{
  // Declare parameters
  this->declare_parameter<double>("max_torque_wheel", 10.0);
  this->declare_parameter<double>("max_torque_front", 10.0);
  this->declare_parameter<double>("max_torque_rear", 10.0);
  this->declare_parameter<std::string>("control_input_topic", "control_input");
  this->declare_parameter<std::string>("torque_command_topic", "/joint_torque_controller/torque_commands");
  this->declare_parameter<double>("publish_rate", 50.0);

  // Get parameters
  max_torque_wheel_ = this->get_parameter("max_torque_wheel").as_double();
  max_torque_front_ = this->get_parameter("max_torque_front").as_double();
  max_torque_rear_ = this->get_parameter("max_torque_rear").as_double();
  std::string control_input_topic = this->get_parameter("control_input_topic").as_string();
  std::string torque_command_topic = this->get_parameter("torque_command_topic").as_string();
  double publish_rate = this->get_parameter("publish_rate").as_double();

  // Initialize torque command message (6 joints in order: Left_front, Left_rear, Left_Wheel, Right_front, Right_rear,
  // Right_Wheel)
  current_torque_commands_.data.resize(6, 0.0);
  received_input_ = false;

  // Create subscription
  subscription_ = create_subscription<control_input_msgs::msg::Inputs>(
      control_input_topic, 10, std::bind(&ControlConverter::control_input_callback, this, std::placeholders::_1));

  // Create publisher
  publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(torque_command_topic, 10);

  // Create timer to periodically publish torque commands
  auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
  timer_ = create_wall_timer(timer_period, std::bind(&ControlConverter::publish_torque_commands, this));

  RCLCPP_INFO(get_logger(), "Control converter node started");
  RCLCPP_INFO(get_logger(), "Subscribing to: %s", control_input_topic.c_str());
  RCLCPP_INFO(get_logger(), "Publishing to: %s", torque_command_topic.c_str());
  RCLCPP_INFO(get_logger(), "Max torques - wheel: %.2f, front: %.2f, rear: %.2f Nm", max_torque_wheel_,
              max_torque_front_, max_torque_rear_);
}

void ControlConverter::control_input_callback(const control_input_msgs::msg::Inputs::SharedPtr msg)
{
  received_input_ = true;

  // If command is not 0, stop the robot (set all torques to zero)
  if (msg->command != 0)
  {
    for (size_t i = 0; i < current_torque_commands_.data.size(); ++i)
    {
      current_torque_commands_.data[i] = 0.0;
    }
    RCLCPP_DEBUG(get_logger(), "Command %d received, stopping robot", msg->command);
    return;
  }

  // Map control inputs to torque commands
  // Joint order: Left_front_joint, Left_rear_joint, Left_Wheel_joint,
  //              Right_front_joint, Right_rear_joint, Right_Wheel_joint

  // Wheel joints: lt/rt control Left_Wheel and Right_Wheel
  // lt/rt typically range from 1.0 (released) to -1.0 (fully pressed)
  // We map: 1.0 (released) -> 0 torque, -1.0 (pressed) -> max_torque
  // Formula: (1.0 - value) * 0.5 maps [1.0, -1.0] to [0.0, 1.0]
  double left_wheel_torque = (1.0 - msg->lt) * 0.5 * max_torque_wheel_;   // Left_Wheel_joint (index 2)
  double right_wheel_torque = (1.0 - msg->rt) * 0.5 * max_torque_wheel_;  // Right_Wheel_joint (index 5)

  // Front joints: rx/lx control Left_front and Right_front
  // rx controls left-right differential for front joints
  // lx can be used for symmetric control
  // double front_combined = (msg->rx + msg->lx) * 0.5;
  double left_front_torque = msg->lx * max_torque_front_;    // Left_front_joint (index 0)
  double right_front_torque = -msg->rx * max_torque_front_;  // Right_front_joint (index 3)

  // Rear joints: ry/ly control Left_rear and Right_rear
  // ry controls left-right differential for rear joints
  // ly can be used for symmetric control
  // double rear_combined = (msg->ry + msg->ly) * 0.5;
  double left_rear_torque = msg->ly * max_torque_rear_;    // Left_rear_joint (index 1)
  double right_rear_torque = -msg->ry * max_torque_rear_;  // Right_rear_joint (index 4)

  // Clamp values to ensure they don't exceed limits
  left_wheel_torque = std::clamp(left_wheel_torque, -max_torque_wheel_, max_torque_wheel_);
  right_wheel_torque = std::clamp(right_wheel_torque, -max_torque_wheel_, max_torque_wheel_);
  left_front_torque = std::clamp(left_front_torque, -max_torque_front_, max_torque_front_);
  right_front_torque = std::clamp(right_front_torque, -max_torque_front_, max_torque_front_);
  left_rear_torque = std::clamp(left_rear_torque, -max_torque_rear_, max_torque_rear_);
  right_rear_torque = std::clamp(right_rear_torque, -max_torque_rear_, max_torque_rear_);

  // Set torque commands in order: Left_front, Left_rear, Left_Wheel, Right_front, Right_rear, Right_Wheel
  current_torque_commands_.data[0] = left_front_torque;
  current_torque_commands_.data[1] = left_rear_torque;
  current_torque_commands_.data[2] = left_wheel_torque;
  current_torque_commands_.data[3] = right_front_torque;
  current_torque_commands_.data[4] = right_rear_torque;
  current_torque_commands_.data[5] = right_wheel_torque;

  RCLCPP_DEBUG(get_logger(),
               "Control input: lt=%.2f, rt=%.2f, rx=%.2f, lx=%.2f, ry=%.2f, ly=%.2f -> Torques: "
               "L_front=%.2f, L_rear=%.2f, L_wheel=%.2f, R_front=%.2f, R_rear=%.2f, R_wheel=%.2f",
               msg->lt, msg->rt, msg->rx, msg->lx, msg->ry, msg->ly, left_front_torque, left_rear_torque,
               left_wheel_torque, right_front_torque, right_rear_torque, right_wheel_torque);
}

void ControlConverter::publish_torque_commands()
{
  // If no input received for a while, send zero torques for safety
  // This is handled by the timer publishing the current_torque_commands_ which is updated
  // by the callback. If no callback happens, current_torque_commands_ remains at zero.
  publisher_->publish(current_torque_commands_);

  // Log published values for debugging (only when non-zero to reduce log spam)
  bool has_non_zero = false;
  for (const auto& torque : current_torque_commands_.data)
  {
    if (std::abs(torque) > 0.01)
    {
      has_non_zero = true;
      break;
    }
  }

  if (has_non_zero)
  {
    RCLCPP_DEBUG(get_logger(), "Publishing torque commands: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                 current_torque_commands_.data[0], current_torque_commands_.data[1], current_torque_commands_.data[2],
                 current_torque_commands_.data[3], current_torque_commands_.data[4], current_torque_commands_.data[5]);
  }
}

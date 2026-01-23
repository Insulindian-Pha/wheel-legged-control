//
// Joint Torque Controller Node Implementation
// Reads joint states and publishes torque commands via ros2_control
//

#include "joint_torque_controller/JointTorqueController.h"
#include <algorithm>

using namespace std::chrono_literals;

JointTorqueController::JointTorqueController() : Node("joint_torque_controller_node"), received_joint_states_(false)
{
  // Declare parameters
  this->declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{ "Left_front_joint", "Left_rear_joint", "Left_Wheel_joint",
                                               "Right_front_joint", "Right_rear_joint", "Right_Wheel_joint" });
  this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
  this->declare_parameter<std::string>("torque_command_topic", "/joint_group_effort_controller/commands");
  this->declare_parameter<std::string>("torque_input_topic", "/joint_torque_controller/torque_commands");
  this->declare_parameter<std::string>("controller_name", "joint_group_effort_controller");
  this->declare_parameter<double>("publish_rate", 50.0);
  this->declare_parameter<double>("max_torque", 30.0);

  // Get parameters
  joint_names_ = this->get_parameter("joint_names").as_string_array();
  joint_state_topic_ = this->get_parameter("joint_state_topic").as_string();
  torque_command_topic_ = this->get_parameter("torque_command_topic").as_string();
  torque_input_topic_ = this->get_parameter("torque_input_topic").as_string();
  controller_name_ = this->get_parameter("controller_name").as_string();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  max_torque_ = this->get_parameter("max_torque").as_double();

  // Initialize joint state maps
  for (const auto& joint_name : joint_names_)
  {
    joint_positions_[joint_name] = 0.0;
    joint_velocities_[joint_name] = 0.0;
    joint_efforts_[joint_name] = 0.0;
    torque_commands_[joint_name] = 0.0;
  }

  // Create subscription for joint states
  joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 10, std::bind(&JointTorqueController::joint_state_callback, this, std::placeholders::_1));

  // Create subscription for torque commands input
  torque_command_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      torque_input_topic_, 10, std::bind(&JointTorqueController::torque_command_callback, this, std::placeholders::_1));

  // Create publisher for torque commands
  torque_command_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(torque_command_topic_, 10);

  // Create timer for periodic publishing
  auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
  timer_ = create_wall_timer(timer_period, std::bind(&JointTorqueController::publish_torque_commands, this));

  RCLCPP_INFO(get_logger(), "Joint Torque Controller node started");
  RCLCPP_INFO(get_logger(), "Subscribing to joint states: %s", joint_state_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Subscribing to torque commands: %s", torque_input_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Publishing torque commands to: %s", torque_command_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Controlling %zu joints", joint_names_.size());
  for (const auto& name : joint_names_)
  {
    RCLCPP_INFO(get_logger(), "  - %s", name.c_str());
  }
  RCLCPP_INFO(get_logger(), "Max torque: %.2f Nm", max_torque_);
  RCLCPP_INFO(get_logger(), "Publish rate: %.1f Hz", publish_rate_);
}

void JointTorqueController::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  received_joint_states_ = true;
  update_joint_states(msg);
}

void JointTorqueController::torque_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // Update torque commands from the received message
  // The message data should be in the same order as joint_names_
  if (msg->data.size() != joint_names_.size())
  {
    RCLCPP_WARN(get_logger(), "Received torque command with %zu values, but expected %zu. Ignoring message.",
                msg->data.size(), joint_names_.size());
    return;
  }

  // Update torque commands
  for (size_t i = 0; i < joint_names_.size() && i < msg->data.size(); ++i)
  {
    torque_commands_[joint_names_[i]] = msg->data[i];
  }

  RCLCPP_DEBUG(get_logger(), "Received torque commands:");
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    RCLCPP_DEBUG(get_logger(), "  %s: %.3f Nm", joint_names_[i].c_str(), torque_commands_[joint_names_[i]]);
  }
}

void JointTorqueController::update_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Update joint states from the message
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    const std::string& joint_name = msg->name[i];

    // Check if this is one of the joints we're controlling
    if (joint_positions_.find(joint_name) != joint_positions_.end())
    {
      // Update position
      if (i < msg->position.size())
      {
        joint_positions_[joint_name] = msg->position[i];
      }

      // Update velocity
      if (i < msg->velocity.size())
      {
        joint_velocities_[joint_name] = msg->velocity[i];
      }

      // Update effort
      if (i < msg->effort.size())
      {
        joint_efforts_[joint_name] = msg->effort[i];
      }
    }
  }
}

void JointTorqueController::publish_torque_commands()
{
  if (!received_joint_states_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No joint states received yet. Publishing zero torques.");
  }

  // Create torque command message
  auto torque_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();

  // Set data in the order of joint_names_
  for (const auto& joint_name : joint_names_)
  {
    // Clamp torque to max_torque limit
    double torque = std::clamp(torque_commands_[joint_name], -max_torque_, max_torque_);
    torque_msg->data.push_back(torque);
  }

  // Publish the command
  torque_command_publisher_->publish(*torque_msg);

  // Log published values for debugging (only when non-zero to reduce log spam)
  bool has_non_zero = false;
  for (const auto& torque : torque_msg->data)
  {
    if (std::abs(torque) > 0.01)
    {
      has_non_zero = true;
      break;
    }
  }

  if (has_non_zero)
  {
    RCLCPP_DEBUG(get_logger(), "Publishing torque commands:");
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      RCLCPP_DEBUG(get_logger(), "  %s: %.3f Nm", joint_names_[i].c_str(), torque_msg->data[i]);
    }
  }
}

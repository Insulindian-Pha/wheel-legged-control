//
// VMC Controller for ros2_control Implementation
// Implements Virtual Model Control for leg linkage control
//

#include "vmc_controller/vmc_controller.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>

namespace vmc_controller
{

VMCController::VMCController()
  : pitch_(0.0)
  , pitch_gyro_(0.0)
  , received_imu_(false)
  , left_F0_(0.0)
  , left_Tp_(0.0)
  , right_F0_(0.0)
  , right_Tp_(0.0)
  , received_force_command_(false)
  , left_front_joint_offset_(0.0)
  , left_rear_joint_offset_(0.0)
  , right_front_joint_offset_(0.0)
  , right_rear_joint_offset_(0.0)
{
}

controller_interface::CallbackReturn VMCController::on_init()
{
  try
  {
    // Declare parameters
    auto_declare<std::string>("left_front_joint_name", "Left_front_joint");
    auto_declare<std::string>("left_rear_joint_name", "Left_rear_joint");
    auto_declare<std::string>("right_front_joint_name", "Right_front_joint");
    auto_declare<std::string>("right_rear_joint_name", "Right_rear_joint");
    auto_declare<std::string>("imu_topic", "/imu/data");
    auto_declare<std::string>("force_command_topic", "/vmc_controller/force_command");
    auto_declare<double>("max_torque", 90.0);

    // VMC杆长参数
    auto_declare<double>("l1", 0.075);
    auto_declare<double>("l2", 0.14);
    auto_declare<double>("l3", 0.14);
    auto_declare<double>("l4", 0.075);
    auto_declare<double>("l5", 0.00);

    // F0和Tp默认值
    auto_declare<double>("left_F0", 0.0);
    auto_declare<double>("left_Tp", 0.0);
    auto_declare<double>("right_F0", 0.0);
    auto_declare<double>("right_Tp", 0.0);

    // 关节初始角度偏置（用于将初始角度设为0点）
    auto_declare<double>("left_front_joint_offset", 0.0);
    auto_declare<double>("left_rear_joint_offset", 0.0);
    auto_declare<double>("right_front_joint_offset", 0.0);
    auto_declare<double>("right_rear_joint_offset", 0.0);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VMCController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Get parameters
  left_front_joint_name_ = get_node()->get_parameter("left_front_joint_name").as_string();
  left_rear_joint_name_ = get_node()->get_parameter("left_rear_joint_name").as_string();
  right_front_joint_name_ = get_node()->get_parameter("right_front_joint_name").as_string();
  right_rear_joint_name_ = get_node()->get_parameter("right_rear_joint_name").as_string();
  imu_topic_ = get_node()->get_parameter("imu_topic").as_string();
  force_command_topic_ = get_node()->get_parameter("force_command_topic").as_string();
  max_torque_ = get_node()->get_parameter("max_torque").as_double();

  l1_ = get_node()->get_parameter("l1").as_double();
  l2_ = get_node()->get_parameter("l2").as_double();
  l3_ = get_node()->get_parameter("l3").as_double();
  l4_ = get_node()->get_parameter("l4").as_double();
  l5_ = get_node()->get_parameter("l5").as_double();

  left_F0_ = get_node()->get_parameter("left_F0").as_double();
  left_Tp_ = get_node()->get_parameter("left_Tp").as_double();
  right_F0_ = get_node()->get_parameter("right_F0").as_double();
  right_Tp_ = get_node()->get_parameter("right_Tp").as_double();

  // 读取关节初始角度偏置
  left_front_joint_offset_ = get_node()->get_parameter("left_front_joint_offset").as_double();
  left_rear_joint_offset_ = get_node()->get_parameter("left_rear_joint_offset").as_double();
  right_front_joint_offset_ = get_node()->get_parameter("right_front_joint_offset").as_double();
  right_rear_joint_offset_ = get_node()->get_parameter("right_rear_joint_offset").as_double();

  // Initialize VMC leg structures
  left_leg_.init_leg_length(l1_, l2_, l3_, l4_, l5_);
  right_leg_.init_leg_length(l1_, l2_, l3_, l4_, l5_);

  // Create subscriptions for IMU and force commands
  imu_subscription_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10, std::bind(&VMCController::imu_callback, this, std::placeholders::_1));

  force_command_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      force_command_topic_, 10, std::bind(&VMCController::force_command_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "VMC Controller configured");
  RCLCPP_INFO(get_node()->get_logger(), "Subscribing to IMU: %s", imu_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Subscribing to force commands: %s", force_command_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Max torque: %.2f Nm", max_torque_);
  RCLCPP_INFO(get_node()->get_logger(), "VMC parameters: l1=%.3f, l2=%.3f, l3=%.3f, l4=%.3f, l5=%.3f", l1_, l2_, l3_,
              l4_, l5_);
  RCLCPP_INFO(get_node()->get_logger(), "Joint offsets: LF=%.3f, LR=%.3f, RF=%.3f, RR=%.3f (rad)",
              left_front_joint_offset_, left_rear_joint_offset_, right_front_joint_offset_, right_rear_joint_offset_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VMCController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Get joint handles
  left_front_joint_cmd_.clear();
  left_rear_joint_cmd_.clear();
  right_front_joint_cmd_.clear();
  right_rear_joint_cmd_.clear();
  left_front_joint_state_.clear();
  left_rear_joint_state_.clear();
  right_front_joint_state_.clear();
  right_rear_joint_state_.clear();

  // Debug: Print all available command interfaces
  RCLCPP_INFO(get_node()->get_logger(), "Available command interfaces (%zu):", command_interfaces_.size());
  for (const auto& interface : command_interfaces_)
  {
    RCLCPP_INFO(get_node()->get_logger(), "  - %s/%s", interface.get_name().c_str(),
                interface.get_interface_name().c_str());
  }

  // Debug: Print all available state interfaces
  RCLCPP_INFO(get_node()->get_logger(), "Available state interfaces (%zu):", state_interfaces_.size());
  for (const auto& interface : state_interfaces_)
  {
    RCLCPP_INFO(get_node()->get_logger(), "  - %s/%s", interface.get_name().c_str(),
                interface.get_interface_name().c_str());
  }

  // Get command interfaces
  // Interface name format is "joint_name/interface_type", so we need to extract joint name
  for (auto& interface : command_interfaces_)
  {
    std::string interface_full_name = interface.get_name();
    std::string interface_type = interface.get_interface_name();

    // Extract joint name from "joint_name/interface_type" format
    size_t pos = interface_full_name.find('/');
    std::string joint_name = (pos != std::string::npos) ? interface_full_name.substr(0, pos) : interface_full_name;

    if (joint_name == left_front_joint_name_ && interface_type == hardware_interface::HW_IF_EFFORT)
    {
      left_front_joint_cmd_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(), "Found left_front_joint command interface");
    }
    if (joint_name == left_rear_joint_name_ && interface_type == hardware_interface::HW_IF_EFFORT)
    {
      left_rear_joint_cmd_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(), "Found left_rear_joint command interface");
    }
    if (joint_name == right_front_joint_name_ && interface_type == hardware_interface::HW_IF_EFFORT)
    {
      right_front_joint_cmd_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(), "Found right_front_joint command interface");
    }
    if (joint_name == right_rear_joint_name_ && interface_type == hardware_interface::HW_IF_EFFORT)
    {
      right_rear_joint_cmd_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(), "Found right_rear_joint command interface");
    }
  }

  // Get state interfaces
  for (auto& interface : state_interfaces_)
  {
    std::string interface_full_name = interface.get_name();
    std::string interface_type = interface.get_interface_name();

    // Extract joint name from "joint_name/interface_type" format
    size_t pos = interface_full_name.find('/');
    std::string joint_name = (pos != std::string::npos) ? interface_full_name.substr(0, pos) : interface_full_name;

    if (joint_name == left_front_joint_name_ && interface_type == hardware_interface::HW_IF_POSITION)
    {
      left_front_joint_state_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(), "Found left_front_joint state interface");
    }
    if (joint_name == left_rear_joint_name_ && interface_type == hardware_interface::HW_IF_POSITION)
    {
      left_rear_joint_state_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(), "Found left_rear_joint state interface");
    }
    if (joint_name == right_front_joint_name_ && interface_type == hardware_interface::HW_IF_POSITION)
    {
      right_front_joint_state_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(), "Found right_front_joint state interface");
    }
    if (joint_name == right_rear_joint_name_ && interface_type == hardware_interface::HW_IF_POSITION)
    {
      right_rear_joint_state_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(), "Found right_rear_joint state interface");
    }
  }

  // Check if all required interfaces are available
  if (left_front_joint_cmd_.empty() || left_rear_joint_cmd_.empty() || right_front_joint_cmd_.empty() ||
      right_rear_joint_cmd_.empty() || left_front_joint_state_.empty() || left_rear_joint_state_.empty() ||
      right_front_joint_state_.empty() || right_rear_joint_state_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Required interfaces not available");
    RCLCPP_ERROR(get_node()->get_logger(),
                 "left_front_cmd: %zu, left_rear_cmd: %zu, right_front_cmd: %zu, right_rear_cmd: %zu",
                 left_front_joint_cmd_.size(), left_rear_joint_cmd_.size(), right_front_joint_cmd_.size(),
                 right_rear_joint_cmd_.size());
    RCLCPP_ERROR(get_node()->get_logger(),
                 "left_front_state: %zu, left_rear_state: %zu, right_front_state: %zu, right_rear_state: %zu",
                 left_front_joint_state_.size(), left_rear_joint_state_.size(), right_front_joint_state_.size(),
                 right_rear_joint_state_.size());
    RCLCPP_ERROR(get_node()->get_logger(), "Looking for joints: %s, %s, %s, %s", left_front_joint_name_.c_str(),
                 left_rear_joint_name_.c_str(), right_front_joint_name_.c_str(), right_rear_joint_name_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  last_update_time_ = get_node()->now();

  RCLCPP_INFO(get_node()->get_logger(), "VMC Controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VMCController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Clear joint handles
  left_front_joint_cmd_.clear();
  left_rear_joint_cmd_.clear();
  right_front_joint_cmd_.clear();
  right_rear_joint_cmd_.clear();
  left_front_joint_state_.clear();
  left_rear_joint_state_.clear();
  right_front_joint_state_.clear();
  right_rear_joint_state_.clear();

  RCLCPP_INFO(get_node()->get_logger(), "VMC Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type VMCController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  // Calculate time step
  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1)  // Sanity check
  {
    dt = 0.002;  // Default 500Hz
  }

  // Get joint positions from state interfaces
  double left_front_pos_raw = left_front_joint_state_[0].get().get_value();
  double left_rear_pos_raw = left_rear_joint_state_[0].get().get_value();
  double right_front_pos_raw = right_front_joint_state_[0].get().get_value();
  double right_rear_pos_raw = right_rear_joint_state_[0].get().get_value();

  // Apply offset to set initial angles as zero point
  double left_front_pos = left_front_pos_raw + left_front_joint_offset_;
  double left_rear_pos = left_rear_pos_raw + left_rear_joint_offset_;
  double right_front_pos = right_front_pos_raw + right_front_joint_offset_;
  double right_rear_pos = right_rear_pos_raw + right_rear_joint_offset_;

  // Update joint angles for VMC calculation
  // 注意：phi1需要加π，但phi4不需要加π（与Simulation.py保持一致）
  // Simulation.py中：右腿使用jAB(phi1)和jAG(phi4)，左腿使用jIO(phi1)和jIJ(phi4)
  // 统一顺序：左腿使用左腿关节(jIJ, jIO)，右腿使用右腿关节(jAB, jAG)
  left_leg_.setPhi1(M_PI + left_rear_pos);     // jIO -> phi1 (左后关节)
  left_leg_.setPhi4(left_front_pos);           // jIJ -> phi4 (左前关节，不加π)
  right_leg_.setPhi1(M_PI + right_front_pos);  // jAB -> phi1 (右前关节)
  right_leg_.setPhi4(right_rear_pos);          // jAG -> phi4 (右后关节，不加π)

  // Set F0 and Tp
  left_leg_.setF0(left_F0_);
  left_leg_.setTp(left_Tp_);
  right_leg_.setF0(right_F0_);
  right_leg_.setTp(right_Tp_);

  // Calculate VMC for left leg
  left_leg_.calc1Left(pitch_, pitch_gyro_, dt);
  left_leg_.calc2();

  // Calculate VMC for right leg
  right_leg_.calc1Right(pitch_, pitch_gyro_, dt);
  right_leg_.calc2();

  right_leg_.getL0();

  // Debug output (only print occasionally to avoid spam)
  static int debug_counter = 0;
  if (debug_counter++ % 100 == 0)  // Print every 100 updates (~0.2 second at 500Hz)
  {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "=== VMC Debug Info ===" << std::endl;
    std::cout << "Raw positions:     LF=" << std::setw(7) << left_front_pos_raw << ", LR=" << std::setw(7)
              << left_rear_pos_raw << ", RF=" << std::setw(7) << right_front_pos_raw << ", RR=" << std::setw(7)
              << right_rear_pos_raw << std::endl;
    std::cout << "Offsets:           LF=" << std::setw(7) << left_front_joint_offset_ << ", LR=" << std::setw(7)
              << left_rear_joint_offset_ << ", RF=" << std::setw(7) << right_front_joint_offset_
              << ", RR=" << std::setw(7) << right_rear_joint_offset_ << std::endl;
    std::cout << "After offset:      LF=" << std::setw(7) << left_front_pos << ", LR=" << std::setw(7) << left_rear_pos
              << ", RF=" << std::setw(7) << right_front_pos << ", RR=" << std::setw(7) << right_rear_pos << std::endl;
    std::cout << "Leg lengths:       LL0=" << std::setw(7) << left_leg_.getL0() << ", RL0=" << std::setw(7)
              << right_leg_.getL0() << std::endl;
    std::cout << "=====================" << std::endl;
  }

  // Clamp and set torques
  double left_front_torque = std::clamp(left_leg_.getTorqueFront(), -max_torque_, max_torque_);
  double left_rear_torque = std::clamp(left_leg_.getTorqueRear(), -max_torque_, max_torque_);
  double right_front_torque = std::clamp(right_leg_.getTorqueFront(), -max_torque_, max_torque_);
  double right_rear_torque = std::clamp(right_leg_.getTorqueRear(), -max_torque_, max_torque_);

  // Set command values
  left_front_joint_cmd_[0].get().set_value(left_front_torque);
  left_rear_joint_cmd_[0].get().set_value(left_rear_torque);
  right_front_joint_cmd_[0].get().set_value(right_front_torque);
  right_rear_joint_cmd_[0].get().set_value(right_rear_torque);

  last_update_time_ = time;

  return controller_interface::return_type::OK;
}

void VMCController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  received_imu_ = true;
  extract_pitch_from_imu(msg, pitch_, pitch_gyro_);
  last_imu_time_ = msg->header.stamp;
}

void VMCController::force_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  received_force_command_ = true;
  if (msg->data.size() >= 4)
  {
    left_F0_ = msg->data[0];
    left_Tp_ = msg->data[1];
    right_F0_ = msg->data[2];
    right_Tp_ = msg->data[3];
    RCLCPP_DEBUG(get_node()->get_logger(), "Received force commands: L_F0=%.3f, L_Tp=%.3f, R_F0=%.3f, R_Tp=%.3f",
                 left_F0_, left_Tp_, right_F0_, right_Tp_);
  }
  else
  {
    RCLCPP_WARN(get_node()->get_logger(), "Received force command with %zu values, but expected 4. Ignoring message.",
                msg->data.size());
  }
}

double VMCController::quaternion_to_pitch(double x, double y, double z, double w)
{
  double sinp = 2 * (w * y - z * x);
  double pitch;
  if (std::abs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp);
  else
    pitch = std::asin(sinp);
  return pitch;
}

void VMCController::extract_pitch_from_imu(const sensor_msgs::msg::Imu::SharedPtr msg, double& pitch,
                                           double& pitch_gyro)
{
  pitch = quaternion_to_pitch(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  pitch_gyro = msg->angular_velocity.y;
}

controller_interface::InterfaceConfiguration VMCController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.push_back(left_front_joint_name_ + "/" + hardware_interface::HW_IF_EFFORT);
  command_interfaces_config.names.push_back(left_rear_joint_name_ + "/" + hardware_interface::HW_IF_EFFORT);
  command_interfaces_config.names.push_back(right_front_joint_name_ + "/" + hardware_interface::HW_IF_EFFORT);
  command_interfaces_config.names.push_back(right_rear_joint_name_ + "/" + hardware_interface::HW_IF_EFFORT);

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration VMCController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.push_back(left_front_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  state_interfaces_config.names.push_back(left_rear_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  state_interfaces_config.names.push_back(right_front_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  state_interfaces_config.names.push_back(right_rear_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);

  return state_interfaces_config;
}

}  // namespace vmc_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(vmc_controller::VMCController, controller_interface::ControllerInterface)

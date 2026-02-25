//
// VMC Controller for ros2_control Implementation
// Implements Virtual Model Control for leg linkage control
//

#include "vmc_controller/vmc_controller.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace vmc_controller {

VMCController::VMCController()
    : pitch_(0.0), pitch_gyro_(0.0), received_imu_(false), left_F0_(0.0),
      left_Tp_(0.0), right_F0_(0.0), right_Tp_(0.0),
      received_force_command_(false), enable_anti_split_(true),
      left_desired_l0_(0.2), right_desired_l0_(0.2), left_desired_theta_(0.0),
      right_desired_theta_(0.0), vmc_reference_topic_(""),
      received_vmc_reference_(false),
      left_front_joint_offset_(0.0), left_rear_joint_offset_(0.0),
      right_front_joint_offset_(0.0), right_rear_joint_offset_(0.0),
      left_front_joint_invert_sign_(1.0), left_rear_joint_invert_sign_(1.0),
      right_front_joint_invert_sign_(1.0), right_rear_joint_invert_sign_(1.0),
      pitch_invert_sign_(1.0), enable_debug_(false),
      debug_print_frequency_(10.0), debug_print_counter_(0) {}

controller_interface::CallbackReturn VMCController::on_init() {
  try {
    // Declare parameters
    auto_declare<std::string>("left_front_joint_name", "Left_front_joint");
    auto_declare<std::string>("left_rear_joint_name", "Left_rear_joint");
    auto_declare<std::string>("right_front_joint_name", "Right_front_joint");
    auto_declare<std::string>("right_rear_joint_name", "Right_rear_joint");
    auto_declare<std::string>("imu_topic", "/imu/data");
    auto_declare<std::string>("force_command_topic",
                              "/vmc_controller/force_command");
    auto_declare<std::string>("vmc_reference_topic",
                              "/vmc_controller/reference");
    auto_declare<std::string>("vmc_state_topic", "/vmc_controller/vmc_state");
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

    // PID控制L0相关参数
    auto_declare<double>("left_desired_l0", 0.2); // 期望的左腿L0值（米）
    auto_declare<double>("right_desired_l0", 0.2); // 期望的右腿L0值（米）

    // PID控制theta相关参数
    auto_declare<double>("left_desired_theta",
                         0.0); // 期望的左腿theta值（弧度）
    auto_declare<double>("right_desired_theta",
                         0.0); // 期望的右腿theta值（弧度）

    // 关节初始角度偏置（用于将初始角度设为0点）
    auto_declare<double>("left_front_joint_offset", 0.0);
    auto_declare<double>("left_rear_joint_offset", 0.0);
    auto_declare<double>("right_front_joint_offset", 0.0);
    auto_declare<double>("right_rear_joint_offset", 0.0);

    // 关节角度取反配置（用于匹配VMC坐标系）
    auto_declare<double>("left_front_joint_invert_sign", 1.0);
    auto_declare<double>("left_rear_joint_invert_sign", 1.0);
    auto_declare<double>("right_front_joint_invert_sign", 1.0);
    auto_declare<double>("right_rear_joint_invert_sign", 1.0);

    // Pitch轴极性配置（用于匹配VMC坐标系）
    auto_declare<double>("pitch_invert_sign", 1.0);

    // 防劈岔补偿开关（使左右腿 theta 之和趋于 0）
    auto_declare<bool>("enable_anti_split", true);

    // 调试参数
    auto_declare<bool>("enable_debug", true);
    auto_declare<double>("debug_print_frequency", 1000.0);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Exception thrown during init stage with message: %s",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VMCController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Get parameters
  left_front_joint_name_ =
      get_node()->get_parameter("left_front_joint_name").as_string();
  left_rear_joint_name_ =
      get_node()->get_parameter("left_rear_joint_name").as_string();
  right_front_joint_name_ =
      get_node()->get_parameter("right_front_joint_name").as_string();
  right_rear_joint_name_ =
      get_node()->get_parameter("right_rear_joint_name").as_string();
  imu_topic_ = get_node()->get_parameter("imu_topic").as_string();
  force_command_topic_ =
      get_node()->get_parameter("force_command_topic").as_string();
  vmc_reference_topic_ =
      get_node()->get_parameter("vmc_reference_topic").as_string();
  vmc_state_topic_ = get_node()->get_parameter("vmc_state_topic").as_string();
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

  // 读取PID控制相关参数
  left_desired_l0_ = get_node()->get_parameter("left_desired_l0").as_double();
  right_desired_l0_ = get_node()->get_parameter("right_desired_l0").as_double();
  left_desired_theta_ =
      get_node()->get_parameter("left_desired_theta").as_double();
  right_desired_theta_ =
      get_node()->get_parameter("right_desired_theta").as_double();

  // 读取关节初始角度偏置
  left_front_joint_offset_ =
      get_node()->get_parameter("left_front_joint_offset").as_double();
  left_rear_joint_offset_ =
      get_node()->get_parameter("left_rear_joint_offset").as_double();
  right_front_joint_offset_ =
      get_node()->get_parameter("right_front_joint_offset").as_double();
  right_rear_joint_offset_ =
      get_node()->get_parameter("right_rear_joint_offset").as_double();

  // 读取关节角度取反配置
  left_front_joint_invert_sign_ =
      get_node()->get_parameter("left_front_joint_invert_sign").as_double();
  left_rear_joint_invert_sign_ =
      get_node()->get_parameter("left_rear_joint_invert_sign").as_double();
  right_front_joint_invert_sign_ =
      get_node()->get_parameter("right_front_joint_invert_sign").as_double();
  right_rear_joint_invert_sign_ =
      get_node()->get_parameter("right_rear_joint_invert_sign").as_double();

  // 读取Pitch轴极性配置
  pitch_invert_sign_ =
      get_node()->get_parameter("pitch_invert_sign").as_double();

  // 读取调试参数
  enable_debug_ = get_node()->get_parameter("enable_debug").as_bool();
  debug_print_frequency_ =
      get_node()->get_parameter("debug_print_frequency").as_double();

  // Initialize VMC leg structures
  left_leg_.init_leg_length(l1_, l2_, l3_, l4_, l5_);
  right_leg_.init_leg_length(l1_, l2_, l3_, l4_, l5_);

  // Initialize PID controllers for L0 control (if enabled)
  left_leg_l0_pid_ = std::make_shared<PID::PidROS>(
      get_node(), "pid_gains.l0_control.left_leg", true, true);
  right_leg_l0_pid_ = std::make_shared<PID::PidROS>(
      get_node(), "pid_gains.l0_control.right_leg", true, true);

  // Initialize PID controllers for theta control (Tp calculation)
  left_leg_theta_pid_ = std::make_shared<PID::PidROS>(
      get_node(), "pid_gains.theta_control.left_leg", true, true);
  right_leg_theta_pid_ = std::make_shared<PID::PidROS>(
      get_node(), "pid_gains.theta_control.right_leg", true, true);

  // 防劈岔补偿 PID：目标为 0（左 theta + 右 theta），反馈为当前两腿 theta 之和
  anti_split_pid_ = std::make_shared<PID::PidROS>(
      get_node(), "pid_gains.anti_split", true, true);
  enable_anti_split_ = get_node()->get_parameter("enable_anti_split").as_bool();

  RCLCPP_INFO(get_node()->get_logger(), "PID control for F0 enabled");
  RCLCPP_INFO(get_node()->get_logger(), "Left desired L0: %.3f m",
              left_desired_l0_);
  RCLCPP_INFO(get_node()->get_logger(), "Right desired L0: %.3f m",
              right_desired_l0_);
  RCLCPP_INFO(get_node()->get_logger(), "PID control for Tp enabled");
  RCLCPP_INFO(get_node()->get_logger(), "Left desired theta: %.3f rad",
              left_desired_theta_);
  RCLCPP_INFO(get_node()->get_logger(), "Right desired theta: %.3f rad",
              right_desired_theta_);
  RCLCPP_INFO(get_node()->get_logger(), "Anti-split compensation: %s",
              enable_anti_split_ ? "enabled" : "disabled");

  // Create subscriptions for IMU and force commands
  imu_subscription_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&VMCController::imu_callback, this, std::placeholders::_1));

  force_command_subscription_ =
      get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
          force_command_topic_, 10,
          std::bind(&VMCController::force_command_callback, this,
                    std::placeholders::_1));

  vmc_reference_subscription_ =
      get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
          vmc_reference_topic_, 10,
          std::bind(&VMCController::vmc_reference_callback, this,
                    std::placeholders::_1));

  // Create publisher for VMC state
  vmc_state_publisher_ =
      get_node()->create_publisher<vmc_controller::msg::VMCState>(
          vmc_state_topic_, 10);

  RCLCPP_INFO(get_node()->get_logger(), "VMC Controller configured");
  RCLCPP_INFO(get_node()->get_logger(), "Subscribing to IMU: %s",
              imu_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Subscribing to force commands: %s",
              force_command_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Subscribing to VMC reference: %s",
              vmc_reference_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Publishing VMC state to: %s",
              vmc_state_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Max torque: %.2f Nm", max_torque_);
  RCLCPP_INFO(get_node()->get_logger(),
              "VMC parameters: l1=%.3f, l2=%.3f, l3=%.3f, l4=%.3f, l5=%.3f",
              l1_, l2_, l3_, l4_, l5_);
  RCLCPP_INFO(get_node()->get_logger(),
              "Joint offsets: LF=%.3f, LR=%.3f, RF=%.3f, RR=%.3f (rad)",
              left_front_joint_offset_, left_rear_joint_offset_,
              right_front_joint_offset_, right_rear_joint_offset_);
  RCLCPP_INFO(get_node()->get_logger(),
              "Joint invert signs: LF=%.1f, LR=%.1f, RF=%.1f, RR=%.1f",
              left_front_joint_invert_sign_, left_rear_joint_invert_sign_,
              right_front_joint_invert_sign_, right_rear_joint_invert_sign_);
  RCLCPP_INFO(get_node()->get_logger(), "Pitch invert sign: %.1f",
              pitch_invert_sign_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
VMCController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
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
  RCLCPP_INFO(get_node()->get_logger(), "Available command interfaces (%zu):",
              command_interfaces_.size());
  for (const auto &interface : command_interfaces_) {
    RCLCPP_INFO(get_node()->get_logger(), "  - %s/%s",
                interface.get_name().c_str(),
                interface.get_interface_name().c_str());
  }

  // Debug: Print all available state interfaces
  RCLCPP_INFO(get_node()->get_logger(),
              "Available state interfaces (%zu):", state_interfaces_.size());
  for (const auto &interface : state_interfaces_) {
    RCLCPP_INFO(get_node()->get_logger(), "  - %s/%s",
                interface.get_name().c_str(),
                interface.get_interface_name().c_str());
  }

  // Get command interfaces
  // Interface name format is "joint_name/interface_type", so we need to extract
  // joint name
  for (auto &interface : command_interfaces_) {
    std::string interface_full_name = interface.get_name();
    std::string interface_type = interface.get_interface_name();

    // Extract joint name from "joint_name/interface_type" format
    size_t pos = interface_full_name.find('/');
    std::string joint_name = (pos != std::string::npos)
                                 ? interface_full_name.substr(0, pos)
                                 : interface_full_name;

    if (joint_name == left_front_joint_name_ &&
        interface_type == hardware_interface::HW_IF_EFFORT) {
      left_front_joint_cmd_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(),
                  "Found left_front_joint command interface");
    }
    if (joint_name == left_rear_joint_name_ &&
        interface_type == hardware_interface::HW_IF_EFFORT) {
      left_rear_joint_cmd_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(),
                  "Found left_rear_joint command interface");
    }
    if (joint_name == right_front_joint_name_ &&
        interface_type == hardware_interface::HW_IF_EFFORT) {
      right_front_joint_cmd_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(),
                  "Found right_front_joint command interface");
    }
    if (joint_name == right_rear_joint_name_ &&
        interface_type == hardware_interface::HW_IF_EFFORT) {
      right_rear_joint_cmd_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(),
                  "Found right_rear_joint command interface");
    }
  }

  // Get state interfaces
  for (auto &interface : state_interfaces_) {
    std::string interface_full_name = interface.get_name();
    std::string interface_type = interface.get_interface_name();

    // Extract joint name from "joint_name/interface_type" format
    size_t pos = interface_full_name.find('/');
    std::string joint_name = (pos != std::string::npos)
                                 ? interface_full_name.substr(0, pos)
                                 : interface_full_name;

    if (joint_name == left_front_joint_name_ &&
        interface_type == hardware_interface::HW_IF_POSITION) {
      left_front_joint_state_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(),
                  "Found left_front_joint state interface");
    }
    if (joint_name == left_rear_joint_name_ &&
        interface_type == hardware_interface::HW_IF_POSITION) {
      left_rear_joint_state_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(),
                  "Found left_rear_joint state interface");
    }
    if (joint_name == right_front_joint_name_ &&
        interface_type == hardware_interface::HW_IF_POSITION) {
      right_front_joint_state_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(),
                  "Found right_front_joint state interface");
    }
    if (joint_name == right_rear_joint_name_ &&
        interface_type == hardware_interface::HW_IF_POSITION) {
      right_rear_joint_state_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(),
                  "Found right_rear_joint state interface");
    }
  }

  // Check if all required interfaces are available
  if (left_front_joint_cmd_.empty() || left_rear_joint_cmd_.empty() ||
      right_front_joint_cmd_.empty() || right_rear_joint_cmd_.empty() ||
      left_front_joint_state_.empty() || left_rear_joint_state_.empty() ||
      right_front_joint_state_.empty() || right_rear_joint_state_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Required interfaces not available");
    RCLCPP_ERROR(get_node()->get_logger(),
                 "left_front_cmd: %zu, left_rear_cmd: %zu, right_front_cmd: "
                 "%zu, right_rear_cmd: %zu",
                 left_front_joint_cmd_.size(), left_rear_joint_cmd_.size(),
                 right_front_joint_cmd_.size(), right_rear_joint_cmd_.size());
    RCLCPP_ERROR(get_node()->get_logger(),
                 "left_front_state: %zu, left_rear_state: %zu, "
                 "right_front_state: %zu, right_rear_state: %zu",
                 left_front_joint_state_.size(), left_rear_joint_state_.size(),
                 right_front_joint_state_.size(),
                 right_rear_joint_state_.size());
    RCLCPP_ERROR(get_node()->get_logger(), "Looking for joints: %s, %s, %s, %s",
                 left_front_joint_name_.c_str(), left_rear_joint_name_.c_str(),
                 right_front_joint_name_.c_str(),
                 right_rear_joint_name_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  last_update_time_ = get_node()->now();

  RCLCPP_INFO(get_node()->get_logger(), "VMC Controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VMCController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
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

controller_interface::return_type
VMCController::update(const rclcpp::Time &time,
                      const rclcpp::Duration &period) {
  // Calculate time step
  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1) // Sanity check
  {
    dt = 0.002; // Default 500Hz
  }

  // Get joint positions from state interfaces
  double left_front_pos_raw = left_front_joint_state_[0].get().get_value();
  double left_rear_pos_raw = left_rear_joint_state_[0].get().get_value();
  double right_front_pos_raw = right_front_joint_state_[0].get().get_value();
  double right_rear_pos_raw = right_rear_joint_state_[0].get().get_value();

  // Apply offset to set initial angles as zero point
  double left_front_pos = left_front_joint_invert_sign_ *
                          (left_front_pos_raw + left_front_joint_offset_);
  double left_rear_pos = left_rear_joint_invert_sign_ *
                         (left_rear_pos_raw + left_rear_joint_offset_);
  double right_front_pos = right_front_joint_invert_sign_ *
                           (right_front_pos_raw + right_front_joint_offset_);
  double right_rear_pos = right_rear_joint_invert_sign_ *
                          (right_rear_pos_raw + right_rear_joint_offset_);

  // Apply pitch polarity
  double pitch_correct = pitch_invert_sign_ * pitch_;
  double pitch_gyro_correct = pitch_invert_sign_ * pitch_gyro_;

  // Normalize joint angles to [-π, π] range to prevent angle accumulation
  auto normalize_angle = [](double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  };

  left_front_pos = normalize_angle(left_front_pos);
  left_rear_pos = normalize_angle(left_rear_pos);
  right_front_pos = normalize_angle(right_front_pos);
  right_rear_pos = normalize_angle(right_rear_pos);

  // Update joint angles for VMC calculation
  double left_phi1 = M_PI + left_rear_pos;    // phi1 (左后关节)
  double left_phi4 = left_front_pos;          // phi4 (左前关节，不加π)
  double right_phi1 = M_PI + right_front_pos; // phi1 (右前关节)
  double right_phi4 = right_rear_pos;         // phi4 (右后关节，不加π)

  left_leg_.setPhi1(left_phi1);
  left_leg_.setPhi4(left_phi4);
  right_leg_.setPhi1(right_phi1);
  right_leg_.setPhi4(right_phi4);

  // Calculate VMC for legs (first pass to get L0 and theta)
  // Note: We need to set F0 first (use previous value or 0) to calculate theta

  // Get current L0 and theta values
  double left_current_l0 = left_leg_.getL0();
  double right_current_l0 = right_leg_.getL0();
  double left_current_theta = left_leg_.getTheta();
  double right_current_theta = right_leg_.getTheta();

  // Set L0 PID targets from desired_l0 (from params or from vmc_reference topic)
  left_leg_l0_pid_->get_pid().setTarget(static_cast<float>(left_desired_l0_));
  right_leg_l0_pid_->get_pid().setTarget(static_cast<float>(right_desired_l0_));

  // Calculate F0 using PID with L0 as feedback
  left_F0_ = left_leg_l0_pid_->update(left_current_l0, time);
  right_F0_ = right_leg_l0_pid_->update(right_current_l0, time);

  // Tp: from force_command (LQR mode) when received; otherwise from theta PID with desired_theta (params or reference)
  if (!received_force_command_) {
    left_leg_theta_pid_->get_pid().setTarget(static_cast<float>(left_desired_theta_));
    right_leg_theta_pid_->get_pid().setTarget(static_cast<float>(right_desired_theta_));
    left_Tp_ = left_leg_theta_pid_->update(left_current_theta, time);
    right_Tp_ = right_leg_theta_pid_->update(right_current_theta, time);
  }

  // 防劈岔补偿：theta_err = 0 - (left_theta + right_theta)，PID 输出 leg_tp
  // 叠加到左右髋 Tp（仿 chassisR_task）
  if (enable_anti_split_ && anti_split_pid_) {
    double theta_sum =
        left_current_theta +
        right_current_theta; // 目标为 0，即期望两腿 theta 之和为 0
    double leg_tp =
        anti_split_pid_->update(static_cast<float>(theta_sum), time);
    left_Tp_ += leg_tp;
    right_Tp_ += leg_tp;
  }

  // Set F0 and Tp
  left_leg_.setF0(left_F0_);
  right_leg_.setF0(right_F0_);
  left_leg_.setTp(left_Tp_);
  right_leg_.setTp(right_Tp_);

  // Recalculate VMC with updated F0 and Tp
  // Apply pitch polarity (reuse the adjusted values from above)
  left_leg_.calc1Left(pitch_correct, pitch_gyro_correct, dt);
  left_leg_.calc2();

  right_leg_.calc1Right(pitch_correct, pitch_gyro_correct, dt);
  right_leg_.calc2();

  // Clamp and set torques
  double left_front_torque_raw =
      std::clamp(left_leg_.getTorqueFront(), -max_torque_, max_torque_);
  double left_rear_torque_raw =
      std::clamp(left_leg_.getTorqueRear(), -max_torque_, max_torque_);
  double right_front_torque_raw =
      std::clamp(right_leg_.getTorqueFront(), -max_torque_, max_torque_);
  double right_rear_torque_raw =
      std::clamp(right_leg_.getTorqueRear(), -max_torque_, max_torque_);

  // Apply invert sign to torques (same as joint positions, because torque
  // direction should match joint axis direction)
  double left_front_torque =
      left_front_joint_invert_sign_ * left_front_torque_raw;
  double left_rear_torque = left_rear_joint_invert_sign_ * left_rear_torque_raw;
  double right_front_torque =
      right_front_joint_invert_sign_ * right_front_torque_raw;
  double right_rear_torque =
      right_rear_joint_invert_sign_ * right_rear_torque_raw;

  // Set command values
  left_front_joint_cmd_[0].get().set_value(left_front_torque);
  left_rear_joint_cmd_[0].get().set_value(left_rear_torque);
  right_front_joint_cmd_[0].get().set_value(right_front_torque);
  right_rear_joint_cmd_[0].get().set_value(right_rear_torque);

  // Publish VMC state
  publish_vmc_state(left_front_torque_raw, left_rear_torque_raw,
                    right_front_torque_raw, right_rear_torque_raw);

  last_update_time_ = time;

  return controller_interface::return_type::OK;
}

void VMCController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  received_imu_ = true;
  extract_pitch_from_imu(msg, pitch_, pitch_gyro_);
  last_imu_time_ = msg->header.stamp;
}

void VMCController::force_command_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  received_force_command_ = true;
  if (msg->data.size() >= 4) {
    left_F0_ = msg->data[0];
    left_Tp_ = msg->data[1];
    right_F0_ = msg->data[2];
    right_Tp_ = msg->data[3];
    RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Received force commands: L_F0=%.3f, L_Tp=%.3f, R_F0=%.3f, R_Tp=%.3f",
        left_F0_, left_Tp_, right_F0_, right_Tp_);
  } else {
    RCLCPP_WARN(get_node()->get_logger(),
                "Received force command with %zu values, but expected 4. "
                "Ignoring message.",
                msg->data.size());
  }
}

void VMCController::vmc_reference_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  received_vmc_reference_ = true;
  if (msg->data.size() >= 4) {
    left_desired_theta_ = msg->data[0];
    left_desired_l0_ = msg->data[1];
    right_desired_theta_ = msg->data[2];
    right_desired_l0_ = msg->data[3];
    RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Received VMC reference: L_theta=%.3f, L_l0=%.3f, R_theta=%.3f, R_l0=%.3f",
        left_desired_theta_, left_desired_l0_, right_desired_theta_, right_desired_l0_);
  } else {
    RCLCPP_WARN(get_node()->get_logger(),
                "Received VMC reference with %zu values, but expected 4. "
                "Ignoring message.",
                msg->data.size());
  }
}

double VMCController::quaternion_to_pitch(double x, double y, double z,
                                          double w) {
  double sinp = 2 * (w * y - z * x);
  double pitch;
  if (std::abs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp);
  else
    pitch = std::asin(sinp);
  return pitch;
}

void VMCController::extract_pitch_from_imu(
    const sensor_msgs::msg::Imu::SharedPtr msg, double &pitch,
    double &pitch_gyro) {
  pitch = quaternion_to_pitch(msg->orientation.x, msg->orientation.y,
                              msg->orientation.z, msg->orientation.w);
  pitch_gyro = msg->angular_velocity.y;
}

bool VMCController::is_valid_number(double value,
                                    const std::string &name) const {
  if (std::isnan(value)) {
    std::cout << name << " is NaN" << std::endl;
    return false;
  }
  if (std::isinf(value)) {
    std::cout << name << " is Inf" << std::endl;
    return false;
  }
  if (std::abs(value) > 1e10) {
    std::cout << name << " is too large: " << std::scientific
              << std::setprecision(6) << value << std::endl;
    return false;
  }
  return true;
}

void VMCController::publish_vmc_state(double left_front_torque_raw,
                                      double left_rear_torque_raw,
                                      double right_front_torque_raw,
                                      double right_rear_torque_raw) {
  // Publish VMC state
  if (vmc_state_publisher_) {
    auto vmc_state_msg = vmc_controller::msg::VMCState();
    vmc_state_msg.header.stamp = get_node()->now();
    vmc_state_msg.header.frame_id = "base_link";

    // Left leg state
    vmc_state_msg.left_theta = left_leg_.getTheta();
    vmc_state_msg.left_d_theta = left_leg_.getDTheta();
    vmc_state_msg.left_l0 = left_leg_.getL0();
    vmc_state_msg.left_f0 = left_F0_;
    vmc_state_msg.left_tp = left_Tp_;
    vmc_state_msg.left_torque_front = left_front_torque_raw;
    vmc_state_msg.left_torque_rear = left_rear_torque_raw;
    vmc_state_msg.left_j11 = left_leg_.getJ11();
    vmc_state_msg.left_j12 = left_leg_.getJ12();
    vmc_state_msg.left_j21 = left_leg_.getJ21();
    vmc_state_msg.left_j22 = left_leg_.getJ22();

    // Right leg state
    vmc_state_msg.right_theta = right_leg_.getTheta();
    vmc_state_msg.right_d_theta = right_leg_.getDTheta();
    vmc_state_msg.right_l0 = right_leg_.getL0();
    vmc_state_msg.right_f0 = right_F0_;
    vmc_state_msg.right_tp = right_Tp_;
    vmc_state_msg.right_torque_front = right_front_torque_raw;
    vmc_state_msg.right_torque_rear = right_rear_torque_raw;
    vmc_state_msg.right_j11 = right_leg_.getJ11();
    vmc_state_msg.right_j12 = right_leg_.getJ12();
    vmc_state_msg.right_j21 = right_leg_.getJ21();
    vmc_state_msg.right_j22 = right_leg_.getJ22();

    // IMU data
    vmc_state_msg.pitch = pitch_;
    vmc_state_msg.pitch_gyro = pitch_gyro_;

    vmc_state_publisher_->publish(vmc_state_msg);
  }
}

controller_interface::InterfaceConfiguration
VMCController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.push_back(left_front_joint_name_ + "/" +
                                            hardware_interface::HW_IF_EFFORT);
  command_interfaces_config.names.push_back(left_rear_joint_name_ + "/" +
                                            hardware_interface::HW_IF_EFFORT);
  command_interfaces_config.names.push_back(right_front_joint_name_ + "/" +
                                            hardware_interface::HW_IF_EFFORT);
  command_interfaces_config.names.push_back(right_rear_joint_name_ + "/" +
                                            hardware_interface::HW_IF_EFFORT);

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
VMCController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.push_back(left_front_joint_name_ + "/" +
                                          hardware_interface::HW_IF_POSITION);
  state_interfaces_config.names.push_back(left_rear_joint_name_ + "/" +
                                          hardware_interface::HW_IF_POSITION);
  state_interfaces_config.names.push_back(right_front_joint_name_ + "/" +
                                          hardware_interface::HW_IF_POSITION);
  state_interfaces_config.names.push_back(right_rear_joint_name_ + "/" +
                                          hardware_interface::HW_IF_POSITION);

  return state_interfaces_config;
}

} // namespace vmc_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(vmc_controller::VMCController,
                       controller_interface::ControllerInterface)

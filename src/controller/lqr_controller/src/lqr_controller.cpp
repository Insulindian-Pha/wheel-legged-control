//
// LQR Controller for ros2_control Implementation
// Implements Linear Quadratic Regulator for wheel-legged robot balance control
//

#include "lqr_controller/lqr_controller.h"
#include <algorithm>
#include <cmath>

namespace lqr_controller {

LQRController::LQRController()
    : left_theta_(0.0), left_d_theta_(0.0), right_theta_(0.0),
      right_d_theta_(0.0), pitch_(0.0), pitch_gyro_(0.0), left_F0_(0.0),
      right_F0_(0.0), received_imu_(false), received_vmc_state_(false),
      x_position_(0.0), x_velocity_(0.0), x_set_(0.0), v_set_(0.0), yaw_(0.0),
      yaw_gyro_(0.0), wheel_radius_(0.06), max_wheel_torque_(4.0) {
  // Initialize LQR gains to zero
  left_wheel_lqr_gains_.fill(0.0);
  right_wheel_lqr_gains_.fill(0.0);
  left_leg_lqr_gains_.fill(0.0);
  right_leg_lqr_gains_.fill(0.0);
  // Initialize state polarity to 1.0 (no inversion by default)
  state_polarity_.fill(1.0);
  left_wheel_gain_polarity_.fill(1.0);
  right_wheel_gain_polarity_.fill(1.0);
  left_leg_gain_polarity_.fill(1.0);
  right_leg_gain_polarity_.fill(1.0);
}

controller_interface::CallbackReturn LQRController::on_init() {
  try {
    // Declare parameters
    auto_declare<std::string>("left_wheel_joint_name", "Left_Wheel_joint");
    auto_declare<std::string>("right_wheel_joint_name", "Right_Wheel_joint");
    auto_declare<std::string>("vmc_state_topic", "/vmc_controller/vmc_state");
    auto_declare<std::string>(
        "imu_topic",
        ""); // Empty: pitch/yaw from VMC and wheel; non-empty: from IMU
    auto_declare<std::string>("force_command_topic",
                              "/vmc_controller/force_command");
    auto_declare<std::string>("lqr_state_topic", "/lqr_controller/lqr_state");
    auto_declare<double>("wheel_radius", 0.06);
    auto_declare<double>("max_wheel_torque", 4.0);
    auto_declare<double>("x_set", 0.0);
    auto_declare<double>("v_set", 0.0);

    // Declare LQR gains for left wheel torque (10 values)
    for (int i = 0; i < 10; ++i) {
      auto_declare<double>("left_wheel_lqr_gains." + std::to_string(i), 0.0);
    }

    // Declare LQR gains for right wheel torque (10 values)
    for (int i = 0; i < 10; ++i) {
      auto_declare<double>("right_wheel_lqr_gains." + std::to_string(i), 0.0);
    }

    // Declare LQR gains for left leg torque (10 values)
    for (int i = 0; i < 10; ++i) {
      auto_declare<double>("left_leg_lqr_gains." + std::to_string(i), 0.0);
    }

    // Declare LQR gains for right leg torque (10 values)
    for (int i = 0; i < 10; ++i) {
      auto_declare<double>("right_leg_lqr_gains." + std::to_string(i), 0.0);
    }

    // Declare state polarity (10 values, one for each state)
    // 0: s, 1: dot_s, 2: fai, 3: dot_fai, 4: theta_ll, 5: dot_theta_ll,
    // 6: theta_lr, 7: dot_theta_lr, 8: theta_b, 9: dot_theta_b
    for (int i = 0; i < 10; ++i) {
      auto_declare<double>("state_polarity." + std::to_string(i), 1.0);
    }

    // Declare gain polarity per output (10 values each): effective_gain = gain
    // * gain_polarity
    for (int i = 0; i < 10; ++i) {
      auto_declare<double>("left_wheel_gain_polarity." + std::to_string(i),
                           1.0);
      auto_declare<double>("right_wheel_gain_polarity." + std::to_string(i),
                           1.0);
      auto_declare<double>("left_leg_gain_polarity." + std::to_string(i), 1.0);
      auto_declare<double>("right_leg_gain_polarity." + std::to_string(i), 1.0);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Exception thrown during init stage with message: %s",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Get parameters
  left_wheel_joint_name_ =
      get_node()->get_parameter("left_wheel_joint_name").as_string();
  right_wheel_joint_name_ =
      get_node()->get_parameter("right_wheel_joint_name").as_string();
  vmc_state_topic_ = get_node()->get_parameter("vmc_state_topic").as_string();
  imu_topic_ = get_node()->get_parameter("imu_topic").as_string();
  received_imu_ = false;
  force_command_topic_ =
      get_node()->get_parameter("force_command_topic").as_string();
  lqr_state_topic_ = get_node()->get_parameter("lqr_state_topic").as_string();
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  max_wheel_torque_ = get_node()->get_parameter("max_wheel_torque").as_double();
  x_set_ = get_node()->get_parameter("x_set").as_double();
  v_set_ = get_node()->get_parameter("v_set").as_double();

  // Get LQR gains for left wheel torque
  for (int i = 0; i < 10; ++i) {
    left_wheel_lqr_gains_[i] =
        get_node()
            ->get_parameter("left_wheel_lqr_gains." + std::to_string(i))
            .as_double();
  }

  // Get LQR gains for right wheel torque
  for (int i = 0; i < 10; ++i) {
    right_wheel_lqr_gains_[i] =
        get_node()
            ->get_parameter("right_wheel_lqr_gains." + std::to_string(i))
            .as_double();
  }

  // Get LQR gains for left leg torque
  for (int i = 0; i < 10; ++i) {
    left_leg_lqr_gains_[i] =
        get_node()
            ->get_parameter("left_leg_lqr_gains." + std::to_string(i))
            .as_double();
  }

  // Get LQR gains for right leg torque
  for (int i = 0; i < 10; ++i) {
    right_leg_lqr_gains_[i] =
        get_node()
            ->get_parameter("right_leg_lqr_gains." + std::to_string(i))
            .as_double();
  }

  // Get state polarity (10 values, one for each state)
  // 0: s, 1: dot_s, 2: fai, 3: dot_fai, 4: theta_ll, 5: dot_theta_ll,
  // 6: theta_lr, 7: dot_theta_lr, 8: theta_b, 9: dot_theta_b
  for (int i = 0; i < 10; ++i) {
    state_polarity_[i] =
        get_node()
            ->get_parameter("state_polarity." + std::to_string(i))
            .as_double();
  }

  // Get gain polarity per output (effective gain = lqr_gain * gain_polarity)
  for (int i = 0; i < 10; ++i) {
    left_wheel_gain_polarity_[i] =
        get_node()
            ->get_parameter("left_wheel_gain_polarity." + std::to_string(i))
            .as_double();
    right_wheel_gain_polarity_[i] =
        get_node()
            ->get_parameter("right_wheel_gain_polarity." + std::to_string(i))
            .as_double();
    left_leg_gain_polarity_[i] =
        get_node()
            ->get_parameter("left_leg_gain_polarity." + std::to_string(i))
            .as_double();
    right_leg_gain_polarity_[i] =
        get_node()
            ->get_parameter("right_leg_gain_polarity." + std::to_string(i))
            .as_double();
  }

  // Create subscription for VMC state
  vmc_state_subscription_ =
      get_node()->create_subscription<vmc_controller::msg::VMCState>(
          vmc_state_topic_, 10,
          std::bind(&LQRController::vmc_state_callback, this,
                    std::placeholders::_1));

  // Create subscription for IMU (optional): when configured,
  // pitch/pitch_gyro/yaw/yaw_gyro come from IMU

  imu_subscription_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&LQRController::imu_callback, this, std::placeholders::_1));
  RCLCPP_INFO(get_node()->get_logger(), "Subscribing to IMU for state: %s",
              imu_topic_.c_str());

  // Create publisher for force commands (Tp updates)
  force_command_publisher_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          force_command_topic_, 10);

  // Create publisher for LQR state
  lqr_state_publisher_ =
      get_node()->create_publisher<lqr_controller::msg::LQRState>(
          lqr_state_topic_, 10);

  RCLCPP_INFO(get_node()->get_logger(), "LQR Controller configured");
  RCLCPP_INFO(get_node()->get_logger(), "Subscribing to VMC state: %s",
              vmc_state_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Publishing force commands to: %s",
              force_command_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Publishing LQR state to: %s",
              lqr_state_topic_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "Wheel radius: %.3f m", wheel_radius_);
  RCLCPP_INFO(get_node()->get_logger(), "Max wheel torque: %.2f Nm",
              max_wheel_torque_);
  RCLCPP_INFO(get_node()->get_logger(), "x_set: %.3f m, v_set: %.3f m/s",
              x_set_, v_set_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LQRController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  // Get wheel joint handles
  left_wheel_cmd_.clear();
  right_wheel_cmd_.clear();
  left_wheel_state_.clear();
  right_wheel_state_.clear();

  // Get command interfaces
  for (auto &interface : command_interfaces_) {
    std::string interface_full_name = interface.get_name();
    std::string interface_type = interface.get_interface_name();

    size_t pos = interface_full_name.find('/');
    std::string joint_name = (pos != std::string::npos)
                                 ? interface_full_name.substr(0, pos)
                                 : interface_full_name;

    if (joint_name == left_wheel_joint_name_ &&
        interface_type == hardware_interface::HW_IF_EFFORT) {
      left_wheel_cmd_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(),
                  "Found left_wheel_joint command interface");
    }
    if (joint_name == right_wheel_joint_name_ &&
        interface_type == hardware_interface::HW_IF_EFFORT) {
      right_wheel_cmd_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(),
                  "Found right_wheel_joint command interface");
    }
  }

  // Get state interfaces
  for (auto &interface : state_interfaces_) {
    std::string interface_full_name = interface.get_name();
    std::string interface_type = interface.get_interface_name();

    size_t pos = interface_full_name.find('/');
    std::string joint_name = (pos != std::string::npos)
                                 ? interface_full_name.substr(0, pos)
                                 : interface_full_name;

    if (joint_name == left_wheel_joint_name_ &&
        interface_type == hardware_interface::HW_IF_VELOCITY) {
      left_wheel_state_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(),
                  "Found left_wheel_joint state interface");
    }
    if (joint_name == right_wheel_joint_name_ &&
        interface_type == hardware_interface::HW_IF_VELOCITY) {
      right_wheel_state_.emplace_back(std::ref(interface));
      RCLCPP_INFO(get_node()->get_logger(),
                  "Found right_wheel_joint state interface");
    }
  }

  // Check if all required interfaces are available
  if (left_wheel_cmd_.empty() || right_wheel_cmd_.empty() ||
      left_wheel_state_.empty() || right_wheel_state_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Required interfaces not available");
    RCLCPP_ERROR(get_node()->get_logger(),
                 "left_wheel_cmd: %zu, right_wheel_cmd: %zu",
                 left_wheel_cmd_.size(), right_wheel_cmd_.size());
    RCLCPP_ERROR(get_node()->get_logger(),
                 "left_wheel_state: %zu, right_wheel_state: %zu",
                 left_wheel_state_.size(), right_wheel_state_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reset state estimation
  x_position_ = 0.0;
  x_velocity_ = 0.0;
  yaw_ = 0.0;
  yaw_gyro_ = 0.0;

  RCLCPP_INFO(get_node()->get_logger(), "LQR Controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LQRController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Clear joint handles
  left_wheel_cmd_.clear();
  right_wheel_cmd_.clear();
  left_wheel_state_.clear();
  right_wheel_state_.clear();

  RCLCPP_INFO(get_node()->get_logger(), "LQR Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
LQRController::update(const rclcpp::Time & /*time*/,
                      const rclcpp::Duration &period) {
  // Calculate time step
  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1) {
    dt = 0.002; // Default 500Hz
  }

  // Check if we have received VMC state
  if (!received_vmc_state_) {
    // Set wheel torques to zero if no VMC state received
    if (!left_wheel_cmd_.empty()) {
      left_wheel_cmd_[0].get().set_value(0.0);
    }
    if (!right_wheel_cmd_.empty()) {
      right_wheel_cmd_[0].get().set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  // Update state estimation (simple integration)
  update_state_estimation(dt);

  // Calculate LQR control
  double wheel_torque_left = 0.0;
  double wheel_torque_right = 0.0;
  double tp_left = 0.0;
  double tp_right = 0.0;
  StateOutputContributions output_contributions;

  calculate_lqr_control(wheel_torque_left, wheel_torque_right, tp_left,
                        tp_right, output_contributions);

  // Clamp wheel torques
  wheel_torque_left =
      std::clamp(wheel_torque_left, -max_wheel_torque_, max_wheel_torque_);
  wheel_torque_right =
      std::clamp(wheel_torque_right, -max_wheel_torque_, max_wheel_torque_);

  // Set wheel torque commands
  left_wheel_cmd_[0].get().set_value(wheel_torque_left);
  right_wheel_cmd_[0].get().set_value(wheel_torque_right);

  // Publish Tp updates to VMC controller
  publish_force_command(left_F0_, tp_left, right_F0_, tp_right);

  // Publish LQR state
  publish_lqr_state(wheel_torque_left, wheel_torque_right, tp_left, tp_right,
                    output_contributions);

  return controller_interface::return_type::OK;
}

void LQRController::vmc_state_callback(
    const vmc_controller::msg::VMCState::SharedPtr msg) {
  received_vmc_state_ = true;

  // Extract VMC state data
  left_theta_ = msg->left_theta;
  left_d_theta_ = msg->left_d_theta;
  right_theta_ = msg->right_theta;
  right_d_theta_ = msg->right_d_theta;
  // Pitch and pitch_gyro: from VMC only when IMU topic is not configured
  pitch_ = msg->pitch;
  pitch_gyro_ = msg->pitch_gyro;

  // Store current F0 values (to preserve when updating Tp)
  left_F0_ = msg->left_f0;
  right_F0_ = msg->right_f0;
}

void LQRController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  received_imu_ = true;
  pitch_ = quaternion_to_pitch(msg->orientation.x, msg->orientation.y,
                               msg->orientation.z, msg->orientation.w);
  pitch_gyro_ = msg->angular_velocity.y;
  yaw_ = quaternion_to_yaw(msg->orientation.x, msg->orientation.y,
                           msg->orientation.z, msg->orientation.w);
  yaw_gyro_ = msg->angular_velocity.z;
}

double LQRController::quaternion_to_pitch(double x, double y, double z,
                                          double w) {
  double sinp = 2 * (w * y - z * x);
  if (std::abs(sinp) >= 1)
    return std::copysign(M_PI / 2, sinp);
  return std::asin(sinp);
}

double LQRController::quaternion_to_yaw(double x, double y, double z,
                                        double w) {
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void LQRController::calculate_lqr_control(
    double &wheel_torque_left, double &wheel_torque_right, double &tp_left,
    double &tp_right, StateOutputContributions &output_contributions) {
  // State vector (10 states):
  // 0: s (位移) - x_position_
  // 1: dot_s (速度) - x_velocity_
  // 2: fai (航向角) - yaw_
  // 3: dot_fai (航向角速度) - yaw_gyro_
  // 4: theta_ll (左腿角度) - left_theta_
  // 5: dot_theta_ll (左腿角速度) - left_d_theta_
  // 6: theta_lr (右腿角度) - right_theta_
  // 7: dot_theta_lr (右腿角速度) - right_d_theta_
  // 8: theta_b (身体角度) - pitch_
  // 9: dot_theta_b (身体角速度) - pitch_gyro_

  // Apply state polarity to each state value
  double s = state_polarity_[0] * x_position_;
  double dot_s = state_polarity_[1] * x_velocity_;
  double fai = state_polarity_[2] * yaw_;
  double dot_fai = state_polarity_[3] * yaw_gyro_;
  double theta_ll = state_polarity_[4] * left_theta_;
  double dot_theta_ll = state_polarity_[5] * left_d_theta_;
  double theta_lr = state_polarity_[6] * right_theta_;
  double dot_theta_lr = state_polarity_[7] * right_d_theta_;
  double theta_b = state_polarity_[8] * pitch_;
  double dot_theta_b = state_polarity_[9] * pitch_gyro_;

  // Effective gain = lqr_gain * gain_polarity (per output, per state)
  // Calculate state output contributions for left wheel
  output_contributions.left_wheel_s_output =
      (left_wheel_lqr_gains_[0] * left_wheel_gain_polarity_[0]) * s +
      (left_wheel_lqr_gains_[1] * left_wheel_gain_polarity_[1]) * dot_s;
  output_contributions.left_wheel_fai_output =
      (left_wheel_lqr_gains_[2] * left_wheel_gain_polarity_[2]) * fai +
      (left_wheel_lqr_gains_[3] * left_wheel_gain_polarity_[3]) * dot_fai;
  output_contributions.left_wheel_theta_ll_output =
      (left_wheel_lqr_gains_[4] * left_wheel_gain_polarity_[4]) * theta_ll +
      (left_wheel_lqr_gains_[5] * left_wheel_gain_polarity_[5]) * dot_theta_ll;
  output_contributions.left_wheel_theta_lr_output =
      (left_wheel_lqr_gains_[6] * left_wheel_gain_polarity_[6]) * theta_lr +
      (left_wheel_lqr_gains_[7] * left_wheel_gain_polarity_[7]) * dot_theta_lr;
  output_contributions.left_wheel_theta_b_output =
      (left_wheel_lqr_gains_[8] * left_wheel_gain_polarity_[8]) * theta_b +
      (left_wheel_lqr_gains_[9] * left_wheel_gain_polarity_[9]) * dot_theta_b;

  // Calculate state output contributions for right wheel
  output_contributions.right_wheel_s_output =
      (right_wheel_lqr_gains_[0] * right_wheel_gain_polarity_[0]) * s +
      (right_wheel_lqr_gains_[1] * right_wheel_gain_polarity_[1]) * dot_s;
  output_contributions.right_wheel_fai_output =
      (right_wheel_lqr_gains_[2] * right_wheel_gain_polarity_[2]) * fai +
      (right_wheel_lqr_gains_[3] * right_wheel_gain_polarity_[3]) * dot_fai;
  output_contributions.right_wheel_theta_ll_output =
      (right_wheel_lqr_gains_[4] * right_wheel_gain_polarity_[4]) * theta_ll +
      (right_wheel_lqr_gains_[5] * right_wheel_gain_polarity_[5]) *
          dot_theta_ll;
  output_contributions.right_wheel_theta_lr_output =
      (right_wheel_lqr_gains_[6] * right_wheel_gain_polarity_[6]) * theta_lr +
      (right_wheel_lqr_gains_[7] * right_wheel_gain_polarity_[7]) *
          dot_theta_lr;
  output_contributions.right_wheel_theta_b_output =
      (right_wheel_lqr_gains_[8] * right_wheel_gain_polarity_[8]) * theta_b +
      (right_wheel_lqr_gains_[9] * right_wheel_gain_polarity_[9]) * dot_theta_b;

  // Calculate state output contributions for left leg
  output_contributions.left_leg_s_output =
      (left_leg_lqr_gains_[0] * left_leg_gain_polarity_[0]) * s +
      (left_leg_lqr_gains_[1] * left_leg_gain_polarity_[1]) * dot_s;
  output_contributions.left_leg_fai_output =
      (left_leg_lqr_gains_[2] * left_leg_gain_polarity_[2]) * fai +
      (left_leg_lqr_gains_[3] * left_leg_gain_polarity_[3]) * dot_fai;
  output_contributions.left_leg_theta_ll_output =
      (left_leg_lqr_gains_[4] * left_leg_gain_polarity_[4]) * theta_ll +
      (left_leg_lqr_gains_[5] * left_leg_gain_polarity_[5]) * dot_theta_ll;
  output_contributions.left_leg_theta_lr_output =
      (left_leg_lqr_gains_[6] * left_leg_gain_polarity_[6]) * theta_lr +
      (left_leg_lqr_gains_[7] * left_leg_gain_polarity_[7]) * dot_theta_lr;
  output_contributions.left_leg_theta_b_output =
      (left_leg_lqr_gains_[8] * left_leg_gain_polarity_[8]) * theta_b +
      (left_leg_lqr_gains_[9] * left_leg_gain_polarity_[9]) * dot_theta_b;

  // Calculate state output contributions for right leg
  output_contributions.right_leg_s_output =
      (right_leg_lqr_gains_[0] * right_leg_gain_polarity_[0]) * s +
      (right_leg_lqr_gains_[1] * right_leg_gain_polarity_[1]) * dot_s;
  output_contributions.right_leg_fai_output =
      (right_leg_lqr_gains_[2] * right_leg_gain_polarity_[2]) * fai +
      (right_leg_lqr_gains_[3] * right_leg_gain_polarity_[3]) * dot_fai;
  output_contributions.right_leg_theta_ll_output =
      (right_leg_lqr_gains_[4] * right_leg_gain_polarity_[4]) * theta_ll +
      (right_leg_lqr_gains_[5] * right_leg_gain_polarity_[5]) * dot_theta_ll;
  output_contributions.right_leg_theta_lr_output =
      (right_leg_lqr_gains_[6] * right_leg_gain_polarity_[6]) * theta_lr +
      (right_leg_lqr_gains_[7] * right_leg_gain_polarity_[7]) * dot_theta_lr;
  output_contributions.right_leg_theta_b_output =
      (right_leg_lqr_gains_[8] * right_leg_gain_polarity_[8]) * theta_b +
      (right_leg_lqr_gains_[9] * right_leg_gain_polarity_[9]) * dot_theta_b;

  // Left wheel torque = sum of all contributions
  wheel_torque_left = output_contributions.left_wheel_s_output +
                      output_contributions.left_wheel_fai_output +
                      output_contributions.left_wheel_theta_ll_output +
                      output_contributions.left_wheel_theta_lr_output +
                      output_contributions.left_wheel_theta_b_output;

  // Right wheel torque = sum of all contributions
  wheel_torque_right = output_contributions.right_wheel_s_output +
                       output_contributions.right_wheel_fai_output +
                       output_contributions.right_wheel_theta_ll_output +
                       output_contributions.right_wheel_theta_lr_output +
                       output_contributions.right_wheel_theta_b_output;

  // Left leg torque (Tp) = sum of all contributions
  tp_left = output_contributions.left_leg_s_output +
            output_contributions.left_leg_fai_output +
            output_contributions.left_leg_theta_ll_output +
            output_contributions.left_leg_theta_lr_output +
            output_contributions.left_leg_theta_b_output;

  // Right leg torque (Tp) = sum of all contributions
  tp_right = output_contributions.right_leg_s_output +
             output_contributions.right_leg_fai_output +
             output_contributions.right_leg_theta_ll_output +
             output_contributions.right_leg_theta_lr_output +
             output_contributions.right_leg_theta_b_output;
}

void LQRController::update_state_estimation(double dt) {
  // Get wheel velocities
  double left_wheel_vel = left_wheel_state_[0].get().get_value();
  double right_wheel_vel = right_wheel_state_[0].get().get_value();

  // Calculate average velocity from wheel speeds (for forward motion)
  double avg_wheel_vel = (left_wheel_vel - right_wheel_vel) / 2.0;
  // Convert angular velocity to linear velocity
  x_velocity_ = avg_wheel_vel * wheel_radius_;
  // Integrate position
  x_position_ += x_velocity_ * dt;
}

void LQRController::publish_force_command(double left_F0, double left_Tp,
                                          double right_F0, double right_Tp) {
  if (force_command_publisher_) {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.resize(4);
    msg.data[0] = left_F0;
    msg.data[1] = left_Tp;
    msg.data[2] = right_F0;
    msg.data[3] = right_Tp;

    force_command_publisher_->publish(msg);
  }
}

void LQRController::publish_lqr_state(
    double wheel_torque_left, double wheel_torque_right, double tp_left,
    double tp_right, const StateOutputContributions &output_contributions) {
  if (lqr_state_publisher_) {
    auto msg = lqr_controller::msg::LQRState();
    msg.header.stamp = get_node()->now();
    msg.header.frame_id = "base_link";

    // LQR outputs
    msg.left_wheel_torque = wheel_torque_left;
    msg.right_wheel_torque = wheel_torque_right;
    msg.left_tp = tp_left;
    msg.right_tp = tp_right;

    // State values used in LQR calculation
    msg.left_theta = left_theta_;
    msg.left_d_theta = left_d_theta_;
    msg.right_theta = right_theta_;
    msg.right_d_theta = right_d_theta_;
    msg.pitch = pitch_;
    msg.pitch_gyro = pitch_gyro_;

    // State estimation
    msg.x_position = x_position_;
    msg.x_velocity = x_velocity_;
    msg.yaw = yaw_;
    msg.yaw_gyro = yaw_gyro_;
    msg.x_set = x_set_;
    msg.v_set = v_set_;
    msg.x_err = x_set_ - x_position_;
    msg.v_err = v_set_ - x_velocity_;

    // Wheel velocities
    if (!left_wheel_state_.empty() && !right_wheel_state_.empty()) {
      msg.left_wheel_velocity = left_wheel_state_[0].get().get_value();
      msg.right_wheel_velocity = right_wheel_state_[0].get().get_value();
    } else {
      msg.left_wheel_velocity = 0.0;
      msg.right_wheel_velocity = 0.0;
    }

    // 使用传入的状态量输出贡献值
    // Left wheel contributions
    msg.left_wheel_s_output = output_contributions.left_wheel_s_output;
    msg.left_wheel_fai_output = output_contributions.left_wheel_fai_output;
    msg.left_wheel_theta_ll_output =
        output_contributions.left_wheel_theta_ll_output;
    msg.left_wheel_theta_lr_output =
        output_contributions.left_wheel_theta_lr_output;
    msg.left_wheel_theta_b_output =
        output_contributions.left_wheel_theta_b_output;

    // Right wheel contributions
    msg.right_wheel_s_output = output_contributions.right_wheel_s_output;
    msg.right_wheel_fai_output = output_contributions.right_wheel_fai_output;
    msg.right_wheel_theta_ll_output =
        output_contributions.right_wheel_theta_ll_output;
    msg.right_wheel_theta_lr_output =
        output_contributions.right_wheel_theta_lr_output;
    msg.right_wheel_theta_b_output =
        output_contributions.right_wheel_theta_b_output;

    // Left leg contributions
    msg.left_leg_s_output = output_contributions.left_leg_s_output;
    msg.left_leg_fai_output = output_contributions.left_leg_fai_output;
    msg.left_leg_theta_ll_output =
        output_contributions.left_leg_theta_ll_output;
    msg.left_leg_theta_lr_output =
        output_contributions.left_leg_theta_lr_output;
    msg.left_leg_theta_b_output = output_contributions.left_leg_theta_b_output;

    // Right leg contributions
    msg.right_leg_s_output = output_contributions.right_leg_s_output;
    msg.right_leg_fai_output = output_contributions.right_leg_fai_output;
    msg.right_leg_theta_ll_output =
        output_contributions.right_leg_theta_ll_output;
    msg.right_leg_theta_lr_output =
        output_contributions.right_leg_theta_lr_output;
    msg.right_leg_theta_b_output =
        output_contributions.right_leg_theta_b_output;

    lqr_state_publisher_->publish(msg);
  }
}

controller_interface::InterfaceConfiguration
LQRController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.push_back(left_wheel_joint_name_ + "/" +
                                            hardware_interface::HW_IF_EFFORT);
  command_interfaces_config.names.push_back(right_wheel_joint_name_ + "/" +
                                            hardware_interface::HW_IF_EFFORT);

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
LQRController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.push_back(left_wheel_joint_name_ + "/" +
                                          hardware_interface::HW_IF_VELOCITY);
  state_interfaces_config.names.push_back(right_wheel_joint_name_ + "/" +
                                          hardware_interface::HW_IF_VELOCITY);

  return state_interfaces_config;
}

double LQRController::normalize_angle(double angle) {
  // Normalize angle to [-π, π] range using fmod for efficiency
  angle = std::fmod(angle, 2.0 * M_PI);
  if (angle > M_PI) {
    angle -= 2.0 * M_PI;
  } else if (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

} // namespace lqr_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(lqr_controller::LQRController,
                       controller_interface::ControllerInterface)

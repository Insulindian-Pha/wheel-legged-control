//
// RL Controller for ros2_control
// ONNX policy (cod_flat_vmc) deployment: 135-dim obs -> 6-dim action.
//

#include "rl_controller/rl_controller.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>

namespace rl_controller {

RLController::RLController()
    : received_vmc_state_(false), received_imu_(false), left_theta_(0.0),
      left_d_theta_(0.0), right_theta_(0.0), right_d_theta_(0.0), left_l0_(0.2),
      right_l0_(0.2), ang_vel_x_(0.0), ang_vel_y_(0.0), ang_vel_z_(0.0),
      proj_grav_x_(0.0), proj_grav_y_(0.0), proj_grav_z_(-1.0),
      lin_vel_cmd_x_(0.0), lin_vel_cmd_y_(0.0), ang_vel_cmd_z_(0.0),
      left_l0_dot_(0.0), right_l0_dot_(0.0), left_l0_prev_(0.2),
      right_l0_prev_(0.2), obs_history_write_idx_(0),
      obs_history_filled_(false),
      ort_env_(ORT_LOGGING_LEVEL_WARNING, "rl_controller") {
  obs_buffer_.fill(0.0f);
  last_action_.fill(0.0f);
  for (auto &step : obs_history_)
    step.fill(0.0f);
}

controller_interface::CallbackReturn RLController::on_init() {
  try {
    auto_declare<std::string>("policy_path", "");
    auto_declare<std::string>("left_wheel_joint_name", "Left_Wheel_joint");
    auto_declare<std::string>("right_wheel_joint_name", "Right_Wheel_joint");
    auto_declare<std::string>("vmc_state_topic", "/vmc_controller/vmc_state");
    auto_declare<std::string>("imu_topic", "/imu/data");
    auto_declare<std::string>("control_input_topic", "/control_input");
    auto_declare<std::string>("vmc_reference_topic",
                              "/vmc_controller/reference");
    auto_declare<std::string>("rl_state_topic", "/rl_controller/rl_state");

    auto_declare<double>("obs_scale_ang_vel", 0.2);
    auto_declare<double>("obs_scale_projected_gravity", 0.05);
    auto_declare<double>("obs_scale_commands", 1.0);
    auto_declare<double>("obs_scale_joint_pos", 0.01);
    auto_declare<double>("obs_scale_joint_vel", 1.5);
    auto_declare<double>("obs_scale_l0", 5.0);
    auto_declare<double>("obs_scale_l0_dot", 0.25);
    auto_declare<double>("obs_scale_actions", 1.0);

    auto_declare<double>("action_scale_theta", 0.5);
    auto_declare<double>("action_scale_l0", 0.1);
    auto_declare<double>("l0_offset", 0.175);
    auto_declare<double>("action_scale_vel", 10.0);
    auto_declare<double>("theta_invert_sign", 1.0);

    auto_declare<double>("v_max", 1.0);
    auto_declare<double>("yaw_max", 1.0);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RLController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  policy_path_ = get_node()->get_parameter("policy_path").as_string();
  left_wheel_joint_name_ =
      get_node()->get_parameter("left_wheel_joint_name").as_string();
  right_wheel_joint_name_ =
      get_node()->get_parameter("right_wheel_joint_name").as_string();
  vmc_state_topic_ = get_node()->get_parameter("vmc_state_topic").as_string();
  imu_topic_ = get_node()->get_parameter("imu_topic").as_string();
  control_input_topic_ =
      get_node()->get_parameter("control_input_topic").as_string();
  vmc_reference_topic_ =
      get_node()->get_parameter("vmc_reference_topic").as_string();
  rl_state_topic_ = get_node()->get_parameter("rl_state_topic").as_string();

  obs_scale_ang_vel_ =
      get_node()->get_parameter("obs_scale_ang_vel").as_double();
  obs_scale_projected_gravity_ =
      get_node()->get_parameter("obs_scale_projected_gravity").as_double();
  obs_scale_commands_ =
      get_node()->get_parameter("obs_scale_commands").as_double();
  obs_scale_joint_pos_ =
      get_node()->get_parameter("obs_scale_joint_pos").as_double();
  obs_scale_joint_vel_ =
      get_node()->get_parameter("obs_scale_joint_vel").as_double();
  obs_scale_l0_ = get_node()->get_parameter("obs_scale_l0").as_double();
  obs_scale_l0_dot_ = get_node()->get_parameter("obs_scale_l0_dot").as_double();
  obs_scale_actions_ =
      get_node()->get_parameter("obs_scale_actions").as_double();

  action_scale_theta_ =
      get_node()->get_parameter("action_scale_theta").as_double();
  action_scale_l0_ = get_node()->get_parameter("action_scale_l0").as_double();
  l0_offset_ = get_node()->get_parameter("l0_offset").as_double();
  action_scale_vel_ = get_node()->get_parameter("action_scale_vel").as_double();
  theta_invert_sign_ =
      get_node()->get_parameter("theta_invert_sign").as_double();

  v_max_ = get_node()->get_parameter("v_max").as_double();
  yaw_max_ = get_node()->get_parameter("yaw_max").as_double();

  wheel_vel_pid_l_ = std::make_shared<PID::PidROS>(
      get_node(), "pid_gains.wheel_vel.left", true, false);
  wheel_vel_pid_r_ = std::make_shared<PID::PidROS>(
      get_node(), "pid_gains.wheel_vel.right", true, false);

  vmc_state_sub_ =
      get_node()->create_subscription<vmc_controller::msg::VMCState>(
          vmc_state_topic_, 10,
          std::bind(&RLController::vmc_state_callback, this,
                    std::placeholders::_1));
  imu_sub_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&RLController::imu_callback, this, std::placeholders::_1));
  control_input_sub_ =
      get_node()->create_subscription<control_input_msgs::msg::Inputs>(
          control_input_topic_, 10,
          std::bind(&RLController::control_input_callback, this,
                    std::placeholders::_1));

  vmc_reference_pub_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          vmc_reference_topic_, 10);
  rl_state_pub_ =
      get_node()->create_publisher<rl_controller::msg::RLState>(rl_state_topic_, 10);

  if (policy_path_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "policy_path is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(1);
    ort_session_ =
        std::make_unique<Ort::Session>(ort_env_, policy_path_.c_str(), opts);

    Ort::AllocatorWithDefaultOptions allocator;
    size_t num_inputs = ort_session_->GetInputCount();
    size_t num_outputs = ort_session_->GetOutputCount();
    if (num_inputs != 1u || num_outputs != 1u) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Expected 1 input and 1 output, got %zu and %zu", num_inputs,
                   num_outputs);
      return controller_interface::CallbackReturn::ERROR;
    }
    auto in_name = ort_session_->GetInputNameAllocated(0, allocator);
    auto out_name = ort_session_->GetOutputNameAllocated(0, allocator);
    input_names_.push_back(in_name ? std::string(in_name.get()) : "");
    output_names_.push_back(out_name ? std::string(out_name.get()) : "");
    input_names_cstr_.push_back(input_names_.back().c_str());
    output_names_cstr_.push_back(output_names_.back().c_str());
  } catch (const Ort::Exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "ONNX load failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "RL Controller configured, policy: %s",
              policy_path_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RLController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  left_wheel_cmd_.clear();
  right_wheel_cmd_.clear();
  left_wheel_state_pos_.clear();
  left_wheel_state_vel_.clear();
  right_wheel_state_pos_.clear();
  right_wheel_state_vel_.clear();

  for (auto &iface : command_interfaces_) {
    std::string name = iface.get_name();
    std::string type = iface.get_interface_name();
    size_t pos = name.find('/');
    std::string joint = (pos != std::string::npos) ? name.substr(0, pos) : name;
    if (joint == left_wheel_joint_name_ &&
        type == hardware_interface::HW_IF_EFFORT)
      left_wheel_cmd_.emplace_back(std::ref(iface));
    if (joint == right_wheel_joint_name_ &&
        type == hardware_interface::HW_IF_EFFORT)
      right_wheel_cmd_.emplace_back(std::ref(iface));
  }
  for (auto &iface : state_interfaces_) {
    std::string name = iface.get_name();
    std::string type = iface.get_interface_name();
    size_t pos = name.find('/');
    std::string joint = (pos != std::string::npos) ? name.substr(0, pos) : name;
    if (joint == left_wheel_joint_name_) {
      if (type == hardware_interface::HW_IF_POSITION)
        left_wheel_state_pos_.emplace_back(std::ref(iface));
      else if (type == hardware_interface::HW_IF_VELOCITY)
        left_wheel_state_vel_.emplace_back(std::ref(iface));
    }
    if (joint == right_wheel_joint_name_) {
      if (type == hardware_interface::HW_IF_POSITION)
        right_wheel_state_pos_.emplace_back(std::ref(iface));
      else if (type == hardware_interface::HW_IF_VELOCITY)
        right_wheel_state_vel_.emplace_back(std::ref(iface));
    }
  }

  if (left_wheel_cmd_.empty() || right_wheel_cmd_.empty() ||
      left_wheel_state_vel_.empty() || right_wheel_state_vel_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Missing wheel interfaces (need effort + velocity)");
    return controller_interface::CallbackReturn::ERROR;
  }

  obs_history_write_idx_ = 0;
  obs_history_filled_ = false;
  last_action_.fill(0.0f);
  left_l0_prev_ = left_l0_;
  right_l0_prev_ = right_l0_;
  if (wheel_vel_pid_l_)
    wheel_vel_pid_l_->reset();
  if (wheel_vel_pid_r_)
    wheel_vel_pid_r_->reset();

  RCLCPP_INFO(get_node()->get_logger(), "RL Controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RLController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  left_wheel_cmd_.clear();
  right_wheel_cmd_.clear();
  left_wheel_state_pos_.clear();
  left_wheel_state_vel_.clear();
  right_wheel_state_pos_.clear();
  right_wheel_state_vel_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

void RLController::vmc_state_callback(
    const vmc_controller::msg::VMCState::SharedPtr msg) {
  received_vmc_state_ = true;
  left_theta_ = msg->left_theta;
  left_d_theta_ = msg->left_d_theta;
  right_theta_ = msg->right_theta;
  right_d_theta_ = msg->right_d_theta;
  left_l0_ = msg->left_l0;
  right_l0_ = msg->right_l0;
}

void RLController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  received_imu_ = true;
  ang_vel_x_ = msg->angular_velocity.x;
  ang_vel_y_ = msg->angular_velocity.y;
  ang_vel_z_ = msg->angular_velocity.z;
  quaternion_to_projected_gravity(msg->orientation.x, msg->orientation.y,
                                  msg->orientation.z, msg->orientation.w,
                                  proj_grav_x_, proj_grav_y_, proj_grav_z_);
}

void RLController::control_input_callback(
    const control_input_msgs::msg::Inputs::SharedPtr msg) {
  lin_vel_cmd_x_ = static_cast<double>(msg->ly) * v_max_;
  lin_vel_cmd_y_ = 0.0;
  ang_vel_cmd_z_ = static_cast<double>(msg->lx) * yaw_max_;
}

void RLController::quaternion_to_projected_gravity(double x, double y, double z,
                                                   double w, double &gx,
                                                   double &gy, double &gz) {
  // Body-frame gravity = R^T * [0, 0, -1]
  gx = -2.0 * (w * x + y * z);
  gy = -2.0 * (w * y - z * x);
  gz = -(w * w - x * x - y * y + z * z);
}

void RLController::build_single_step_obs(
    std::array<float, NUM_OBS_PER_STEP> &step_obs) {
  double dt = 0.002;
  if (received_vmc_state_) {
    left_l0_dot_ = (left_l0_ - left_l0_prev_) / dt;
    right_l0_dot_ = (right_l0_ - right_l0_prev_) / dt;
    left_l0_prev_ = left_l0_;
    right_l0_prev_ = right_l0_;
  }

  float wpos_l = 0.0f, wpos_r = 0.0f, wvel_l = 0.0f, wvel_r = 0.0f;
  if (!left_wheel_state_pos_.empty())
    wpos_l = static_cast<float>(left_wheel_state_pos_[0].get().get_value());
  if (!right_wheel_state_pos_.empty())
    wpos_r = static_cast<float>(right_wheel_state_pos_[0].get().get_value());
  if (!left_wheel_state_vel_.empty())
    wvel_l = static_cast<float>(left_wheel_state_vel_[0].get().get_value());
  if (!right_wheel_state_vel_.empty())
    wvel_r = static_cast<float>(right_wheel_state_vel_[0].get().get_value());

  step_obs[0] = static_cast<float>(ang_vel_x_ * obs_scale_ang_vel_);
  step_obs[1] = static_cast<float>(ang_vel_y_ * obs_scale_ang_vel_);
  step_obs[2] = static_cast<float>(ang_vel_z_ * obs_scale_ang_vel_);
  step_obs[3] = static_cast<float>(proj_grav_x_ * obs_scale_projected_gravity_);
  step_obs[4] = static_cast<float>(proj_grav_y_ * obs_scale_projected_gravity_);
  step_obs[5] = static_cast<float>(proj_grav_z_ * obs_scale_projected_gravity_);
  step_obs[6] = static_cast<float>(lin_vel_cmd_x_ * obs_scale_commands_);
  step_obs[7] = static_cast<float>(lin_vel_cmd_y_ * obs_scale_commands_);
  step_obs[8] = static_cast<float>(ang_vel_cmd_z_ * obs_scale_commands_);
  step_obs[9] = static_cast<float>(left_theta_ * obs_scale_joint_pos_);
  step_obs[10] = static_cast<float>(right_theta_ * obs_scale_joint_pos_);
  step_obs[11] = static_cast<float>(left_d_theta_ * obs_scale_joint_vel_);
  step_obs[12] = static_cast<float>(right_d_theta_ * obs_scale_joint_vel_);
  step_obs[13] = static_cast<float>(left_l0_ * obs_scale_l0_);
  step_obs[14] = static_cast<float>(right_l0_ * obs_scale_l0_);
  step_obs[15] = static_cast<float>(left_l0_dot_ * obs_scale_l0_dot_);
  step_obs[16] = static_cast<float>(right_l0_dot_ * obs_scale_l0_dot_);
  step_obs[17] = static_cast<float>(wpos_l * obs_scale_joint_pos_);
  step_obs[18] = static_cast<float>(wpos_r * obs_scale_joint_pos_);
  step_obs[19] = static_cast<float>(wvel_l * obs_scale_joint_vel_);
  step_obs[20] = static_cast<float>(wvel_r * obs_scale_joint_vel_);
  for (int i = 0; i < ACTION_DIM; ++i)
    step_obs[21 + i] = static_cast<float>(last_action_[i] * obs_scale_actions_);
}

void RLController::push_obs_history(
    const std::array<float, NUM_OBS_PER_STEP> &step_obs) {
  for (int i = 0; i < NUM_OBS_PER_STEP; ++i)
    obs_history_[obs_history_write_idx_][i] = step_obs[i];
  obs_history_write_idx_ = (obs_history_write_idx_ + 1) % OBS_HISTORY_LEN;
  if (obs_history_write_idx_ == 0)
    obs_history_filled_ = true;
}

void RLController::flatten_obs_to_buffer() {
  int idx = 0;
  int start = obs_history_filled_ ? obs_history_write_idx_ : 0;
  for (int s = 0; s < OBS_HISTORY_LEN; ++s) {
    int k =
        obs_history_filled_
            ? (start + s) % OBS_HISTORY_LEN
            : ((s <= obs_history_write_idx_) ? s : (int)obs_history_write_idx_);
    for (int i = 0; i < NUM_OBS_PER_STEP; ++i)
      obs_buffer_[idx++] = obs_history_[k][i];
  }
}

bool RLController::run_inference(std::array<float, ACTION_DIM> &action_out) {
  if (!ort_session_)
    return false;
  try {
    std::array<int64_t, 2> input_shape = {1, OBS_DIM};
    Ort::MemoryInfo mem_info =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        mem_info, obs_buffer_.data(), obs_buffer_.size(), input_shape.data(),
        input_shape.size());

    Ort::RunOptions run_options;
    auto output_tensors =
        ort_session_->Run(run_options, input_names_cstr_.data(), &input_tensor,
                          1, output_names_cstr_.data(), 1);
    float *out_data = output_tensors[0].GetTensorMutableData<float>();
    for (size_t i = 0; i < ACTION_DIM; ++i)
      action_out[i] = out_data[i];
    return true;
  } catch (const Ort::Exception &e) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                          1000, "ONNX Run failed: %s", e.what());
    return false;
  }
}

void RLController::unscale_action(const std::array<float, ACTION_DIM> &raw,
                                  double &theta_l, double &l0_l, double &vel_l,
                                  double &theta_r, double &l0_r,
                                  double &vel_r) {
  theta_l = raw[0] * action_scale_theta_ * theta_invert_sign_;
  l0_l = raw[1] * action_scale_l0_ + l0_offset_;
  vel_l = raw[2] * action_scale_vel_;
  theta_r = raw[3] * action_scale_theta_ * theta_invert_sign_;
  l0_r = raw[4] * action_scale_l0_ + l0_offset_;
  vel_r = raw[5] * action_scale_vel_;
}

void RLController::publish_vmc_reference(double theta_l, double l0_l,
                                         double theta_r, double l0_r) {
  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data = {theta_l, l0_l, theta_r, l0_r};
  vmc_reference_pub_->publish(msg);
}

controller_interface::return_type
RLController::update(const rclcpp::Time & time,
                     const rclcpp::Duration &period) {
  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1)
    dt = 0.002;

  std::array<float, NUM_OBS_PER_STEP> step_obs;
  build_single_step_obs(step_obs);
  push_obs_history(step_obs);
  flatten_obs_to_buffer();

  // H1 fix: only run policy when 5-step observation history is filled
  if (!obs_history_filled_) {
    publish_vmc_reference(0.0, l0_offset_, 0.0, l0_offset_);
    if (!left_wheel_cmd_.empty())
      left_wheel_cmd_[0].get().set_value(0.0);
    if (!right_wheel_cmd_.empty())
      right_wheel_cmd_[0].get().set_value(0.0);
    if (rl_state_pub_) {
      rl_controller::msg::RLState msg;
      msg.header.stamp = get_node()->now();
      msg.header.frame_id = "base_link";
      msg.obs_history_filled = false;
      msg.received_vmc_state = received_vmc_state_;
      msg.received_imu = received_imu_;
      msg.proj_grav_x = proj_grav_x_;
      msg.proj_grav_y = proj_grav_y_;
      msg.proj_grav_z = proj_grav_z_;
      msg.left_theta = left_theta_;
      msg.left_d_theta = left_d_theta_;
      msg.left_l0 = left_l0_;
      msg.right_theta = right_theta_;
      msg.right_d_theta = right_d_theta_;
      msg.right_l0 = right_l0_;
      msg.theta_l_ref = 0.0;
      msg.l0_l_ref = l0_offset_;
      msg.theta_r_ref = 0.0;
      msg.l0_r_ref = l0_offset_;
      msg.lin_vel_cmd_x = lin_vel_cmd_x_;
      msg.ang_vel_cmd_z = ang_vel_cmd_z_;
      rl_state_pub_->publish(msg);
    }
    return controller_interface::return_type::OK;
  }

  std::array<float, ACTION_DIM> raw_action;
  if (!run_inference(raw_action)) {
    if (!left_wheel_cmd_.empty())
      left_wheel_cmd_[0].get().set_value(0.0);
    if (!right_wheel_cmd_.empty())
      right_wheel_cmd_[0].get().set_value(0.0);
    if (rl_state_pub_) {
      rl_controller::msg::RLState msg;
      msg.header.stamp = get_node()->now();
      msg.header.frame_id = "base_link";
      msg.obs_history_filled = obs_history_filled_;
      msg.received_vmc_state = received_vmc_state_;
      msg.received_imu = received_imu_;
      msg.proj_grav_x = proj_grav_x_;
      msg.proj_grav_y = proj_grav_y_;
      msg.proj_grav_z = proj_grav_z_;
      msg.left_theta = left_theta_;
      msg.left_l0 = left_l0_;
      msg.right_theta = right_theta_;
      msg.right_l0 = right_l0_;
      msg.lin_vel_cmd_x = lin_vel_cmd_x_;
      msg.ang_vel_cmd_z = ang_vel_cmd_z_;
      rl_state_pub_->publish(msg);
    }
    return controller_interface::return_type::OK;
  }

  // H4 fix: clip policy output to [-1,1] to match training (tanh) and avoid unbounded commands
  for (size_t i = 0; i < ACTION_DIM; ++i)
    raw_action[i] = std::max(-1.0f, std::min(1.0f, raw_action[i]));

  for (size_t i = 0; i < ACTION_DIM; ++i)
    last_action_[i] = raw_action[i];

  double theta_l, l0_l, vel_l, theta_r, l0_r, vel_r;
  unscale_action(raw_action, theta_l, l0_l, vel_l, theta_r, l0_r, vel_r);
  publish_vmc_reference(theta_l, l0_l, theta_r, l0_r);

  double wvel_l = 0.0, wvel_r = 0.0;
  if (!left_wheel_state_vel_.empty())
    wvel_l = left_wheel_state_vel_[0].get().get_value();
  if (!right_wheel_state_vel_.empty())
    wvel_r = right_wheel_state_vel_[0].get().get_value();

  wheel_vel_pid_l_->get_pid().setTarget(static_cast<float>(vel_l));
  wheel_vel_pid_r_->get_pid().setTarget(static_cast<float>(vel_r));
  double torque_l = static_cast<double>(
      wheel_vel_pid_l_->update(static_cast<float>(wvel_l), time));
  double torque_r = static_cast<double>(
      wheel_vel_pid_r_->update(static_cast<float>(wvel_r), time));

  // #region agent log
  update_count_++;
  double max_delta = 0.0;
  if (prev_raw_valid_) {
    for (int i = 0; i < ACTION_DIM; ++i) {
      double d = std::fabs(static_cast<double>(raw_action[i]) - prev_raw_action_[i]);
      if (d > max_delta) max_delta = d;
    }
  }
  for (size_t i = 0; i < ACTION_DIM; ++i) prev_raw_action_[i] = raw_action[i];
  prev_raw_valid_ = true;
  const bool throttle = (update_count_ % 125 == 0);
  if (throttle || max_delta > 0.8) {
    auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now().time_since_epoch())
                  .count();
    std::ostringstream js;
    js << "{\"timestamp\":" << ts
       << ",\"location\":\"rl_controller.cpp:update\",\"message\":\"rl_update\""
       << ",\"data\":{\"count\":" << update_count_
       << ",\"obs_filled\":" << (obs_history_filled_ ? "true" : "false")
       << ",\"vmc\":" << (received_vmc_state_ ? "true" : "false")
       << ",\"imu\":" << (received_imu_ ? "true" : "false")
       << ",\"obs0\":" << obs_buffer_[0] << ",\"obs1\":" << obs_buffer_[1]
       << ",\"obs2\":" << obs_buffer_[2] << ",\"obs3\":" << obs_buffer_[3]
       << ",\"obs4\":" << obs_buffer_[4] << ",\"obs5\":" << obs_buffer_[5]
       << ",\"raw0\":" << raw_action[0] << ",\"raw1\":" << raw_action[1]
       << ",\"raw2\":" << raw_action[2] << ",\"raw3\":" << raw_action[3]
       << ",\"raw4\":" << raw_action[4] << ",\"raw5\":" << raw_action[5]
       << ",\"theta_l\":" << theta_l << ",\"l0_l\":" << l0_l << ",\"vel_l\":" << vel_l
       << ",\"theta_r\":" << theta_r << ",\"l0_r\":" << l0_r << ",\"vel_r\":" << vel_r
       << ",\"torque_l\":" << torque_l << ",\"torque_r\":" << torque_r
       << ",\"wvel_l\":" << wvel_l << ",\"wvel_r\":" << wvel_r
       << ",\"max_delta\":" << max_delta << "}"
       << ",\"hypothesisId\":\"H1,H2,H3,H4,H5\",\"runId\":\"post-fix\"}";
    std::ofstream ofs("/home/kyx/code/wheel-legged-control/.cursor/debug.log",
                      std::ios::app);
    if (ofs) ofs << js.str() << "\n";
  }
  // #endregion

  // Publish rl_state for debug
  if (rl_state_pub_) {
    rl_controller::msg::RLState msg;
    msg.header.stamp = get_node()->now();
    msg.header.frame_id = "base_link";
    msg.obs_history_filled = obs_history_filled_;
    msg.received_vmc_state = received_vmc_state_;
    msg.received_imu = received_imu_;
    msg.proj_grav_x = proj_grav_x_;
    msg.proj_grav_y = proj_grav_y_;
    msg.proj_grav_z = proj_grav_z_;
    msg.ang_vel_x = ang_vel_x_;
    msg.ang_vel_y = ang_vel_y_;
    msg.ang_vel_z = ang_vel_z_;
    msg.left_theta = left_theta_;
    msg.left_d_theta = left_d_theta_;
    msg.left_l0 = left_l0_;
    msg.right_theta = right_theta_;
    msg.right_d_theta = right_d_theta_;
    msg.right_l0 = right_l0_;
    msg.raw0 = raw_action[0];
    msg.raw1 = raw_action[1];
    msg.raw2 = raw_action[2];
    msg.raw3 = raw_action[3];
    msg.raw4 = raw_action[4];
    msg.raw5 = raw_action[5];
    msg.theta_l_ref = theta_l;
    msg.l0_l_ref = l0_l;
    msg.theta_r_ref = theta_r;
    msg.l0_r_ref = l0_r;
    msg.vel_l_ref = vel_l;
    msg.vel_r_ref = vel_r;
    msg.left_wheel_velocity = wvel_l;
    msg.right_wheel_velocity = wvel_r;
    msg.torque_l = torque_l;
    msg.torque_r = torque_r;
    msg.lin_vel_cmd_x = lin_vel_cmd_x_;
    msg.ang_vel_cmd_z = ang_vel_cmd_z_;
    rl_state_pub_->publish(msg);
  }

  if (!left_wheel_cmd_.empty())
    left_wheel_cmd_[0].get().set_value(torque_l);
  if (!right_wheel_cmd_.empty())
    right_wheel_cmd_[0].get().set_value(torque_r);

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
RLController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.push_back(left_wheel_joint_name_ + "/" +
                      hardware_interface::HW_IF_EFFORT);
  cfg.names.push_back(right_wheel_joint_name_ + "/" +
                      hardware_interface::HW_IF_EFFORT);
  return cfg;
}

controller_interface::InterfaceConfiguration
RLController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.push_back(left_wheel_joint_name_ + "/" +
                      hardware_interface::HW_IF_POSITION);
  cfg.names.push_back(left_wheel_joint_name_ + "/" +
                      hardware_interface::HW_IF_VELOCITY);
  cfg.names.push_back(right_wheel_joint_name_ + "/" +
                      hardware_interface::HW_IF_POSITION);
  cfg.names.push_back(right_wheel_joint_name_ + "/" +
                      hardware_interface::HW_IF_VELOCITY);
  return cfg;
}

} // namespace rl_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rl_controller::RLController,
                       controller_interface::ControllerInterface)

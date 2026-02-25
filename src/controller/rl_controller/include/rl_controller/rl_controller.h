//
// RL Controller for ros2_control
// Deploys ONNX policy (cod_flat_vmc) with 135-dim obs -> 6-dim action;
// drives VMC via reference topic and wheel joints via effort (P velocity
// control).
//

#ifndef RLCONTROLLER_H
#define RLCONTROLLER_H

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "control_input_msgs/msg/inputs.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "vmc_controller/msg/vmc_state.hpp"
#include "rl_controller/msg/rl_state.hpp"

#include <onnxruntime_cxx_api.h>

#include "pid_ros.hpp"

namespace rl_controller {

// Single-step actor obs dim and history length (COD_VMC_IO)
constexpr int NUM_OBS_PER_STEP = 27;
constexpr int OBS_HISTORY_LEN = 5;
constexpr int OBS_DIM = OBS_HISTORY_LEN * NUM_OBS_PER_STEP; // 135
constexpr int ACTION_DIM = 6;

class RLController : public controller_interface::ControllerInterface {
public:
  RLController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

protected:
  // Wheel joint handles
  std::vector<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      left_wheel_cmd_;
  std::vector<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      right_wheel_cmd_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      left_wheel_state_pos_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      left_wheel_state_vel_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      right_wheel_state_pos_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      right_wheel_state_vel_;

  std::string left_wheel_joint_name_;
  std::string right_wheel_joint_name_;

  // Subscriptions
  rclcpp::Subscription<vmc_controller::msg::VMCState>::SharedPtr vmc_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr
      control_input_sub_;

  // Publisher: VMC reference [left_desired_theta, left_desired_l0,
  // right_desired_theta, right_desired_l0]
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      vmc_reference_pub_;
  rclcpp::Publisher<rl_controller::msg::RLState>::SharedPtr rl_state_pub_;

  std::string vmc_state_topic_;
  std::string rl_state_topic_;
  std::string imu_topic_;
  std::string control_input_topic_;
  std::string vmc_reference_topic_;

  // Latest data from topics
  bool received_vmc_state_;
  bool received_imu_;
  double left_theta_, left_d_theta_, right_theta_, right_d_theta_;
  double left_l0_, right_l0_;
  double ang_vel_x_, ang_vel_y_, ang_vel_z_;
  double proj_grav_x_, proj_grav_y_, proj_grav_z_;
  double lin_vel_cmd_x_, lin_vel_cmd_y_, ang_vel_cmd_z_;
  double left_l0_dot_, right_l0_dot_;
  double left_l0_prev_, right_l0_prev_;

  // Observation: 5 steps x 27, then flattened to 135 for ONNX
  std::array<float, OBS_DIM> obs_buffer_;
  std::array<std::array<float, NUM_OBS_PER_STEP>, OBS_HISTORY_LEN> obs_history_;
  int obs_history_write_idx_;
  bool obs_history_filled_;

  // Last action (6-dim, unscaled) for next step's obs[21:27] (stored scaled in
  // obs)
  std::array<float, ACTION_DIM> last_action_;

  // #region agent log
  int update_count_{0};
  std::array<float, ACTION_DIM> prev_raw_action_{};
  bool prev_raw_valid_{false};
  // #endregion

  // ONNX
  std::string policy_path_;
  std::unique_ptr<Ort::Session> ort_session_;
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<const char *> input_names_cstr_;
  std::vector<const char *> output_names_cstr_;
  Ort::Env ort_env_;

  // Obs scales (match LeggedLab cod_vmc)
  double obs_scale_ang_vel_;
  double obs_scale_projected_gravity_;
  double obs_scale_commands_;
  double obs_scale_joint_pos_;
  double obs_scale_joint_vel_;
  double obs_scale_l0_;
  double obs_scale_l0_dot_;
  double obs_scale_actions_;

  // Action scaling
  double action_scale_theta_;
  double action_scale_l0_;
  double l0_offset_;
  double action_scale_vel_;
  double theta_invert_sign_;  // 1.0 or -1.0 if VMC theta convention opposite to policy

  // Fall detection: when proj_grav_z > threshold, body is tilted (lying). Upright ≈ -1.
  double fall_threshold_proj_grav_z_;

  // Wheel velocity control (pid_gains.wheel_vel.left/right, same as vmc_controller)
  std::shared_ptr<PID::PidROS> wheel_vel_pid_l_;
  std::shared_ptr<PID::PidROS> wheel_vel_pid_r_;
  double v_max_;   // ly -> lin_vel_x
  double yaw_max_; // lx -> ang_vel_z

  void vmc_state_callback(const vmc_controller::msg::VMCState::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void
  control_input_callback(const control_input_msgs::msg::Inputs::SharedPtr msg);

  void build_single_step_obs(std::array<float, NUM_OBS_PER_STEP> &step_obs);
  void push_obs_history(const std::array<float, NUM_OBS_PER_STEP> &step_obs);
  void flatten_obs_to_buffer();
  bool run_inference(std::array<float, ACTION_DIM> &action_out);
  void unscale_action(const std::array<float, ACTION_DIM> &raw, double &theta_l,
                      double &l0_l, double &vel_l, double &theta_r,
                      double &l0_r, double &vel_r);
  void publish_vmc_reference(double theta_l, double l0_l, double theta_r,
                             double l0_r);

  static void quaternion_to_projected_gravity(double x, double y, double z,
                                              double w, double &gx, double &gy,
                                              double &gz);
};

} // namespace rl_controller

#endif // RLCONTROLLER_H

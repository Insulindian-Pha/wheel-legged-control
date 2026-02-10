#ifndef PID_ROS_H
#define PID_ROS_H

#include "pid.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <pid_controller/msg/pid_state.hpp>
#include <string>
#include <array>

namespace PID
{

/**
 * @class PidROS
 * @brief PID控制器的ROS2包装类，支持参数服务器配置和实时调参
 *
 * 这个类包装了基础的PID类，添加了ROS2参数服务器支持：
 * - 从参数服务器读取PID参数
 * - 支持实时更新参数（通过参数回调）
 * - 支持YAML配置文件初始化
 */
class PidROS
{
public:
  /**
   * @brief 构造函数（接受Node）
   *
   * @param node ROS2节点指针（用于访问参数服务器）
   * @param param_prefix 参数前缀（例如："pid_gains.position.joint1"）
   * @param use_parameter_callback 是否启用参数回调来实时更新参数（默认true）
   * @param publish_state 是否发布PID状态话题（默认true）
   */
  PidROS(rclcpp::Node::SharedPtr node, const std::string& param_prefix, bool use_parameter_callback = true,
         bool publish_state = true);

  /**
   * @brief 构造函数（接受LifecycleNode）
   *
   * @param node ROS2生命周期节点指针（用于访问参数服务器）
   * @param param_prefix 参数前缀（例如："pid_gains.position.joint1"）
   * @param use_parameter_callback 是否启用参数回调来实时更新参数（默认true）
   * @param publish_state 是否发布PID状态话题（默认true）
   */
  PidROS(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string& param_prefix,
         bool use_parameter_callback = true, bool publish_state = true);

  /**
   * @brief 从参数服务器初始化PID参数
   *
   * @return true 如果成功读取所有参数
   * @return false 如果参数读取失败
   */
  bool initialize_from_ros_parameters();

  /**
   * @brief 更新PID控制器
   *
   * @param target 目标值
   * @param feedback 反馈值
   * @param time 当前ROS2时间
   * @return float PID输出值
   */
  float update(float feedback, const rclcpp::Time& time);

  /**
   * @brief 重置PID控制器
   */
  void reset();

  /**
   * @brief 获取底层PID控制器（用于访问其他方法）
   *
   * @return PID& PID控制器引用
   */
  PID& get_pid()
  {
    return pid_;
  }

  /**
   * @brief 获取PID增益
   *
   * @return std::array<float, 3> [Kp, Ki, Kd]
   */
  std::array<float, 3> get_gains() const;

private:
  /**
   * @brief 发布PID状态到话题
   *
   * @param time 当前时间戳
   */
  void publish_state(const rclcpp::Time& time);

  /**
   * @brief 参数回调函数，当参数改变时自动更新PID参数
   *
   * @param parameters 改变的参数列表
   * @return rcl_interfaces::msg::SetParametersResult
   */
  rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter>& parameters);

  rclcpp::Node::SharedPtr node_;                               // ROS2节点指针（如果使用Node）
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;  // ROS2生命周期节点指针（如果使用LifecycleNode）
  std::string param_prefix_;                                   // 参数前缀
  PID pid_;                                                    // 底层PID控制器
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;  // 参数回调句柄

  // 辅助方法：获取logger（统一接口）
  rclcpp::Logger get_logger();

  // 状态发布相关
  bool publish_state_;                                                           // 是否发布状态
  rclcpp::Publisher<pid_controller::msg::PidState>::SharedPtr state_publisher_;  // 状态发布者
  std::string state_topic_name_;                                                 // 状态话题名称

  // 参数名称
  std::string param_target_;
  std::string param_p_;
  std::string param_i_;
  std::string param_d_;
  std::string param_max_;
  std::string param_min_;
  std::string param_integral_limit_;
  std::string param_integral_separation_threshold_;
};

}  // namespace PID

#endif  // PID_ROS_H

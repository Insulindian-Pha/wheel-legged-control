#include "pid_ros.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <array>
#include <algorithm>
#include <cmath>

namespace PID
{

PidROS::PidROS(rclcpp::Node::SharedPtr node, const std::string& param_prefix, bool use_parameter_callback,
               bool publish_state)
  : node_(node)
  , lifecycle_node_(nullptr)
  , param_prefix_(param_prefix)
  , pid_(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f)  // 临时初始化，将从参数服务器读取
  , publish_state_(publish_state)
{
  // 构建参数名称
  param_target_ = param_prefix_ + ".target";
  param_p_ = param_prefix_ + ".kp";
  param_i_ = param_prefix_ + ".ki";
  param_d_ = param_prefix_ + ".kd";
  param_max_ = param_prefix_ + ".u_clamp_max";
  param_min_ = param_prefix_ + ".u_clamp_min";
  param_integral_limit_ = param_prefix_ + ".i_clamp_max";
  param_integral_separation_threshold_ = param_prefix_ + ".i_separation_threshold";

  // 声明参数（带默认值）
  node_->declare_parameter<double>(param_target_, 0.0);
  node_->declare_parameter<double>(param_p_, 0.0);
  node_->declare_parameter<double>(param_i_, 0.0);
  node_->declare_parameter<double>(param_d_, 0.0);
  node_->declare_parameter<double>(param_max_, 1.0);
  node_->declare_parameter<double>(param_min_, -1.0);
  node_->declare_parameter<double>(param_integral_limit_, 0.0);
  node_->declare_parameter<double>(param_integral_separation_threshold_, 0.0);

  // 初始化参数
  if (!initialize_from_ros_parameters())
  {
    RCLCPP_WARN(get_logger(), "Failed to initialize PID parameters from parameter server for prefix: %s",
                param_prefix_.c_str());
  }

  // 注册参数回调（如果启用）
  if (use_parameter_callback)
  {
    param_callback_handle_ =
        node_->add_on_set_parameters_callback(std::bind(&PidROS::parameter_callback, this, std::placeholders::_1));
  }

  // 创建状态发布者（如果启用）
  if (publish_state_)
  {
    // 构建话题名称：将参数前缀中的点替换为斜杠，并添加/pid_state后缀
    // 例如：pid_gains.position.Left_front_joint -> /pid_gains/position/Left_front_joint/pid_state
    state_topic_name_ = "/" + param_prefix_;
    std::replace(state_topic_name_.begin(), state_topic_name_.end(), '.', '/');
    state_topic_name_ += "/pid_state";

    state_publisher_ = node_->create_publisher<pid_controller::msg::PidState>(state_topic_name_, 10);
    RCLCPP_DEBUG(get_logger(), "PID state publisher created for topic: %s", state_topic_name_.c_str());
  }
}

PidROS::PidROS(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string& param_prefix,
               bool use_parameter_callback, bool publish_state)
  : node_(nullptr)
  , lifecycle_node_(node)
  , param_prefix_(param_prefix)
  , pid_(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f)  // 临时初始化，将从参数服务器读取
  , publish_state_(publish_state)
{
  // 构建参数名称
  param_target_ = param_prefix_ + ".target";
  param_p_ = param_prefix_ + ".kp";
  param_i_ = param_prefix_ + ".ki";
  param_d_ = param_prefix_ + ".kd";
  param_max_ = param_prefix_ + ".u_clamp_max";
  param_min_ = param_prefix_ + ".u_clamp_min";
  param_integral_limit_ = param_prefix_ + ".i_clamp_max";
  param_integral_separation_threshold_ = param_prefix_ + ".i_separation_threshold";

  // 初始化参数
  if (!initialize_from_ros_parameters())
  {
    RCLCPP_WARN(get_logger(), "Failed to initialize PID parameters from parameter server for prefix: %s",
                param_prefix_.c_str());
  }

  // 注册参数回调（如果启用）
  if (use_parameter_callback)
  {
    param_callback_handle_ = lifecycle_node_->add_on_set_parameters_callback(
        std::bind(&PidROS::parameter_callback, this, std::placeholders::_1));
  }

  // 创建状态发布者（如果启用）
  if (publish_state_)
  {
    // 构建话题名称：将参数前缀中的点替换为斜杠，并添加/pid_state后缀
    // 例如：pid_gains.position.Left_front_joint -> /pid_gains/position/Left_front_joint/pid_state
    state_topic_name_ = "/" + param_prefix_;
    std::replace(state_topic_name_.begin(), state_topic_name_.end(), '.', '/');
    state_topic_name_ += "/pid_state";

    state_publisher_ = lifecycle_node_->create_publisher<pid_controller::msg::PidState>(state_topic_name_, 10);
    RCLCPP_DEBUG(get_logger(), "PID state publisher created for topic: %s", state_topic_name_.c_str());
  }
}

rclcpp::Logger PidROS::get_logger()
{
  if (node_)
  {
    return node_->get_logger();
  }
  else if (lifecycle_node_)
  {
    return lifecycle_node_->get_logger();
  }
  return rclcpp::get_logger("pid_controller");
}

bool PidROS::initialize_from_ros_parameters()
{
  try
  {
    // 读取PID增益参数
    double target, kp, ki, kd, max, min, integral_limit, integral_separation_threshold;

    if (node_)
    {
      target = node_->get_parameter(param_target_).as_double();
      kp = node_->get_parameter(param_p_).as_double();
      ki = node_->get_parameter(param_i_).as_double();
      kd = node_->get_parameter(param_d_).as_double();
      max = node_->get_parameter(param_max_).as_double();
      min = node_->get_parameter(param_min_).as_double();
      integral_limit = node_->get_parameter(param_integral_limit_).as_double();
      integral_separation_threshold = 0.0;
      if (node_->has_parameter(param_integral_separation_threshold_))
      {
        integral_separation_threshold = node_->get_parameter(param_integral_separation_threshold_).as_double();
      }
    }
    else if (lifecycle_node_)
    {
      target = lifecycle_node_->get_parameter(param_target_).as_double();
      kp = lifecycle_node_->get_parameter(param_p_).as_double();
      ki = lifecycle_node_->get_parameter(param_i_).as_double();
      kd = lifecycle_node_->get_parameter(param_d_).as_double();
      max = lifecycle_node_->get_parameter(param_max_).as_double();
      min = lifecycle_node_->get_parameter(param_min_).as_double();
      integral_limit = lifecycle_node_->get_parameter(param_integral_limit_).as_double();
      integral_separation_threshold = 0.0;
      if (lifecycle_node_->has_parameter(param_integral_separation_threshold_))
      {
        integral_separation_threshold =
            lifecycle_node_->get_parameter(param_integral_separation_threshold_).as_double();
      }
    }
    else
    {
      return false;
    }

    // 更新PID控制器参数
    pid_.setTarget(static_cast<float>(target));
    pid_.setK(static_cast<float>(kp), static_cast<float>(ki), static_cast<float>(kd));
    pid_.setMax(static_cast<float>(std::max(std::abs(max), std::abs(min))));  // 使用较大的绝对值
    pid_.setIntegralLimit(static_cast<float>(integral_limit));
    pid_.setIntegralSeparation(static_cast<float>(integral_separation_threshold));

    RCLCPP_DEBUG(get_logger(),
                 "PID parameters loaded for '%s': Kp=%.3f, Ki=%.3f, Kd=%.3f, Max=%.3f, I_Limit=%.3f, "
                 "I_Separation=%.3f",
                 param_prefix_.c_str(), kp, ki, kd, std::max(std::abs(max), std::abs(min)), integral_limit,
                 integral_separation_threshold);

    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Exception while reading PID parameters for '%s': %s", param_prefix_.c_str(), e.what());
    return false;
  }
}

float PidROS::update(float feedback, const rclcpp::Time& time)
{
  float output = pid_.Update(feedback, time);

  // 发布状态（如果启用）
  if (publish_state_)
  {
    publish_state(time);
  }

  return output;
}

void PidROS::reset()
{
  pid_.reset();
}

std::array<float, 3> PidROS::get_gains() const
{
  // 从PID类获取当前增益值
  return { pid_.getKp(), pid_.getKi(), pid_.getKd() };
}

rcl_interfaces::msg::SetParametersResult PidROS::parameter_callback(const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool pid_updated = false;

  for (const auto& param : parameters)
  {
    // 检查是否是当前PID的参数
    if (param.get_name().find(param_prefix_) == 0)
    {
      pid_updated = true;
      break;
    }
  }

  // 如果有相关参数改变，重新初始化
  if (pid_updated)
  {
    if (initialize_from_ros_parameters())
    {
      RCLCPP_INFO(get_logger(), "PID parameters updated for '%s'", param_prefix_.c_str());
    }
    else
    {
      result.successful = false;
      result.reason = "Failed to update PID parameters";
    }
  }

  return result;
}

void PidROS::publish_state(const rclcpp::Time& time)
{
  if (!state_publisher_)
  {
    return;
  }

  pid_controller::msg::PidState state_msg;
  state_msg.header.stamp = time;
  state_msg.header.frame_id = "";  // 可以设置为关节名称或其他标识

  // 期望值/反馈值/误差值
  state_msg.target = pid_.getTarget();
  state_msg.feedback = pid_.getFeedback();
  state_msg.error = pid_.getError();

  // PID增益
  state_msg.kp = pid_.getKp();
  state_msg.ki = pid_.getKi();
  state_msg.kd = pid_.getKd();

  // 各项输出
  state_msg.p_out = pid_.getPOutput();
  state_msg.i_out = pid_.getIOutput();
  state_msg.d_out = pid_.getDOutput();

  // 总输出
  state_msg.out = pid_.getOutput();

  state_publisher_->publish(state_msg);
}

}  // namespace PID

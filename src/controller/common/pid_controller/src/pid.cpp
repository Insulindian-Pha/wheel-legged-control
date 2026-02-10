#include "pid.hpp"
#include <algorithm>
#include <cmath>

namespace PID
{
PID::PID(float kp, float ki, float kd, float max, float integral_limit, float integral_separation_threshold_)
  : max_(max)
  , min_(-max)
  , integral_limit_(integral_limit)
  , integral_separation_threshold_(integral_separation_threshold_)
{
  // 设定增益
  k_[0] = kp;
  k_[1] = ki;
  k_[2] = kd;
  // 初始化时间记录为空
  previous_time_.reset();
}

float PID::Update(float feedback, const rclcpp::Time& time)
{
  setFeedback(feedback);

  error_ = target_ - feedback_;

  // 计算时间差 dt
  float dt = 0.0f;
  if (previous_time_.has_value())
  {
    rclcpp::Duration period = time - previous_time_.value();
    dt = static_cast<float>(period.seconds());

    // 安全检查：防止异常的时间间隔
    if (dt <= 0.0f || dt > 1.0f)
    {
      // 如果时间异常，使用默认值（假设500Hz，即0.002秒）
      dt = 0.002f;
    }
  }
  else
  {
    // 第一次调用，没有历史时间，使用默认dt
    dt = 0.002f;  // 假设500Hz采样率
  }

  // 更新时间为下一次调用做准备
  previous_time_ = time;

  // 比例项
  k_out_[0] = k_[0] * error_;

  // 积分项处理（使用dt）
  // 积分隔离：只有当误差小于阈值时才进行积分累积
  // 当阈值为0时，禁用积分隔离功能，积分始终工作
  if (integral_separation_threshold_ == 0.0f || std::abs(error_) < integral_separation_threshold_)
  {
    integral_ += error_ * dt;  // 使用dt进行积分

    // 积分限幅：限制积分累积值的范围
    if (integral_limit_ > 0.0f)
    {
      integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
    }
  }

  k_out_[1] = k_[1] * integral_;

  // 微分项（使用dt）
  float error_derivative = 0.0f;
  if (dt > 0.0f)
  {
    error_derivative = (error_ - previous_error_) / dt;
  }
  k_out_[2] = k_[2] * error_derivative;

  // 计算输出
  output_ = k_out_[0] + k_out_[1] + k_out_[2];

  // 输出限幅
  output_ = std::clamp(output_, min_, max_);

  // 积分抗饱和：如果输出饱和，则回退积分累积
  if ((output_ >= max_ && error_ > 0) || (output_ <= min_ && error_ < 0))
  {
    integral_ -= error_ * dt;  // 使用dt进行回退
    // 确保积分不超出限制
    if (integral_limit_ > 0.0f)
    {
      integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
    }
  }

  // 更新历史误差
  previous_error_ = error_;

  return output_;
}

}  // namespace PID

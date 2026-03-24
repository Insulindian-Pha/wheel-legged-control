#include "lqr_controller/odom/odom_publisher.hpp"

#include <algorithm>
#include <iterator>

namespace lqr_controller
{
namespace odom
{

OdomPublisher::OdomPublisher(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
  const std::string& imu_topic,
  const std::string& odom_topic,
  const std::string& odom_frame_id,
  const std::string& base_frame_id,
  bool enable_odom_tf,
  bool enable_odom_msg,
  bool require_imu_for_odom,
  bool odom_use_pitch,
  double odom_publish_rate)
{
  node_ = node;
  enable_odom_tf_ = enable_odom_tf;
  enable_odom_msg_ = enable_odom_msg;
  require_imu_for_odom_ = require_imu_for_odom;
  odom_use_pitch_ = odom_use_pitch;
  odom_publish_rate_ = odom_publish_rate;

  odom_frame_id_ = odom_frame_id;
  base_frame_id_ = base_frame_id;

  if (enable_odom_msg_)
  {
    odom_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
  }
  if (enable_odom_tf_)
  {
    odom_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  }

  // IMU subscription: orientation (RPY) + gyro for odom pose / twist.
  imu_subscription_ = node_->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, 10, std::bind(&OdomPublisher::imu_callback, this, std::placeholders::_1));
}

void OdomPublisher::reset()
{
  estimator_.reset();
  received_imu_ = false;
  roll_ = 0.0;
  pitch_ = 0.0;
  yaw_ = 0.0;
  yaw_gyro_ = 0.0;
  pitch_gyro_ = 0.0;
  roll_gyro_ = 0.0;
  last_odom_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void OdomPublisher::update(double dt, double v_body)
{
  if (require_imu_for_odom_ && !received_imu_)
  {
    return;
  }

  // Update planar pose using the current yaw from IMU.
  estimator_.update(dt, v_body, yaw_);

  if (!enable_odom_tf_ && !enable_odom_msg_)
  {
    return;
  }

  const auto now = node_->now();
  if (odom_publish_rate_ > 0.0 && last_odom_publish_time_.nanoseconds() != 0)
  {
    const auto min_interval = rclcpp::Duration::from_seconds(1.0 / odom_publish_rate_);
    if ((now - last_odom_publish_time_) < min_interval)
    {
      return;
    }
  }
  last_odom_publish_time_ = now;

  // Build pose orientation (RPY from IMU; pitch optional zero per odom_use_pitch_).
  const double roll = roll_;
  const double pitch = odom_use_pitch_ ? pitch_ : 0.0;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw_);
  q.normalize();

  geometry_msgs::msg::Quaternion quat;
  quat.x = q.x();
  quat.y = q.y();
  quat.z = q.z();
  quat.w = q.w();

  // TF
  if (enable_odom_tf_ && odom_tf_broadcaster_)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now;
    tf_msg.header.frame_id = odom_frame_id_;
    tf_msg.child_frame_id = base_frame_id_;

    tf_msg.transform.translation.x = estimator_.x();
    tf_msg.transform.translation.y = estimator_.y();
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = quat;
    odom_tf_broadcaster_->sendTransform(tf_msg);
  }

  // /odom
  if (enable_odom_msg_ && odom_publisher_)
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;

    odom_msg.pose.pose.position.x = estimator_.x();
    odom_msg.pose.pose.position.y = estimator_.y();
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = quat;

    odom_msg.twist.twist.linear.x = estimator_.vx();
    odom_msg.twist.twist.linear.y = estimator_.vy();
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = roll_gyro_;
    odom_msg.twist.twist.angular.y = odom_use_pitch_ ? pitch_gyro_ : 0.0;
    odom_msg.twist.twist.angular.z = yaw_gyro_;

    std::fill(std::begin(odom_msg.pose.covariance), std::end(odom_msg.pose.covariance), 0.0);
    std::fill(std::begin(odom_msg.twist.covariance), std::end(odom_msg.twist.covariance), 0.0);

    odom_publisher_->publish(odom_msg);
  }
}

void OdomPublisher::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  received_imu_ = true;

  tf2::Quaternion q_imu(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  q_imu.normalize();
  tf2::Matrix3x3 m(q_imu);
  m.getRPY(roll_, pitch_, yaw_);

  roll_gyro_ = msg->angular_velocity.x;
  pitch_gyro_ = msg->angular_velocity.y;
  yaw_gyro_ = msg->angular_velocity.z;
}

}  // namespace odom
}  // namespace lqr_controller


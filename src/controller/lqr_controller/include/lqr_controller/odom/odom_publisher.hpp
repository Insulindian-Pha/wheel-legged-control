#ifndef LQR_CONTROLLER_ODOM_PUBLISHER_HPP_
#define LQR_CONTROLLER_ODOM_PUBLISHER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "lqr_controller/odom/odom_estimator.hpp"

// TF
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace lqr_controller
{
namespace odom
{

class OdomPublisher
{
public:
  OdomPublisher(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::string& imu_topic,
    const std::string& odom_topic,
    const std::string& odom_frame_id,
    const std::string& base_frame_id,
    bool enable_odom_tf,
    bool enable_odom_msg,
    bool require_imu_for_odom,
    bool odom_use_pitch,
    double odom_publish_rate);

  void reset();

  // dt: controller周期时长
  // v_body: 机体系前向速度 (base_link 的 +X)，单位 m/s
  void update(double dt, double v_body);

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  static double quaternion_to_pitch(double x, double y, double z, double w);
  static double quaternion_to_yaw(double x, double y, double z, double w);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // Estimator (x,y,vx,vy in world frame)
  PlanarOdomEstimator estimator_;

  // IMU state
  bool received_imu_{false};
  double yaw_{0.0};
  double pitch_{0.0};
  double yaw_gyro_{0.0};
  double pitch_gyro_{0.0};
  double roll_gyro_{0.0};

  // Publishers / TF
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

  // Params
  bool enable_odom_tf_{true};
  bool enable_odom_msg_{true};
  bool require_imu_for_odom_{true};
  bool odom_use_pitch_{true};
  double odom_publish_rate_{50.0};

  std::string odom_frame_id_;
  std::string base_frame_id_;
  rclcpp::Time last_odom_publish_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace odom
}  // namespace lqr_controller

#endif  // LQR_CONTROLLER_ODOM_PUBLISHER_HPP_


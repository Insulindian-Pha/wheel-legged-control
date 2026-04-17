#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

class ImuTfBroadcaster : public rclcpp::Node
{
public:
  ImuTfBroadcaster()
  : Node("imu_tf_broadcaster"), broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu_broadcaster/imu");
    parent_frame_ = declare_parameter<std::string>("parent_frame", "odom");
    child_frame_ = declare_parameter<std::string>("child_frame", "base_link");
    use_imu_stamp_ = declare_parameter<bool>("use_imu_stamp", true);

    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ImuTfBroadcaster::imu_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Publishing TF from IMU topic '%s': %s -> %s",
      imu_topic_.c_str(), parent_frame_.c_str(), child_frame_.c_str());
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = child_frame_;
    if (use_imu_stamp_) {
      transform.header.stamp = msg->header.stamp;
    } else {
      transform.header.stamp = now();
    }
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = msg->orientation;
    broadcaster_->sendTransform(transform);
  }

  std::string imu_topic_;
  std::string parent_frame_;
  std::string child_frame_;
  bool use_imu_stamp_{true};
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}

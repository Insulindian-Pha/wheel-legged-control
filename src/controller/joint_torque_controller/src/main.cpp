//
// Main function for joint_torque_controller node
//

#include "joint_torque_controller/JointTorqueController.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTorqueController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

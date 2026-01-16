//
// Main function for control_converter node
//

#include "control_converter/ControlConverter.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

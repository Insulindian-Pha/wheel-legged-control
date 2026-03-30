//
// Created by tlab-uav on 24-9-13.
//

#include "joystick_input/JoystickInput.h"

using std::placeholders::_1;

JoystickInput::JoystickInput() : Node("joysick_input_node")
{
  publisher_ = create_publisher<control_input_msgs::msg::Inputs>("control_input", 10);
  subscription_ =
      create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoystickInput::joy_callback, this, _1));
}

void JoystickInput::joy_callback(sensor_msgs::msg::Joy::SharedPtr msg)
{
  // 独立的 start/stop 命令：start 锁存（按下 START 后保持 1，直到按下 STOP 置 0）
  const bool stop_btn = (msg->buttons.size() > 6) ? static_cast<bool>(msg->buttons[6]) : false;
  const bool start_btn = (msg->buttons.size() > 7) ? static_cast<bool>(msg->buttons[7]) : false;
  const bool stop_rising = stop_btn && !prev_stop_btn_;
  const bool start_rising = start_btn && !prev_start_btn_;

  if (stop_rising) {
    start_latched_ = false;
  } else if (start_rising) {
    start_latched_ = true;
  }

  inputs_.start = start_latched_;
  inputs_.stop = stop_rising;
  prev_stop_btn_ = stop_btn;
  prev_start_btn_ = start_btn;

  if (msg->buttons[1] && msg->buttons[4])
  {
    inputs_.command = 1;  // LB + B
  }
  else if (msg->buttons[0] && msg->buttons[4])
  {
    inputs_.command = 2;  // LB + A
  }
  else if (msg->buttons[2] && msg->buttons[4])
  {
    inputs_.command = 3;  // LB + X
  }
  else if (msg->buttons[3] && msg->buttons[4])
  {
    inputs_.command = 4;  // LB + Y
  }
  else if (msg->axes[2] != 1 && msg->buttons[1])
  {
    inputs_.command = 5;  // LT + B
  }
  else if (msg->axes[2] != 1 && msg->buttons[0])
  {
    inputs_.command = 6;  // LT + A
  }
  else if (msg->axes[2] != 1 && msg->buttons[2])
  {
    inputs_.command = 7;  // LT + X
  }
  else if (msg->axes[2] != 1 && msg->buttons[3])
  {
    inputs_.command = 8;  // LT + Y
  }
  else if (msg->buttons[7])
  {
    inputs_.command = 9;  // START
  }
  else
  {
    inputs_.command = 0;
    inputs_.lx = -msg->axes[0];
    inputs_.ly = msg->axes[1];
    inputs_.rx = -msg->axes[3];
    inputs_.ry = msg->axes[4];
    inputs_.lt = msg->axes[2];
    inputs_.rt = msg->axes[5];
  }
  publisher_->publish(inputs_);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoystickInput>();
  spin(node);
  rclcpp::shutdown();
  return 0;
}

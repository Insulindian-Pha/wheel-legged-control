#!/bin/bash
# 重装经典 Gazebo 脚本

set -e

echo "=== 步骤 1: 卸载现有的 Gazebo 包 ==="
sudo apt remove --purge gazebo* libgazebo* -y

echo "=== 步骤 2: 清理残留配置和缓存 ==="
sudo apt autoremove -y
sudo apt autoclean

echo "=== 步骤 3: 更新软件包列表 ==="
sudo apt update

echo "=== 步骤 4: 安装经典 Gazebo 11 ==="
sudo apt install gazebo11 gazebo11-common libgazebo11-dev -y

echo "=== 步骤 5: 安装 ROS 2 Humble 的 Gazebo 集成包 ==="
sudo apt install ros-humble-gazebo-ros-pkgs -y

echo "=== 步骤 6: 验证安装 ==="
echo "Gazebo 版本:"
gazebo --version

echo ""
echo "=== 安装完成！ ==="
echo "如果遇到问题，可以尝试："
echo "1. 检查 ROS 2 环境: source /opt/ros/humble/setup.bash"
echo "2. 检查 Gazebo 模型路径: echo \$GAZEBO_MODEL_PATH"


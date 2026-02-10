#!/bin/bash
# wheel-legged-control 项目依赖安装脚本
# 适用于 Ubuntu 22.04 (Jammy) + ROS 2 Humble

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}=== wheel-legged-control 依赖安装脚本 ===${NC}"
echo "工作空间: $WORKSPACE_ROOT"
echo ""

# 2. 检测 ROS 2 发行版
if [ -z "$ROS_DISTRO" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        export ROS_DISTRO=humble
        echo -e "${GREEN}[2/6] 检测到 ROS 2 Humble${NC}"
    elif [ -f /opt/ros/jazzy/setup.bash ]; then
        export ROS_DISTRO=jazzy
        echo -e "${GREEN}[2/6] 检测到 ROS 2 Jazzy${NC}"
    elif [ -f /opt/ros/iron/setup.bash ]; then
        export ROS_DISTRO=iron
        echo -e "${GREEN}[2/6] 检测到 ROS 2 Iron${NC}"
    else
        echo -e "${RED}未找到 ROS 2 安装，请先安装 ROS 2:${NC}"
        echo "  Ubuntu 22.04: sudo apt install ros-humble-desktop"
        echo "  Ubuntu 24.04: sudo apt install ros-jazzy-desktop"
        exit 1
    fi
else
    echo -e "${GREEN}[2/6] 使用已设置的 ROS_DISTRO=$ROS_DISTRO${NC}"
fi

# 3. 更新 apt 并安装系统依赖
echo -e "${GREEN}[3/6] 安装系统依赖...${NC}"
sudo apt update
sudo apt install -y \
    libglfw3-dev \
    python3-pykdl \
    python3-venv \
    python3-pip \
    python3-numpy \
    python3-pynput

# python3-mujoco: 优先 apt，若无则 pip
if apt-cache show python3-mujoco &>/dev/null; then
    sudo apt install -y python3-mujoco
else
    echo -e "${YELLOW}通过 pip 安装 mujoco...${NC}"
    pip3 install --user mujoco || true
fi

# 4. 安装 Gazebo Classic 与 ROS 2 集成
echo -e "${GREEN}[4/6] 安装 Gazebo Classic 与 ROS 2 集成...${NC}"
sudo apt install -y gazebo libgazebo-dev
sudo apt install -y ros-$ROS_DISTRO-gazebo-ros-pkgs 2>/dev/null || echo -e "${YELLOW}gazebo-ros-pkgs 可能在此发行版不可用${NC}"

# 5. 安装 ROS 2 依赖
echo -e "${GREEN}[5/6] 安装 ROS 2 依赖...${NC}"
source /opt/ros/$ROS_DISTRO/setup.bash

# mujoco_ros2_control 所需的 ROS 2 包
sudo apt install -y \
    ros-$ROS_DISTRO-control-toolbox \
    ros-$ROS_DISTRO-backward-ros \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-transmission-interface \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-controller-interface \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-position-controllers \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-joint-state-publisher-gui

# Gazebo Classic 仿真所需的 gazebo_ros2_control（提供 libgazebo_ros2_control.so）
sudo apt install -y ros-$ROS_DISTRO-gazebo-ros2-control 2>/dev/null || echo -e "${YELLOW}gazebo-ros2-control 可能在此发行版不可用，Gazebo 仿真将无法加载插件${NC}"

# 6. 使用 rosdep 安装其余依赖（若可用）
echo -e "${GREEN}[6/6] 运行 rosdep 检查...${NC}"
if command -v rosdep &>/dev/null; then
    cd "$WORKSPACE_ROOT"
    rosdep update 2>/dev/null || true
    rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || echo -e "${YELLOW}rosdep 部分包可能未找到，已安装的依赖应足够构建${NC}"
else
    echo -e "${YELLOW}未安装 rosdep，跳过。可执行: sudo apt install python3-rosdep && rosdep update${NC}"
fi

echo ""
echo -e "${GREEN}=== 安装完成 ===${NC}"
echo "请执行以下命令构建项目:"
echo "  source /opt/ros/$ROS_DISTRO/setup.bash"
echo "  cd $WORKSPACE_ROOT"
echo "  ./make.sh"
echo ""
echo "若之前禁用了 Cursor 源，恢复命令:"
echo "  sudo mv /etc/apt/sources.list.d/cursor.list.disabled /etc/apt/sources.list.d/cursor.list"

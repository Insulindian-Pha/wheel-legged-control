# COD-2026RoboMaster-Balance ROS2 功能包

这是辽宁科技大学COD战队的RM2026-串联腿闭链结构机器人的ROS2描述包。

## 机器人结构

COD-2026RoboMaster-Balance是一个四轮腿式机器人，具有复杂的串联腿闭链结构。

## 依赖

确保安装了以下 ROS2 包：

```bash
sudo apt install ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-rviz2 \
                 ros-humble-gazebo-ros \
                 ros-humble-xacro
```

## 构建

```bash
cd ~/your_workspace
colcon build --packages-select cod_2026_balance
source install/setup.bash
```

## 使用方法

### 1. 在 RViz2 中可视化机器人

启动 RViz2 显示机器人模型并使用 GUI 控制关节：

```bash
ros2 launch cod_2026_balance display.launch.py
```

这将启动：
- `robot_state_publisher`: 发布机器人 TF 变换
- `joint_state_publisher_gui`: GUI 界面用于手动控制关节
- `rviz2`: 可视化工具

### 2. 在经典 Gazebo 中仿真

启动经典 Gazebo 仿真环境：

```bash
ros2 launch cod_2026_balance gazebo.launch.py
```

这将启动：
- 经典 Gazebo 仿真环境
- 在 Gazebo 中生成机器人模型
- `robot_state_publisher`: 发布机器人状态
- 静态 TF 发布器 (base_link → base_footprint)

### 3. 在 MuJoCo 中仿真

启动 MuJoCo 仿真环境：

```bash
ros2 launch cod_2026_balance mujoco.launch.py
```

或者指定 MuJoCo 可执行文件路径：

```bash
ros2 launch cod_2026_balance mujoco.launch.py mujoco_exe:=/path/to/mujoco/simulate
```

这将启动：
- MuJoCo 仿真环境
- 加载 MJCF 模型文件
- 显示机器人3D模型

**注意**: 需要先安装 MuJoCo 和相应的命令行工具。MuJoCo 的安装请参考 [MuJoCo 官方文档](https://mujoco.readthedocs.io/)。

## 文件结构

```
COD-2026RoboMaster-Balance/
├── CMakeLists.txt           # ROS2 ament_cmake 构建配置
├── package.xml              # ROS2 包清单 (format 3)
├── launch/
│   ├── display.launch.py    # RViz2 启动文件
│   ├── gazebo.launch.py     # 经典Gazebo 启动文件
│   └── mujoco.launch.py     # MuJoCo 启动文件
├── meshes/
│   └── *.STL                # 机器人3D模型文件
├── urdf/
│   └── COD_2026_Balance_2_0.urdf  # 机器人 URDF 描述文件
├── MJCF/                    # MuJoCo 模型文件
├── USD/                     # Isaac Sim 模型文件
└── Pictures/                # 图片资源
```

## 原始文件说明

本包基于辽宁科技大学COD战队开源的RM2026机器人模型文件创建，原始文件包含：
- **MJCF**: MuJoCo仿真格式文件
- **USD**: Isaac Sim仿真格式文件
- **URDF**: ROS机器人描述格式文件

| MJCF in Mujoco | USD in IsaacSim5.0 |
| :---: | :---: |
| ![描述1](./Pictures/MJCF%20in%20Mujoco.png) | ![描述2](./Pictures/USD%20in%20IsaacSim5.0.png) |

## 许可证

BSD

## 作者

- 原始模型：辽宁科技大学COD战队
- ROS2包：基于原始URDF模型创建

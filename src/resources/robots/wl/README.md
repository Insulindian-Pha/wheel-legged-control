# WL 轮腿式机器人 ROS2 包

这是一个轮腿式机器人的 URDF 描述包，已从 ROS1 完整迁移到 ROS2。

## 机器人结构

- **基座**: base_link
- **左前腿**: lf0_Joint (髋关节), lf1_Joint (膝关节)
- **左轮**: l_wheel_Joint (连续旋转)
- **右前腿**: rf0_Joint (髋关节), rf1_Joint (膝关节)
- **右轮**: r_wheel_Joint (连续旋转)

总共 6 个自由度（4 个腿部关节 + 2 个轮子）。

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
colcon build --packages-select wl
source install/setup.bash
```

## 使用方法

### 1. 在 RViz2 中可视化机器人

启动 RViz2 显示机器人模型并使用 GUI 控制关节：

```bash
ros2 launch wl display.launch.py
```

这将启动：
- `robot_state_publisher`: 发布机器人 TF 变换
- `joint_state_publisher_gui`: GUI 界面用于手动控制关节
- `rviz2`: 可视化工具

### 2. 在 Gazebo 中仿真

启动 Gazebo 仿真环境：

```bash
ros2 launch wl gazebo.launch.py
```

这将启动：
- Gazebo 仿真环境
- 在 Gazebo 中生成机器人模型
- `robot_state_publisher`: 发布机器人状态
- 静态 TF 发布器 (base_link → base_footprint)
- 发布校准消息到 `/calibrated` 话题

## 文件结构

```
wl/
├── CMakeLists.txt           # ROS2 ament_cmake 构建配置
├── package.xml              # ROS2 包清单 (format 3)
├── config/
│   ├── display.rviz         # RViz2 配置文件
│   └── joint_names_wheel_legged_open.yaml  # 关节名称配置
├── launch/
│   ├── display.launch.py    # RViz2 启动文件
│   └── gazebo.launch.py     # Gazebo 启动文件
├── meshes/
│   ├── base_link.STL
│   ├── lf0_Link.STL
│   ├── lf1_Link.STL
│   ├── l_wheel_Link.STL
│   ├── rf0_Link.STL
│   ├── rf1_Link.STL
│   └── r_wheel_Link.STL
└── urdf/
    └── wl.urdf              # 机器人 URDF 描述文件
```

## ROS1 到 ROS2 迁移内容

### 已完成的迁移项目：

1. ✅ **package.xml**: 从 format 2 升级到 format 3，更新依赖项
   - `catkin` → `ament_cmake`
   - `rviz` → `rviz2`
   - 移除 `roslaunch` 依赖

2. ✅ **CMakeLists.txt**: 完全重写使用 ament_cmake
   - 移除 catkin 宏
   - 使用 ament_cmake API
   - 添加测试支持

3. ✅ **启动文件**: 从 XML 格式转换为 Python 格式
   - `display.launch` → `display.launch.py`
   - `gazebo.launch` → `gazebo.launch.py`
   - 使用 ROS2 launch API

4. ✅ **URDF 文件**: 更新网格路径
   - 从相对路径 `../meshes/` 改为 `package://wl/meshes/`
   - 更符合 ROS2 最佳实践

5. ✅ **RViz 配置**: 创建 ROS2 兼容的 RViz 配置文件
   - 适配 RViz2 格式
   - 配置机器人显示和 TF 可视化

### 主要变更说明：

- **TF**: 使用 `tf2_ros` 替代 ROS1 的 `tf`
- **Gazebo**: 使用 `spawn_entity.py` 替代 `spawn_model`
- **话题命令**: 使用 `ros2 topic pub` 替代 `rostopic pub`
- **包查找**: 使用 `ament_index_python` 替代 `rospkg`

## 后续开发建议

1. **添加 Gazebo 插件**
   - 为关节添加控制器插件
   - 为轮子添加差速驱动插件
   - 添加传感器（IMU、激光雷达等）

2. **控制器开发**
   - 实现轮腿混合运动控制
   - 开发平衡控制算法
   - 添加轨迹规划

3. **测试**
   - 添加单元测试
   - 添加集成测试

## 许可证

BSD

## 作者

- 原始模型：从 SolidWorks 导出
- ROS2 迁移：[维护者]

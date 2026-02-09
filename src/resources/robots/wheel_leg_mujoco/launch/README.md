# wheel_leg_mujoco Launch 文件

## 使用方法

### 1. 仅查看模型（使用 MuJoCo Viewer）

启动 MuJoCo viewer 来查看机器人模型：

```bash
ros2 launch wheel_leg_mujoco mujoco.launch.py
```

或者指定自定义的 MuJoCo 命令：

```bash
ros2 launch wheel_leg_mujoco mujoco.launch.py mujoco_cmd:="python3 -m mujoco.viewer"
```

### 2. 运行完整仿真（包含 VMC 控制）

运行完整的仿真，包括 VMC 控制和键盘输入：

```bash
ros2 launch wheel_leg_mujoco mujoco.launch.py sim_mode:=simulation
```

## 参数说明

- `mujoco_cmd`: MuJoCo 命令（默认: `python3 -m mujoco.viewer`）
- `sim_mode`: 仿真模式
  - `viewer`: 仅查看模型（默认）
  - `simulation`: 运行完整仿真（Simulation.py）

## 注意事项

1. 确保已安装 MuJoCo Python 包：
   ```bash
   pip3 install mujoco
   ```

2. 如果运行完整仿真，还需要安装其他依赖：
   ```bash
   pip3 install pynput numpy
   ```

3. 工作目录会自动设置为正确的位置，以确保 mesh 文件路径正确。


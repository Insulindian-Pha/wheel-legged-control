# COD-2026RoboMaster-Balance VMC 验证指南

## 概述

使用 `vmc_verification.py` 脚本来验证 COD-2026RoboMaster-Balance 机器人的 VMC 控制器计算是否正确。

## 使用方法

### 1. 启动仿真

在第一个终端启动 MuJoCo 仿真：

```bash
ros2 launch bringup mujoco_ros2_control.launch.py
```

确保使用 COD-2026RoboMaster-Balance 的配置文件。

### 2. 运行验证脚本

在第二个终端运行验证脚本（**从项目根目录运行**）：

```bash
# 方法1：从项目根目录运行（推荐）
cd /home/yons/code/learn_repo/wheel-legged-control
python3 src/resources/robots/wheel_leg_mujoco/scripts/vmc_verification.py \
    --config src/bringup/config/cod_balance_controllers.yaml

# 方法2：如果当前在 scripts 目录，使用相对路径
cd src/resources/robots/wheel_leg_mujoco/scripts
python3 vmc_verification.py \
    --config ../../../../bringup/config/cod_balance_controllers.yaml

# 方法3：使用绝对路径
python3 /home/yons/code/learn_repo/wheel-legged-control/src/resources/robots/wheel_leg_mujoco/scripts/vmc_verification.py \
    --config /home/yons/code/learn_repo/wheel-legged-control/src/bringup/config/cod_balance_controllers.yaml
```

### 3. 脚本参数

```bash
python3 src/resources/robots/wheel_leg_mujoco/scripts/vmc_verification.py --help
```

主要参数：
- `--config`: 配置文件路径（YAML格式，包含 vmc_controller 配置）
- `--print-frequency`: 打印频率（Hz），默认 10Hz
- `--auto-calibrate`: 启用自动校准模式
- `--balance-threshold`: 平衡状态检测的 pitch 角度阈值（弧度），默认 0.05 rad
- `--balance-stable-time`: 平衡状态需要保持的稳定时间（秒），默认 2.0 秒

### 4. 自动校准模式

如果需要自动校准偏移值（**从项目根目录运行**）：

```bash
# 从项目根目录
cd /home/yons/code/learn_repo/wheel-legged-control
python3 src/resources/robots/wheel_leg_mujoco/scripts/vmc_verification.py \
    --config src/bringup/config/cod_balance_controllers.yaml \
    --auto-calibrate \
    --balance-threshold 0.05 \
    --balance-stable-time 2.0
```

## 关节映射关系

脚本会自动检测机器人类型并应用正确的映射：

### COD-2026RoboMaster-Balance 映射

根据 `vmc_controller.cpp` 的实现：

- **左腿**：
  - `phi1 = π + Left_rear_joint` (左后关节)
  - `phi4 = Left_front_joint` (左前关节，不加π)

- **右腿**：
  - `phi1 = π + Right_front_joint` (右前关节)
  - `phi4 = Right_rear_joint` (右后关节，不加π)

## 输出说明

脚本会定期打印以下信息：

1. **输入数据**：
   - 连杆长度（l1, l2, l3, l4, l5）
   - 关节位置（原始值和应用偏移后的值）
   - Pitch 角度和角速度
   - 力命令（F0, Tp）

2. **中间计算值**：
   - L0（腿长）
   - phi0（腿的角度）
   - theta（末端角度）
   - d_theta（末端角速度）
   - 雅可比矩阵系数（j11, j12, j21, j22）

3. **最终力矩计算结果**：
   - 四个关节的力矩值
   - 力矩限制检查

4. **错误和警告**：
   - 数值有效性检查
   - 力矩限制警告

## 配置文件格式

配置文件应该是 YAML 格式，包含 `vmc_controller` 部分：

```yaml
vmc_controller:
  ros__parameters:
    # 关节名称配置
    left_front_joint_name: "Left_front_joint"
    left_rear_joint_name: "Left_rear_joint"
    right_front_joint_name: "Right_front_joint"
    right_rear_joint_name: "Right_rear_joint"
    
    # VMC杆长参数 (单位: m)
    l1: 0.255184
    l2: 0.216280
    l3: 0.255184
    l4: 0.216280
    l5: 0.0
    
    # 关节初始角度偏置 (单位: rad)
    left_front_joint_offset: 0.15
    left_rear_joint_offset: 0.0
    right_front_joint_offset: -0.112
    right_rear_joint_offset: 0.0
    
    # 其他参数...
    max_torque: 90.0
    imu_topic: "/imu/data"
    force_command_topic: "/vmc_controller/force_command"
```

## 故障排除

### 问题1：缺少关节状态数据

**症状**：脚本显示 "等待关节状态数据..."

**解决**：
- 确保仿真已启动
- 检查 `/joint_states` 话题是否发布数据：
  ```bash
  ros2 topic echo /joint_states
  ```

### 问题2：缺少 IMU 数据

**症状**：脚本显示 "未接收到IMU数据，使用默认值 pitch=0"

**解决**：
- 检查 `/imu/data` 话题是否发布数据：
  ```bash
  ros2 topic echo /imu/data
  ```
- 如果不需要 IMU，这是正常的（脚本会使用 pitch=0）

### 问题3：计算值异常

**症状**：出现 NaN、Inf 或过大的值

**可能原因**：
- 关节位置超出合理范围
- 杆长参数配置错误
- 偏移值配置错误

**解决**：
- 检查配置文件中的杆长参数
- 检查关节偏移值
- 查看详细的中间计算值以定位问题

## 注意事项

1. **关节映射**：脚本会自动检测机器人类型（通过关节名称），并应用正确的映射关系。

2. **坐标系一致性**：确保配置文件中的参数与控制器中的参数一致。

3. **实时性**：脚本以 10Hz 频率打印详细信息，不会影响控制器的实时性能。

4. **自动校准**：自动校准模式需要机器人处于平衡状态，且保持稳定一段时间。


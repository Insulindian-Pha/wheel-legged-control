# VMC控制器电机零点位置校准指南

## 概述

VMC控制器使用关节角度偏置（`joint_offset`）来定义电机的零点位置。这些偏置值用于将机器人处于期望初始位置时的关节角度转换为控制器内部的零角度。

## 工作原理

在 `vmc_controller.cpp` 的 `update()` 函数中（第272-275行），关节偏移值被**加**到原始关节位置：

```cpp
double left_front_pos = left_front_pos_raw + left_front_joint_offset_;
double left_rear_pos = left_rear_pos_raw + left_rear_joint_offset_;
double right_front_pos = right_front_pos_raw + right_front_joint_offset_;
double right_rear_pos = right_rear_pos_raw + right_rear_joint_offset_;
```

然后这些位置被用于VMC计算（第278-281行）：

```cpp
left_leg_.setPhi1(M_PI / 2.0f + left_front_pos);
left_leg_.setPhi4(M_PI / 2.0f + left_rear_pos);
right_leg_.setPhi1(M_PI / 2.0f + right_front_pos);
right_leg_.setPhi4(M_PI / 2.0f + right_rear_pos);
```

## 确定零点位置的步骤

### 方法1：使用校准脚本（推荐）

使用提供的 `read_joint_positions.py` 脚本：

1. **将机器人放置在期望的初始位置（零点位置）**
   - 确保机器人处于平衡状态或期望的初始姿态

2. **获取关节位置值**
   ```bash
   # 方法A：从ROS2话题读取
   ros2 topic echo /joint_states --once
   
   # 方法B：从控制器日志中查看
   # 方法C：从其他工具或传感器读取
   ```

3. **运行校准脚本**
   ```bash
   python3 src/controller/vmc_controller/scripts/read_joint_positions.py
   ```

4. **按提示输入关节位置**
   - 脚本会提示选择输入单位（弧度或度）
   - 依次输入四个关节的位置值：
     - 左前关节 (Left_front_joint)
     - 左后关节 (Left_rear_joint)
     - 右前关节 (Right_front_joint)
     - 右后关节 (Right_rear_joint)

5. **复制生成的配置**
   - 脚本会自动计算偏移值并生成YAML配置格式
   - 将输出复制到 `cod_vmc_config.yaml` 文件中

### 方法2：手动计算

1. **将机器人放置在期望的初始位置（零点位置）**
   - 确保机器人处于平衡状态或期望的初始姿态
   - 记录此时各关节的物理位置

2. **读取当前关节角度**
   ```bash
   # 使用ros2 topic echo
   ros2 topic echo /joint_states --once
   ```

3. **计算偏移值**
   - 假设读取到的关节角度为 `pos_raw`
   - 要使控制器认为此时角度为0，需要：`pos = pos_raw + offset = 0`
   - 因此：`offset = -pos_raw`
   
   例如：
   - 如果 `left_front_pos_raw = 1.4` rad
   - 则 `left_front_joint_offset = -1.4` rad
   - 这样 `left_front_pos = 1.4 + (-1.4) = 0.0`

4. **更新配置文件**
   - 编辑 `cod_vmc_config.yaml` 或相应的配置文件
   - 将计算得到的偏移值填入对应的参数

### 方法3：通过日志输出确定

1. **临时修改代码添加调试输出**
   - 在 `vmc_controller.cpp` 的 `update()` 函数中添加：
   ```cpp
   RCLCPP_INFO(get_node()->get_logger(), 
               "Raw positions: LF=%.3f, LR=%.3f, RF=%.3f, RR=%.3f (rad)",
               left_front_pos_raw, left_rear_pos_raw, 
               right_front_pos_raw, right_rear_pos_raw);
   ```

2. **启动控制器并观察日志**
   - 记录机器人处于初始位置时的原始角度值
   - 计算偏移值：`offset = -pos_raw`

## 注意事项

1. **符号问题**：
   - 代码中使用的是**加法**：`pos = pos_raw + offset`
   - 配置文件中的注释说"减去"是错误的，实际是**加上**偏移值

2. **VMC算法的坐标系**：
   - 在设置VMC角度时，还会加上 `M_PI/2`（90度）
   - 这是VMC算法的坐标系定义，不需要在偏移值中考虑

3. **左右对称性**：
   - 通常左腿和右腿的偏移值符号相反
   - 例如：`left_front_joint_offset = 1.4`，`right_front_joint_offset = -1.4`

4. **单位**：
   - 所有角度值使用**弧度（rad）**，不是度（degree）
   - 1度 = π/180 ≈ 0.01745 弧度

5. **验证**：
   - 设置偏移值后，启动控制器
   - 检查日志输出（第116-117行会打印偏移值）
   - 确认机器人处于初始位置时，控制器计算的关节角度接近0

## 当前配置示例

根据 `cod_vmc_config.yaml`：

```yaml
left_front_joint_offset: 1.4   # 左前关节初始角度偏置
left_rear_joint_offset: 1.6    # 左后关节初始角度偏置
right_front_joint_offset: -1.4  # 右前关节初始角度偏置
right_rear_joint_offset: -1.6  # 右后关节初始角度偏置
```

这意味着：
- 当机器人处于初始位置时，如果原始关节角度为：
  - `left_front_pos_raw = -1.4` rad → `left_front_pos = -1.4 + 1.4 = 0.0`
  - `left_rear_pos_raw = -1.6` rad → `left_rear_pos = -1.6 + 1.6 = 0.0`
  - `right_front_pos_raw = 1.4` rad → `right_front_pos = 1.4 + (-1.4) = 0.0`
  - `right_rear_pos_raw = 1.6` rad → `right_rear_pos = 1.6 + (-1.6) = 0.0`

## 故障排除

如果控制器行为异常：

1. **检查偏移值符号**：确保偏移值与原始角度符号相反
2. **检查单位**：确保使用弧度而不是度
3. **检查关节名称**：确保配置文件中的关节名称与实际硬件匹配
4. **查看日志**：控制器启动时会打印所有偏移值（第116-117行）


std_msgs/Header header

# Debug flags / 调试标志
bool obs_history_filled    # 5 步观测历史是否已填满
bool received_vmc_state    # 是否已收到 VMC 状态
bool received_imu         # 是否已收到 IMU 数据

# Observation (body frame) / 观测（机体坐标系）：投影重力、角速度
float64 proj_grav_x       # 投影重力 x
float64 proj_grav_y       # 投影重力 y
float64 proj_grav_z       # 投影重力 z
float64 ang_vel_x         # 角速度 x (rad/s)
float64 ang_vel_y         # 角速度 y (rad/s)
float64 ang_vel_z         # 角速度 z (rad/s)

# Current VMC state (from vmc_state topic) / 当前 VMC 状态（来自 vmc_state 话题）
float64 left_theta        # 左腿 theta (rad)
float64 left_d_theta      # 左腿 theta 角速度 (rad/s)
float64 left_l0           # 左腿 L0 长度 (m)
float64 right_theta       # 右腿 theta (rad)
float64 right_d_theta     # 右腿 theta 角速度 (rad/s)
float64 right_l0          # 右腿 L0 长度 (m)

# Policy output (after clip) / 策略输出（裁剪后）：原始动作 [0..5]
float64 raw0              # 左腿 theta 动作
float64 raw1              # 左腿 L0 动作
float64 raw2              # 左轮速度动作
float64 raw3              # 右腿 theta 动作
float64 raw4              # 右腿 L0 动作
float64 raw5              # 右轮速度动作

# Unscaled reference sent to VMC / 发送给 VMC 的未缩放参考
float64 theta_l_ref       # 左腿 theta 参考 (rad)
float64 l0_l_ref          # 左腿 L0 参考 (m)
float64 theta_r_ref       # 右腿 theta 参考 (rad)
float64 l0_r_ref          # 右腿 L0 参考 (m)

# Wheel: policy velocity command and actual, torque / 轮子：策略速度指令与实际值、力矩
float64 vel_l_ref         # 左轮速度参考 (m/s)
float64 vel_r_ref         # 右轮速度参考 (m/s)
float64 left_wheel_velocity   # 左轮实际角速度 (rad/s)
float64 right_wheel_velocity # 右轮实际角速度 (rad/s)
float64 torque_l         # 左轮力矩 (Nm)
float64 torque_r         # 右轮力矩 (Nm)

# Command input (from control_input) / 控制输入（来自 control_input）
float64 lin_vel_cmd_x     # 线速度指令 x (m/s)
float64 ang_vel_cmd_z     # 航向角速度指令 (rad/s)

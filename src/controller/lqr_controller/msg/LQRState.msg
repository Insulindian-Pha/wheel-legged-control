std_msgs/Header header

# LQR输出量
float64 left_wheel_torque    # 左轮毂电机力矩 (Nm)
float64 right_wheel_torque   # 右轮毂电机力矩 (Nm)
float64 left_tp              # 左腿Tp (Nm)
float64 right_tp             # 右腿Tp (Nm)

# 状态值（用于LQR计算）
float64 left_theta           # 左腿theta (rad)
float64 left_d_theta         # 左腿d_theta (rad/s)
float64 right_theta          # 右腿theta (rad)
float64 right_d_theta        # 右腿d_theta (rad/s)
float64 pitch                # 俯仰角 (rad)
float64 pitch_gyro           # 俯仰角速度 (rad/s)

# 状态估计值
float64 x_position           # 当前位置 (m) - s
float64 x_velocity           # 当前速度 (m/s) - dot_s
float64 yaw                  # 航向角 (rad) - fai
float64 yaw_rate             # 航向角速度 (rad/s) - dot_fai
float64 x_set                # 期望位置 (m)
float64 v_set                # 期望速度 (m/s)
float64 x_err                # 位置误差 (m)
float64 v_err                # 速度误差 (m/s)

# 轮速
float64 left_wheel_velocity  # 左轮角速度 (rad/s)
float64 right_wheel_velocity # 右轮角速度 (rad/s)


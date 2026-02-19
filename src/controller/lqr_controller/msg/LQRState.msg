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
float64 yaw_gyro             # 航向角速度 (rad/s) - dot_fai
float64 x_set                # 期望位置 (m)
float64 v_set                # 期望速度 (m/s)
float64 x_err                # 位置误差 (m)
float64 v_err                # 速度误差 (m/s)
float64 fai_set              # 期望航向角 (rad)
float64 dot_fai_set           # 期望航向角速度 (rad/s)
float64 fai_err               # 航向角误差 (rad)
float64 dot_fai_err           # 航向角速度误差 (rad/s)

# 轮速
float64 left_wheel_velocity  # 左轮角速度 (rad/s)
float64 right_wheel_velocity # 右轮角速度 (rad/s)

# 状态量输出贡献（用于观察每个状态量对输出的影响）
# 每个状态量对 = 状态量 + d_状态量（应用增益后的组合输出）
# 左轮力矩输出贡献
float64 left_wheel_s_output           # 左轮：位置+速度状态输出贡献 (s*gain[0] + dot_s*gain[1])
float64 left_wheel_fai_output         # 左轮：航向状态输出贡献 (fai*gain[2] + dot_fai*gain[3])
float64 left_wheel_theta_ll_output   # 左轮：左腿状态输出贡献 (theta_ll*gain[4] + dot_theta_ll*gain[5])
float64 left_wheel_theta_lr_output    # 左轮：右腿状态输出贡献 (theta_lr*gain[6] + dot_theta_lr*gain[7])
float64 left_wheel_theta_b_output     # 左轮：身体状态输出贡献 (theta_b*gain[8] + dot_theta_b*gain[9])

# 右轮力矩输出贡献
float64 right_wheel_s_output          # 右轮：位置+速度状态输出贡献 (s*gain[0] + dot_s*gain[1])
float64 right_wheel_fai_output        # 右轮：航向状态输出贡献 (fai*gain[2] + dot_fai*gain[3])
float64 right_wheel_theta_ll_output   # 右轮：左腿状态输出贡献 (theta_ll*gain[4] + dot_theta_ll*gain[5])
float64 right_wheel_theta_lr_output    # 右轮：右腿状态输出贡献 (theta_lr*gain[6] + dot_theta_lr*gain[7])
float64 right_wheel_theta_b_output     # 右轮：身体状态输出贡献 (theta_b*gain[8] + dot_theta_b*gain[9])

# 左腿Tp输出贡献
float64 left_leg_s_output              # 左腿：位置+速度状态输出贡献 (s*gain[0] + dot_s*gain[1])
float64 left_leg_fai_output            # 左腿：航向状态输出贡献 (fai*gain[2] + dot_fai*gain[3])
float64 left_leg_theta_ll_output       # 左腿：左腿状态输出贡献 (theta_ll*gain[4] + dot_theta_ll*gain[5])
float64 left_leg_theta_lr_output       # 左腿：右腿状态输出贡献 (theta_lr*gain[6] + dot_theta_lr*gain[7])
float64 left_leg_theta_b_output        # 左腿：身体状态输出贡献 (theta_b*gain[8] + dot_theta_b*gain[9])

# 右腿Tp输出贡献
float64 right_leg_s_output             # 右腿：位置+速度状态输出贡献 (s*gain[0] + dot_s*gain[1])
float64 right_leg_fai_output           # 右腿：航向状态输出贡献 (fai*gain[2] + dot_fai*gain[3])
float64 right_leg_theta_ll_output      # 右腿：左腿状态输出贡献 (theta_ll*gain[4] + dot_theta_ll*gain[5])
float64 right_leg_theta_lr_output     # 右腿：右腿状态输出贡献 (theta_lr*gain[6] + dot_theta_lr*gain[7])
float64 right_leg_theta_b_output       # 右腿：身体状态输出贡献 (theta_b*gain[8] + dot_theta_b*gain[9])


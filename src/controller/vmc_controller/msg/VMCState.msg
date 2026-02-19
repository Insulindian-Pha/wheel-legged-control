std_msgs/Header header

# 左腿VMC状态
float64 left_theta          # theta角度 (rad)
float64 left_d_theta         # theta角速度 (rad/s)
float64 left_l0              # 腿长L0 (m)
float64 left_f0              # 虚拟力F0 (N)
float64 left_tp              # 虚拟力矩Tp (Nm)
float64 left_torque_front    # 前关节力矩 (Nm)
float64 left_torque_rear     # 后关节力矩 (Nm)
float64 left_j11             # 左腿 Jacobian j11
float64 left_j12             # 左腿 Jacobian j12
float64 left_j21             # 左腿 Jacobian j21
float64 left_j22             # 左腿 Jacobian j22

# 右腿VMC状态
float64 right_theta          # theta角度 (rad)
float64 right_d_theta        # theta角速度 (rad/s)
float64 right_l0             # 腿长L0 (m)
float64 right_f0              # 虚拟力F0 (N)
float64 right_tp              # 虚拟力矩Tp (Nm)
float64 right_torque_front    # 前关节力矩 (Nm)
float64 right_torque_rear     # 后关节力矩 (Nm)
float64 right_j11             # 右腿 Jacobian j11
float64 right_j12             # 右腿 Jacobian j12
float64 right_j21             # 右腿 Jacobian j21
float64 right_j22             # 右腿 Jacobian j22

# IMU相关数据
float64 pitch                 # 俯仰角 (rad)
float64 pitch_gyro            # 俯仰角速度 (rad/s)


std_msgs/Header header

# 期望值 / 反馈值 / 误差值
float64 target
float64 feedback
float64 error

# PID增益
float64 kp
float64 ki
float64 kd

# 各项输出
float64 p_out
float64 i_out
float64 d_out

# 总输出
float64 out


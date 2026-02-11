# LQR控制器状态量更新说明

## 更新概述

已将LQR控制器从6状态量模型更新为10状态量模型,以匹配Python脚本(`lagrange.py`)计算的K矩阵格式。

## 状态向量定义

### 新的10状态量 (与Python脚本一致):

| 索引 | 状态名称     | 描述            | C++变量          |
| ---- | ------------ | --------------- | ---------------- |
| 0    | s            | 位移            | `x_position_`    |
| 1    | dot_s        | 速度            | `x_velocity_`    |
| 2    | fai          | 航向角          | `yaw_`           |
| 3    | dot_fai      | 航向角速度      | `yaw_rate_`      |
| 4    | theta_ll     | 左腿角度        | `left_theta_`    |
| 5    | dot_theta_ll | 左腿角速度      | `left_d_theta_`  |
| 6    | theta_lr     | 右腿角度        | `right_theta_`   |
| 7    | dot_theta_lr | 右腿角速度      | `right_d_theta_` |
| 8    | theta_b      | 身体角度(pitch) | `pitch_`         |
| 9    | dot_theta_b  | 身体角速度      | `pitch_gyro_`    |

## K矩阵结构

K矩阵维度: **4×10**

### 4个控制输出:
1. **左轮力矩** (K[0:9]) - `left_wheel_lqr_gains_`
2. **右轮力矩** (K[10:19]) - `right_wheel_lqr_gains_`
3. **左腿力矩/Tp** (K[20:29]) - `left_leg_lqr_gains_`
4. **右腿力矩/Tp** (K[30:39]) - `right_leg_lqr_gains_`

## 代码更改

### 1. 头文件 (`lqr_controller.h`)

**旧结构:**
```cpp
std::array<double, 12> left_lqr_gains_;
std::array<double, 12> right_lqr_gains_;
std::array<double, 12> left_lqr_gain_polarity_;
std::array<double, 12> right_lqr_gain_polarity_;
```

**新结构:**
```cpp
std::array<double, 10> left_wheel_lqr_gains_;   // 左轮力矩增益
std::array<double, 10> right_wheel_lqr_gains_;  // 右轮力矩增益
std::array<double, 10> left_leg_lqr_gains_;     // 左腿力矩增益
std::array<double, 10> right_leg_lqr_gains_;    // 右腿力矩增益
```

**新增状态变量:**
```cpp
double yaw_;       // 航向角 (fai)
double yaw_rate_;  // 航向角速度 (dot_fai)
```

### 2. 实现文件 (`lqr_controller.cpp`)

#### 参数声明更新:
- 从12个参数/腿 改为 10个参数/输出
- 移除了gain_polarity参数
- 新增4组增益参数(每组10个)

#### LQR控制计算更新:
```cpp
// 左轮力矩 = K[0:9] * state[0:9]
wheel_torque_left = left_wheel_lqr_gains_[0] * x_position_ +
                    left_wheel_lqr_gains_[1] * x_velocity_ +
                    left_wheel_lqr_gains_[2] * yaw_ +
                    left_wheel_lqr_gains_[3] * yaw_rate_ +
                    left_wheel_lqr_gains_[4] * left_theta_ +
                    left_wheel_lqr_gains_[5] * left_d_theta_ +
                    left_wheel_lqr_gains_[6] * right_theta_ +
                    left_wheel_lqr_gains_[7] * right_d_theta_ +
                    left_wheel_lqr_gains_[8] * pitch_ +
                    left_wheel_lqr_gains_[9] * pitch_gyro_;
```

#### 状态估计更新:
- 新增yaw和yaw_rate的计算
- yaw_rate从左右轮速差计算: `(right_vel - left_vel) * wheel_radius / wheel_base_width`
- yaw通过积分yaw_rate得到

### 3. 配置文件 (`cod_lqr_vmc_config.yaml`)

**旧配置:**
- `left_lqr_gains`: 12个值
- `right_lqr_gains`: 12个值
- `left_lqr_gain_polarity`: 12个值
- `right_lqr_gain_polarity`: 12个值

**新配置:**
- `left_wheel_lqr_gains`: 10个值 (左轮力矩)
- `right_wheel_lqr_gains`: 10个值 (右轮力矩)
- `left_leg_lqr_gains`: 10个值 (左腿力矩/Tp)
- `right_leg_lqr_gains`: 10个值 (右腿力矩/Tp)

## 如何使用Python脚本输出更新配置

1. 运行Python脚本计算K值:
```bash
cd /home/yons/code/learn_repo/wheel-legged-control/src/controller/lqr_controller/scripts
./setup_and_run.sh
```

2. 复制终端输出的YAML格式K值

3. 直接粘贴到 `cod_lqr_vmc_config.yaml` 的对应位置

## 注意事项

1. **航向角计算**: 当前yaw_rate使用轮速差计算,假设轮距为0.3m,可能需要根据实际机器人参数调整

2. **状态量符号**: 所有状态量直接使用原始值,不再有极性反转

3. **K矩阵符号**: Python脚本输出的K值已经包含正确的符号,直接使用即可

4. **配置兼容性**: 旧的配置文件格式不再兼容,需要使用新格式

## 测试建议

1. 先使用Python脚本的默认参数测试
2. 观察机器人响应,特别是航向控制
3. 如果航向控制不稳定,可能需要调整yaw_rate计算中的轮距参数
4. 可以通过发布的`/lqr_controller/lqr_state`话题监控所有状态量

## 文件清单

修改的文件:
- `src/controller/lqr_controller/include/lqr_controller/lqr_controller.h`
- `src/controller/lqr_controller/src/lqr_controller.cpp`
- `src/bringup/config/cod_lqr_vmc_config.yaml`

参考文件:
- `src/controller/lqr_controller/scripts/lagrange.py` (K矩阵计算)
- `src/controller/lqr_controller/scripts/setup_and_run.sh` (运行脚本)



#ifndef VMC_CALC_H
#define VMC_CALC_H

#include <cstdint>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace vmc_controller
{

class VMCLeg
{
public:
  // 构造函数
  VMCLeg();

  // 初始化杆长参数
  void init_leg_length(double l1, double l2, double l3, double l4, double l5);

  // 计算theta和d_theta给lqr用，同时也计算腿长L0
  void calc1Right(double pitch, double pitch_gyro, double dt);
  void calc1Left(double pitch, double pitch_gyro, double dt);

  // 计算期望的关节输出力矩
  void calc2();

  // 离地检测（可选功能）
  bool groundDetectionRight();
  bool groundDetectionLeft();

  // Setter方法 - 设置输入参数
  void setPhi1(double phi1);
  void setPhi4(double phi4);
  void setF0(double F0);
  void setTp(double Tp);

  // Getter方法 - 获取计算结果和状态
  double getTorqueFront() const;
  double getTorqueRear() const;
  double getTheta() const;
  double getDTheta() const;
  double getL0() const;
  double getPhi0() const;
  double getJ11() const;
  double getJ12() const;
  double getJ21() const;
  double getJ22() const;

private:
  // 左右两腿的公共参数，固定不变
  double l5_;  // AE长度，单位为m
  double l1_;  // 单位为m
  double l2_;  // 单位为m
  double l3_;  // 单位为m
  double l4_;  // 单位为m

  // 关节角度（输入）
  double phi1_;
  double phi4_;

  // 力和力矩（输入）
  double F0_;
  double Tp_;

  // 计算结果（输出）
  double torque_set_[2];

  // 状态变量（可用于外部读取）
  double theta_;
  double d_theta_;  // theta的一阶导数
  double L0_;       // 腿长

  static constexpr double PI = M_PI;

  double XB, YB;  // B点的坐标
  double XD, YD;  // D点的坐标

  double XC, YC;  // C点的直角坐标
  double phi0;    // C点的极坐标角度
  double alpha;
  double d_alpha;

  double lBD;  // BD两点的距离

  double d_phi0;     // 现在C点角度phi0的变换率
  double last_phi0;  // 上一次C点角度，用于计算角度phi0的变换率d_phi0

  double A0, B0, C0;  // 中间变量
  double phi2, phi3;

  double j11, j12, j21, j22;  // 笛卡尔空间力到关节空间的力的雅可比矩阵系数

  double last_d_theta;
  double dd_theta;  // theta的二阶导数

  double d_L0;   // L0的一阶导数
  double dd_L0;  // L0的二阶导数
  double last_L0;
  double last_d_L0;

  // 闭链求解内部状态（被动关节 warm-start）
  double passive_phi2_;
  double passive_phi3_;

  double FN;  // 支持力

  uint8_t first_flag;
  uint8_t leg_flag;  // 腿长完成标志

  void calcKinematicsCommon(bool is_right_leg, double pitch, double pitch_gyro, double dt);
  bool calcKinematicsPinocchio(double& solved_phi2, double& solved_phi3, double& out_l0, double& out_phi0,
                               double& out_d_l0_d_phi1, double& out_d_l0_d_phi4, double& out_d_phi0_d_phi1,
                               double& out_d_phi0_d_phi4);
};

}  // namespace vmc_controller

#endif  // VMC_CALC_H

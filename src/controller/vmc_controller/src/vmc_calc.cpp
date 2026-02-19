#include "vmc_controller/vmc_calc.h"
#include <cmath>

namespace vmc_controller {

VMCLeg::VMCLeg() : first_flag(0), leg_flag(0) {
  // 初始化所有变量为0
  l5_ = l1_ = l2_ = l3_ = l4_ = 0.0;
  XB = YB = XD = YD = 0.0;
  XC = YC = L0_ = phi0 = 0.0;
  alpha = d_alpha = 0.0;
  lBD = 0.0;
  d_phi0 = last_phi0 = 0.0;
  A0 = B0 = C0 = 0.0;
  phi2 = phi3 = phi1_ = phi4_ = 0.0;
  j11 = j12 = j21 = j22 = 0.0;
  torque_set_[0] = torque_set_[1] = 0.0;
  F0_ = Tp_ = 0.0;
  theta_ = d_theta_ = last_d_theta = dd_theta = 0.0;
  d_L0 = dd_L0 = last_L0 = last_d_L0 = 0.0;
  FN = 0.0;
}

void VMCLeg::calc1Right(double pitch, double pitch_gyro, double dt) {
  double PitchR = -pitch;
  double PitchGyroR = -pitch_gyro;

  YD = l4_ * sin(phi4_);       // D的y坐标
  YB = l1_ * sin(phi1_);       // B的y坐标
  XD = l5_ + l4_ * cos(phi4_); // D的x坐标
  XB = l1_ * cos(phi1_);       // B的x坐标

  lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));

  A0 = 2 * l2_ * (XD - XB);
  B0 = 2 * l2_ * (YD - YB);
  C0 = l2_ * l2_ + lBD * lBD - l3_ * l3_;
  phi2 = 2 * atan2((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), A0 + C0);
  phi3 = atan2(YB - YD + l2_ * sin(phi2), XB - XD + l2_ * cos(phi2));
  // C点直角坐标
  XC = l1_ * cos(phi1_) + l2_ * cos(phi2);
  YC = l1_ * sin(phi1_) + l2_ * sin(phi2);
  // C点极坐标
  L0_ = sqrt((XC - l5_ / 2.0) * (XC - l5_ / 2.0) + YC * YC);

  phi0 = atan2(YC, (XC - l5_ / 2.0)); // phi0用于计算lqr需要的theta
  alpha = PI / 2.0 - phi0;

  if (first_flag == 0) {
    last_phi0 = phi0;
    first_flag = 1;
  }
  d_phi0 =
      (phi0 - last_phi0) / dt; // 计算phi0变化率，d_phi0用于计算lqr需要的d_theta
  d_alpha = 0.0 - d_phi0;

  theta_ = PI / 2.0 - PitchR - phi0; // 得到状态变量1
  d_theta_ = (-PitchGyroR - d_phi0); // 得到状态变量2

  last_phi0 = phi0;

  d_L0 = (L0_ - last_L0) / dt;     // 腿长L0的一阶导数
  dd_L0 = (d_L0 - last_d_L0) / dt; // 腿长L0的二阶导数

  last_d_L0 = d_L0;
  last_L0 = L0_;

  dd_theta = (d_theta_ - last_d_theta) / dt;
  last_d_theta = d_theta_;
}

void VMCLeg::calc1Left(double pitch, double pitch_gyro, double dt) {
  double PitchL = pitch;
  double PitchGyroL = pitch_gyro;

  YD = l4_ * sin(phi4_);       // D的y坐标
  YB = l1_ * sin(phi1_);       // B的y坐标
  XD = l5_ + l4_ * cos(phi4_); // D的x坐标
  XB = l1_ * cos(phi1_);       // B的x坐标

  lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));

  A0 = 2 * l2_ * (XD - XB);
  B0 = 2 * l2_ * (YD - YB);
  C0 = l2_ * l2_ + lBD * lBD - l3_ * l3_;
  phi2 = 2 * atan2((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), A0 + C0);
  phi3 = atan2(YB - YD + l2_ * sin(phi2), XB - XD + l2_ * cos(phi2));
  // C点直角坐标
  XC = l1_ * cos(phi1_) + l2_ * cos(phi2);
  YC = l1_ * sin(phi1_) + l2_ * sin(phi2);
  // C点极坐标
  L0_ = sqrt((XC - l5_ / 2.0) * (XC - l5_ / 2.0) + YC * YC);

  phi0 = atan2(YC, (XC - l5_ / 2.0)); // phi0用于计算lqr需要的theta
  alpha = PI / 2.0 - phi0;

  if (first_flag == 0) {
    last_phi0 = phi0;
    first_flag = 1;
  }
  d_phi0 =
      (phi0 - last_phi0) / dt; // 计算phi0变化率，d_phi0用于计算lqr需要的d_theta
  d_alpha = 0.0 - d_phi0;

  theta_ = PI / 2.0 - PitchL - phi0; // 得到状态变量1
  d_theta_ = (-PitchGyroL - d_phi0); // 得到状态变量2

  last_phi0 = phi0;

  d_L0 = (L0_ - last_L0) / dt;     // 腿长L0的一阶导数
  dd_L0 = (d_L0 - last_d_L0) / dt; // 腿长L0的二阶导数

  last_d_L0 = d_L0;
  last_L0 = L0_;

  dd_theta = (d_theta_ - last_d_theta) / dt;
  last_d_theta = d_theta_;
}

void VMCLeg::calc2() {
  j11 = (l1_ * sin(phi0 - phi3) * sin(phi1_ - phi2)) / sin(phi3 - phi2);
  j12 = (l1_ * cos(phi0 - phi3) * sin(phi1_ - phi2)) / (L0_ * sin(phi3 - phi2));
  j21 = (l4_ * sin(phi0 - phi2) * sin(phi3 - phi4_)) / sin(phi3 - phi2);
  j22 = (l4_ * cos(phi0 - phi2) * sin(phi3 - phi4_)) / (L0_ * sin(phi3 - phi2));

  // 得到输出轴期望力矩，F0为五连杆机构末端沿腿的推力
  torque_set_[0] = j11 * F0_ + j12 * Tp_; // Front关节力矩
  // Tp为沿中心轴的力矩
  torque_set_[1] = j21 * F0_ + j22 * Tp_; // Rear关节力矩
}

bool VMCLeg::groundDetectionRight() {
  // 腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度
  FN = F0_ * cos(theta_) + Tp_ * sin(theta_) / L0_ + 6.0;

  if (FN < 5.0) { // 离地了
    return true;
  } else {
    return false;
  }
}

bool VMCLeg::groundDetectionLeft() {
  // 腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度
  FN = F0_ * cos(theta_) + Tp_ * sin(theta_) / L0_ + 6.0;

  if (FN < 5.0) { // 离地了
    return true;
  } else {
    return false;
  }
}

// Setter方法
void VMCLeg::init_leg_length(double l1, double l2, double l3, double l4,
                             double l5) {
  l1_ = l1;
  l2_ = l2;
  l3_ = l3;
  l4_ = l4;
  l5_ = l5;
}
void VMCLeg::setPhi1(double phi1) { phi1_ = phi1; }

void VMCLeg::setPhi4(double phi4) { phi4_ = phi4; }

void VMCLeg::setF0(double F0) { F0_ = F0; }

void VMCLeg::setTp(double Tp) { Tp_ = Tp; }

// Getter方法
double VMCLeg::getTorqueFront() const { return torque_set_[0]; }

double VMCLeg::getTorqueRear() const { return torque_set_[1]; }

double VMCLeg::getTheta() const { return theta_; }

double VMCLeg::getDTheta() const { return d_theta_; }

double VMCLeg::getL0() const { return L0_; }

double VMCLeg::getPhi0() const { return phi0; }

double VMCLeg::getJ11() const { return j11; }

double VMCLeg::getJ12() const { return j12; }

double VMCLeg::getJ21() const { return j21; }

double VMCLeg::getJ22() const { return j22; }

} // namespace vmc_controller

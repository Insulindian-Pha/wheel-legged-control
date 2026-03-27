#include "vmc_controller/vmc_calc.h"
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <memory>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>

namespace vmc_controller
{

namespace
{
class FiveBarPinocchioSolver
{
public:
  FiveBarPinocchioSolver(double l1, double l2, double l3, double l4, double l5)
  : l5_(l5)
  {
    using namespace pinocchio;
    model_ = Model();

    const JointIndex root = 0;
    const Inertia inert = Inertia::Zero();
    const SE3 left_joint_placement = SE3::Identity();
    const JointIndex j1 = model_.addJoint(root, JointModelRZ(), left_joint_placement, "phi1_joint");
    model_.appendBodyToJoint(j1, inert, SE3::Identity());

    const JointIndex j2 = model_.addJoint(j1, JointModelRZ(), SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(l1, 0.0, 0.0)),
                                          "phi2_joint");
    model_.appendBodyToJoint(j2, inert, SE3::Identity());

    const SE3 right_base_placement(Eigen::Matrix3d::Identity(), Eigen::Vector3d(l5, 0.0, 0.0));
    const JointIndex j4 = model_.addJoint(root, JointModelRZ(), right_base_placement, "phi4_joint");
    model_.appendBodyToJoint(j4, inert, SE3::Identity());

    const JointIndex j3 = model_.addJoint(j4, JointModelRZ(), SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(l4, 0.0, 0.0)),
                                          "phi3_joint");
    model_.appendBodyToJoint(j3, inert, SE3::Identity());

    left_tip_frame_id_ = model_.addFrame(Frame("left_tip", j2, j2, SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(l2, 0.0, 0.0)),
                                               OP_FRAME));
    right_tip_frame_id_ = model_.addFrame(Frame("right_tip", j3, j3, SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(l3, 0.0, 0.0)),
                                                OP_FRAME));

    data_ = pinocchio::Data(model_);
    q_.setZero(model_.nq);
  }

  bool solve(double phi1, double phi4, double& phi2_guess, double& phi3_guess, Eigen::Vector2d& c_point,
             Eigen::Matrix2d& d_c_d_qa, Eigen::Matrix2d& d_task_d_qa, double& l0, double& phi0)
  {
    q_[0] = phi1;
    q_[2] = phi4;
    q_[1] = phi2_guess;
    q_[3] = phi3_guess;

    constexpr int kMaxIter = 12;
    constexpr double kTol = 1e-8;
    constexpr double kDamping = 1e-9;
    bool converged = false;

    for (int i = 0; i < kMaxIter; ++i)
    {
      pinocchio::forwardKinematics(model_, data_, q_);
      pinocchio::updateFramePlacements(model_, data_);

      const Eigen::Vector3d p_l = data_.oMf[left_tip_frame_id_].translation();
      const Eigen::Vector3d p_r = data_.oMf[right_tip_frame_id_].translation();
      const Eigen::Vector2d r = p_l.head<2>() - p_r.head<2>();
      if (r.norm() < kTol)
      {
        converged = true;
        break;
      }

      Eigen::Matrix<double, 6, Eigen::Dynamic> J_l(6, model_.nv);
      Eigen::Matrix<double, 6, Eigen::Dynamic> J_r(6, model_.nv);
      pinocchio::computeFrameJacobian(model_, data_, q_, left_tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_l);
      pinocchio::computeFrameJacobian(model_, data_, q_, right_tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_r);
      const Eigen::Matrix<double, 2, 4> Jc = (J_l.topRows<2>() - J_r.topRows<2>());

      Eigen::Matrix2d Jp;
      Jp.col(0) = Jc.col(1);
      Jp.col(1) = Jc.col(3);

      Eigen::Matrix2d lhs = Jp.transpose() * Jp + kDamping * Eigen::Matrix2d::Identity();
      Eigen::Vector2d rhs = -Jp.transpose() * r;
      const Eigen::Vector2d delta = lhs.ldlt().solve(rhs);

      q_[1] += delta[0];
      q_[3] += delta[1];
      if (!std::isfinite(q_[1]) || !std::isfinite(q_[3]))
      {
        return false;
      }
    }

    if (!converged)
    {
      return false;
    }

    phi2_guess = q_[1];
    phi3_guess = q_[3];

    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);
    Eigen::Matrix<double, 6, Eigen::Dynamic> J_l(6, model_.nv);
    Eigen::Matrix<double, 6, Eigen::Dynamic> J_r(6, model_.nv);
    pinocchio::computeFrameJacobian(model_, data_, q_, left_tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_l);
    pinocchio::computeFrameJacobian(model_, data_, q_, right_tip_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_r);

    const Eigen::Vector2d p_l = data_.oMf[left_tip_frame_id_].translation().head<2>();
    const Eigen::Vector2d p_r = data_.oMf[right_tip_frame_id_].translation().head<2>();
    c_point = 0.5 * (p_l + p_r);

    const Eigen::Matrix<double, 2, 4> J_constraint = (J_l.topRows<2>() - J_r.topRows<2>());
    const Eigen::Matrix<double, 2, 4> J_c = J_l.topRows<2>();
    Eigen::Matrix2d Jp_con;
    Jp_con.col(0) = J_constraint.col(1);
    Jp_con.col(1) = J_constraint.col(3);
    Eigen::Matrix2d Ja_con;
    Ja_con.col(0) = J_constraint.col(0);
    Ja_con.col(1) = J_constraint.col(2);

    Eigen::Matrix2d Jp_c;
    Jp_c.col(0) = J_c.col(1);
    Jp_c.col(1) = J_c.col(3);
    Eigen::Matrix2d Ja_c;
    Ja_c.col(0) = J_c.col(0);
    Ja_c.col(1) = J_c.col(2);

    const Eigen::Matrix2d d_qp_d_qa = -Jp_con.colPivHouseholderQr().solve(Ja_con);
    d_c_d_qa = Ja_c + Jp_c * d_qp_d_qa;

    const double dx = c_point[0] - l5_ / 2.0;
    const double dy = c_point[1];
    l0 = std::sqrt(dx * dx + dy * dy);
    phi0 = std::atan2(dy, dx);
    if (l0 < 1e-8)
    {
      return false;
    }

    const Eigen::RowVector2d grad_l(dx / l0, dy / l0);
    const Eigen::RowVector2d grad_phi(-dy / (l0 * l0), dx / (l0 * l0));
    d_task_d_qa.row(0) = grad_l * d_c_d_qa;
    d_task_d_qa.row(1) = grad_phi * d_c_d_qa;
    return true;
  }

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex left_tip_frame_id_;
  pinocchio::FrameIndex right_tip_frame_id_;
  Eigen::VectorXd q_;
  double l5_;
};

}  // namespace

VMCLeg::VMCLeg() : first_flag(0), leg_flag(0)
{
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
  passive_phi2_ = passive_phi3_ = 0.0;
  FN = 0.0;
}

bool VMCLeg::calcKinematicsPinocchio(double& solved_phi2, double& solved_phi3, double& out_l0, double& out_phi0,
                                     double& out_d_l0_d_phi1, double& out_d_l0_d_phi4, double& out_d_phi0_d_phi1,
                                     double& out_d_phi0_d_phi4)
{
  static thread_local std::unique_ptr<FiveBarPinocchioSolver> solver;
  static thread_local double last_l1 = std::numeric_limits<double>::quiet_NaN();
  static thread_local double last_l2 = std::numeric_limits<double>::quiet_NaN();
  static thread_local double last_l3 = std::numeric_limits<double>::quiet_NaN();
  static thread_local double last_l4 = std::numeric_limits<double>::quiet_NaN();
  static thread_local double last_l5 = std::numeric_limits<double>::quiet_NaN();
  if (!solver || std::abs(last_l1 - l1_) > 1e-12 || std::abs(last_l2 - l2_) > 1e-12 || std::abs(last_l3 - l3_) > 1e-12 ||
      std::abs(last_l4 - l4_) > 1e-12 || std::abs(last_l5 - l5_) > 1e-12)
  {
    solver = std::make_unique<FiveBarPinocchioSolver>(l1_, l2_, l3_, l4_, l5_);
    last_l1 = l1_;
    last_l2 = l2_;
    last_l3 = l3_;
    last_l4 = l4_;
    last_l5 = l5_;
  }
  Eigen::Vector2d c;
  Eigen::Matrix2d d_c_d_qa;
  Eigen::Matrix2d d_task_d_qa;
  bool ok = solver->solve(phi1_, phi4_, passive_phi2_, passive_phi3_, c, d_c_d_qa, d_task_d_qa, out_l0, out_phi0);
  if (!ok)
  {
    return false;
  }

  solved_phi2 = passive_phi2_;
  solved_phi3 = passive_phi3_;
  out_d_l0_d_phi1 = d_task_d_qa(0, 0);
  out_d_l0_d_phi4 = d_task_d_qa(0, 1);
  out_d_phi0_d_phi1 = d_task_d_qa(1, 0);
  out_d_phi0_d_phi4 = d_task_d_qa(1, 1);
  return std::isfinite(out_l0) && std::isfinite(out_phi0);
}

void VMCLeg::calcKinematicsCommon(bool is_right_leg, double pitch, double pitch_gyro, double dt)
{
  double solved_phi2 = passive_phi2_;
  double solved_phi3 = passive_phi3_;
  double solved_l0 = last_L0;
  double solved_phi0 = last_phi0;
  double d_l0_d_phi1 = std::numeric_limits<double>::quiet_NaN();
  double d_l0_d_phi4 = std::numeric_limits<double>::quiet_NaN();
  double d_phi0_d_phi1 = std::numeric_limits<double>::quiet_NaN();
  double d_phi0_d_phi4 = std::numeric_limits<double>::quiet_NaN();

  const bool pin_ok = calcKinematicsPinocchio(solved_phi2, solved_phi3, solved_l0, solved_phi0, d_l0_d_phi1,
                                               d_l0_d_phi4, d_phi0_d_phi1, d_phi0_d_phi4);
  if (!pin_ok)
  {
    solved_phi2 = passive_phi2_;
    solved_phi3 = passive_phi3_;
    solved_l0 = L0_;
    solved_phi0 = phi0;
    d_l0_d_phi1 = 0.0;
    d_l0_d_phi4 = 0.0;
    d_phi0_d_phi1 = 0.0;
    d_phi0_d_phi4 = 0.0;
  }

  phi2 = solved_phi2;
  phi3 = solved_phi3;
  L0_ = solved_l0;
  phi0 = solved_phi0;
  alpha = PI / 2.0 - phi0;

  if (first_flag == 0)
  {
    last_phi0 = phi0;
    first_flag = 1;
  }
  d_phi0 = (phi0 - last_phi0) / dt;  // 计算phi0变化率，d_phi0用于计算lqr需要的d_theta
  d_alpha = 0.0 - d_phi0;

  const double pitch_term = is_right_leg ? -pitch : pitch;
  const double pitch_gyro_term = is_right_leg ? -pitch_gyro : pitch_gyro;
  theta_ = PI / 2.0 - pitch_term - phi0;          // 得到状态变量1
  d_theta_ = (-pitch_gyro_term - d_phi0);         // 得到状态变量2

  last_phi0 = phi0;

  d_L0 = (L0_ - last_L0) / dt;      // 腿长L0的一阶导数
  dd_L0 = (d_L0 - last_d_L0) / dt;  // 腿长L0的二阶导数

  last_d_L0 = d_L0;
  last_L0 = L0_;

  dd_theta = (d_theta_ - last_d_theta) / dt;
  last_d_theta = d_theta_;

  // 保持和旧接口一致的系数语义：j11/j12对应第一执行器，j21/j22对应第二执行器
  j11 = d_l0_d_phi1;
  j12 = d_phi0_d_phi1;
  j21 = d_l0_d_phi4;
  j22 = d_phi0_d_phi4;
}

void VMCLeg::calc1Right(double pitch, double pitch_gyro, double dt)
{
  calcKinematicsCommon(true, pitch, pitch_gyro, dt);
}

void VMCLeg::calc1Left(double pitch, double pitch_gyro, double dt)
{
  calcKinematicsCommon(false, pitch, pitch_gyro, dt);
}

void VMCLeg::calc2()
{
  // 得到输出轴期望力矩，F0为五连杆机构末端沿腿的推力
  torque_set_[0] = j11 * F0_ + j12 * Tp_;  // Front关节力矩
  // Tp为沿中心轴的力矩
  torque_set_[1] = j21 * F0_ + j22 * Tp_;  // Rear关节力矩
}

bool VMCLeg::groundDetectionRight()
{
  // 腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度
  FN = F0_ * cos(theta_) + Tp_ * sin(theta_) / L0_ + 6.0;

  if (FN < 5.0)
  {  // 离地了
    return true;
  }
  else
  {
    return false;
  }
}

bool VMCLeg::groundDetectionLeft()
{
  // 腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度
  FN = F0_ * cos(theta_) + Tp_ * sin(theta_) / L0_ + 6.0;

  if (FN < 5.0)
  {  // 离地了
    return true;
  }
  else
  {
    return false;
  }
}

// Setter方法
void VMCLeg::init_leg_length(double l1, double l2, double l3, double l4, double l5)
{
  l1_ = l1;
  l2_ = l2;
  l3_ = l3;
  l4_ = l4;
  l5_ = l5;
}
void VMCLeg::setPhi1(double phi1)
{
  phi1_ = phi1;
}

void VMCLeg::setPhi4(double phi4)
{
  phi4_ = phi4;
}

void VMCLeg::setF0(double F0)
{
  F0_ = F0;
}

void VMCLeg::setTp(double Tp)
{
  Tp_ = Tp;
}

// Getter方法
double VMCLeg::getTorqueFront() const
{
  return torque_set_[0];
}

double VMCLeg::getTorqueRear() const
{
  return torque_set_[1];
}

double VMCLeg::getTheta() const
{
  return theta_;
}

double VMCLeg::getDTheta() const
{
  return d_theta_;
}

double VMCLeg::getL0() const
{
  return L0_;
}

double VMCLeg::getPhi0() const
{
  return phi0;
}

double VMCLeg::getJ11() const
{
  return j11;
}

double VMCLeg::getJ12() const
{
  return j12;
}

double VMCLeg::getJ21() const
{
  return j21;
}

double VMCLeg::getJ22() const
{
  return j22;
}

}  // namespace vmc_controller

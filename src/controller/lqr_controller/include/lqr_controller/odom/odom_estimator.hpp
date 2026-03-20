#ifndef LQR_CONTROLLER_PLANAR_ODOM_ESTIMATOR_HPP_
#define LQR_CONTROLLER_PLANAR_ODOM_ESTIMATOR_HPP_

#include <cmath>

namespace lqr_controller
{
namespace odom
{

// Simple planar odometry estimator:
// - Integrates body-frame forward velocity (v_body along +X_body)
// - Converts it into world-frame velocity using current yaw
// - Outputs world-frame (x, y) and world-frame (vx, vy)
class PlanarOdomEstimator
{
public:
  void reset()
  {
    x_world_ = 0.0;
    y_world_ = 0.0;
    vx_world_ = 0.0;
    vy_world_ = 0.0;
  }

  void update(double dt, double v_body, double yaw)
  {
    vx_world_ = v_body * std::cos(yaw);
    vy_world_ = v_body * std::sin(yaw);
    x_world_ += vx_world_ * dt;
    y_world_ += vy_world_ * dt;
  }

  double x() const { return x_world_; }
  double y() const { return y_world_; }
  double vx() const { return vx_world_; }
  double vy() const { return vy_world_; }

private:
  double x_world_{0.0};
  double y_world_{0.0};
  double vx_world_{0.0};
  double vy_world_{0.0};
};

}  // namespace odom
}  // namespace lqr_controller

#endif  // LQR_CONTROLLER_PLANAR_ODOM_ESTIMATOR_HPP_


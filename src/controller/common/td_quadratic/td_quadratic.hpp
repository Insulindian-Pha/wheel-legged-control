#ifndef TD_QUADRATIC_H
#define TD_QUADRATIC_H

namespace common {
class TDquadratic {
public:
  /**
   * @brief 二阶TD微分跟踪器的参数，一般来说确定好R就可以
   *
   * @param r       决定x1快慢的跟踪因子，越大则越快，小则反之。单位2*pi*Hz
   * @param max_x2  跟踪器的最大输出
   * @param h       采样周期，单位s
   */
  TDquadratic(float r = 300.0f, float h = 0.001f) : r(r), h(h) {}

  float getX1() { return x1; }

  float getX2() { return x2; }

  /** @brief Set sampling period (s), e.g. control period dt */
  void setH(float h_in) { h = h_in; }

  /** @brief Set tracking factor r (larger = faster tracking) */
  void setR(float r_in) { r = r_in; }

  /** @brief Reset internal state to zero (x1=0, x2=0) */
  void reset() {
    x1 = 0.0f;
    x2 = 0.0f;
  }

  /**
   * @brief TD微分跟踪器，其主要作用为消除系统的初期超调
   *
   * @param u 跟踪信号
   * @return float 跟踪器输出
   */
  float Calc(float u) { // 计算公式
    u_ = u;
    x1 += x2 * h;
    x2 += (-2.0f * r * x2 - r * r * (x1 - u_)) * h;
    return x1;
  }

private:
  float u_;
  float x1, x2;
  float r, h = 0.001;
};
} // namespace common
#endif // TD_QUADRATIC_H
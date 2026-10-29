#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Dense>

namespace tools
{

/**
 * 8 维匀速模型卡尔曼滤波器（装甲板位姿）
 *
 * 状态 x = [px, py, pz, yaw, vx, vy, vz, vyaw]^T      (8x1)
 * 观测 z = [px, py, pz, yaw]^T                        (4x1)
 *
 * 速度单位取决于 dt 的约定：dt = 1.0 时为 m/帧，dt = 0.01 时为 m/s。
 *
 * 用法：
 *   tools::KalmanFilter kf(x0, P0, Q, R);
 *   for (每一帧) {
 *     kf.predict(dt);
 *     kf.update(z);
 *     // kf.x 即为该帧的滤波结果
 *   }
 */
class KalmanFilter
{
public:
  static constexpr int kStateDim = 8;
  static constexpr int kMeasDim = 4;

  // 状态估计与协方差，公开出来方便打印调试
  Eigen::VectorXd x;  // 8x1
  Eigen::MatrixXd P;  // 8x8

  // 最近一次 update 的中间量，用于观察滤波器是否自洽
  Eigen::VectorXd lastInnovation;  // 4x1，yaw 分量已做 π 周期归一化
  double lastNis = 0.0;            // 归一化新息平方 ȳᵀS⁻¹ȳ，正常应接近 4

  /**
   * @param x0 初始状态 8x1，通常取第一帧观测 + 速度置零
   * @param P0 初始协方差 8x8，速度部分应给较大值，否则速度学不出来
   * @param Q  过程噪声 8x8，位置项与速度项的量级不同
   * @param R  观测噪声 4x4，可由数据差分估计
   */
  KalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R);

  /**
   * 预测一步：
   *   F = [ I4  dt·I4 ]
   *       [ 0     I4  ]
   *   x = F·x
   *   P = F·P·Fᵀ + Q
   */
  void predict(double dt);

  /**
   * 用观测 z 校正：
   *   S  = H·P·Hᵀ + R
   *   K  = P·Hᵀ·S⁻¹
   *   ȳ  = z − H·x     （yaw 分量先做 π 周期归一化）
   *   x += K·ȳ
   *   P  = (I − K·H)·P
   */
  void update(const Eigen::VectorXd & z);

  // 调参用：不重建滤波器即可换掉 Q / R
  void setQ(const Eigen::MatrixXd & Q) { Q_ = Q; }
  void setR(const Eigen::MatrixXd & R) { R_ = R; }
  const Eigen::MatrixXd & Q() const { return Q_; }
  const Eigen::MatrixXd & R() const { return R_; }

private:
  /**
   * 把角度归一到 [-pi/2, pi/2)。
   * 装甲板具有 π 周期对称性（PnP 正反解歧义），所以是 mod π 而非 mod 2π。
   * 直接用 std::remainder(angle, M_PI) 即可得到该区间。
   */
  static double wrapYaw(double angle);

  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd H_;  // 4x8，左半 I4、右半 0
  Eigen::MatrixXd I_;  // 8x8 单位阵
};

}  // namespace tools

#endif  // KALMAN_FILTER_HPP

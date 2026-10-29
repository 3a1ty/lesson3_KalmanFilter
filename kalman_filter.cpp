#include "kalman_filter.hpp"

#include <cmath>    
#include <stdexcept>   
namespace tools
{

KalmanFilter::KalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R)
: x(x0), P(P0), Q_(Q), R_(R),
  H_(Eigen::MatrixXd::Zero(kMeasDim, kStateDim)),
  I_(Eigen::MatrixXd::Identity(kStateDim, kStateDim))
{

  H_.leftCols(kMeasDim).setIdentity();

  lastInnovation = Eigen::VectorXd::Zero(kMeasDim);
  lastNis = 0.0;
}

double KalmanFilter::wrapYaw(double angle)
{
  return std::remainder(angle, M_PI);
}

void KalmanFilter::predict(double dt)
{
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(kStateDim, kStateDim);
  F.topRightCorner(kMeasDim, kMeasDim) = dt * Eigen::MatrixXd::Identity(kMeasDim, kMeasDim);

  x = F * x;
  P = F * P * F.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd & z)
{
  Eigen::MatrixXd S = H_ * P * H_.transpose() + R_;
  Eigen::MatrixXd K = P * H_.transpose() * S.inverse();
  Eigen::VectorXd innov = z - H_ * x;
  innov(3) = wrapYaw(innov(3));

  lastInnovation = innov;
  lastNis = innov.transpose() * S.ldlt().solve(innov);

  x = x + K * innov;
  P = (I_ - K * H_) * P;
}

}  // namespace tools

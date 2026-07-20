#include "imu/initializer.h"

#include <Eigen/Geometry>

namespace {

// Mean and per-axis variance of a vector quantity over the window.
struct MeanVar {
  Eigen::Vector3d mean;
  Eigen::Vector3d var;
};

template <typename Accessor>
MeanVar mean_var(const std::vector<ImuMeasurement>& imu, Accessor get) {
  const double n = static_cast<double>(imu.size());
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const ImuMeasurement& m : imu) mean += get(m);
  mean /= n;
  Eigen::Vector3d var = Eigen::Vector3d::Zero();
  for (const ImuMeasurement& m : imu) {
    const Eigen::Vector3d d = get(m) - mean;
    var += d.cwiseProduct(d);
  }
  var /= n;
  return {mean, var};
}

}  // namespace

InitResult initialize_static(const std::vector<ImuMeasurement>& imu,
                             const InitParams& params) {
  InitResult result;
  if (imu.empty()) return result;  // ok stays false

  const MeanVar accel = mean_var(
      imu, [](const ImuMeasurement& m) { return m.linear_acceleration; });
  const MeanVar gyro =
      mean_var(imu, [](const ImuMeasurement& m) { return m.angular_velocity; });

  result.accel_mean_norm = accel.mean.norm();
  result.max_accel_var = accel.var.maxCoeff();
  result.max_gyro_var = gyro.var.maxCoeff();

  // Stationarity gate.
  if (result.max_accel_var > params.max_accel_var ||
      result.max_gyro_var > params.max_gyro_var) {
    return result;  // ok stays false
  }

  State s;  // p = v = b_a = 0
  s.gravity = Eigen::Vector3d(0.0, 0.0, -params.gravity_magnitude);
  s.b_g = gyro.mean;

  // Align attitude to gravity: rotate the measured "up" (f_hat) onto world up.
  // This pins roll and pitch; the rotation axis lies in the horizontal plane,
  // so yaw stays zero (gauge-fixed by definition).
  const Eigen::Vector3d f_hat = accel.mean.normalized();
  s.R = Sophus::SO3d(
      Eigen::Quaterniond::FromTwoVectors(f_hat, Eigen::Vector3d::UnitZ()));

  // Covariance seed: diagonal from per-block standard deviations.
  auto sq = [](double v) { return v * v; };
  ErrorState diag;
  diag.segment<3>(kIdxTheta).setConstant(sq(params.theta_std));
  diag.segment<3>(kIdxPos).setConstant(sq(params.pos_std));
  diag.segment<3>(kIdxVel).setConstant(sq(params.vel_std));
  diag.segment<3>(kIdxBiasGyro).setConstant(sq(params.bias_gyro_std));
  diag.segment<3>(kIdxBiasAccel).setConstant(sq(params.bias_accel_std));
  diag.segment<2>(kIdxGravity).setConstant(sq(params.gravity_std));  // 2-DOF
  diag.segment<3>(kIdxExtRot).setConstant(sq(params.ext_rot_std));
  diag.segment<3>(kIdxExtPos).setConstant(sq(params.ext_trans_std));

  result.ok = true;
  result.state = s;
  result.cov = diag.asDiagonal();
  return result;
}

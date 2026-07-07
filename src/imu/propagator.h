#pragma once

#include <Eigen/Core>
#include <utility>

#include "imu/state.h"
#include "math/skew.h"
#include "types.h"

struct NoiseParams {
  double gyro_noise_std{0.0};   // rad/s / sqrt(Hz)
  double accel_noise_std{0.0};  // m/s² / sqrt(Hz)
  double gyro_rw_std{0.0};      // rad/s² / sqrt(Hz)  (gyro bias random walk)
  double accel_rw_std{0.0};     // m/s³ / sqrt(Hz)   (accel bias random walk)
};

// Single-step IMU propagation using the midpoint integration rule.
// One step spans [m0.stamp, m1.stamp].
class ImuPropagator {
 public:
  explicit ImuPropagator(const NoiseParams& noise);

  // Propagate state only.
  State propagate(const State& s0, const ImuMeasurement& m0,
                  const ImuMeasurement& m1) const;

  // Propagate state and error-state covariance (17×17).
  // Uses a first-order discrete-time Jacobian and continuous-time noise model.
  std::pair<State, Eigen::Matrix<double, 17, 17>> propagate_with_covariance(
      const State& s0, const Eigen::Matrix<double, 17, 17>& P,
      const ImuMeasurement& m0, const ImuMeasurement& m1) const;

 private:
  NoiseParams noise_;
};

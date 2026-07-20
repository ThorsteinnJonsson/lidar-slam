#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

#include "state/s2.h"

// Full navigation state, including the LiDAR-to-IMU extrinsic.
// Error-state δx ∈ R²³ is ordered
//   [δθ, δp, δv, δb_g, δb_a, δg, δθ_ext, δp_ext].
// Every block is a 3-vector except δg, which is 2-DOF: gravity lives on a
// fixed-magnitude sphere (S²) so it can tilt but not stretch. See state/s2.h.
//
// The extrinsic (R_imu_lidar, p_imu_lidar) maps a point from the lidar frame
// into the IMU frame. It is seeded from calibration and then estimated online
// as a random constant (no process noise), driven by the point-to-plane
// measurement. Its translation needs motion excitation to be observable.
struct State {
  Sophus::SO3d R;               // body rotation in world frame
  Eigen::Vector3d p;            // position in world frame         (m)
  Eigen::Vector3d v;            // velocity in world frame         (m/s)
  Eigen::Vector3d b_g;          // gyroscope bias                  (rad/s)
  Eigen::Vector3d b_a;          // accelerometer bias              (m/s²)
  Eigen::Vector3d gravity;      // gravity vector in world frame   (m/s²)
  Sophus::SO3d R_imu_lidar;     // lidar->IMU rotation
  Eigen::Vector3d p_imu_lidar;  // lidar->IMU translation      (m)

  State()
      : p(Eigen::Vector3d::Zero()),
        v(Eigen::Vector3d::Zero()),
        b_g(Eigen::Vector3d::Zero()),
        b_a(Eigen::Vector3d::Zero()),
        gravity(Eigen::Vector3d(0.0, 0.0, -9.81)),
        p_imu_lidar(Eigen::Vector3d::Zero()) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Dimension of the error state
// δx = [δθ, δp, δv, δb_g, δb_a, δg, δθ_ext, δp_ext]; every block is a 3-vector
// except δg, which is 2-DOF (S²).
inline constexpr int kErrorDim = 23;

// Block offsets within the error state.
inline constexpr int kIdxTheta = 0;
inline constexpr int kIdxPos = 3;
inline constexpr int kIdxVel = 6;
inline constexpr int kIdxBiasGyro = 9;
inline constexpr int kIdxBiasAccel = 12;
inline constexpr int kIdxGravity = 15;  // 2-DOF
inline constexpr int kIdxExtRot = 17;
inline constexpr int kIdxExtPos = 20;

// Error-state increment vector, ordered as above.
using ErrorState = Eigen::Matrix<double, kErrorDim, 1>;

// Square operator over the error state: covariance P, transition Jacobians
// (Fc/Fd), process noise Qd, and gain products (KH, S) all share this shape.
using ErrorMatrix = Eigen::Matrix<double, kErrorDim, kErrorDim>;

// Apply an error-state increment to a nominal state.
// Rotations use the right perturbation R' = R · Exp(δθ); gravity uses the S²
// retraction (tilt only, magnitude fixed); every other block is vector
// addition. Inverse of boxminus.
inline State boxplus(const State& s, const ErrorState& dx) {
  State out;
  out.R = s.R * Sophus::SO3d::exp(dx.segment<3>(kIdxTheta));
  out.p = s.p + dx.segment<3>(kIdxPos);
  out.v = s.v + dx.segment<3>(kIdxVel);
  out.b_g = s.b_g + dx.segment<3>(kIdxBiasGyro);
  out.b_a = s.b_a + dx.segment<3>(kIdxBiasAccel);
  out.gravity = s2_boxplus(s.gravity, dx.segment<2>(kIdxGravity));
  out.R_imu_lidar =
      s.R_imu_lidar * Sophus::SO3d::exp(dx.segment<3>(kIdxExtRot));
  out.p_imu_lidar = s.p_imu_lidar + dx.segment<3>(kIdxExtPos);
  return out;
}

// Difference of two states: the increment dx with boxplus(b, dx) == a.
// The rotation parts are δθ = log(b.R⁻¹ · a.R); gravity is the S² difference;
// the rest is plain subtraction.
inline ErrorState boxminus(const State& a, const State& b) {
  ErrorState dx;
  dx.segment<3>(kIdxTheta) = (b.R.inverse() * a.R).log();
  dx.segment<3>(kIdxPos) = a.p - b.p;
  dx.segment<3>(kIdxVel) = a.v - b.v;
  dx.segment<3>(kIdxBiasGyro) = a.b_g - b.b_g;
  dx.segment<3>(kIdxBiasAccel) = a.b_a - b.b_a;
  dx.segment<2>(kIdxGravity) = s2_boxminus(a.gravity, b.gravity);
  dx.segment<3>(kIdxExtRot) = (b.R_imu_lidar.inverse() * a.R_imu_lidar).log();
  dx.segment<3>(kIdxExtPos) = a.p_imu_lidar - b.p_imu_lidar;
  return dx;
}

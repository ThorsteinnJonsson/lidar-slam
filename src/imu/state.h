#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

#include "imu/s2.h"

// Full 17-DOF navigation state.
// Error-state δx ∈ R¹⁷ is ordered [δθ, δp, δv, δb_g, δb_a, δg]. Every block is
// a 3-vector except δg, which is 2-DOF: gravity lives on a fixed-magnitude
// sphere (S²) so it can tilt but not stretch. See imu/s2.h.
struct State {
  Sophus::SO3d R;           // body rotation in world frame
  Eigen::Vector3d p;        // position in world frame         (m)
  Eigen::Vector3d v;        // velocity in world frame         (m/s)
  Eigen::Vector3d b_g;      // gyroscope bias                  (rad/s)
  Eigen::Vector3d b_a;      // accelerometer bias              (m/s²)
  Eigen::Vector3d gravity;  // gravity vector in world frame   (m/s²)

  State()
      : p(Eigen::Vector3d::Zero()),
        v(Eigen::Vector3d::Zero()),
        b_g(Eigen::Vector3d::Zero()),
        b_a(Eigen::Vector3d::Zero()),
        gravity(Eigen::Vector3d(0.0, 0.0, -9.81)) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Error-state vector, ordered [δθ, δp, δv, δb_g, δb_a, δg]; δg is 2-DOF (S²).
using Vector17 = Eigen::Matrix<double, 17, 1>;

// Apply an error-state increment to a nominal state.
// Rotation uses the right perturbation R' = R · Exp(δθ); gravity uses the S²
// retraction (tilt only, magnitude fixed); every other block is vector
// addition. Inverse of boxminus.
inline State boxplus(const State& s, const Vector17& dx) {
  State out;
  out.R = s.R * Sophus::SO3d::exp(dx.segment<3>(0));
  out.p = s.p + dx.segment<3>(3);
  out.v = s.v + dx.segment<3>(6);
  out.b_g = s.b_g + dx.segment<3>(9);
  out.b_a = s.b_a + dx.segment<3>(12);
  out.gravity = s2_boxplus(s.gravity, dx.segment<2>(15));
  return out;
}

// Difference of two states: the increment dx with boxplus(b, dx) == a.
// The rotation part is δθ = log(b.R⁻¹ · a.R); gravity is the S² difference; the
// rest is plain subtraction.
inline Vector17 boxminus(const State& a, const State& b) {
  Vector17 dx;
  dx.segment<3>(0) = (b.R.inverse() * a.R).log();
  dx.segment<3>(3) = a.p - b.p;
  dx.segment<3>(6) = a.v - b.v;
  dx.segment<3>(9) = a.b_g - b.b_g;
  dx.segment<3>(12) = a.b_a - b.b_a;
  dx.segment<2>(15) = s2_boxminus(a.gravity, b.gravity);
  return dx;
}

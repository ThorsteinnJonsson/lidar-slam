#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

// Full 18-DOF navigation state.
// Error-state δx ∈ R¹⁸ is ordered [δθ, δp, δv, δb_g, δb_a, δg],
// with each block being a 3-vector.
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

// Error-state vector, ordered [δθ, δp, δv, δb_g, δb_a, δg].
using Vector18 = Eigen::Matrix<double, 18, 1>;

// Apply an error-state increment to a nominal state.
// Rotation uses the right perturbation R' = R · Exp(δθ); every other block is
// vector addition. Inverse of boxminus.
inline State boxplus(const State& s, const Vector18& dx) {
  State out;
  out.R = s.R * Sophus::SO3d::exp(dx.segment<3>(0));
  out.p = s.p + dx.segment<3>(3);
  out.v = s.v + dx.segment<3>(6);
  out.b_g = s.b_g + dx.segment<3>(9);
  out.b_a = s.b_a + dx.segment<3>(12);
  out.gravity = s.gravity + dx.segment<3>(15);
  return out;
}

// Difference of two states: the increment dx with boxplus(b, dx) == a.
// The rotation part is δθ = log(b.R⁻¹ · a.R); the rest is plain subtraction.
inline Vector18 boxminus(const State& a, const State& b) {
  Vector18 dx;
  dx.segment<3>(0) = (b.R.inverse() * a.R).log();
  dx.segment<3>(3) = a.p - b.p;
  dx.segment<3>(6) = a.v - b.v;
  dx.segment<3>(9) = a.b_g - b.b_g;
  dx.segment<3>(12) = a.b_a - b.b_a;
  dx.segment<3>(15) = a.gravity - b.gravity;
  return dx;
}

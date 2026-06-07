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

#pragma once

#include <Eigen/Core>

// Skew-symmetric (cross-product) matrix of a 3-vector: skew(a) * b ==
// a.cross(b).
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d s;
  // clang-format off
  s <<     0, -v.z(),  v.y(),
       v.z(),      0, -v.x(),
      -v.y(),  v.x(),      0;
  // clang-format on
  return s;
}

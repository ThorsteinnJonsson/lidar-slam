#pragma once

#include <Eigen/Core>
#include <cmath>
#include <sophus/so3.hpp>

// Gravity lives on a sphere of fixed radius |g|: it can tilt but never change
// magnitude. These helpers implement the 2-DOF tangent-space (S^2) operations
// the error state uses for the gravity block, matching FAST-LIO2 / IKFoM. A
// free R^3 gravity would let the filter absorb accelerometer-bias error by
// tilting gravity off vertical; pinning the magnitude removes that DOF.

// Orthonormal 3x2 basis of the tangent plane at g: the two directions g can
// tilt, both perpendicular to g.
inline Eigen::Matrix<double, 3, 2> s2_tangent_basis(const Eigen::Vector3d& g) {
  const Eigen::Vector3d u = g.normalized();
  // Seed with a reference axis not parallel to u to avoid a degenerate basis.
  const Eigen::Vector3d ref = std::abs(u.z()) < 0.9 ? Eigen::Vector3d::UnitZ()
                                                    : Eigen::Vector3d::UnitX();
  const Eigen::Vector3d b1 = (ref - ref.dot(u) * u).normalized();
  const Eigen::Vector3d b2 = u.cross(b1);
  Eigen::Matrix<double, 3, 2> basis;
  basis.col(0) = b1;
  basis.col(1) = b2;
  return basis;
}

// Retract a 2-DOF perturbation onto the sphere: rotate g by Exp(B(g)*delta).
// Preserves |g| exactly since it is a pure rotation.
inline Eigen::Vector3d s2_boxplus(const Eigen::Vector3d& g,
                                  const Eigen::Vector2d& delta) {
  const Eigen::Vector3d phi = s2_tangent_basis(g) * delta;
  return Sophus::SO3d::exp(phi) * g;
}

// Inverse of s2_boxplus: the 2-vector delta with s2_boxplus(b, delta) == a
// (a and b assumed to share the same magnitude). delta is the shortest geodesic
// rotation carrying b onto a, expressed in b's tangent basis.
inline Eigen::Vector2d s2_boxminus(const Eigen::Vector3d& a,
                                   const Eigen::Vector3d& b) {
  const Eigen::Vector3d ua = a.normalized();
  const Eigen::Vector3d ub = b.normalized();
  const Eigen::Vector3d cross = ub.cross(ua);
  const double s = cross.norm();
  const double c = ub.dot(ua);
  Eigen::Vector3d phi = Eigen::Vector3d::Zero();
  if (s > 1e-12) phi = std::atan2(s, c) * cross / s;  // else a == b, delta = 0
  return s2_tangent_basis(b).transpose() * phi;
}

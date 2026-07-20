#include "estimator/measurement.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <sophus/so3.hpp>
#include <vector>

#include "map/association.h"
#include "state/state.h"

namespace {

using Vec3 = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;

State sample_state() {
  State s;
  s.R = Sophus::SO3d::exp(Vec3(0.3, -0.7, 0.2));
  s.p = Vec3(1, 2, 3);
  s.v = Vec3(-0.5, 0.1, 0.4);
  s.b_g = Vec3(0.01, -0.02, 0.03);
  s.b_a = Vec3(0.1, 0.2, -0.1);
  s.gravity = Vec3(0.05, -0.05, -9.8);
  // A non-trivial extrinsic, so the finite difference exercises the extrinsic
  // Jacobian blocks rather than a degenerate identity.
  s.R_imu_lidar = Sophus::SO3d::exp(Vec3(0.02, -0.05, 0.03));
  s.p_imu_lidar = Vec3(-0.05, 0.01, 0.06);
  return s;
}

// Matches are associated in the lidar frame, so `point` is p_L.
PlaneMatch make_match(const Vec3f& p_L, const Vec3f& normal, float residual) {
  return PlaneMatch{p_L, normal.normalized(), residual};
}

std::vector<PlaneMatch> sample_matches() {
  return {
      make_match({1, 2, 3}, {0, 0, 1}, 0.10f),
      make_match({-2, 0.5f, 1}, {1, 0, 0}, -0.05f),
      make_match({0.4f, -1.5f, 2}, {1, 1, 1}, 0.20f),
      make_match({3, 3, -1}, {0.2f, -0.9f, 0.3f}, 0.0f),
  };
}

// Residual h = n.(R (R_ext p_L + p_ext) + p) + d at state `s`, for the given
// match and plane offset d. The extrinsic is part of the state, so perturbing
// its blocks moves the residual too.
double residual_at(const State& s, const PlaneMatch& m, double d) {
  const Vec3 p_L = m.point.cast<double>();
  const Vec3 n = m.normal.cast<double>();
  const Vec3 p_I = s.R_imu_lidar * p_L + s.p_imu_lidar;
  const Vec3 p_W = s.R * p_I + s.p;
  return n.dot(p_W) + d;
}

}  // namespace

TEST(Measurement, JacobianMatchesFiniteDifference) {
  const State s = sample_state();
  const auto matches = sample_matches();
  const auto lin = build_measurement(s, matches);

  ASSERT_EQ(lin.H.rows(), static_cast<Eigen::Index>(matches.size()));
  ASSERT_EQ(lin.z.size(), static_cast<Eigen::Index>(matches.size()));

  const double eps = 1e-6;
  for (Eigen::Index i = 0; i < lin.H.rows(); ++i) {
    // Reconstruct the plane offset d so the analytic residual at `s` equals the
    // stored residual; the finite difference then perturbs from a consistent
    // operating point.
    const double d = static_cast<double>(matches[i].residual) -
                     residual_at(s, matches[i], 0.0);

    for (int j = 0; j < kErrorDim; ++j) {
      ErrorState delta = ErrorState::Zero();
      delta(j) = eps;
      const double r_plus = residual_at(boxplus(s, delta), matches[i], d);
      const double r_minus = residual_at(boxplus(s, -delta), matches[i], d);
      const double numeric = (r_plus - r_minus) / (2 * eps);

      EXPECT_NEAR(lin.H(i, j), numeric, 1e-5) << "row " << i << " col " << j;
    }
  }
}

TEST(Measurement, OnlyPoseAndExtrinsicBlocksAreNonzero) {
  const State s = sample_state();
  const auto lin = build_measurement(s, sample_matches());

  // Velocity, biases, and gravity (columns 6..16) carry no information from a
  // single frame; they are corrected only through covariance cross-terms.
  EXPECT_TRUE(lin.H.middleCols(kIdxVel, kIdxExtRot - kIdxVel).isZero(0.0));
  // Pose (rotation + position) and extrinsic blocks are not all zero.
  EXPECT_GT(lin.H.leftCols<6>().cwiseAbs().sum(), 0.0);
  EXPECT_GT(lin.H.rightCols<6>().cwiseAbs().sum(), 0.0);
}

TEST(Measurement, ResidualVectorEchoesMatchResiduals) {
  const State s = sample_state();
  const auto matches = sample_matches();
  const auto lin = build_measurement(s, matches);

  for (Eigen::Index i = 0; i < lin.z.size(); ++i)
    EXPECT_FLOAT_EQ(static_cast<float>(lin.z(i)), matches[i].residual);
}

TEST(Measurement, EmptyMatchesYieldEmptySystem) {
  const State s = sample_state();
  const auto lin = build_measurement(s, {});

  EXPECT_EQ(lin.H.rows(), 0);
  EXPECT_EQ(lin.H.cols(), kErrorDim);
  EXPECT_EQ(lin.z.size(), 0);
}

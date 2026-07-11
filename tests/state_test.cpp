#include "state/state.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <random>
#include <sophus/so3.hpp>

namespace {

using Vec3 = Eigen::Vector3d;
using Vec2 = Eigen::Vector2d;

// A state with all blocks set to distinct, non-trivial values so a bug in any
// single segment shows up.
State sample_state() {
  State s;
  s.R = Sophus::SO3d::exp(Vec3(0.3, -0.7, 0.2));
  s.p = Vec3(1, 2, 3);
  s.v = Vec3(-0.5, 0.1, 0.4);
  s.b_g = Vec3(0.01, -0.02, 0.03);
  s.b_a = Vec3(0.1, 0.2, -0.1);
  s.gravity = Vec3(0.05, -0.05, -9.8);
  return s;
}

Vector17 random_increment(uint32_t seed, double scale) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> d(-scale, scale);
  Vector17 dx;
  for (int i = 0; i < 17; ++i) dx(i) = d(rng);
  return dx;
}

}  // namespace

TEST(State, BoxplusZeroIsIdentity) {
  const State s = sample_state();
  const State s2 = boxplus(s, Vector17::Zero());

  EXPECT_TRUE(s2.R.matrix().isApprox(s.R.matrix()));
  EXPECT_TRUE(s2.p.isApprox(s.p));
  EXPECT_TRUE(s2.v.isApprox(s.v));
  EXPECT_TRUE(s2.b_g.isApprox(s.b_g));
  EXPECT_TRUE(s2.b_a.isApprox(s.b_a));
  EXPECT_TRUE(s2.gravity.isApprox(s.gravity));
}

TEST(State, BoxminusOfSelfIsZero) {
  const State s = sample_state();
  EXPECT_TRUE(boxminus(s, s).isZero(1e-12));
}

TEST(State, BoxminusInvertsBoxplus) {
  const State s = sample_state();
  const Vector17 dx = random_increment(/*seed=*/7, /*scale=*/0.4);

  const State s2 = boxplus(s, dx);
  const Vector17 recovered = boxminus(s2, s);

  EXPECT_TRUE(recovered.isApprox(dx, 1e-9))
      << "recovered:\n"
      << recovered.transpose() << "\nexpected:\n"
      << dx.transpose();
}

TEST(State, BoxplusReconstructsFromBoxminus) {
  // boxplus(b, boxminus(a, b)) == a, the dual round trip.
  const State a = sample_state();
  State b = sample_state();
  b.R = a.R * Sophus::SO3d::exp(Vec3(0.1, 0.05, -0.2));
  b.p += Vec3(0.5, -0.5, 0.25);
  b.v += Vec3(0.2, 0.2, 0.2);

  const State out = boxplus(b, boxminus(a, b));

  EXPECT_TRUE(out.R.matrix().isApprox(a.R.matrix(), 1e-9));
  EXPECT_TRUE(out.p.isApprox(a.p, 1e-9));
  EXPECT_TRUE(out.v.isApprox(a.v, 1e-9));
  EXPECT_TRUE(out.b_g.isApprox(a.b_g, 1e-9));
  EXPECT_TRUE(out.b_a.isApprox(a.b_a, 1e-9));
  EXPECT_TRUE(out.gravity.isApprox(a.gravity, 1e-9));
}

TEST(State, BoxplusRotatesWithRightPerturbation) {
  const State s = sample_state();
  const Vec3 dtheta(0.05, -0.1, 0.07);
  Vector17 dx = Vector17::Zero();
  dx.segment<3>(0) = dtheta;

  const State s2 = boxplus(s, dx);
  const Sophus::SO3d expected = s.R * Sophus::SO3d::exp(dtheta);

  EXPECT_TRUE(s2.R.matrix().isApprox(expected.matrix(), 1e-12));
  // The non-rotation blocks are untouched.
  EXPECT_TRUE(s2.p.isApprox(s.p));
  EXPECT_TRUE(s2.v.isApprox(s.v));
}

TEST(State, BoxplusAddsVectorBlocksLinearly) {
  const State s = sample_state();
  Vector17 dx = Vector17::Zero();
  dx.segment<3>(3) = Vec3(1, -2, 3);     // δp
  dx.segment<3>(6) = Vec3(0.1, 0.2, 0);  // δv
  dx.segment<3>(9) = Vec3(0, 0, 0.01);   // δb_g
  dx.segment<3>(12) = Vec3(0.5, 0, 0);   // δb_a

  const State s2 = boxplus(s, dx);

  EXPECT_TRUE(s2.p.isApprox(s.p + dx.segment<3>(3)));
  EXPECT_TRUE(s2.v.isApprox(s.v + dx.segment<3>(6)));
  EXPECT_TRUE(s2.b_g.isApprox(s.b_g + dx.segment<3>(9)));
  EXPECT_TRUE(s2.b_a.isApprox(s.b_a + dx.segment<3>(12)));
  // Rotation untouched when δθ is zero; gravity untouched when δg is zero.
  EXPECT_TRUE(s2.R.matrix().isApprox(s.R.matrix()));
  EXPECT_TRUE(s2.gravity.isApprox(s.gravity));
}

TEST(State, BoxplusTiltsGravityKeepingMagnitude) {
  const State s = sample_state();
  Vector17 dx = Vector17::Zero();
  dx.segment<2>(15) = Vec2(0.05, -0.03);  // δg on S² (tilt, not stretch)

  const State s2 = boxplus(s, dx);

  EXPECT_NEAR(s2.gravity.norm(), s.gravity.norm(), 1e-9);  // magnitude fixed
  EXPECT_FALSE(s2.gravity.isApprox(s.gravity));            // but it moved
  // Everything else untouched.
  EXPECT_TRUE(s2.R.matrix().isApprox(s.R.matrix()));
  EXPECT_TRUE(s2.p.isApprox(s.p));
}

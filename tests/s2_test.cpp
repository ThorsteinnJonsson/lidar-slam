#include "state/s2.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <random>

namespace {

using Vec3 = Eigen::Vector3d;
using Vec2 = Eigen::Vector2d;

// Gravity-like vector: magnitude ~9.8, tilted slightly off vertical.
Vec3 sample_gravity() { return Vec3(0.3, -0.2, -9.8); }

}  // namespace

TEST(S2, TangentBasisIsOrthonormalAndPerpendicular) {
  const Vec3 g = sample_gravity();
  const Eigen::Matrix<double, 3, 2> b = s2_tangent_basis(g);

  // Columns orthonormal.
  EXPECT_NEAR(b.col(0).norm(), 1.0, 1e-12);
  EXPECT_NEAR(b.col(1).norm(), 1.0, 1e-12);
  EXPECT_NEAR(b.col(0).dot(b.col(1)), 0.0, 1e-12);
  // Both perpendicular to g (the forbidden radial direction).
  EXPECT_NEAR(b.col(0).dot(g), 0.0, 1e-12);
  EXPECT_NEAR(b.col(1).dot(g), 0.0, 1e-12);
}

TEST(S2, BoxplusPreservesMagnitude) {
  const Vec3 g = sample_gravity();
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> d(-0.5, 0.5);
  for (int i = 0; i < 100; ++i) {
    const Vec2 delta(d(rng), d(rng));
    const Vec3 g2 = s2_boxplus(g, delta);
    EXPECT_NEAR(g2.norm(), g.norm(), 1e-9);  // stays on the sphere
  }
}

TEST(S2, BoxplusZeroIsIdentity) {
  const Vec3 g = sample_gravity();
  EXPECT_TRUE(s2_boxplus(g, Vec2::Zero()).isApprox(g));
}

TEST(S2, BoxminusInvertsBoxplus) {
  const Vec3 g = sample_gravity();
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> d(-0.4, 0.4);
  for (int i = 0; i < 100; ++i) {
    const Vec2 delta(d(rng), d(rng));
    const Vec3 g2 = s2_boxplus(g, delta);
    const Vec2 recovered = s2_boxminus(g2, g);
    EXPECT_TRUE(recovered.isApprox(delta, 1e-9))
        << "delta " << delta.transpose() << " recovered "
        << recovered.transpose();
  }
}

TEST(S2, BoxminusOfEqualIsZero) {
  const Vec3 g = sample_gravity();
  EXPECT_TRUE(s2_boxminus(g, g).isZero(1e-12));
}

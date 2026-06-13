#include "map/association.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <random>
#include <sophus/se3.hpp>
#include <vector>

#include "map/ikd_tree.h"

namespace {

using Vec3 = Eigen::Vector3f;

// An orthonormal basis (u, v) spanning the plane with unit normal n.
void plane_basis(const Vec3& n, Vec3& u, Vec3& v) {
  const Vec3 e = (std::abs(n.x()) < 0.9f) ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
  u = n.cross(e).normalized();
  v = n.cross(u);  // already unit since n and u are orthonormal
}

// Sample `count` points on the plane through p0 with unit normal n, each
// perturbed along the normal by up to `noise`.
std::vector<Vec3> plane_cloud(const Vec3& n, const Vec3& p0, int count,
                              float noise, uint32_t seed) {
  Vec3 u, v;
  plane_basis(n, u, v);
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> ab(-5.0f, 5.0f);
  std::uniform_real_distribution<float> nz(-noise, noise);
  std::vector<Vec3> pts;
  pts.reserve(count);
  for (int i = 0; i < count; ++i)
    pts.push_back(p0 + ab(rng) * u + ab(rng) * v + nz(rng) * n);
  return pts;
}

}  // namespace

TEST(Association, RecoversSyntheticPlane) {
  const Vec3 n_true = Vec3(1, 1, 1).normalized();
  const Vec3 p0(-2, -2, -2);  // on the plane; n.p0 < 0 so d_true > 0
  const float d_true = -n_true.dot(p0);
  ASSERT_GT(d_true, 0.0f);  // ensures the fitted normal matches n_true in sign

  IkdTree map;
  map.build(plane_cloud(n_true, p0, /*count=*/400, /*noise=*/0.0f, /*seed=*/1));

  Vec3 u, v;
  plane_basis(n_true, u, v);
  const Sophus::SE3f identity;  // body frame == world frame

  // With d_true > 0 the fitted plane is exactly (n_true, d_true), so a point
  // offset by h along the normal has residual exactly h (sign included).
  for (float h : {0.05f, -0.05f, 0.2f}) {
    const Vec3 on_plane = p0 + 1.0f * u - 1.0f * v;
    const Vec3 q = on_plane + h * n_true;
    const auto matches = associate_planes(map, {q}, identity);

    ASSERT_EQ(matches.size(), 1u) << "h=" << h;
    EXPECT_NEAR(std::abs(matches[0].normal.dot(n_true)), 1.0f, 1e-3f);
    EXPECT_NEAR(matches[0].residual, h, 5e-3f) << "h=" << h;
    EXPECT_TRUE(matches[0].point.isApprox(q));  // reported in body frame
  }
}

TEST(Association, DistanceGateRejectsFarPoints) {
  const Vec3 n_true(0, 0, 1);
  const Vec3 p0(0, 0, -2);
  IkdTree map;
  map.build(plane_cloud(n_true, p0, /*count=*/300, /*noise=*/0.0f, /*seed=*/2));

  // Far from any map point: the farthest of the 5 neighbors exceeds the gate.
  const auto matches =
      associate_planes(map, {Vec3(100, 100, 100)}, Sophus::SE3f());
  EXPECT_TRUE(matches.empty());
}

TEST(Association, PlanarityGateRejectsNonPlanar) {
  // A clearly non-planar neighborhood: the origin sits 1/sqrt(3) off the plane
  // x + y + z = 1 formed by the other three points.
  const std::vector<Vec3> pts = {
      {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
  IkdTree map;
  map.build(pts);

  const auto matches =
      associate_planes(map, {Vec3(0.2f, 0.2f, 0.2f)}, Sophus::SE3f());
  EXPECT_TRUE(matches.empty());
}

TEST(Association, AppliesBodyToWorldTransform) {
  // World plane z = -2 (normal up, d = 2 > 0).
  const Vec3 n_true(0, 0, 1);
  const Vec3 p0(0, 0, -2);
  IkdTree map;
  map.build(plane_cloud(n_true, p0, /*count=*/400, /*noise=*/0.0f, /*seed=*/3));

  // A non-trivial body->world transform; the point is given in the body frame.
  const Sophus::SE3f T_world_body(Sophus::SO3f::exp(Vec3(0.1f, -0.2f, 0.5f)),
                                  Vec3(5, -3, 1));
  const float h = 0.07f;
  const Vec3 q_world(1, 1, -2 + h);
  const Vec3 p_body = T_world_body.inverse() * q_world;

  const auto matches = associate_planes(map, {p_body}, T_world_body);
  ASSERT_EQ(matches.size(), 1u);
  EXPECT_NEAR(std::abs(matches[0].normal.dot(n_true)), 1.0f, 1e-3f);
  EXPECT_NEAR(matches[0].residual, h, 5e-3f);
  EXPECT_TRUE(matches[0].point.isApprox(p_body));  // body frame, not world
}

TEST(Association, MatchesWholePlanarSheet) {
  const Vec3 n_true = Vec3(0.2f, -0.3f, 1.0f).normalized();
  const Vec3 p0(1, 2, -5);
  const auto sheet =
      plane_cloud(n_true, p0, /*count=*/500, /*noise=*/0.02f, /*seed=*/4);

  IkdTree map;
  map.build(sheet);

  // Query the sheet points themselves (body == world).
  const auto matches = associate_planes(map, sheet, Sophus::SE3f());

  // Noise (0.02) is under the planarity threshold, so the large majority match.
  EXPECT_GE(matches.size(), sheet.size() * 9 / 10);

  // A 5-NN plane fit on noisy points scatters by ~10 degrees per match, so only
  // require each normal to roughly agree, but the average must have no
  // systematic bias away from the true normal.
  Vec3 mean_normal = Vec3::Zero();
  for (const auto& m : matches) {
    const float aligned = m.normal.dot(n_true);
    EXPECT_GT(std::abs(aligned), 0.9f);
    EXPECT_LT(std::abs(m.residual), 0.1f);
    mean_normal += aligned > 0 ? m.normal : -m.normal;
  }
  mean_normal.normalize();
  EXPECT_GT(mean_normal.dot(n_true), 0.99f);
}

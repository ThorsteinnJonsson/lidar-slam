#include "estimator/iekf.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <sophus/so3.hpp>
#include <vector>

#include "estimator/measurement.h"
#include "imu/state.h"

namespace {

using Vec3 = Eigen::Vector3d;
using Matrix18 = Eigen::Matrix<double, 18, 18>;

State true_state() {
  State s;
  s.R = Sophus::SO3d::exp(Vec3(0.2, -0.1, 0.15));
  s.p = Vec3(0.5, -1.0, 2.0);
  s.v = Vec3(0.3, 0.0, -0.2);
  s.b_g = Vec3(0.01, 0.0, -0.01);
  s.b_a = Vec3(0.0, 0.05, 0.0);
  s.gravity = Vec3(0.0, 0.0, -9.81);
  return s;
}

// A set of plane correspondences whose normals span all three axes so the
// stacked system constrains the full 6-DOF pose. Each point lies exactly on its
// plane at `x_true`, so residuals are zero there.
struct Scene {
  std::vector<Vec3> p_I;
  std::vector<Vec3> n;
  std::vector<double> d;
};

Scene make_scene(const State& x_true) {
  const std::vector<Vec3> normals = {Vec3(1, 0, 0), Vec3(0, 1, 0),
                                     Vec3(0, 0, 1), Vec3(1, 1, 1).normalized(),
                                     Vec3(1, -1, 0).normalized()};
  const std::vector<Vec3> points = {Vec3(2, 1, -1), Vec3(-1, 3, 2),
                                    Vec3(0, -2, 4), Vec3(3, 0, 1),
                                    Vec3(-2, -1, -3)};
  Scene sc;
  for (const Vec3& n_raw : normals) {
    const Vec3 n = n_raw.normalized();
    for (const Vec3& p : points) {
      const Vec3 p_W = x_true.R * p + x_true.p;
      sc.p_I.push_back(p);
      sc.n.push_back(n);
      sc.d.push_back(-n.dot(p_W));  // residual is zero at x_true
    }
  }
  return sc;
}

// Builds (H, z) at any state by recomputing each residual against the fixed
// planes, exactly the way the pipeline will (associate gives the current
// residual, build_measurement forms H).
MeasurementFn scene_measure(const Scene& sc) {
  return [&sc](const State& x) {
    std::vector<PlaneMatch> matches;
    matches.reserve(sc.p_I.size());
    for (size_t i = 0; i < sc.p_I.size(); ++i) {
      const Vec3 p_W = x.R * sc.p_I[i] + x.p;
      const double r = sc.n[i].dot(p_W) + sc.d[i];
      matches.push_back({sc.p_I[i].cast<float>(), sc.n[i].cast<float>(),
                         static_cast<float>(r)});
    }
    return build_measurement(x, matches);
  };
}

// Append gross-outlier correspondences: real geometry (point + normal) but a
// plane offset that puts the point a fixed distance off, regardless of pose.
void add_outliers(Scene& sc, const State& x_true, int count, double offset) {
  for (int i = 0; i < count; ++i) {
    const Vec3 p(1.0 + i, -2.0 + 0.5 * i, 0.5 * i);
    const Vec3 n = Vec3(0.3 + 0.1 * i, 1.0, -0.4).normalized();
    const Vec3 p_W = x_true.R * p + x_true.p;
    sc.p_I.push_back(p);
    sc.n.push_back(n);
    sc.d.push_back(-n.dot(p_W) +
                   offset);  // residual ~ offset at any nearby pose
  }
}

double pose_error(const State& a, const State& b) {
  return boxminus(a, b).head<6>().norm();  // rotation + position only
}

}  // namespace

TEST(Iekf, PullsStateTowardTruth) {
  const State x_true = true_state();
  const Scene sc = make_scene(x_true);

  Vector18 err = Vector18::Zero();
  err.segment<3>(0) = Vec3(0.05, -0.04, 0.03);  // rotation error
  err.segment<3>(3) = Vec3(0.1, -0.08, 0.06);   // position error
  const State x_hat = boxplus(x_true, err);

  const Matrix18 P = Matrix18::Identity() * 0.25;
  const auto result = iterated_update(x_hat, P, scene_measure(sc), {});

  EXPECT_GT(pose_error(x_hat, x_true), 0.1);
  EXPECT_LT(pose_error(result.state, x_true), 1e-3);
  EXPECT_TRUE(result.converged);
}

TEST(Iekf, IterationImprovesOnSingleStep) {
  const State x_true = true_state();
  const Scene sc = make_scene(x_true);

  // A large enough error that one linearization undershoots.
  Vector18 err = Vector18::Zero();
  err.segment<3>(0) = Vec3(0.3, -0.2, 0.25);
  err.segment<3>(3) = Vec3(0.3, -0.25, 0.2);
  const State x_hat = boxplus(x_true, err);
  const Matrix18 P = Matrix18::Identity() * 0.25;

  IekfConfig one;
  one.max_iterations = 1;
  const auto single = iterated_update(x_hat, P, scene_measure(sc), one);

  IekfConfig many;
  many.max_iterations = 10;
  const auto iterated = iterated_update(x_hat, P, scene_measure(sc), many);

  EXPECT_EQ(single.iterations, 1);
  EXPECT_LT(pose_error(single.state, x_true), pose_error(x_hat, x_true));
  EXPECT_LT(pose_error(iterated.state, x_true),
            pose_error(single.state, x_true));
  EXPECT_LT(pose_error(iterated.state, x_true), 1e-3);
  EXPECT_TRUE(iterated.converged);
}

TEST(Iekf, CovarianceShrinksAndStaysValid) {
  const State x_true = true_state();
  const Scene sc = make_scene(x_true);

  Vector18 err = Vector18::Zero();
  err.segment<3>(0) = Vec3(0.02, -0.01, 0.015);
  err.segment<3>(3) = Vec3(0.03, -0.02, 0.01);
  const State x_hat = boxplus(x_true, err);

  const Matrix18 P = Matrix18::Identity() * 0.25;
  const auto result = iterated_update(x_hat, P, scene_measure(sc), {});
  const Matrix18& Pp = result.covariance;

  // Symmetric.
  EXPECT_LT((Pp - Pp.transpose()).norm(), 1e-12);
  // Positive semidefinite.
  const Eigen::SelfAdjointEigenSolver<Matrix18> es(Pp);
  EXPECT_GT(es.eigenvalues().minCoeff(), -1e-9);
  // Strictly less total uncertainty than the prior.
  EXPECT_LT(Pp.trace(), P.trace());
  // The observed pose block shrank. (Locals: the template commas in
  // topLeftCorner<6, 6> would otherwise be parsed as extra macro arguments.)
  const double pose_trace_after = Pp.topLeftCorner<6, 6>().trace();
  const double pose_trace_before = P.topLeftCorner<6, 6>().trace();
  EXPECT_LT(pose_trace_after, pose_trace_before);
}

TEST(Iekf, OutlierGateRecoversFromBadCorrespondences) {
  const State x_true = true_state();
  Scene sc = make_scene(x_true);
  add_outliers(sc, x_true, /*count=*/8, /*offset=*/10.0);

  Vector18 err = Vector18::Zero();
  err.segment<3>(0) = Vec3(0.02, -0.01, 0.015);
  err.segment<3>(3) = Vec3(0.03, -0.02, 0.01);
  const State x_hat = boxplus(x_true, err);
  const Matrix18 P = Matrix18::Identity() * 0.25;

  IekfConfig ungated;
  ungated.reject_outliers = false;
  const auto without = iterated_update(x_hat, P, scene_measure(sc), ungated);

  IekfConfig gated;  // reject_outliers defaults to true
  const auto with = iterated_update(x_hat, P, scene_measure(sc), gated);

  // The outliers drag the ungated solution off truth; the gate recovers it.
  EXPECT_GT(pose_error(without.state, x_true), 0.05);
  EXPECT_LT(pose_error(with.state, x_true), 1e-3);
  EXPECT_LT(pose_error(with.state, x_true), pose_error(without.state, x_true));
  EXPECT_TRUE(with.converged);
}

TEST(Iekf, NoCorrespondencesLeavePriorUnchanged) {
  const State x_hat = true_state();
  const Matrix18 P = Matrix18::Identity() * 0.25;

  const MeasurementFn empty = [](const State& x) {
    return build_measurement(x, {});
  };
  const auto result = iterated_update(x_hat, P, empty, {});

  EXPECT_EQ(result.iterations, 0);
  EXPECT_TRUE(result.converged);
  EXPECT_TRUE(result.covariance.isApprox(P));
  EXPECT_LT(pose_error(result.state, x_hat), 1e-12);
}

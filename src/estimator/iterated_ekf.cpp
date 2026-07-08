#include "estimator/iterated_ekf.h"

#include <algorithm>
#include <cmath>

#include "estimator/measurement.h"

IteratedEkf::IteratedEkf(const NoiseParams& noise,
                         const Sophus::SE3d& T_imu_lidar, const IekfConfig& cfg,
                         const PlaneAssocParams& assoc, const State& x0,
                         const Eigen::Matrix<double, 17, 17>& P0,
                         const LocalMapParams& map_params)
    : map_(map_params),
      x_(x0),
      P_(P0),
      propagator_(noise),
      T_imu_lidar_(T_imu_lidar),
      cfg_(cfg),
      assoc_(assoc) {}

EkfResult IteratedEkf::process_scan(
    const std::vector<ImuMeasurement>& imu,
    const std::vector<Eigen::Vector3f>& points_lidar) {
  // ── Predict: advance (x, P) to scan end over consecutive IMU pairs
  // ──────────
  for (size_t i = 0; i + 1 < imu.size(); ++i) {
    std::tie(x_, P_) =
        propagator_.propagate_with_covariance(x_, P_, imu[i], imu[i + 1]);
  }

  // Lidar points into the IMU frame once; the extrinsic is fixed, so only the
  // pose changes between iterations.
  std::vector<Eigen::Vector3f> points_imu;
  points_imu.reserve(points_lidar.size());
  const Sophus::SE3f T_il = T_imu_lidar_.cast<float>();
  for (const Eigen::Vector3f& p_L : points_lidar) {
    points_imu.push_back(T_il * p_L);
  }

  // ── Update: re-associate against the live iterate each iteration
  // ────────────
  // Keep the last iterate's correspondences for registration diagnostics.
  std::vector<PlaneMatch> last_matches;
  const MeasurementFn measure = [&](const State& x) {
    const Sophus::SE3f T_WI(x.R.cast<float>(), x.p.cast<float>());
    last_matches = associate_planes(map_.tree(), points_imu, T_WI, assoc_);
    return build_measurement(x, last_matches);
  };
  EkfResult result = iterated_update(x_, P_, measure, cfg_);
  x_ = result.state;
  P_ = result.covariance;

  // Registration diagnostics: how many scan points found a nearby plane, and
  // the median absolute point-to-plane distance at the final iterate.
  result.num_scan_points = static_cast<int>(points_imu.size());
  result.num_matches = static_cast<int>(last_matches.size());
  if (!last_matches.empty()) {
    std::vector<double> abs_res;
    abs_res.reserve(last_matches.size());
    for (const PlaneMatch& m : last_matches)
      abs_res.push_back(std::abs(static_cast<double>(m.residual)));
    const size_t mid = abs_res.size() / 2;
    std::nth_element(abs_res.begin(), abs_res.begin() + mid, abs_res.end());
    result.median_abs_residual = abs_res[mid];
  }

  // Mean rotation and (gravity-removed) linear acceleration over the window.
  // a_world = R * f + g cancels gravity: at rest f = -R^T g so this is ~0.
  if (!imu.empty()) {
    double sum_w = 0.0;
    double sum_a = 0.0;
    for (const ImuMeasurement& m : imu) {
      sum_w += m.angular_velocity.norm();
      sum_a += (x_.R * m.linear_acceleration + x_.gravity).norm();
    }
    const double n = static_cast<double>(imu.size());
    result.mean_omega = sum_w / n;
    result.mean_acc = sum_a / n;
  }

  // ── Map: fold in the registered scan, then bound to the sliding window
  // ──────
  const Sophus::SE3d T_WI(x_.R, x_.p);
  std::vector<Eigen::Vector3f> world;
  world.reserve(points_imu.size());
  for (const Eigen::Vector3f& p_I : points_imu) {
    world.push_back((T_WI * p_I.cast<double>()).cast<float>());
  }
  map_.insert(std::move(world));
  map_.recenter(x_.p.cast<float>());

  return result;
}

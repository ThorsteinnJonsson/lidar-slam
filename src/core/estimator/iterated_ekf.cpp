#include "estimator/iterated_ekf.h"

#include "estimator/measurement.h"

IteratedEkf::IteratedEkf(const NoiseParams& noise, const IekfConfig& cfg,
                         const PlaneAssocParams& assoc, const State& x0,
                         const ErrorMatrix& P0,
                         const LocalMapParams& map_params)
    : map_(map_params),
      x_(x0),
      P_(P0),
      propagator_(noise),
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

  // World-from-lidar for a given state: T_WL = T_WI . T_imu_lidar. The
  // extrinsic is part of the state now, so this changes between iterations.
  const auto world_from_lidar = [](const State& x) {
    return Sophus::SE3d(x.R * x.R_imu_lidar, x.R * x.p_imu_lidar + x.p);
  };

  // ── Update: re-associate against the live iterate each iteration
  // ────────────
  // Association runs in the LIDAR frame (raw points + T_world_lidar), so the
  // points need no per-iteration transform and each PlaneMatch reports p_L,
  // which the extrinsic Jacobian blocks need directly.
  const MeasurementFn measure = [&](const State& x) {
    const Sophus::SE3f T_WL = world_from_lidar(x).cast<float>();
    return build_measurement(
        x, associate_planes(map_.tree(), points_lidar, T_WL, assoc_));
  };
  const EkfResult result = iterated_update(x_, P_, measure, cfg_);
  x_ = result.state;
  P_ = result.covariance;

  // ── Map: fold in the registered scan, then bound to the sliding window
  // ──────
  const Sophus::SE3d T_WL = world_from_lidar(x_);
  std::vector<Eigen::Vector3f> world;
  world.reserve(points_lidar.size());
  for (const Eigen::Vector3f& p_L : points_lidar) {
    world.push_back((T_WL * p_L.cast<double>()).cast<float>());
  }
  map_.insert(std::move(world));
  map_.recenter(x_.p.cast<float>());

  return result;
}

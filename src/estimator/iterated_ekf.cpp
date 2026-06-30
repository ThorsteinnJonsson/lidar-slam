#include "estimator/iterated_ekf.h"

#include "estimator/measurement.h"

IteratedEkf::IteratedEkf(const NoiseParams& noise,
                         const Sophus::SE3d& T_imu_lidar, const IekfConfig& cfg,
                         const PlaneAssocParams& assoc, const State& x0,
                         const Eigen::Matrix<double, 18, 18>& P0,
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
  const MeasurementFn measure = [&](const State& x) {
    const Sophus::SE3f T_WI(x.R.cast<float>(), x.p.cast<float>());
    return build_measurement(
        x, associate_planes(map_.tree(), points_imu, T_WI, assoc_));
  };
  const EkfResult result = iterated_update(x_, P_, measure, cfg_);
  x_ = result.state;
  P_ = result.covariance;

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

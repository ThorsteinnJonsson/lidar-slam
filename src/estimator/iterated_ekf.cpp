#include "estimator/iterated_ekf.h"

#include <cmath>

#include "estimator/measurement.h"

bool box_needs_slide(const Eigen::Vector3f& center, const Eigen::Vector3f& lo,
                     const Eigen::Vector3f& hi, float margin) {
  for (int a = 0; a < 3; ++a) {
    if (center[a] - lo[a] < margin || hi[a] - center[a] < margin) return true;
  }
  return false;
}

void crop_to_box(IkdTree& tree, const Eigen::Vector3f& lo,
                 const Eigen::Vector3f& hi) {
  constexpr float kBig = 1e9f;
  const Eigen::Vector3f big = Eigen::Vector3f::Constant(kBig);
  for (int a = 0; a < 3; ++a) {
    // Slab below lo[a]: full extent on the other axes, capped just under lo[a]
    // so points exactly on the kept boundary survive.
    Eigen::Vector3f below_max = big;
    below_max[a] = std::nextafter(lo[a], -kBig);
    tree.remove_box(-big, below_max);
    // Slab above hi[a].
    Eigen::Vector3f above_min = -big;
    above_min[a] = std::nextafter(hi[a], kBig);
    tree.remove_box(above_min, big);
  }
}

IteratedEkf::IteratedEkf(const NoiseParams& noise,
                         const Sophus::SE3d& T_imu_lidar, const IekfConfig& cfg,
                         const PlaneAssocParams& assoc, const State& x0,
                         const Eigen::Matrix<double, 18, 18>& P0,
                         const MapCropParams& crop)
    : x_(x0),
      P_(P0),
      propagator_(noise),
      T_imu_lidar_(T_imu_lidar),
      cfg_(cfg),
      assoc_(assoc),
      crop_(crop) {}

void IteratedEkf::crop_map() {
  if (!crop_.enabled) return;
  const Eigen::Vector3f center = x_.p.cast<float>();
  const Eigen::Vector3f half =
      Eigen::Vector3f::Constant(crop_.keep_half_extent);
  if (!box_initialized_) {
    box_min_ = center - half;
    box_max_ = center + half;
    box_initialized_ = true;
    return;  // box just seated; nothing outside it yet
  }
  if (!box_needs_slide(center, box_min_, box_max_, crop_.slide_margin)) return;
  box_min_ = center - half;
  box_max_ = center + half;
  crop_to_box(map_, box_min_, box_max_);
}

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
    return build_measurement(x,
                             associate_planes(map_, points_imu, T_WI, assoc_));
  };
  const EkfResult result = iterated_update(x_, P_, measure, cfg_);
  x_ = result.state;
  P_ = result.covariance;

  // ── Map: insert the registered scan in world frame
  // ──────────────────────────
  const Sophus::SE3d T_WI(x_.R, x_.p);
  std::vector<Eigen::Vector3f> world;
  world.reserve(points_imu.size());
  for (const Eigen::Vector3f& p_I : points_imu) {
    world.push_back((T_WI * p_I.cast<double>()).cast<float>());
  }
  if (map_.size() == 0) {
    map_.build(std::move(world));
  } else {
    map_.insert(world);
  }

  // ── Map: bound to the local sliding-window cube
  // ──────────────────────────────
  crop_map();

  return result;
}

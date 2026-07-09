#include "estimator/iterated_ekf.h"

#include <algorithm>
#include <cmath>
#include <numbers>

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

  // Prior (post-predict, pre-update) uncertainty in the weakly-observed
  // roll/pitch and gravity directions (temp onset-burst diagnostic).
  const double prior_att_rp = std::sqrt(P_(0, 0) + P_(1, 1));
  const double prior_grav = std::sqrt(P_(15, 15) + P_(16, 16));

  // Lidar points into the IMU frame once; the extrinsic is fixed, so only the
  // pose changes between iterations.
  std::vector<Eigen::Vector3f> points_imu;
  points_imu.reserve(points_lidar.size());
  const Sophus::SE3f T_il = T_imu_lidar_.cast<float>();
  for (const Eigen::Vector3f& p_L : points_lidar) {
    points_imu.push_back(T_il * p_L);
  }

  // Pre-update inconsistency at the propagated prior (temp onset probe): fit
  // the predicted+deskewed scan to the map before the update moves anything.
  double prior_medres = 0.0;
  double prior_vbias = 0.0;
  {
    const Sophus::SE3f T_prior(x_.R.cast<float>(), x_.p.cast<float>());
    const std::vector<PlaneMatch> pm =
        associate_planes(map_.tree(), points_imu, T_prior, assoc_);
    if (!pm.empty()) {
      std::vector<double> ar;
      ar.reserve(pm.size());
      double vb = 0.0;
      for (const PlaneMatch& m : pm) {
        ar.push_back(std::abs(static_cast<double>(m.residual)));
        vb += static_cast<double>(m.normal.z() * m.residual);
      }
      const size_t mid = ar.size() / 2;
      std::nth_element(ar.begin(), ar.begin() + mid, ar.end());
      prior_medres = ar[mid];
      prior_vbias = vb / static_cast<double>(pm.size());
    }
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

  constexpr double kRadToDeg = 180.0 / std::numbers::pi;
  result.prior_att_rp_std_deg = prior_att_rp * kRadToDeg;
  result.prior_grav_std_deg = prior_grav * kRadToDeg;
  result.prior_medres = prior_medres;
  result.prior_vbias = prior_vbias;

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

    // Plane population that constrains roll/pitch (temp): near-vertical normals
    // (ground/ceiling) and the normal-weighted mean residual.
    int n_vert = 0;
    double vbias = 0.0;
    for (const PlaneMatch& m : last_matches) {
      if (std::abs(m.normal.z()) > 0.8f) ++n_vert;
      vbias += static_cast<double>(m.normal.z() * m.residual);
    }
    const double n_match = static_cast<double>(last_matches.size());
    result.vert_normal_frac = static_cast<double>(n_vert) / n_match;
    result.vert_resid_bias = vbias / n_match;
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

#include "estimator/iterated_ekf.h"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <sophus/se3.hpp>
#include <vector>

#include "imu/state.h"
#include "types.h"

namespace {

using Vec3 = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;
using Matrix17 = Eigen::Matrix<double, 17, 17>;

// Six faces of an axis-aligned room sampled on a regular grid. The normals span
// all three axes, so a single scan constrains the full 6-DOF pose.
std::vector<Vec3> room_points() {
  std::vector<Vec3> pts;
  const double lo = -4.0, hi = 4.0, step = 1.0, wall = 5.0;
  for (double a = lo; a <= hi + 1e-9; a += step) {
    for (double b = lo; b <= hi + 1e-9; b += step) {
      pts.emplace_back(wall, a, b);   // +x
      pts.emplace_back(-wall, a, b);  // -x
      pts.emplace_back(a, wall, b);   // +y
      pts.emplace_back(a, -wall, b);  // -y
      pts.emplace_back(a, b, wall);   // +z
      pts.emplace_back(a, b, -wall);  // -z
    }
  }
  return pts;
}

// The fixed world room as seen from the lidar at pose T_WI (extrinsic = I, so
// the lidar frame coincides with the IMU frame).
std::vector<Vec3f> scan_from(const std::vector<Vec3>& world,
                             const Sophus::SE3d& T_WI) {
  const Sophus::SE3d T_IW = T_WI.inverse();
  std::vector<Vec3f> out;
  out.reserve(world.size());
  for (const Vec3& p_W : world) out.push_back((T_IW * p_W).cast<float>());
  return out;
}

ImuMeasurement imu_at(uint64_t t_ns, const Vec3& gyro, const Vec3& accel) {
  ImuMeasurement m;
  m.stamp = Timestamp::from_nsec(t_ns);
  m.angular_velocity = gyro;
  m.linear_acceleration = accel;
  return m;
}

double pose_err(const State& a, const Sophus::SE3d& T) {
  const Vec3 dr = (T.so3().inverse() * a.R).log();
  const Vec3 dp = a.p - T.translation();
  return std::sqrt(dr.squaredNorm() + dp.squaredNorm());
}

NoiseParams test_noise() {
  NoiseParams n;
  n.gyro_noise_std = 1e-3;
  n.accel_noise_std = 1e-2;
  n.gyro_rw_std = 1e-4;
  n.accel_rw_std = 1e-4;
  return n;
}

State origin_state() {
  State s;
  s.gravity = Vec3(0.0, 0.0, -9.81);
  return s;
}

}  // namespace

TEST(IteratedEkf, SeatsMapOnFirstScan) {
  const auto world = room_points();
  IteratedEkf ekf(test_noise(), Sophus::SE3d(), IekfConfig{},
                  PlaneAssocParams{}, origin_state(),
                  Matrix17::Identity() * 0.01);

  const auto scan = scan_from(world, Sophus::SE3d());
  const std::vector<ImuMeasurement> imu = {
      imu_at(0, Vec3::Zero(), Vec3(0, 0, 9.81))};
  const auto r = ekf.process_scan(imu, scan);

  // Empty map yields no correspondences, so the update is a no-op and the scan
  // just seats the map.
  EXPECT_EQ(r.iterations, 0);
  EXPECT_GT(ekf.map().size(), 0u);
  EXPECT_LT(pose_err(ekf.state(), Sophus::SE3d()), 1e-9);
}

TEST(IteratedEkf, StaticSensorHoldsPose) {
  const auto world = room_points();
  IteratedEkf ekf(test_noise(), Sophus::SE3d(), IekfConfig{},
                  PlaneAssocParams{}, origin_state(),
                  Matrix17::Identity() * 0.01);

  const Sophus::SE3d T_true;  // identity, sensor does not move
  const auto scan = scan_from(world, T_true);
  const Vec3 gyro = Vec3::Zero();
  const Vec3 accel(0, 0, 9.81);  // specific force while holding still
  const uint64_t dt = 100'000'000;

  for (int i = 0; i < 20; ++i) {
    const std::vector<ImuMeasurement> imu = {
        imu_at(static_cast<uint64_t>(i) * dt, gyro, accel),
        imu_at(static_cast<uint64_t>(i + 1) * dt, gyro, accel)};
    ekf.process_scan(imu, scan);
    EXPECT_LT(pose_err(ekf.state(), T_true), 1e-3);
  }
}

TEST(IteratedEkf, ConstantVelocityTracksTruth) {
  const auto world = room_points();
  const Vec3 v_true(0.2, 0.0, 0.0);

  // Pose is initialized at truth (so the map seats in the truth frame) but the
  // velocity is seeded wrong, so the prediction drifts and the update has to
  // pull it back and recover the velocity through the covariance cross-terms.
  State x0 = origin_state();
  x0.v = Vec3(0.1, 0.0, 0.0);
  IteratedEkf ekf(test_noise(), Sophus::SE3d(), IekfConfig{},
                  PlaneAssocParams{}, x0, Matrix17::Identity() * 0.1);

  const Vec3 gyro = Vec3::Zero();
  const Vec3 accel(0, 0, 9.81);  // constant-velocity specific force
  const uint64_t dt = 100'000'000;
  const double dts = 0.1;
  const int N = 20;

  double max_err = 0.0;
  for (int i = 0; i < N; ++i) {
    const Sophus::SE3d T_true(Sophus::SO3d(), v_true * (dts * i));
    const auto scan = scan_from(world, T_true);

    std::vector<ImuMeasurement> imu;
    if (i == 0) {
      imu = {imu_at(0, gyro, accel)};  // no predict on the first scan
    } else {
      imu = {imu_at(static_cast<uint64_t>(i - 1) * dt, gyro, accel),
             imu_at(static_cast<uint64_t>(i) * dt, gyro, accel)};
    }
    ekf.process_scan(imu, scan);

    const double e = pose_err(ekf.state(), T_true);
    if (i >= 1) max_err = std::max(max_err, e);
  }

  // The error stays bounded across the run (no accumulating drift)...
  EXPECT_LT(max_err, 0.1);
  // ...and the filter has converged onto the trajectory by the end.
  const Sophus::SE3d T_final(Sophus::SO3d(), v_true * (dts * (N - 1)));
  EXPECT_LT(pose_err(ekf.state(), T_final), 0.02);
}

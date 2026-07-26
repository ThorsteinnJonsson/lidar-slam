#include "pipeline/slam_pipeline.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <sophus/se3.hpp>
#include <vector>

#include "state/state.h"
#include "types.h"

namespace {

using Vec3 = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;

constexpr uint64_t kImuDtNs = 5'000'000;  // 200 Hz
constexpr size_t kInitCount = 60;
const Vec3 kGravityUp(0, 0, 9.81);  // specific force at rest, gravity = -9.81 z

// Six walls of an axis-aligned room; normals span all axes so a scan constrains
// the full pose.
std::vector<Vec3> room_points() {
  std::vector<Vec3> pts;
  for (double a = -4.0; a <= 4.0 + 1e-9; a += 1.0) {
    for (double b = -4.0; b <= 4.0 + 1e-9; b += 1.0) {
      pts.emplace_back(5.0, a, b);
      pts.emplace_back(-5.0, a, b);
      pts.emplace_back(a, 5.0, b);
      pts.emplace_back(a, -5.0, b);
      pts.emplace_back(a, b, 5.0);
      pts.emplace_back(a, b, -5.0);
    }
  }
  return pts;
}

std::vector<Vec3f> scan_from(const std::vector<Vec3>& world,
                             const Sophus::SE3d& T_WI) {
  const Sophus::SE3d T_IW = T_WI.inverse();
  std::vector<Vec3f> out;
  out.reserve(world.size());
  for (const Vec3& p_W : world) out.push_back((T_IW * p_W).cast<float>());
  return out;
}

ImuMeasurement imu_at(uint64_t t_ns, const Vec3& accel) {
  ImuMeasurement m;
  m.stamp = Timestamp::from_nsec(t_ns);
  m.angular_velocity = Vec3::Zero();
  m.linear_acceleration = accel;
  return m;
}

// A scan stamped at `stamp_ns`, with per-point times spread over `span_ns`.
PointCloud make_scan(const std::vector<Vec3f>& pts, uint64_t stamp_ns,
                     uint64_t span_ns) {
  PointCloud pc;
  pc.stamp = Timestamp::from_nsec(stamp_ns);
  pc.reserve(pts.size());
  for (size_t i = 0; i < pts.size(); ++i) {
    pc.x.push_back(pts[i].x());
    pc.y.push_back(pts[i].y());
    pc.z.push_back(pts[i].z());
    pc.intensity.push_back(0.0f);
    pc.ring.push_back(0);
    pc.t_offset_ns.push_back(static_cast<uint32_t>(span_ns * i / pts.size()));
  }
  return pc;
}

PipelineConfig test_config() {
  PipelineConfig c;
  c.noise.gyro_noise_std = 1e-3;
  c.noise.accel_noise_std = 1e-2;
  c.noise.gyro_rw_std = 1e-4;
  c.noise.accel_rw_std = 1e-4;
  c.init.gravity_magnitude = 9.81;  // match the fed specific force
  c.init_imu_count = kInitCount;    // identity extrinsic by default
  return c;
}

double pose_err(const State& a, const Sophus::SE3d& T) {
  const Vec3 dr = (T.so3().inverse() * a.R).log();
  const Vec3 dp = a.p - T.translation();
  return std::sqrt(dr.squaredNorm() + dp.squaredNorm());
}

}  // namespace

TEST(SlamPipeline, InitializesAfterStaticWindowAndDropsEarlyScans) {
  SlamPipeline p(test_config());

  // A scan before initialization is dropped; nothing to take.
  p.add_scan(make_scan(scan_from(room_points(), Sophus::SE3d()), 0, kImuDtNs));
  EXPECT_TRUE(p.take_ready().empty());
  EXPECT_FALSE(p.initialized());

  uint64_t t = 0;
  std::optional<InitResult> init;
  for (size_t i = 0; i < kInitCount; ++i) {
    if (auto r = p.add_imu(imu_at(t, kGravityUp))) init = r;
    t += kImuDtNs;
  }
  EXPECT_TRUE(p.initialized());
  ASSERT_TRUE(init.has_value());
  EXPECT_TRUE(init->ok);
}

TEST(SlamPipeline, StaticSensorHoldsPose) {
  SlamPipeline p(test_config());
  const auto world = room_points();
  const auto scan_pts =
      scan_from(world, Sophus::SE3d());  // identity, stationary

  uint64_t t = 0;
  for (size_t i = 0; i < kInitCount; ++i) {
    p.add_imu(imu_at(t, kGravityUp));
    t += kImuDtNs;
  }
  ASSERT_TRUE(p.initialized());

  size_t processed = 0;
  for (int k = 0; k < 20; ++k) {
    const uint64_t scan_start = t;
    for (int j = 0; j < 20; ++j) {  // ~100 ms of IMU covering the scan
      p.add_imu(imu_at(t, kGravityUp));
      t += kImuDtNs;
    }
    const uint64_t latest = t - kImuDtNs;
    p.add_scan(make_scan(scan_pts, scan_start, latest - scan_start));
    processed += p.take_ready().size();
  }

  EXPECT_GT(processed, 0u);
  EXPECT_EQ(p.scans(), processed);
  EXPECT_GT(p.map_size(), 0u);
  // A stationary sensor must stay at the origin it initialized to.
  EXPECT_LT(pose_err(p.state(), Sophus::SE3d()), 0.05);
}

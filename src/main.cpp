#include <spdlog/spdlog.h>

#include <algorithm>
#include <deque>
#include <fstream>
#include <optional>
#include <vector>

#include "estimator/iterated_ekf.h"
#include "imu/buffer.h"
#include "imu/initializer.h"
#include "imu/propagator.h"
#include "imu/state.h"
#include "io/bag_reader.h"
#include "io/calibration.h"
#include "io/ros_deserializer.h"
#include "map/association.h"
#include "preprocess/deskew.h"
#include "preprocess/voxel_grid.h"
#include "types.h"

namespace {

constexpr size_t kInitImuCount = 200;  // static IMU samples used to initialize
constexpr double kVoxelLeaf = 0.5;     // map/scan downsample leaf size (m)

// Scan time window [t_start, t_end] from the per-point time offsets.
std::optional<std::pair<uint64_t, uint64_t>> scan_window(const PointCloud& c) {
  if (c.t_offset_ns.empty()) return std::nullopt;
  const auto [lo, hi] =
      std::minmax_element(c.t_offset_ns.begin(), c.t_offset_ns.end());
  const uint64_t base = c.stamp.to_nsec();
  return std::make_pair(base + *lo, base + *hi);
}

std::vector<Eigen::Vector3f> to_points(const PointCloud& c) {
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(c.size());
  for (size_t i = 0; i < c.size(); ++i)
    pts.emplace_back(c.x[i], c.y[i], c.z[i]);
  return pts;
}

}  // namespace

int main() {
  spdlog::flush_on(spdlog::level::info);  // flush progress as it is logged
  spdlog::info("lidar-slam initializing...");

  const std::string dataset = "datasets/ntu_viral/eee_03";
  const auto imu_cal = load_imu_calibration(dataset + "/imu_v100.yaml");
  const auto lidar_cal = load_lidar_calibration(dataset + "/lidar_horz.yaml");
  const Sophus::SE3d T_imu_lidar = imu_from_lidar(imu_cal, lidar_cal);

  spdlog::info("IMU  topic: {}, LiDAR topic: {}", imu_cal.topic,
               lidar_cal.topic);

  const NoiseParams noise{.gyro_noise_std = imu_cal.gyro_noise_std,
                          .accel_noise_std = imu_cal.accel_noise_std,
                          .gyro_rw_std = imu_cal.gyro_rw_std,
                          .accel_rw_std = imu_cal.accel_rw_std};

  BagReader reader(dataset + "/eee_03.bag");

  ImuBuffer imu_buffer;
  std::deque<PointCloud> pending;        // clouds awaiting IMU coverage
  std::vector<ImuMeasurement> init_imu;  // first samples, for static init
  std::optional<IteratedEkf> ekf;
  uint64_t last_ref = 0;  // scan-end of the previously processed scan
  size_t scans = 0;

  std::ofstream traj_out("trajectory.tum");

  // Process every pending scan whose IMU window [last_ref, t_end] is buffered.
  const auto drain = [&] {
    while (!pending.empty()) {
      const PointCloud& cloud = pending.front();
      const auto window = scan_window(cloud);
      if (!window) {
        pending.pop_front();
        continue;
      }
      const auto [t_start, t_end] = *window;
      if (!imu_buffer.covers(last_ref, t_end)) break;  // need more IMU

      const auto traj = build_scan_trajectory(imu_buffer, t_start, t_end);
      const PointCloud undist = deskew(cloud, traj, T_imu_lidar, t_end);
      const PointCloud filtered = voxel_downsample(undist, kVoxelLeaf);

      const auto imu_window = imu_buffer.get_between(last_ref, t_end);
      const EkfResult r = ekf->process_scan(imu_window, to_points(filtered));
      last_ref = t_end;
      ++scans;

      const Eigen::Vector3d& p = ekf->state().p;
      const Eigen::Quaterniond q = ekf->state().R.unit_quaternion();
      traj_out << Timestamp::from_nsec(t_end).to_sec() << ' ' << p.x() << ' '
               << p.y() << ' ' << p.z() << ' ' << q.x() << ' ' << q.y() << ' '
               << q.z() << ' ' << q.w() << '\n';

      if (scans % 100 == 0) {
        spdlog::info(
            "scan {:5} | pos [{:7.2f},{:7.2f},{:7.2f}] | map {:6} pts | "
            "iters {} {}",
            scans, p.x(), p.y(), p.z(), ekf->map().size(), r.iterations,
            r.converged ? "conv" : "");
      }
      pending.pop_front();
    }
  };

  reader.read_messages(
      {imu_cal.topic, lidar_cal.topic},
      [&](const std::string& topic, uint64_t /*stamp_ns*/,
          std::span<const std::byte> data) {
        if (topic == imu_cal.topic) {
          const ImuMeasurement m = deserialize_imu(data);
          imu_buffer.push(m);
          if (!ekf) {
            init_imu.push_back(m);
            if (init_imu.size() >= kInitImuCount) {
              const InitResult init = initialize_static(init_imu, InitParams{});
              if (init.ok) {
                spdlog::info(
                    "Static init OK: |accel| = {:.4f} m/s^2, accel_var = "
                    "{:.2e}, gyro_var = {:.2e}, b_g = [{:.2e}, {:.2e}, {:.2e}]",
                    init.accel_mean_norm, init.max_accel_var, init.max_gyro_var,
                    init.state.b_g.x(), init.state.b_g.y(), init.state.b_g.z());
                ekf.emplace(noise, T_imu_lidar, IekfConfig{},
                            PlaneAssocParams{}, init.state, init.cov);
                last_ref = init_imu.back().stamp.to_nsec();
              } else {
                // Platform is moving: slide the window and keep waiting for a
                // stationary stretch.
                init_imu.erase(init_imu.begin());
              }
            }
          }
        } else if (topic == lidar_cal.topic &&
                   ekf) {  // drop clouds that arrive before initialization
          pending.push_back(deserialize_pointcloud2(data));
        }
        if (ekf) drain();
      });

  spdlog::info("Done. Processed {} scans, final map {} points.", scans,
               ekf ? ekf->map().size() : 0u);
  return 0;
}

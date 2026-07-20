#include <spdlog/spdlog.h>

#include <algorithm>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <string_view>
#include <vector>

#include "estimator/iterated_ekf.h"
#include "imu/buffer.h"
#include "imu/initializer.h"
#include "imu/propagator.h"
#include "io/bag_reader.h"
#include "io/calibration.h"
#include "io/ros_deserializer.h"
#include "map/association.h"
#include "preprocess/deskew.h"
#include "preprocess/voxel_grid.h"
#include "state/state.h"
#include "types.h"
#include "viz/visualizer.h"

namespace {

constexpr size_t kInitImuCount = 200;  // static IMU samples used to initialize
constexpr double kVoxelLeaf = 0.5;     // map/scan downsample leaf size (m)

// Lidar-to-IMU clock offset (seconds). The OS1 lidar stamps its points ahead
// of the IMU clock on NTU VIRAL;
// TODO: promote to a per-dataset calibration parameter instead of hard-coding.
constexpr double kLidarTimeOffsetSec = -0.1;
constexpr int64_t kLidarTimeOffsetNs =
    static_cast<int64_t>(kLidarTimeOffsetSec * 1e9);

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

  // NTU VIRAL sequence to run. Hard-coded for now (later: command-line arg).
  // Options: eee_02 | eee_03 | rtp_03 | tnp_01.
  constexpr std::string_view kSequence = "eee_03";
  const std::string dataset = "datasets/ntu_viral/" + std::string(kSequence);

  // rtp_03 and tnp_01 ship no imu_v100.yaml. Across NTU VIRAL the IMU topic and
  // body-IMU extrinsic are constant ("/imu/imu", identity), and the noise is
  // hard-coded below rather than read from the file, so a default calibration
  // covers the sequences that lack the yaml.
  const std::filesystem::path imu_yaml = dataset + "/imu_v100.yaml";
  const ImuCalibration imu_cal = std::filesystem::exists(imu_yaml)
                                     ? load_imu_calibration(imu_yaml)
                                     : ImuCalibration{.topic = "/imu/imu"};
  const auto lidar_cal = load_lidar_calibration(dataset + "/lidar_horz.yaml");
  const auto prism_cal = load_prism_calibration(dataset + "/leica_prism.yaml");
  const Sophus::SE3d T_imu_lidar = imu_from_lidar(imu_cal, lidar_cal);

  // Prism position in the IMU frame: the lever arm to apply to our estimate so
  // it can be compared against the Leica prism ground truth.
  const Eigen::Vector3d t_imu_prism =
      (imu_cal.T_body_imu.inverse() * prism_cal.T_body_prism).translation();

  spdlog::info("IMU  topic: {}, LiDAR topic: {}, GT topic: {}", imu_cal.topic,
               lidar_cal.topic, prism_cal.topic);

  // FAST-LIO2 tuning parity: deliberately LOOSE process noise (gyr_cov/acc_cov
  // 0.1, b_gyr_cov/b_acc_cov 1e-4, stored as std = sqrt(cov)) instead of the
  // datasheet values from imu_v100.yaml. The datasheet Q makes the filter
  // overconfident: the covariance stays so tight that the takeoff registration
  // misfit gets absorbed as attitude/gravity error and then locks in as a
  // permanent ~5 deg map tilt. Loose Q lets the bias and attitude keep
  // re-adapting, so the same transient heals: final gravity tilt 5.2 -> 3.6 deg
  // and ATE 0.32 -> 0.11 on eee_03.
  // TODO: bisect which of the loosened terms carries the improvement, and
  // consider scaling the datasheet values instead of replacing them.
  const NoiseParams noise{.gyro_noise_std = 0.3162,   // sqrt(0.1)
                          .accel_noise_std = 0.3162,  // sqrt(0.1)
                          .gyro_rw_std = 0.01,        // sqrt(1e-4)
                          .accel_rw_std = 0.01};      // sqrt(1e-4)

  BagReader reader(dataset + "/" + std::string(kSequence) + ".bag");

  ImuBuffer imu_buffer;
  std::deque<PointCloud> pending;        // clouds awaiting IMU coverage
  std::vector<ImuMeasurement> init_imu;  // first samples, for static init
  std::optional<IteratedEkf> ekf;
  uint64_t last_ref = 0;  // scan-end of the previously processed scan
  size_t scans = 0;
  size_t converged_scans = 0;  // scans whose iEKF hit convergence_tol
  size_t total_iters = 0;      // iEKF iterations summed over all scans
  size_t gt_msgs = 0;

  // Trajectory and ground-truth outputs (TUM format) for offline evaluation.
  // Per-sequence subdir so running another sequence does not clobber the last.
  const std::filesystem::path eval_dir =
      std::filesystem::path("evaluation") / std::string(kSequence);
  std::filesystem::create_directories(eval_dir);
  std::ofstream traj_out(eval_dir / "trajectory.tum");  // IMU pose
  std::ofstream prism_out(eval_dir / "prism.tum");      // estimate at the prism
  std::ofstream gt_out(eval_dir / "gt.tum");            // Leica prism GT
  // Fixed, 9-decimal precision so the ~1.6e9 timestamps keep nanosecond
  // resolution (the default 6 significant digits would destroy them).
  for (std::ofstream* os : {&traj_out, &prism_out, &gt_out})
    *os << std::fixed << std::setprecision(9);

  // Live visualization stream (no-op unless built with
  // LIDAR_SLAM_ENABLE_RERUN).
  Visualizer viz;

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

      const auto traj =
          build_scan_trajectory(imu_buffer, t_start, t_end, ekf->state());
      const PointCloud undist = deskew(cloud, traj, T_imu_lidar, t_end);
      const PointCloud filtered = voxel_downsample(undist, kVoxelLeaf);

      const auto imu_window = imu_buffer.get_between(last_ref, t_end);
      const std::vector<Eigen::Vector3f> scan_lidar = to_points(filtered);
      const EkfResult r = ekf->process_scan(imu_window, scan_lidar);
      last_ref = t_end;
      ++scans;
      converged_scans += r.converged ? 1 : 0;
      total_iters += static_cast<size_t>(r.iterations);

      const Eigen::Vector3d& p = ekf->state().p;
      const Eigen::Quaterniond q = ekf->state().R.unit_quaternion();
      // Write the corrected (offset-shifted) stamp: after the lidar<->IMU time
      // offset the scan-end time is on the IMU/GT reference clock, so this is
      // the pose's true time for GT association.
      const double t_sec = Timestamp::from_nsec(t_end).to_sec();
      traj_out << t_sec << ' ' << p.x() << ' ' << p.y() << ' ' << p.z() << ' '
               << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';

      // Estimate at the prism (lever arm applied), for prism-to-prism ATE.
      // Orientation is irrelevant to position-only GT, so write identity.
      const Eigen::Vector3d p_prism = p + ekf->state().R * t_imu_prism;
      prism_out << t_sec << ' ' << p_prism.x() << ' ' << p_prism.y() << ' '
                << p_prism.z() << " 0 0 0 1\n";

      if (scans % 100 == 0) {
        spdlog::info(
            "scan {:5} | pos [{:7.2f},{:7.2f},{:7.2f}] | map {:6} pts | "
            "iters {} {}",
            scans, p.x(), p.y(), p.z(), ekf->map().size(), r.iterations,
            r.converged ? "conv" : "");
      }

      // Live visualization (no-op unless built with Rerun). Transform the scan
      // to world with the post-update pose; log the full map periodically since
      // it is large.
      const Sophus::SE3d T_wi(ekf->state().R, ekf->state().p);
      const Sophus::SE3d T_wl = T_wi * T_imu_lidar;
      std::vector<Eigen::Vector3f> scan_world;
      scan_world.reserve(scan_lidar.size());
      for (const Eigen::Vector3f& p_l : scan_lidar) {
        scan_world.push_back((T_wl * p_l.cast<double>()).cast<float>());
      }
      viz.set_scan(static_cast<int64_t>(scans));
      viz.log_scan(scan_world);
      viz.log_pose(T_wi);
      if (scans % 50 == 0) viz.log_map(ekf->map());

      pending.pop_front();
    }
  };

  reader.read_messages(
      {imu_cal.topic, lidar_cal.topic, prism_cal.topic},
      [&](const std::string& topic, uint64_t /*stamp_ns*/,
          std::span<const std::byte> data) {
        if (topic == prism_cal.topic) {
          // Ground truth: independent of the filter, dump every message.
          const PoseStampedMsg gt = deserialize_pose_stamped(data);
          gt_out << gt.stamp.to_sec() << ' ' << gt.position.x() << ' '
                 << gt.position.y() << ' ' << gt.position.z() << " 0 0 0 1\n";
          ++gt_msgs;
          return;
        }
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
                // FAST-LIO2 tuning parity: lidar point noise sigma =
                // sqrt(LASER_POINT_COV = 1e-3) ~ 0.0316 m, 10x less information
                // per point than the 0.01 default. Part of the loose-tuning fix
                // for the takeoff gravity tilt (see NoiseParams above).
                IekfConfig ekf_cfg;
                ekf_cfg.sigma = 0.0316;
                ekf.emplace(noise, T_imu_lidar, ekf_cfg, PlaneAssocParams{},
                            init.state, init.cov);
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
          PointCloud pc = deserialize_pointcloud2(data);
          // Shift the scan stamp onto the IMU clock by the lidar-to-IMU time
          // offset, so scan_window, trajectory, deskew, and the IMU window all
          // move together.
          pc.stamp = Timestamp::from_nsec(static_cast<uint64_t>(
              static_cast<int64_t>(pc.stamp.to_nsec()) + kLidarTimeOffsetNs));
          pending.push_back(std::move(pc));
        }
        if (ekf) drain();
      });

  // Final full map at the last scan index.
  if (ekf) {
    viz.set_scan(static_cast<int64_t>(scans));
    viz.log_map(ekf->map());
  }

  spdlog::info(
      "Done. Processed {} scans, final map {} points, {} GT poses written.",
      scans, ekf ? ekf->map().size() : 0u, gt_msgs);
  if (scans > 0) {
    spdlog::info(
        "iEKF: {}/{} scans converged ({:.1f}%), mean {:.2f} iters/scan.",
        converged_scans, scans,
        100.0 * static_cast<double>(converged_scans) /
            static_cast<double>(scans),
        static_cast<double>(total_iters) / static_cast<double>(scans));
  }
  return 0;
}

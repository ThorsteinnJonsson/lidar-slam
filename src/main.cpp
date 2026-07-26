#include <spdlog/spdlog.h>

#include <CLI/CLI.hpp>
#include <deque>
#include <filesystem>
#include <memory>
#include <numbers>
#include <optional>
#include <vector>

#include "estimator/iterated_ekf.h"
#include "imu/buffer.h"
#include "imu/initializer.h"
#include "imu/propagator.h"
#include "io/bag_reader.h"
#include "io/dataset_loader.h"
#include "io/params.h"
#include "io/result_writer.h"
#include "io/ros_deserializer.h"
#include "map/association.h"
#include "preprocess/deskew.h"
#include "preprocess/voxel_grid.h"
#include "state/state.h"
#include "types.h"
#include "viz/visualizer.h"

namespace {

struct RunConfig {
  std::string format;
  std::filesystem::path sequence_dir;
  std::filesystem::path params_path;
  std::filesystem::path output_dir;  // defaulted to evaluation/<sequence>
};

// Parse the command line. Returns nullopt when the program should exit
// immediately (--help or a parse error); `exit_code` then holds the return
// code.
std::optional<RunConfig> parse_args(int argc, char** argv, int& exit_code) {
  CLI::App app{"LiDAR-inertial SLAM (FAST-LIO2 style)"};
  RunConfig cfg;
  app.add_option("--format", cfg.format, "Dataset format")
      ->required()
      ->check(CLI::IsMember({"NTU_VIRAL", "HILTI_22", "FAST_LIVO2"}));
  app.add_option("--sequence", cfg.sequence_dir,
                 "Path to the sequence directory")
      ->required()
      ->check(CLI::ExistingDirectory);
  app.add_option("--params", cfg.params_path, "Path to the parameter YAML")
      ->required()
      ->check(CLI::ExistingFile);
  app.add_option("--output", cfg.output_dir,
                 "Directory for TUM output (default: evaluation/<sequence>)");
  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    exit_code = app.exit(e);
    return std::nullopt;
  }
  if (cfg.output_dir.empty())
    cfg.output_dir =
        std::filesystem::path("evaluation") / cfg.sequence_dir.filename();
  return cfg;
}

}  // namespace

int main(int argc, char** argv) {
  int exit_code = 0;
  const std::optional<RunConfig> cfg_opt = parse_args(argc, argv, exit_code);
  if (!cfg_opt) return exit_code;
  const RunConfig& cfg = *cfg_opt;

  spdlog::flush_on(spdlog::level::info);  // flush progress as it is logged
  spdlog::info("lidar-slam initializing...");

  std::vector<std::string> param_warnings;
  const Params params = load_params(cfg.params_path, &param_warnings);
  for (const std::string& w : param_warnings) spdlog::warn("{}", w);

  spdlog::info("Format: {}, sequence: {}, params: {}", cfg.format,
               cfg.sequence_dir.string(), cfg.params_path.string());

  const int64_t lidar_time_offset_ns =
      static_cast<int64_t>(params.lidar_time_offset_sec * 1e9);

  const std::unique_ptr<DatasetLoader> loader =
      make_loader(cfg.format, cfg.sequence_dir, params);
  const Sophus::SE3d T_imu_lidar = loader->T_imu_lidar();
  const Eigen::Vector3d t_imu_gt = loader->imu_to_gt_point();

  spdlog::info("IMU topic: {}, LiDAR topic: {}, GT topic: {}",
               loader->imu_topic(), loader->lidar_topic(),
               loader->gt_topic().empty() ? "(none)" : loader->gt_topic());

  BagReader reader(loader->bag_path());

  ImuBuffer imu_buffer;
  std::deque<PointCloud> pending;        // clouds awaiting IMU coverage
  std::vector<ImuMeasurement> init_imu;  // first samples, for static init
  std::optional<IteratedEkf> ekf;
  uint64_t last_ref = 0;  // scan-end of the previously processed scan
  size_t scans = 0;
  size_t converged_scans = 0;  // scans whose iEKF hit convergence_tol
  size_t total_iters = 0;      // iEKF iterations summed over all scans

  // TUM outputs for offline evaluation (see ResultWriter). External-file GT is
  // dumped once up front; topic-streamed GT flows through write_gt below.
  ResultWriter writer(cfg.output_dir, loader->has_gt(), t_imu_gt);
  size_t gt_msgs = 0;
  if (loader->has_gt() && loader->gt_topic().empty())
    gt_msgs = loader->write_external_gt(writer.gt_stream());

  // Live visualization stream (no-op unless built with
  // LIDAR_SLAM_ENABLE_RERUN).
  Visualizer viz;

  // Process every pending scan whose IMU window [last_ref, t_end] is buffered.
  const auto drain = [&] {
    while (!pending.empty()) {
      const PointCloud& cloud = pending.front();
      const auto window = scan_time_window(cloud);
      if (!window) {
        pending.pop_front();
        continue;
      }
      const auto [t_start, t_end] = *window;
      // Drop a scan that does not advance past the last one processed (Livox
      // publishes occasionally overlapping/out-of-order sweeps); otherwise the
      // [last_ref, t_end] IMU window would invert.
      if (last_ref != 0 && t_end <= last_ref) {
        pending.pop_front();
        continue;
      }
      if (!imu_buffer.covers(last_ref, t_end)) break;  // need more IMU

      const auto traj =
          build_scan_trajectory(imu_buffer, t_start, t_end, ekf->state());
      // Deskew with the current extrinsic estimate (seeded from YAML, refined
      // online) so it stays consistent with the update below.
      const Sophus::SE3d T_il_est(ekf->state().R_imu_lidar,
                                  ekf->state().p_imu_lidar);
      const PointCloud undist = deskew(cloud, traj, T_il_est, t_end);
      const PointCloud filtered =
          voxel_downsample(undist, params.scan_voxel_leaf);

      const auto imu_window = imu_buffer.get_between(last_ref, t_end);
      const std::vector<Eigen::Vector3f> scan_lidar = filtered.xyz();
      const EkfResult r = ekf->process_scan(imu_window, scan_lidar);
      last_ref = t_end;
      ++scans;
      converged_scans += r.converged ? 1 : 0;
      total_iters += static_cast<size_t>(r.iterations);

      // The offset-corrected scan-end time is on the IMU/GT reference clock, so
      // it is the pose's true time for GT association.
      const double t_sec = Timestamp::from_nsec(t_end).to_sec();
      writer.write_pose(t_sec, ekf->state());

      const Eigen::Vector3d& p = ekf->state().p;
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
      const Sophus::SE3d T_wl = T_wi * Sophus::SE3d(ekf->state().R_imu_lidar,
                                                    ekf->state().p_imu_lidar);
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

  std::vector<std::string> topics = {loader->imu_topic(),
                                     loader->lidar_topic()};
  if (!loader->gt_topic().empty()) topics.push_back(loader->gt_topic());

  reader.read_messages(
      topics, [&](const std::string& topic, uint64_t /*stamp_ns*/,
                  std::span<const std::byte> data) {
        if (topic == loader->gt_topic()) {
          // Ground truth: independent of the filter, dump every message.
          if (const std::optional<GtRow> gt = loader->decode_gt(data)) {
            writer.write_gt(*gt);
            ++gt_msgs;
          }
          return;
        }
        if (topic == loader->imu_topic()) {
          ImuMeasurement m = deserialize_imu(data);
          m.linear_acceleration *= loader->imu_accel_scale();
          imu_buffer.push(m);
          if (!ekf) {
            init_imu.push_back(m);
            if (init_imu.size() >= params.init_imu_count) {
              const InitResult init = initialize_static(init_imu, params.init);
              if (init.ok) {
                spdlog::info(
                    "Static init OK: |accel| = {:.4f} m/s^2, accel_var = "
                    "{:.2e}, gyro_var = {:.2e}, b_g = [{:.2e}, {:.2e}, {:.2e}]",
                    init.accel_mean_norm, init.max_accel_var, init.max_gyro_var,
                    init.state.b_g.x(), init.state.b_g.y(), init.state.b_g.z());
                // Seed the extrinsic from the YAML calibration.
                State x0 = init.state;
                x0.R_imu_lidar = T_imu_lidar.so3();
                x0.p_imu_lidar = T_imu_lidar.translation();
                ErrorMatrix P0 = init.cov;
                if (!params.enable_extrinsic_estimation) {
                  // Pin the extrinsic with a negligible prior variance: the
                  // update cannot move it and there is no process noise to
                  // reinflate it, so the filter behaves as if the calibration
                  // were fixed. Non-zero to keep P invertible.
                  P0.diagonal().segment<6>(kIdxExtRot).setConstant(1e-12);
                }
                ekf.emplace(params.noise, params.iekf, params.assoc, x0, P0,
                            params.map);
                last_ref = init_imu.back().stamp.to_nsec();
              } else {
                // Platform is moving: slide the window and keep waiting for a
                // stationary stretch.
                init_imu.erase(init_imu.begin());
              }
            }
          }
        } else if (topic == loader->lidar_topic() &&
                   ekf) {  // drop clouds that arrive before initialization
          PointCloud pc = loader->decode_cloud(data);
          // Shift the scan stamp onto the IMU clock by the lidar-to-IMU time
          // offset, so the scan window, trajectory, deskew, and the IMU window
          // all move together.
          pc.stamp = Timestamp::from_nsec(static_cast<uint64_t>(
              static_cast<int64_t>(pc.stamp.to_nsec()) + lidar_time_offset_ns));
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
    // How far the online extrinsic estimate drifted from the YAML seed. Large
    // translation drift suggests it is being driven by unobservable directions
    // rather than genuine miscalibration.
    const Eigen::Vector3d d_rot =
        (T_imu_lidar.so3().inverse() * ekf->state().R_imu_lidar).log();
    const Eigen::Vector3d d_trans =
        ekf->state().p_imu_lidar - T_imu_lidar.translation();
    spdlog::info(
        "Extrinsic: drot {:.3f} deg | dtrans {:.4f} m | p_il [{:+.4f}, "
        "{:+.4f}, {:+.4f}] (seed [{:+.4f}, {:+.4f}, {:+.4f}])",
        d_rot.norm() * 180.0 / std::numbers::pi, d_trans.norm(),
        ekf->state().p_imu_lidar.x(), ekf->state().p_imu_lidar.y(),
        ekf->state().p_imu_lidar.z(), T_imu_lidar.translation().x(),
        T_imu_lidar.translation().y(), T_imu_lidar.translation().z());
  }
  return 0;
}

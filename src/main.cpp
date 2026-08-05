#include <spdlog/spdlog.h>

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>
#include <numbers>
#include <optional>
#include <vector>

#include "estimator/iekf.h"
#include "imu/initializer.h"
#include "io/bag_reader.h"
#include "io/dataset_loader.h"
#include "io/params.h"
#include "io/result_writer.h"
#include "io/ros_deserializer.h"
#include "pipeline/slam_pipeline.h"
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

// ── Diagnostics ──────────────────────────────────────────────────────────────

void log_static_init(const InitResult& init) {
  spdlog::info(
      "Static init OK: |accel| = {:.4f} m/s^2, accel_var = {:.2e}, gyro_var = "
      "{:.2e}, b_g = [{:.2e}, {:.2e}, {:.2e}]",
      init.accel_mean_norm, init.max_accel_var, init.max_gyro_var,
      init.state.b_g.x(), init.state.b_g.y(), init.state.b_g.z());
}

void log_progress(size_t scans, const State& s, size_t map_size,
                  const EkfResult& r) {
  spdlog::info(
      "scan {:5} | pos [{:7.2f},{:7.2f},{:7.2f}] | map {:6} pts | iters {} {}",
      scans, s.p.x(), s.p.y(), s.p.z(), map_size, r.iterations,
      r.converged ? "conv" : "");
}

void log_summary(size_t scans, size_t map_size, size_t gt_msgs,
                 size_t converged_scans, size_t total_iters) {
  spdlog::info(
      "Done. Processed {} scans, final map {} points, {} GT poses written.",
      scans, map_size, gt_msgs);
  if (scans == 0) return;
  spdlog::info(
      "iEKF: {}/{} scans converged ({:.1f}%), mean {:.2f} iters/scan.",
      converged_scans, scans,
      100.0 * static_cast<double>(converged_scans) / static_cast<double>(scans),
      static_cast<double>(total_iters) / static_cast<double>(scans));
}

// How far the online extrinsic estimate drifted from the calibration seed.
// Large translation drift suggests it is being driven by unobservable
// directions rather than genuine miscalibration.
void log_extrinsic_drift(const Sophus::SE3d& seed, const State& s) {
  const Eigen::Vector3d d_rot = (seed.so3().inverse() * s.R_imu_lidar).log();
  const Eigen::Vector3d d_trans = s.p_imu_lidar - seed.translation();
  spdlog::info(
      "Extrinsic: drot {:.3f} deg | dtrans {:.4f} m | p_il [{:+.4f}, {:+.4f}, "
      "{:+.4f}] (seed [{:+.4f}, {:+.4f}, {:+.4f}])",
      d_rot.norm() * 180.0 / std::numbers::pi, d_trans.norm(),
      s.p_imu_lidar.x(), s.p_imu_lidar.y(), s.p_imu_lidar.z(),
      seed.translation().x(), seed.translation().y(), seed.translation().z());
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

  PipelineConfig pcfg;
  pcfg.noise = params.noise;
  pcfg.init = params.init;
  pcfg.iekf = params.iekf;
  pcfg.assoc = params.assoc;
  pcfg.map = params.map;
  pcfg.T_imu_lidar = T_imu_lidar;
  pcfg.scan_voxel_leaf = params.scan_voxel_leaf;
  pcfg.lidar_time_offset_ns = lidar_time_offset_ns;
  pcfg.init_imu_count = params.init_imu_count;
  pcfg.enable_extrinsic_estimation = params.enable_extrinsic_estimation;
  SlamPipeline pipeline(pcfg);

  // TUM outputs for offline evaluation (see ResultWriter). External-file GT is
  // dumped once up front; topic-streamed GT flows through write_gt below.
  ResultWriter writer(cfg.output_dir, loader->has_gt(), t_imu_gt);
  size_t gt_msgs = 0;
  if (loader->has_gt() && loader->gt_topic().empty())
    gt_msgs = loader->write_external_gt(writer.gt_stream());

  // Live visualization stream (no-op unless built with
  // LIDAR_SLAM_ENABLE_RERUN).
  Visualizer viz;

  // Sink each finished scan: write the pose, log progress, drive the viewer.
  const auto handle_results =
      [&](const std::vector<SlamPipeline::Result>& results) {
        for (const SlamPipeline::Result& res : results) {
          writer.write_pose(Timestamp::from_nsec(res.t_ns).to_sec(), res.state);
          if (res.scan_index % 100 == 0)
            log_progress(res.scan_index, res.state, res.map_size, res.ekf);

          // Transform the scan to world with the post-update pose; log the full
          // map periodically since it is large.
          const Sophus::SE3d T_wi(res.state.R, res.state.p);
          const Sophus::SE3d T_wl =
              T_wi * Sophus::SE3d(res.state.R_imu_lidar, res.state.p_imu_lidar);
          std::vector<Eigen::Vector3f> scan_world;
          scan_world.reserve(res.scan_lidar.size());
          for (const Eigen::Vector3f& p_l : res.scan_lidar)
            scan_world.push_back((T_wl * p_l.cast<double>()).cast<float>());
          viz.set_scan(static_cast<int64_t>(res.scan_index));
          viz.log_scan(scan_world);
          viz.log_pose(T_wi);
          if (res.scan_index % 50 == 0) viz.log_map(pipeline.map());
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
          if (const std::optional<InitResult> init = pipeline.add_imu(m))
            log_static_init(*init);
        } else if (topic == loader->lidar_topic() && pipeline.initialized()) {
          pipeline.add_scan(loader->decode_cloud(data));
        }
        handle_results(pipeline.take_ready());
      });

  // Final full map at the last scan index.
  if (pipeline.initialized()) {
    viz.set_scan(static_cast<int64_t>(pipeline.scans()));
    viz.log_map(pipeline.map());
  }

  log_summary(pipeline.scans(), pipeline.map_size(), gt_msgs,
              pipeline.converged_scans(), pipeline.total_iters());
  if (pipeline.scans() > 0) log_extrinsic_drift(T_imu_lidar, pipeline.state());
  return 0;
}

#include "pipeline/slam_pipeline.h"

#include "preprocess/deskew.h"
#include "preprocess/voxel_grid.h"

SlamPipeline::SlamPipeline(const PipelineConfig& cfg) : cfg_(cfg) {}

std::optional<InitResult> SlamPipeline::add_imu(const ImuMeasurement& m) {
  imu_buffer_.push(m);
  if (ekf_) return std::nullopt;

  init_imu_.push_back(m);
  if (init_imu_.size() < cfg_.init_imu_count) return std::nullopt;

  InitResult init = initialize_static(init_imu_, cfg_.init);
  if (!init.ok) {
    // Platform is moving: slide the window and keep waiting for a stationary
    // stretch.
    init_imu_.erase(init_imu_.begin());
    return std::nullopt;
  }

  // Seed the extrinsic from the calibration.
  State x0 = init.state;
  x0.R_imu_lidar = cfg_.T_imu_lidar.so3();
  x0.p_imu_lidar = cfg_.T_imu_lidar.translation();
  ErrorMatrix P0 = init.cov;
  if (!cfg_.enable_extrinsic_estimation) {
    // Pin the extrinsic with a negligible prior variance: the update cannot
    // move it and there is no process noise to reinflate it, so the filter
    // behaves as if the calibration were fixed. Non-zero to keep P invertible.
    P0.diagonal().segment<6>(kIdxExtRot).setConstant(1e-12);
  }
  ekf_.emplace(cfg_.noise, cfg_.iekf, cfg_.assoc, x0, P0, cfg_.map);
  last_ref_ = init_imu_.back().stamp.to_nsec();
  return init;
}

void SlamPipeline::add_scan(PointCloud cloud) {
  if (!ekf_) return;  // drop clouds that arrive before initialization
  // Shift the scan stamp onto the IMU clock by the lidar-to-IMU time offset, so
  // the scan window, trajectory, deskew, and the IMU window all move together.
  cloud.stamp = Timestamp::from_nsec(static_cast<uint64_t>(
      static_cast<int64_t>(cloud.stamp.to_nsec()) + cfg_.lidar_time_offset_ns));
  pending_.push_back(std::move(cloud));
}

std::vector<SlamPipeline::Result> SlamPipeline::take_ready() {
  std::vector<Result> out;
  while (!pending_.empty()) {
    const PointCloud& cloud = pending_.front();
    const auto window = scan_time_window(cloud);
    if (!window) {
      pending_.pop_front();
      continue;
    }
    const auto [t_start, t_end] = *window;
    // Drop a scan that does not advance past the last one processed (e.g. Livox
    // publishes occasionally overlapping/out-of-order sweeps).
    if (last_ref_ != 0 && t_end <= last_ref_) {
      pending_.pop_front();
      continue;
    }
    if (!imu_buffer_.covers(last_ref_, t_end)) break;  // need more IMU

    const auto traj =
        build_scan_trajectory(imu_buffer_, t_start, t_end, ekf_->state());
    // Deskew with the current extrinsic estimate so it stays consistent with
    // the update below.
    const Sophus::SE3d T_il_est(ekf_->state().R_imu_lidar,
                                ekf_->state().p_imu_lidar);
    const PointCloud undist = deskew(cloud, traj, T_il_est, t_end);
    const PointCloud filtered = voxel_downsample(undist, cfg_.scan_voxel_leaf);

    const auto imu_window = imu_buffer_.get_between(last_ref_, t_end);
    std::vector<Eigen::Vector3f> scan_lidar = filtered.xyz();
    const EkfResult r = ekf_->process_scan(imu_window, scan_lidar);
    last_ref_ = t_end;
    ++scans_;
    converged_scans_ += r.converged ? 1 : 0;
    total_iters_ += static_cast<size_t>(r.iterations);

    out.push_back(Result{t_end, scans_, ekf_->state(), r, ekf_->map().size(),
                         std::move(scan_lidar)});
    pending_.pop_front();
  }
  return out;
}

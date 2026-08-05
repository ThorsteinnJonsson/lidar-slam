#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <deque>
#include <optional>
#include <sophus/se3.hpp>
#include <vector>

#include "estimator/iterated_ekf.h"
#include "imu/buffer.h"
#include "imu/initializer.h"
#include "imu/propagator.h"
#include "map/association.h"
#include "map/ikd_tree.h"
#include "map/local_map.h"
#include "state/state.h"
#include "types.h"

struct PipelineConfig {
  NoiseParams noise;
  InitParams init;
  IekfConfig iekf;
  PlaneAssocParams assoc;
  LocalMapParams map;
  Sophus::SE3d T_imu_lidar;
  double scan_voxel_leaf{0.5};
  int64_t lidar_time_offset_ns{0};
  size_t init_imu_count{200};
  bool enable_extrinsic_estimation{false};
};

class SlamPipeline {
 public:
  struct Result {
    uint64_t t_ns;      // scan-end time, on the IMU/GT clock (offset-corrected)
    size_t scan_index;  // 1-based
    State state;
    EkfResult ekf;
    size_t map_size;
    // Deskewed, downsampled scan in the lidar frame (for visualization).
    std::vector<Eigen::Vector3f> scan_lidar;
  };

  explicit SlamPipeline(const PipelineConfig& cfg);

  // Push one IMU sample.
  // Returns the InitResult on the call that completes static initialization,
  // else nullopt.
  std::optional<InitResult> add_imu(const ImuMeasurement& m);

  // Push one raw scan.
  // Applies the lidar-to-IMU time offset. Only valid after initialization.
  void add_scan(PointCloud cloud);

  // Process every pending scan whose IMU window is now covered, in order.
  std::vector<Result> take_ready();

  bool initialized() const { return ekf_.has_value(); }
  const State& state() const { return ekf_->state(); }
  const IkdTree& map() const { return ekf_->map(); }
  size_t map_size() const { return ekf_ ? ekf_->map().size() : 0u; }
  size_t scans() const { return scans_; }
  size_t converged_scans() const { return converged_scans_; }
  size_t total_iters() const { return total_iters_; }

 private:
  PipelineConfig cfg_;
  ImuBuffer imu_buffer_;
  std::deque<PointCloud> pending_;
  std::vector<ImuMeasurement> init_imu_;  // window for static init
  std::optional<IteratedEkf> ekf_;
  uint64_t last_ref_ = 0;  // scan-end of the previously processed scan
  size_t scans_ = 0;
  size_t converged_scans_ = 0;
  size_t total_iters_ = 0;
};

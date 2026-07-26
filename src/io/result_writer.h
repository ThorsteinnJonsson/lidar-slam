#pragma once

#include <Eigen/Core>
#include <filesystem>
#include <fstream>
#include <ostream>

#include "io/dataset_loader.h"  // GtRow
#include "state/state.h"

// Writes the TUM evaluation outputs for a run:
//   trajectory.tum   estimated IMU pose in the world frame
//   estimate_gt.tum  estimate at the ground-truth reference point (lever arm)
//   gt.tum           ground truth (position only)
// The estimate_gt/gt files are opened only when the dataset provides ground
// truth. All streams use fixed 9-decimal precision so the ~1.6e9 timestamps
// keep nanosecond resolution.
class ResultWriter {
 public:
  ResultWriter(const std::filesystem::path& output_dir, bool has_gt,
               const Eigen::Vector3d& imu_to_gt_point);

  // Trajectory pose (the IMU pose) plus, when GT is available, the estimate at
  // the GT reference point (lever arm applied). `t_sec` is the pose time on the
  // GT/IMU clock.
  void write_pose(double t_sec, const State& state);

  // A ground-truth sample (position only).
  void write_gt(const GtRow& gt);

  // Output stream for a bulk external-file GT dump
  // (DatasetLoader::write_external_gt). Only meaningful when has_gt.
  std::ostream& gt_stream() { return gt_out_; }

 private:
  bool has_gt_;
  Eigen::Vector3d imu_to_gt_point_;
  std::ofstream traj_out_;
  std::ofstream est_gt_out_;
  std::ofstream gt_out_;
};

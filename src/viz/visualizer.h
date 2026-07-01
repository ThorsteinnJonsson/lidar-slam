#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <sophus/se3.hpp>
#include <string>
#include <vector>

#include "map/ikd_tree.h"

// Thin wrapper over a Rerun recording stream for live 3D debugging of the SLAM
// (map, registered scans, sensor pose).
//
// Built without LIDAR_SLAM_ENABLE_RERUN, every method is a no-op and Rerun is
// not linked, so callers need no #ifdefs. When enabled, construction spawns the
// Rerun viewer; if that fails (e.g. the viewer binary is not on PATH) it logs a
// warning and degrades to no-ops rather than aborting the run.
class Visualizer {
 public:
  explicit Visualizer(const std::string& app_id = "lidar_slam");
  ~Visualizer();

  Visualizer(const Visualizer&) = delete;
  Visualizer& operator=(const Visualizer&) = delete;

  // Position subsequent logs at this scan index on the "scan" timeline.
  void set_scan(int64_t scan_index);

  // Log the current registered scan (world frame) and the sensor pose.
  void log_scan(const std::vector<Eigen::Vector3f>& points_world);
  void log_pose(const Sophus::SE3d& T_world_imu);

  // Log the accumulated map. Takes the tree and collects its live points only
  // when enabled, so disabled builds pay nothing.
  void log_map(const IkdTree& map);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;  // null when built without Rerun
};

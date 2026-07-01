#include "viz/visualizer.h"

#ifdef LIDAR_SLAM_ENABLE_RERUN

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <rerun.hpp>

struct Visualizer::Impl {
  rerun::RecordingStream rec;
  bool ok = false;

  explicit Impl(const std::string& app_id) : rec(app_id) {
    if (const rerun::Error err = rec.spawn(); err.is_err()) {
      spdlog::warn("Rerun: viewer spawn failed ({}); visualization disabled",
                   err.description);
      return;
    }
    ok = true;
    // Z-up, right-handed world so the viewer's default orientation matches
    // ours.
    rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);
  }
};

namespace {

std::vector<rerun::Position3D> to_positions(
    const std::vector<Eigen::Vector3f>& pts) {
  std::vector<rerun::Position3D> out;
  out.reserve(pts.size());
  for (const Eigen::Vector3f& p : pts) out.emplace_back(p.x(), p.y(), p.z());
  return out;
}

}  // namespace

Visualizer::Visualizer(const std::string& app_id)
    : impl_(std::make_unique<Impl>(app_id)) {}
Visualizer::~Visualizer() = default;

void Visualizer::set_scan(int64_t scan_index) {
  if (impl_->ok) impl_->rec.set_time_sequence("scan", scan_index);
}

void Visualizer::log_scan(const std::vector<Eigen::Vector3f>& points_world) {
  if (!impl_->ok) return;
  impl_->rec.log("world/scan", rerun::Points3D(to_positions(points_world))
                                   .with_colors(rerun::Color(0xff, 0x50, 0x50))
                                   .with_radii(0.05f));
}

void Visualizer::log_pose(const Sophus::SE3d& T_world_imu) {
  if (!impl_->ok) return;
  const Eigen::Vector3f t = T_world_imu.translation().cast<float>();
  const Eigen::Matrix3f r = T_world_imu.rotationMatrix().cast<float>();
  std::array<float, 9> cols;  // Eigen is column-major, as Mat3x3 expects
  std::copy(r.data(), r.data() + 9, cols.begin());
  impl_->rec.log("world/pose", rerun::Transform3D()
                                   .with_translation({t.x(), t.y(), t.z()})
                                   .with_mat3x3(cols));
}

void Visualizer::log_map(const IkdTree& map) {
  if (!impl_->ok) return;
  impl_->rec.log("world/map", rerun::Points3D(to_positions(map.collect()))
                                  .with_colors(rerun::Color(0xa0, 0xa0, 0xa0))
                                  .with_radii(0.02f));
}

#else  // LIDAR_SLAM_ENABLE_RERUN not defined: no-op build

struct Visualizer::Impl {};

Visualizer::Visualizer(const std::string&) {}
Visualizer::~Visualizer() = default;
void Visualizer::set_scan(int64_t) {}
void Visualizer::log_scan(const std::vector<Eigen::Vector3f>&) {}
void Visualizer::log_pose(const Sophus::SE3d&) {}
void Visualizer::log_map(const IkdTree&) {}

#endif

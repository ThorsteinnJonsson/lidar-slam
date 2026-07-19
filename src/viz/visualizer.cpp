#include "viz/visualizer.h"

#ifdef LIDAR_SLAM_ENABLE_RERUN

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
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

// Map a normalized value t in [0, 1] to an RGB color by sweeping hue from blue
// (low) through green to red (high), with saturation and value pinned to 1.
rerun::Color hue_color(float t) {
  // Blue (hue sextant 4) at t=0 down to red (sextant 0) at t=1.
  const float h = (1.0f - std::clamp(t, 0.0f, 1.0f)) * 4.0f;
  const float x = 1.0f - std::abs(std::fmod(h, 2.0f) - 1.0f);
  float r = 0.0f, g = 0.0f, b = 0.0f;
  if (h < 1.0f) {
    r = 1.0f, g = x;
  } else if (h < 2.0f) {
    r = x, g = 1.0f;
  } else if (h < 3.0f) {
    g = 1.0f, b = x;
  } else if (h < 4.0f) {
    g = x, b = 1.0f;
  } else if (h < 5.0f) {
    r = x, b = 1.0f;
  } else {
    r = 1.0f, b = x;
  }
  const auto to8 = [](float c) { return static_cast<uint8_t>(c * 255.0f); };
  return rerun::Color(to8(r), to8(g), to8(b));
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
                                   .with_radii(0.10f));
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
  const std::vector<Eigen::Vector3f> pts = map.collect();

  // Color by height over a fixed z-range so the gradient is stable across
  // frames (a per-frame min/max rescales the colors as the map grows). Points
  // outside the range clamp to the endpoint colors (hue_color clamps t to [0,
  // 1]).
  constexpr float kColorZMin = -2.0f;
  constexpr float kColorZMax = 30.0f;
  constexpr float kColorZSpan = kColorZMax - kColorZMin;

  std::vector<rerun::Color> colors;
  colors.reserve(pts.size());
  for (const Eigen::Vector3f& p : pts)
    colors.push_back(hue_color((p.z() - kColorZMin) / kColorZSpan));

  impl_->rec.log(
      "world/map",
      rerun::Points3D(to_positions(pts)).with_colors(colors).with_radii(0.02f));
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

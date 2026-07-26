#include "preprocess/voxel_grid.h"

#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace {

// Running per-voxel accumulator. Sums in double precision even though the cloud
// stores floats, to avoid precision loss when many coordinates are added.
struct VoxelAccum {
  Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
  double sum_intensity{0.0};
  uint32_t count{0};
};

}  // namespace

PointCloud voxel_downsample(const PointCloud& cloud, double leaf_size) {
  assert(leaf_size > 0.0);

  // Map a point to a single integer voxel key by bit-packing its grid indices.
  // 21 bits per axis, biased to non-negative: range ±2^20 voxels, i.e. ±524 km
  // at a 0.5 m leaf, far beyond the extent of any single scan.
  const auto voxel_key = [leaf_size](float x, float y, float z) -> int64_t {
    // floor (not truncation) so negative coordinates bucket correctly.
    const int64_t ix = static_cast<int64_t>(std::floor(x / leaf_size));
    const int64_t iy = static_cast<int64_t>(std::floor(y / leaf_size));
    const int64_t iz = static_cast<int64_t>(std::floor(z / leaf_size));
    constexpr int64_t kBias = 1 << 20;
    constexpr int64_t kMask = 0x1FFFFF;  // 21 bits
    return ((ix + kBias) & kMask) | (((iy + kBias) & kMask) << 21) |
           (((iz + kBias) & kMask) << 42);
  };

  // Pass 1: accumulate points into their voxels.
  std::unordered_map<int64_t, VoxelAccum> grid;
  grid.reserve(cloud.size());  // upper bound; avoids rehashing
  for (size_t i = 0; i < cloud.size(); ++i) {
    const float x = cloud.x[i], y = cloud.y[i], z = cloud.z[i];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    auto& a = grid[voxel_key(x, y, z)];
    a.sum += Eigen::Vector3d(x, y, z);
    a.sum_intensity += cloud.intensity[i];
    ++a.count;
  }

  // Pass 2: emit one centroid per occupied voxel.
  PointCloud out;
  out.stamp = cloud.stamp;
  out.frame_id = cloud.frame_id;
  out.reserve(grid.size());
  for (const auto& [key, a] : grid) {
    const Eigen::Vector3d c = a.sum / a.count;
    out.x.push_back(static_cast<float>(c.x()));
    out.y.push_back(static_cast<float>(c.y()));
    out.z.push_back(static_cast<float>(c.z()));
    out.intensity.push_back(static_cast<float>(a.sum_intensity / a.count));
  }
  return out;
}

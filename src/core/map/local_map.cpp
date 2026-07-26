#include "map/local_map.h"

#include <cmath>

bool box_needs_slide(const Eigen::Vector3f& center, const Eigen::Vector3f& lo,
                     const Eigen::Vector3f& hi, float margin) {
  // Per axis, the distance to the nearer face; slide if any is within margin.
  return ((center - lo).cwiseMin(hi - center).array() < margin).any();
}

void crop_to_box(IkdTree& tree, const Eigen::Vector3f& lo,
                 const Eigen::Vector3f& hi) {
  constexpr float kBig = 1e9f;
  const Eigen::Vector3f big = Eigen::Vector3f::Constant(kBig);
  for (int a = 0; a < 3; ++a) {
    // Slab below lo[a]: full extent on the other axes, capped just under lo[a]
    // so points exactly on the kept boundary survive.
    Eigen::Vector3f below_max = big;
    below_max[a] = std::nextafter(lo[a], -kBig);
    tree.remove_box(-big, below_max);
    // Slab above hi[a].
    Eigen::Vector3f above_min = -big;
    above_min[a] = std::nextafter(hi[a], kBig);
    tree.remove_box(above_min, big);
  }
}

void LocalMap::insert(std::vector<Eigen::Vector3f> world_points) {
  // First scan seats the map; nothing to dedup against yet.
  if (tree_.size() == 0) {
    tree_.build(std::move(world_points));
    return;
  }
  if (!params_.voxel_on_insert) {
    tree_.insert(world_points);
    return;
  }

  // Voxel-on-insert: keep a point only when no existing map point lies within
  // voxel_leaf of it. The batch is tested against the current tree (not against
  // itself); the per-scan downsample already spaces points out, so intra-batch
  // duplicates are negligible.
  const float leaf2 = params_.voxel_leaf * params_.voxel_leaf;
  std::vector<Eigen::Vector3f> kept;
  kept.reserve(world_points.size());
  std::vector<Eigen::Vector3f> nbr_pts;
  std::vector<float> nbr_d2;
  for (const Eigen::Vector3f& p : world_points) {
    tree_.knn(p, 1, nbr_pts, nbr_d2);
    if (nbr_d2.empty() || nbr_d2[0] >= leaf2) kept.push_back(p);
  }
  if (!kept.empty()) tree_.insert(kept);
}

void LocalMap::recenter(const Eigen::Vector3f& sensor_pos) {
  const MapCropParams& crop = params_.crop;
  if (!crop.enabled) return;
  const Eigen::Vector3f half =
      Eigen::Vector3f::Constant(crop.keep_half_extent());
  if (!box_initialized_) {
    box_min_ = sensor_pos - half;
    box_max_ = sensor_pos + half;
    box_initialized_ = true;
    return;  // box just seated; nothing outside it yet
  }
  if (!box_needs_slide(sensor_pos, box_min_, box_max_, crop.slide_margin())) {
    return;
  }
  box_min_ = sensor_pos - half;
  box_max_ = sensor_pos + half;
  crop_to_box(tree_, box_min_, box_max_);
}

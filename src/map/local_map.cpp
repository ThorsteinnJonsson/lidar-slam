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
  if (tree_.size() == 0) {
    tree_.build(std::move(world_points));
  } else {
    tree_.insert(world_points);
  }
}

void LocalMap::recenter(const Eigen::Vector3f& sensor_pos) {
  if (!crop_.enabled) return;
  const Eigen::Vector3f half =
      Eigen::Vector3f::Constant(crop_.keep_half_extent);
  if (!box_initialized_) {
    box_min_ = sensor_pos - half;
    box_max_ = sensor_pos + half;
    box_initialized_ = true;
    return;  // box just seated; nothing outside it yet
  }
  if (!box_needs_slide(sensor_pos, box_min_, box_max_, crop_.slide_margin)) {
    return;
  }
  box_min_ = sensor_pos - half;
  box_max_ = sensor_pos + half;
  crop_to_box(tree_, box_min_, box_max_);
}

#pragma once

#include <Eigen/Core>
#include <vector>

#include "map/ikd_tree.h"

// The SLAM map and its growth policy: an ikd-tree plus a sliding-window bound.
// Owns map management (insertion, cropping, and later voxel-on-insert) so the
// estimator can stay focused on estimation and just drive it.

// Sliding-window map bound (FAST-LIO2 "map sliding"). The map is kept to a cube
// of half-side keep_half_extent centered near the sensor; when the sensor comes
// within slide_margin of a face the cube recenters on the current position and
// everything outside is box-deleted. Bounds memory and keeps k-NN fast on long
// trajectories. Note: on a small loop where every surface stays in range the
// box never slides, so this is a no-op there.
struct MapCropParams {
  bool enabled{true};
  float keep_half_extent{150.0f};  // half side of the local map cube (m)
  float slide_margin{30.0f};       // recenter when within this of a face (m)
};

// True if `center` is within `margin` of any face of the closed box [lo, hi].
bool box_needs_slide(const Eigen::Vector3f& center, const Eigen::Vector3f& lo,
                     const Eigen::Vector3f& hi, float margin);

// Box-delete every point outside the closed cube [lo, hi] from `tree` (six
// outer slabs). Points exactly on the cube boundary are kept.
void crop_to_box(IkdTree& tree, const Eigen::Vector3f& lo,
                 const Eigen::Vector3f& hi);

class LocalMap {
 public:
  explicit LocalMap(const MapCropParams& crop = {}) : crop_(crop) {}

  // Insert registered world-frame points; builds the tree on the first call.
  void insert(std::vector<Eigen::Vector3f> world_points);

  // Slide the keep-cube to follow the sensor, box-deleting points that fall
  // outside it. The cube seats on the first call and only slides thereafter
  // when the sensor nears a face.
  void recenter(const Eigen::Vector3f& sensor_pos);

  // Read access for association / k-NN.
  const IkdTree& tree() const { return tree_; }
  size_t size() const { return tree_.size(); }

 private:
  IkdTree tree_;
  MapCropParams crop_;
  Eigen::Vector3f box_min_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f box_max_{Eigen::Vector3f::Zero()};
  bool box_initialized_{false};
};

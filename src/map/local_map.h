#pragma once

#include <Eigen/Core>
#include <vector>

#include "map/ikd_tree.h"

// The SLAM map and its growth policy: an ikd-tree plus a sliding-window bound.
// Owns map management (insertion, cropping, and later voxel-on-insert) so the
// estimator can stay focused on estimation and just drive it.

// Sliding-window map bound (FAST-LIO2 "map sliding"). The map is kept to a cube
// centered near the sensor; when the sensor comes within the slide margin of a
// face the cube recenters on the current position and everything outside is
// box-deleted. Bounds memory and keeps k-NN fast on long trajectories.
//
// Both extents derive from the sensor range so the kept cube always contains
// everything the lidar can currently observe. The margin must exceed the range
// (FAST-LIO2 uses MOV_THRESHOLD = 1.5): if the sensor got closer than its range
// to a face, it would be observing surface outside the cube, and cropping would
// delete points association still needs. keep_factor > margin_factor keeps a
// stable band inside the margin so a recenter does not immediately retrigger.
struct MapCropParams {
  bool enabled{true};
  float sensor_range{300.0f};  // lidar detection range (m)
  float margin_factor{1.5f};   // slide within margin_factor*range of a face
  float keep_factor{2.0f};     // keep cube half-side = keep_factor*range

  float slide_margin() const { return margin_factor * sensor_range; }
  float keep_half_extent() const { return keep_factor * sensor_range; }
};

// True if `center` is within `margin` of any face of the closed box [lo, hi].
bool box_needs_slide(const Eigen::Vector3f& center, const Eigen::Vector3f& lo,
                     const Eigen::Vector3f& hi, float margin);

// Box-delete every point outside the closed cube [lo, hi] from `tree` (six
// outer slabs). Points exactly on the cube boundary are kept.
void crop_to_box(IkdTree& tree, const Eigen::Vector3f& lo,
                 const Eigen::Vector3f& hi);

// Map management policy: the sliding-window crop plus voxel-on-insert
// resolution control. Both default on; defaults are sane for a lidar-scale
// outdoor scene.
struct LocalMapParams {
  MapCropParams crop;
  bool voxel_on_insert{true};
  float voxel_leaf{0.5f};  // map resolution (m); match the per-scan leaf
};

class LocalMap {
 public:
  explicit LocalMap(const LocalMapParams& params = {}) : params_(params) {}

  // Insert registered world-frame points; builds the tree on the first call.
  // With voxel-on-insert enabled, a point is kept only when no existing map
  // point lies within voxel_leaf of it, so re-observing a surface does not pile
  // up near-duplicates and the map holds a fixed resolution.
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
  LocalMapParams params_;
  Eigen::Vector3f box_min_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f box_max_{Eigen::Vector3f::Zero()};
  bool box_initialized_{false};
};

#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>

#include "estimator/iekf.h"
#include "imu/propagator.h"
#include "imu/state.h"
#include "map/association.h"
#include "map/ikd_tree.h"
#include "types.h"

// Stateful driver that ties the iEKF to the map across scans: predict over the
// scan's IMU window, run the iterated update against the local map, then fold
// the registered scan into the map. Owns the running estimate (x, P) and the
// ikd-tree.
//
// The pure update math lives in iterated_update (iekf.h); this class supplies
// the real MeasurementFn (associate_planes + build_measurement, re-associated
// every iteration against the live iterate) and manages map growth.

// Sliding-window map bound (FAST-LIO2 "map sliding"). The map is kept to a cube
// of half-side keep_half_extent centered near the sensor; when the sensor comes
// within slide_margin of a face the cube recenters on the current position and
// everything outside is box-deleted. Bounds memory and keeps k-NN fast on long
// trajectories.
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

class IteratedEkf {
 public:
  IteratedEkf(const NoiseParams& noise, const Sophus::SE3d& T_imu_lidar,
              const IekfConfig& cfg, const PlaneAssocParams& assoc,
              const State& x0, const Eigen::Matrix<double, 18, 18>& P0,
              const MapCropParams& crop = {});

  // Process one scan. `imu` spans [previous scan ref, this scan ref] with
  // interpolated endpoints (as ImuBuffer::get_between returns); the state is
  // propagated across consecutive pairs. `points_lidar` is the deskewed,
  // downsampled scan in the lidar frame. After the update the registered points
  // are inserted into the map. On the first scan the map is empty, so the
  // update is a no-op and the scan simply seats the map.
  EkfResult process_scan(const std::vector<ImuMeasurement>& imu,
                         const std::vector<Eigen::Vector3f>& points_lidar);

  const State& state() const { return x_; }
  const Eigen::Matrix<double, 18, 18>& covariance() const { return P_; }
  const IkdTree& map() const { return map_; }

 private:
  // Bound the map to the local sliding-window cube around the current position.
  void crop_map();

  IkdTree map_;
  State x_;
  Eigen::Matrix<double, 18, 18> P_;
  ImuPropagator propagator_;
  Sophus::SE3d T_imu_lidar_;
  IekfConfig cfg_;
  PlaneAssocParams assoc_;
  MapCropParams crop_;
  Eigen::Vector3f box_min_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f box_max_{Eigen::Vector3f::Zero()};
  bool box_initialized_{false};
};

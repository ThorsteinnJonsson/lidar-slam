#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>

#include "estimator/iekf.h"
#include "imu/propagator.h"
#include "imu/state.h"
#include "map/association.h"
#include "map/ikd_tree.h"
#include "map/local_map.h"
#include "types.h"

// Stateful driver that ties the iEKF to the map across scans: predict over the
// scan's IMU window, run the iterated update against the local map, then fold
// the registered scan into the map. Owns the running estimate (x, P) and the
// ikd-tree.
//
// The pure update math lives in iterated_update (iekf.h); this class supplies
// the real MeasurementFn (associate_planes + build_measurement, re-associated
// every iteration against the live iterate). Map growth and the sliding-window
// bound are delegated to LocalMap.
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
  const IkdTree& map() const { return map_.tree(); }

 private:
  LocalMap map_;
  State x_;
  Eigen::Matrix<double, 18, 18> P_;
  ImuPropagator propagator_;
  Sophus::SE3d T_imu_lidar_;
  IekfConfig cfg_;
  PlaneAssocParams assoc_;
};

#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>

#include "map/ikd_tree.h"

// A point-to-plane correspondence between a scan point and a local map surface.
struct PlaneMatch {
  Eigen::Vector3f point;  // source point, in the body frame passed to associate
  Eigen::Vector3f normal;  // unit plane normal, world frame
  float residual;          // signed point-to-plane distance, world frame
};

// Thresholds for accepting a correspondence.
struct PlaneAssocParams {
  int num_neighbors = 5;            // neighbors used to fit the plane
  float max_neighbor_dist2 = 5.0f;  // reject if farthest neighbor beyond (m^2)
  float max_plane_dist = 0.1f;      // reject if any neighbor farther from plane

  // FAST-LIO2 range-normalized correspondence gate. Keep a point only if its
  // distance to the fitted plane is small relative to sqrt(range):
  //   s = 1 - 0.9 * |point-plane dist| / sqrt(range),  keep if s > min_quality.
  // Tolerates proportionally larger residuals at long range (where the fit is
  // noisier) while rejecting near points that sit clearly off their plane.
  float min_quality = 0.9f;
};

// For each body-frame point, find the local map plane it lies on and emit a
// point-to-plane correspondence. Points are transformed to world via
// T_world_body for the map lookup; the returned `point` is the original
// body-frame point (the iEKF folds the lidar->imu extrinsic into T_world_body
// beforehand). Points without a valid nearby plane are dropped, so the output
// is generally shorter than the input.
std::vector<PlaneMatch> associate_planes(
    const IkdTree& map, const std::vector<Eigen::Vector3f>& points_body,
    const Sophus::SE3f& T_world_body, const PlaneAssocParams& params = {});

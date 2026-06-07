#pragma once

#include <cstdint>
#include <sophus/se3.hpp>
#include <vector>

#include "imu/buffer.h"
#include "types.h"

// A body (IMU) pose in the world frame at a given time.
struct StampedPose {
  uint64_t t_ns{0};
  Sophus::SE3d T_W_I;
};

// Build the IMU trajectory spanning a scan window [t_start_ns, t_end_ns] by
// integrating the buffered IMU measurements. The returned poses are expressed
// relative to the scan start (first pose = identity); only intra-scan relative
// motion matters for deskewing, so the absolute world frame is irrelevant here.
//
// TODO(velocity): this is rotation-only for now — translation is left at zero
// because we have no velocity estimate yet. Once the iEKF provides a full state
// (Phase 5/6), integrate position via ImuPropagator and feed the bias in too.
std::vector<StampedPose> build_scan_trajectory(const ImuBuffer& imu,
                                               uint64_t t_start_ns,
                                               uint64_t t_end_ns);

// Undistort a scan by transforming every point into the lidar frame at the
// reference time t_ref_ns (typically scan end). Pure function over a supplied
// trajectory, so it works unchanged with an IMU-only trajectory now and with an
// EKF-propagated one later.
//
//   p_ref = T_I_L⁻¹ · T_W_I(t_ref)⁻¹ · T_W_I(t_i) · T_I_L · p_i
PointCloud deskew(const PointCloud& cloud,
                  const std::vector<StampedPose>& trajectory,
                  const Sophus::SE3d& T_imu_lidar, uint64_t t_ref_ns);

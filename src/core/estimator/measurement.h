#pragma once

#include <Eigen/Core>
#include <vector>

#include "map/association.h"
#include "state/state.h"

// Point-to-plane measurement Jacobian: one row per correspondence, columns
// ordered like the error state [δθ, δp, δv, δb_g, δb_a, δg] (δg is 2-DOF).
using MeasurementJacobian = Eigen::Matrix<double, Eigen::Dynamic, kErrorDim>;

// Stacked linearization of the point-to-plane measurements about a state.
struct LinearizedMeasurement {
  // Measurement Jacobian, one row per correspondence (m x kErrorDim).
  MeasurementJacobian H;
  // Signed point-to-plane residuals, one per correspondence (m).
  Eigen::VectorXd z;
};

// Build H and z for the iEKF update from plane correspondences linearized about
// `s`. The matches must have been produced by associate_planes at this same
// state, associated in the LIDAR frame (T_world_lidar), so PlaneMatch.point is
// the raw lidar-frame point p_L and PlaneMatch.residual is the current
// residual.
//
// Residual: h = n.(R (R_ext p_L + p_ext) + p) + d, with the extrinsic
// (R_ext, p_ext) = (R_imu_lidar, p_imu_lidar) part of the state. With right
// perturbations R' = R Exp(δθ) and R_ext' = R_ext Exp(δθ_ext),
//   δh = -n^T R [p_I]x δθ + n^T δp - n^T R R_ext [p_L]x δθ_ext + n^T R δp_ext,
// so the pose and extrinsic blocks are nonzero; velocity, biases, and gravity
// are unobserved by a single frame and are corrected only through the
// covariance cross-terms.
LinearizedMeasurement build_measurement(const State& s,
                                        const std::vector<PlaneMatch>& matches);

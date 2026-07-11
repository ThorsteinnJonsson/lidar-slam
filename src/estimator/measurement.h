#pragma once

#include <Eigen/Core>
#include <vector>

#include "imu/state.h"
#include "map/association.h"

// Stacked linearization of the point-to-plane measurements about a state.
struct LinearizedMeasurement {
  // Measurement Jacobian, one row per correspondence (m x 17). Columns are
  // ordered like the error state [δθ, δp, δv, δb_g, δb_a, δg] (δg is 2-DOF).
  Eigen::Matrix<double, Eigen::Dynamic, 17> H;
  // Signed point-to-plane residuals, one per correspondence (m).
  Eigen::VectorXd z;
};

// Build H and z for the iEKF update from plane correspondences linearized about
// `s`. The matches must have been produced by associate_planes at this same
// state, so PlaneMatch.residual is the current residual and PlaneMatch.point is
// the point in the IMU/body frame (p_I = T_imu_lidar . p_L).
//
// Residual: h = n.(R p_I + p) + d. With the right perturbation R' = R Exp(δθ),
//   δh = -n^T R [p_I]x δθ + n^T δp,
// so only the rotation and position blocks are nonzero; velocity, biases, and
// gravity are unobserved by a single frame and are corrected only through the
// covariance cross-terms.
LinearizedMeasurement build_measurement(const State& s,
                                        const std::vector<PlaneMatch>& matches);

#pragma once

#include <Eigen/Core>

#include "estimator/measurement.h"

// Drop point-to-plane correspondences whose residual is too large to be an
// inlier given the current estimate, using a chi-squared test on the
// Mahalanobis distance. For each row i the normalized squared residual is
//
//   d_i^2 = z_i^2 / (H_i P H_i^T + sigma^2),
//
// where H_i P H_i^T + sigma^2 is the innovation variance (prior state
// uncertainty projected into the measurement plus the measurement noise R =
// sigma^2). Under the inlier hypothesis d_i^2 ~ chi-squared with one DOF, so a
// row is kept when d_i^2 <= chi2_thresh. Normalizing by the innovation variance
// (not sigma^2 alone) loosens the gate when the prior is uncertain and tightens
// it as the estimate firms up.
//
// Returns the surviving rows of `m` as a new LinearizedMeasurement; the input
// is unchanged. An empty result is valid.
LinearizedMeasurement gate_measurement(const LinearizedMeasurement& m,
                                       const Eigen::Matrix<double, 17, 17>& P,
                                       double sigma, double chi2_thresh);

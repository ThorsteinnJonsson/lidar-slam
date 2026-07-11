#pragma once

#include <Eigen/Core>
#include <functional>

#include "estimator/measurement.h"
#include "imu/state.h"

// Tuning for the iterated EKF measurement update.
struct IekfConfig {
  double sigma = 0.01;     // point-to-plane measurement std (m), R = sigma^2 I
  int max_iterations = 5;  // cap on the iterate loop
  double convergence_tol = 1e-3;  // stop when ||delta_x|| drops below this

  bool reject_outliers = true;  // gate correspondences by Mahalanobis distance
  double outlier_chi2 = 3.841;  // chi-squared(1) threshold (95th percentile)
};

// Result of an update: corrected state, covariance, and loop diagnostics.
struct EkfResult {
  State state;
  Eigen::Matrix<double, 17, 17> covariance;
  int iterations = 0;  // iterations actually run
  bool converged =
      false;  // true if the loop hit convergence_tol before the cap
};

// Builds the linearized measurement (H, z) about a given state. Supplied by the
// caller: tests pass a synthetic analytic-plane builder, the pipeline (5.5)
// passes associate_planes + build_measurement.
using MeasurementFn = std::function<LinearizedMeasurement(const State&)>;

// Iterated error-state EKF measurement update (FAST-LIO2 / IKFoM style).
//
// Starting from the propagated prior (x_hat, P), iterate:
// clang-format off
//   re-linearize (H, z) = measure(x_j)
//   K   = (H^T R^-1 H + P^-1)^-1 H^T R^-1          (17x17 inversion, not m x m)
//   dx  = -K z - (I - K H) boxminus(x_j, x_hat)
//   x_{j+1} = boxplus(x_j, dx)
// clang-format on
// until ||dx|| < tol or max_iterations. Then P+ = (I - K H) P, resymmetrized.
//
// With no correspondences the prior is returned unchanged.
EkfResult iterated_update(const State& x_hat,
                          const Eigen::Matrix<double, 17, 17>& P,
                          const MeasurementFn& measure, const IekfConfig& cfg);

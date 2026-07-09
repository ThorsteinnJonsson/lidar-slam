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
  // Diagnostic: total attitude correction the update applied to the propagated
  // prior, log(x_hat.R^-1 * x.R) in degrees. Large sustained values mean the
  // map is fighting the prediction (drift being induced or corrected).
  double update_dtheta_deg = 0.0;

  // Registration diagnostics (temp): correspondences at the final iterate, to
  // tell a sparse-map registration failure (few matches / large residuals) from
  // a bias observability transient. Filled by IteratedEkf::process_scan.
  int num_matches = 0;      // accepted point-to-plane correspondences
  int num_scan_points = 0;  // scan points offered to association
  double median_abs_residual =
      0.0;  // median |point-plane dist| (m) over matches
  // Motion over the scan's IMU window, to split the onset residual into a
  // rotation-coupled vs translation-coupled channel: if medres tracks
  // mean_omega the warp is rotational (extrinsic / gyro deskew), if it tracks
  // mean_acc it is translational (accel/velocity deskew).
  double mean_omega = 0.0;  // mean |gyro| over the window (rad/s)
  double mean_acc = 0.0;    // mean |linear accel|, gravity removed (m/s^2)

  // Registration attitude-bias diagnostics (temp): the update's attitude
  // correction expressed in the WORLD frame (deg). A persistent horizontal
  // component, accumulated over the run, reveals a directional normal bias that
  // gravity absorbs as tilt. Plus the plane population that constrains roll and
  // pitch: the fraction of matches with near-vertical normals (ground/ceiling)
  // and the normal-weighted mean residual, which a systematic vertical map
  // error would bias away from zero.
  Eigen::Vector3d update_dtheta_world_deg = Eigen::Vector3d::Zero();
  double vert_normal_frac = 0.0;  // fraction of matches with |n_z| > 0.8
  double vert_resid_bias = 0.0;   // mean n_z * residual over matches (m)

  // Onset-burst diagnostics (temp): the PRIOR (post-predict, pre-update)
  // uncertainty in the weakly-observed directions, in degrees. If the attitude
  // roll/pitch and gravity priors inflate at motion onset (when the ground
  // planes that pin them drop out), the update is free to yank attitude, which
  // is the suspected source of the ~4 deg onset kick.
  double prior_att_rp_std_deg = 0.0;  // sqrt(P00+P11), roll/pitch attitude
  double prior_grav_std_deg = 0.0;    // sqrt(P15,15+P16,16), gravity tilt

  // Pre-update inconsistency (temp): the point-to-plane fit at the propagated
  // prior, before the update tilts anything. If these already blow up at onset
  // the predicted+deskewed scan is wrong (prediction/deskew problem); if they
  // stay small the mismatch is manufactured by the update.
  double prior_medres = 0.0;  // median |residual| at the prior (m)
  double prior_vbias = 0.0;   // mean n_z * residual at the prior (m)
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

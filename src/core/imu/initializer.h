#pragma once

#include <Eigen/Core>
#include <vector>

#include "state/state.h"
#include "types.h"

// Static initialization of the navigation state from a window of IMU samples
// taken while the platform is at rest.
//
// With the platform stationary the accelerometer reads the specific force
// f = -R^T * gravity (it senses "up"), and the gyroscope reads its own bias.
// From the mean specific force we recover:
//   - the initial attitude R, by tilting the body so the measured "up" aligns
//     with world up. This fixes roll and pitch. Yaw is a gauge freedom (no
//     sensor observes absolute heading at rest), so we define it to zero.
//   - gravity, pinned to (0, 0, -|g|) with |g| the known local value.
// The gyro bias is the mean angular rate. The accel bias is left at zero: at
// rest it is indistinguishable from gravity, so the filter learns it later
// under motion.

struct InitParams {
  double gravity_magnitude{9.78};  // local g (NTU VIRAL, Singapore)

  // Stationarity gate: reject the window if any accel/gyro axis variance
  // exceeds these. Units are (m/s^2)^2 and (rad/s)^2 respectively.
  double max_accel_var{0.05};
  double max_gyro_var{0.01};

  // Initial covariance seed, given as per-block standard deviations. Position
  // and gyro bias are pinned tight (position is a gauge freedom, gyro bias is
  // well observed at rest); attitude carries ~1 deg of roll/pitch uncertainty
  // and gravity a matching tilt allowance. Accel bias is deliberately seeded
  // tight too (see below).
  double theta_std{0.017};  // attitude       ~1 deg
  double pos_std{1e-3};     // position       (origin by definition)
  double vel_std{1e-2};     // velocity       (at rest)
  double bias_gyro_std{1e-3};
  // Tight seed. Accel bias is unobservable at rest, so a loose prior lets the
  // filter dump the motion-onset registration inconsistency into b_a (it blew
  // up to ~1.5 m/s^2 with a 0.1 seed), which then tilts gravity. Pinning b_a
  // near its known-small init keeps the onset transient physical; the bias is
  // still learned under motion. 0.01 minimizes the onset gravity tilt without
  // starving the real (small, nonzero) bias.
  double bias_accel_std{0.01};
  double gravity_std{1e-2};
  // LiDAR-IMU extrinsic seed uncertainty. Deliberately tight: the YAML value is
  // a mechanical calibration good to about a millimeter, and from a single
  // frame the extrinsic is degenerate with the pose (δp_ext and δp shift the
  // world point identically), so only platform motion separates them. A loose
  // seed (1 deg / 1 cm) let the extrinsic drift ~3-4 sigma on eee_03 and absorb
  // registration misfit, costing ~8% ATE. Keep it pinned enough that the filter
  // can only make micro-refinements.
  double ext_rot_std{0.0035};   // ~0.2 deg
  double ext_trans_std{0.002};  // 2 mm
};

struct InitResult {
  bool ok{false};  // false if the window is not stationary
  State state;
  ErrorMatrix cov{ErrorMatrix::Zero()};

  // Diagnostics for logging.
  double accel_mean_norm{0.0};
  double max_accel_var{0.0};
  double max_gyro_var{0.0};
};

// Attempt static initialization over the given IMU window.
// Returns ok == false (with diagnostics filled) if the platform is moving.
InitResult initialize_static(const std::vector<ImuMeasurement>& imu,
                             const InitParams& params);

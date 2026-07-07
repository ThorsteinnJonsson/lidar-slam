#pragma once

#include <Eigen/Core>
#include <vector>

#include "imu/state.h"
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

  // Initial covariance seed, given as per-block standard deviations.
  // Most blocks are tight: roll/pitch are measured from gravity, while yaw and
  // position are gauge freedoms we fix by definition. Accel bias is the one
  // genuinely unknown quantity, so it gets a loose seed.
  double theta_std{0.017};  // attitude       ~1 deg
  double pos_std{1e-3};     // position       (origin by definition)
  double vel_std{1e-2};     // velocity       (at rest)
  double bias_gyro_std{1e-3};
  double bias_accel_std{0.1};  // loose: learned under motion
  double gravity_std{1e-2};
};

struct InitResult {
  bool ok{false};  // false if the window is not stationary
  State state;
  Eigen::Matrix<double, 17, 17> cov{Eigen::Matrix<double, 17, 17>::Zero()};

  // Diagnostics for logging.
  double accel_mean_norm{0.0};
  double max_accel_var{0.0};
  double max_gyro_var{0.0};
};

// Attempt static initialization over the given IMU window.
// Returns ok == false (with diagnostics filled) if the platform is moving.
InitResult initialize_static(const std::vector<ImuMeasurement>& imu,
                             const InitParams& params);

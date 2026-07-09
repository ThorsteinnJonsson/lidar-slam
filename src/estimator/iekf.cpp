#include "estimator/iekf.h"

#include <Eigen/Dense>
#include <numbers>

#include "estimator/outlier.h"

EkfResult iterated_update(const State& x_hat,
                          const Eigen::Matrix<double, 17, 17>& P,
                          const MeasurementFn& measure, const IekfConfig& cfg) {
  using Matrix17 = Eigen::Matrix<double, 17, 17>;

  const double inv_r = 1.0 / (cfg.sigma * cfg.sigma);  // R = sigma^2 I
  const Matrix17 I17 = Matrix17::Identity();
  const Matrix17 P_inv = P.inverse();

  State x = x_hat;
  Eigen::Matrix<double, Eigen::Dynamic, 17> H;
  Eigen::Matrix<double, 17, Eigen::Dynamic> K;
  int iters = 0;
  bool converged = false;
  bool have_gain = false;

  while (iters < cfg.max_iterations) {
    LinearizedMeasurement lm = measure(x);
    if (cfg.reject_outliers && lm.H.rows() > 0)
      lm = gate_measurement(lm, P, cfg.sigma, cfg.outlier_chi2);
    if (lm.H.rows() == 0) break;  // nothing observes the state this iteration
    H = lm.H;

    // Information-form gain: invert the 17x17 system, never the m x m
    // innovation.
    const Matrix17 S = H.transpose() * (inv_r * H) + P_inv;
    K = S.ldlt().solve(H.transpose() * inv_r);  // 17 x m
    have_gain = true;

    const Matrix17 KH = K * H;
    // Measurement pull plus the prior pull back toward x_hat, re-expressed in
    // the current tangent space (zero on the first iteration where x == x_hat).
    const Vector17 dx = -K * lm.z - (I17 - KH) * boxminus(x, x_hat);

    x = boxplus(x, dx);
    ++iters;

    if (dx.norm() < cfg.convergence_tol) {
      converged = true;
      break;
    }
  }

  EkfResult result;
  result.state = x;
  result.iterations = iters;
  // An empty system leaves the prior untouched, which counts as trivially done.
  result.converged = converged || !have_gain;
  // Diagnostic: how far the update rotated the state off the propagated prior.
  constexpr double kRadToDeg = 180.0 / std::numbers::pi;
  const Eigen::Vector3d dtheta_body = (x_hat.R.inverse() * x.R).log();
  result.update_dtheta_deg = dtheta_body.norm() * kRadToDeg;
  // Same correction rotated into the world frame (Ad_{R} = R for SO3), so its
  // horizontal component lines up with the gravity-tilt axis.
  result.update_dtheta_world_deg = (x_hat.R * dtheta_body) * kRadToDeg;
  if (have_gain) {
    const Matrix17 KH = K * H;
    const Matrix17 P_upd = (I17 - KH) * P;
    result.covariance = 0.5 * (P_upd + P_upd.transpose());  // resymmetrize
  } else {
    result.covariance = P;
  }
  return result;
}

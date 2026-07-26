#include "estimator/iekf.h"

#include <Eigen/Dense>

#include "estimator/outlier.h"

EkfResult iterated_update(const State& x_hat, const ErrorMatrix& P,
                          const MeasurementFn& measure, const IekfConfig& cfg) {
  const double inv_r = 1.0 / (cfg.sigma * cfg.sigma);  // R = sigma^2 I
  const ErrorMatrix I17 = ErrorMatrix::Identity();
  const ErrorMatrix P_inv = P.inverse();

  State x = x_hat;
  MeasurementJacobian H;
  KalmanGain K;
  int iters = 0;
  bool converged = false;
  bool have_gain = false;

  while (iters < cfg.max_iterations) {
    LinearizedMeasurement lm = measure(x);
    if (cfg.reject_outliers && lm.H.rows() > 0)
      lm = gate_measurement(lm, P, cfg.sigma, cfg.outlier_chi2);
    if (lm.H.rows() == 0) break;  // nothing observes the state this iteration
    H = lm.H;

    // Information-form gain: invert the kErrorDim system, never the m x m
    // innovation.
    const ErrorMatrix S = H.transpose() * (inv_r * H) + P_inv;
    K = S.ldlt().solve(H.transpose() * inv_r);  // kErrorDim x m
    have_gain = true;

    const ErrorMatrix KH = K * H;
    // Measurement pull plus the prior pull back toward x_hat, re-expressed in
    // the current tangent space (zero on the first iteration where x == x_hat).
    const ErrorState dx = -K * lm.z - (I17 - KH) * boxminus(x, x_hat);

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
  if (have_gain) {
    const ErrorMatrix KH = K * H;
    const ErrorMatrix P_upd = (I17 - KH) * P;
    result.covariance = 0.5 * (P_upd + P_upd.transpose());  // resymmetrize
  } else {
    result.covariance = P;
  }
  return result;
}

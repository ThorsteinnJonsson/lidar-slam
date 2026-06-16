#include "estimator/iekf.h"

#include <Eigen/Dense>

#include "estimator/outlier.h"

EkfResult iterated_update(const State& x_hat,
                          const Eigen::Matrix<double, 18, 18>& P,
                          const MeasurementFn& measure, const IekfConfig& cfg) {
  using Matrix18 = Eigen::Matrix<double, 18, 18>;

  const double inv_r = 1.0 / (cfg.sigma * cfg.sigma);  // R = sigma^2 I
  const Matrix18 I18 = Matrix18::Identity();
  const Matrix18 P_inv = P.inverse();

  State x = x_hat;
  Eigen::Matrix<double, Eigen::Dynamic, 18> H;
  Eigen::Matrix<double, 18, Eigen::Dynamic> K;
  int iters = 0;
  bool converged = false;
  bool have_gain = false;

  while (iters < cfg.max_iterations) {
    LinearizedMeasurement lm = measure(x);
    if (cfg.reject_outliers && lm.H.rows() > 0)
      lm = gate_measurement(lm, P, cfg.sigma, cfg.outlier_chi2);
    if (lm.H.rows() == 0) break;  // nothing observes the state this iteration
    H = lm.H;

    // Information-form gain: invert the 18x18 system, never the m x m
    // innovation.
    const Matrix18 S = H.transpose() * (inv_r * H) + P_inv;
    K = S.ldlt().solve(H.transpose() * inv_r);  // 18 x m
    have_gain = true;

    const Matrix18 KH = K * H;
    // Measurement pull plus the prior pull back toward x_hat, re-expressed in
    // the current tangent space (zero on the first iteration where x == x_hat).
    const Vector18 dx = -K * lm.z - (I18 - KH) * boxminus(x, x_hat);

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
    const Matrix18 KH = K * H;
    const Matrix18 P_upd = (I18 - KH) * P;
    result.covariance = 0.5 * (P_upd + P_upd.transpose());  // resymmetrize
  } else {
    result.covariance = P;
  }
  return result;
}

#include "estimator/outlier.h"

#include <vector>

LinearizedMeasurement gate_measurement(const LinearizedMeasurement& m,
                                       const Eigen::Matrix<double, 17, 17>& P,
                                       double sigma, double chi2_thresh) {
  const Eigen::Index rows = m.H.rows();
  const double r = sigma * sigma;  // measurement noise variance

  // Precompute P H^T (17 x m) so each innovation variance H_i P H_i^T is a
  // single row-column dot product.
  const Eigen::Matrix<double, 17, Eigen::Dynamic> PHt = P * m.H.transpose();

  std::vector<Eigen::Index> keep;
  keep.reserve(static_cast<size_t>(rows));
  for (Eigen::Index i = 0; i < rows; ++i) {
    const double innovation_var = m.H.row(i).dot(PHt.col(i)) + r;
    const double d2 = m.z(i) * m.z(i) / innovation_var;
    if (d2 <= chi2_thresh) keep.push_back(i);
  }

  LinearizedMeasurement out;
  out.H = Eigen::Matrix<double, Eigen::Dynamic, 17>(
      static_cast<Eigen::Index>(keep.size()), 17);
  out.z = Eigen::VectorXd(static_cast<Eigen::Index>(keep.size()));
  for (size_t k = 0; k < keep.size(); ++k) {
    out.H.row(static_cast<Eigen::Index>(k)) = m.H.row(keep[k]);
    out.z(static_cast<Eigen::Index>(k)) = m.z(keep[k]);
  }
  return out;
}

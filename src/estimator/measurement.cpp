#include "estimator/measurement.h"

#include "math/skew.h"

LinearizedMeasurement build_measurement(
    const State& s, const std::vector<PlaneMatch>& matches) {
  const Eigen::Index m = static_cast<Eigen::Index>(matches.size());

  LinearizedMeasurement out;
  out.H = MeasurementJacobian::Zero(m, kErrorDim);
  out.z = Eigen::VectorXd::Zero(m);

  const Eigen::Matrix3d R = s.R.matrix();
  for (Eigen::Index i = 0; i < m; ++i) {
    const Eigen::Vector3d p_I = matches[i].point.cast<double>();
    const Eigen::Vector3d n = matches[i].normal.cast<double>();

    out.H.block<1, 3>(i, 0) = -n.transpose() * R * skew(p_I);  // δθ
    out.H.block<1, 3>(i, 3) = n.transpose();                   // δp
    out.z(i) = static_cast<double>(matches[i].residual);
  }
  return out;
}

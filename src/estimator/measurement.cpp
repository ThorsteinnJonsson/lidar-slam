#include "estimator/measurement.h"

namespace {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d s;
  // clang-format off
  s <<     0, -v.z(),  v.y(),
       v.z(),      0, -v.x(),
      -v.y(),  v.x(),      0;
  // clang-format on
  return s;
}

}  // namespace

LinearizedMeasurement build_measurement(
    const State& s, const std::vector<PlaneMatch>& matches) {
  const Eigen::Index m = static_cast<Eigen::Index>(matches.size());

  LinearizedMeasurement out;
  out.H = Eigen::Matrix<double, Eigen::Dynamic, 18>::Zero(m, 18);
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

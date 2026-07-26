#include "estimator/measurement.h"

#include "math/skew.h"

LinearizedMeasurement build_measurement(
    const State& s, const std::vector<PlaneMatch>& matches) {
  const Eigen::Index m = static_cast<Eigen::Index>(matches.size());

  LinearizedMeasurement out;
  out.H = MeasurementJacobian::Zero(m, kErrorDim);
  out.z = Eigen::VectorXd::Zero(m);

  const Eigen::Matrix3d R = s.R.matrix();
  const Eigen::Matrix3d R_ext = s.R_imu_lidar.matrix();
  for (Eigen::Index i = 0; i < m; ++i) {
    // Matches are associated in the lidar frame, so `point` is p_L.
    const Eigen::Vector3d p_L = matches[i].point.cast<double>();
    const Eigen::Vector3d p_I = R_ext * p_L + s.p_imu_lidar;
    const Eigen::Vector3d n = matches[i].normal.cast<double>();
    const Eigen::RowVector3d nR = n.transpose() * R;

    out.H.block<1, 3>(i, kIdxTheta) = -nR * skew(p_I);
    out.H.block<1, 3>(i, kIdxPos) = n.transpose();
    // Extrinsic rotation: R_ext' = R_ext Exp(δθ_ext) moves p_I by
    // -R_ext [p_L]× δθ_ext, which world-frame becomes -R R_ext [p_L]×.
    out.H.block<1, 3>(i, kIdxExtRot) = -nR * R_ext * skew(p_L);
    // Extrinsic translation: p_ext shifts p_I directly, so p_W by R δp_ext.
    out.H.block<1, 3>(i, kIdxExtPos) = nR;
    out.z(i) = static_cast<double>(matches[i].residual);
  }
  return out;
}

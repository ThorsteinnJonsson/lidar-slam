#include "preprocess/deskew.h"

#include <algorithm>

namespace {

// Interpolate a body pose at time t_ns from a (time-sorted) trajectory.
// Clamps to the endpoints when t_ns lies outside the covered range.
Sophus::SE3d interpolate_pose(const std::vector<StampedPose>& traj,
                              uint64_t t_ns) {
  if (t_ns <= traj.front().t_ns) return traj.front().T_W_I;
  if (t_ns >= traj.back().t_ns) return traj.back().T_W_I;

  // First pose with t_ns strictly greater than the query.
  auto hi = std::upper_bound(
      traj.begin(), traj.end(), t_ns,
      [](uint64_t t, const StampedPose& p) { return t < p.t_ns; });
  auto lo = std::prev(hi);

  const double alpha = static_cast<double>(t_ns - lo->t_ns) /
                       static_cast<double>(hi->t_ns - lo->t_ns);

  // SE3 interpolation via the Lie-algebra tangent between the two poses.
  const Sophus::SE3d rel = lo->T_W_I.inverse() * hi->T_W_I;
  return lo->T_W_I * Sophus::SE3d::exp(alpha * rel.log());
}

}  // namespace

std::vector<StampedPose> build_scan_trajectory(const ImuBuffer& imu,
                                               uint64_t t_start_ns,
                                               uint64_t t_end_ns) {
  const std::vector<ImuMeasurement> meas =
      imu.get_between(t_start_ns, t_end_ns);

  std::vector<StampedPose> traj;
  traj.reserve(meas.size());

  // Integrate gyro from identity. Position stays zero (rotation-only); see the
  // TODO(velocity) note in the header.
  const Eigen::Vector3d zero_p = Eigen::Vector3d::Zero();
  Sophus::SO3d R;  // identity
  traj.emplace_back(meas.front().stamp.to_nsec(), Sophus::SE3d(R, zero_p));

  for (size_t k = 1; k < meas.size(); ++k) {
    const double dt = static_cast<double>(meas[k].stamp.to_nsec() -
                                          meas[k - 1].stamp.to_nsec()) *
                      1e-9;
    // Midpoint angular velocity (bias assumed zero until the EKF estimates it).
    const Eigen::Vector3d w_mid =
        0.5 * (meas[k - 1].angular_velocity + meas[k].angular_velocity);
    R = R * Sophus::SO3d::exp(w_mid * dt);
    traj.emplace_back(meas[k].stamp.to_nsec(), Sophus::SE3d(R, zero_p));
  }

  return traj;
}

PointCloud deskew(const PointCloud& cloud,
                  const std::vector<StampedPose>& trajectory,
                  const Sophus::SE3d& T_imu_lidar, uint64_t t_ref_ns) {
  PointCloud out = cloud;
  if (trajectory.empty()) return out;

  const Sophus::SE3d T_lidar_imu = T_imu_lidar.inverse();
  const Sophus::SE3d T_W_I_ref = interpolate_pose(trajectory, t_ref_ns);
  const Sophus::SE3d T_W_I_ref_inv = T_W_I_ref.inverse();
  const uint64_t base_ns = cloud.stamp.to_nsec();

  for (size_t i = 0; i < cloud.size(); ++i) {
    const uint64_t t_i = base_ns + cloud.t_offset_ns[i];
    const Sophus::SE3d T_W_I_i = interpolate_pose(trajectory, t_i);

    // Relative body motion from the point's capture time to the reference time,
    // re-expressed in the lidar frame.
    const Sophus::SE3d correction =
        T_lidar_imu * (T_W_I_ref_inv * T_W_I_i) * T_imu_lidar;

    const Eigen::Vector3d p_i(cloud.x[i], cloud.y[i], cloud.z[i]);
    const Eigen::Vector3d p_ref = correction * p_i;

    out.x[i] = static_cast<float>(p_ref.x());
    out.y[i] = static_cast<float>(p_ref.y());
    out.z[i] = static_cast<float>(p_ref.z());
  }

  return out;
}

#include "io/result_writer.h"

#include <iomanip>

ResultWriter::ResultWriter(const std::filesystem::path& output_dir, bool has_gt,
                           const Eigen::Vector3d& imu_to_gt_point)
    : has_gt_(has_gt), imu_to_gt_point_(imu_to_gt_point) {
  std::filesystem::create_directories(output_dir);
  traj_out_.open(output_dir / "trajectory.tum");
  if (has_gt_) {
    est_gt_out_.open(output_dir / "estimate_gt.tum");
    gt_out_.open(output_dir / "gt.tum");
  }
  for (std::ofstream* os : {&traj_out_, &est_gt_out_, &gt_out_})
    *os << std::fixed << std::setprecision(9);
}

void ResultWriter::write_pose(double t_sec, const State& state) {
  const Eigen::Vector3d& p = state.p;
  const Eigen::Quaterniond q = state.R.unit_quaternion();
  traj_out_ << t_sec << ' ' << p.x() << ' ' << p.y() << ' ' << p.z() << ' '
            << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';

  // Estimate at the GT reference point. Orientation is irrelevant to
  // position-only GT, so write identity.
  if (has_gt_) {
    const Eigen::Vector3d p_gt = p + state.R * imu_to_gt_point_;
    est_gt_out_ << t_sec << ' ' << p_gt.x() << ' ' << p_gt.y() << ' '
                << p_gt.z() << " 0 0 0 1\n";
  }
}

void ResultWriter::write_gt(const GtRow& gt) {
  gt_out_ << gt.t_sec << ' ' << gt.pos.x() << ' ' << gt.pos.y() << ' '
          << gt.pos.z() << " 0 0 0 1\n";
}

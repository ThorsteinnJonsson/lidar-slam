#include "io/result_writer.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::filesystem::path temp_dir(const std::string& name) {
  const auto dir =
      std::filesystem::temp_directory_path() / "lidar_slam_rw_test" / name;
  std::filesystem::remove_all(dir);
  return dir;
}

std::vector<std::vector<double>> read_rows(const std::filesystem::path& p) {
  std::vector<std::vector<double>> rows;
  std::ifstream in(p);
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    std::vector<double> vals;
    double v;
    while (ss >> v) vals.push_back(v);
    rows.push_back(vals);
  }
  return rows;
}

State pose(double px, double py, double pz) {
  State s;
  s.p = {px, py, pz};  // R defaults to identity
  return s;
}

}  // namespace

TEST(ResultWriter, WritesTrajectoryAndLeverArmEstimate) {
  const auto dir = temp_dir("with_gt");
  {
    ResultWriter w(dir, /*has_gt=*/true, Eigen::Vector3d(0.1, 0.0, 0.0));
    w.write_pose(100.5, pose(1.0, 2.0, 3.0));
    w.write_gt(GtRow{100.5, Eigen::Vector3d(5.0, 6.0, 7.0)});
  }  // streams flush on destruction

  const auto traj = read_rows(dir / "trajectory.tum");
  ASSERT_EQ(traj.size(), 1u);
  ASSERT_EQ(traj[0].size(), 8u);  // t tx ty tz qx qy qz qw
  EXPECT_DOUBLE_EQ(traj[0][0], 100.5);
  EXPECT_DOUBLE_EQ(traj[0][1], 1.0);
  EXPECT_DOUBLE_EQ(traj[0][7], 1.0);  // identity quaternion w

  // Estimate at the GT point: identity rotation, so p + lever = (1.1, 2, 3).
  const auto est = read_rows(dir / "estimate_gt.tum");
  ASSERT_EQ(est.size(), 1u);
  EXPECT_NEAR(est[0][1], 1.1, 1e-9);
  EXPECT_NEAR(est[0][2], 2.0, 1e-9);
  EXPECT_NEAR(est[0][3], 3.0, 1e-9);

  const auto gt = read_rows(dir / "gt.tum");
  ASSERT_EQ(gt.size(), 1u);
  EXPECT_DOUBLE_EQ(gt[0][1], 5.0);
  EXPECT_DOUBLE_EQ(gt[0][3], 7.0);
}

TEST(ResultWriter, NoGtWritesOnlyTrajectory) {
  const auto dir = temp_dir("no_gt");
  {
    ResultWriter w(dir, /*has_gt=*/false, Eigen::Vector3d::Zero());
    w.write_pose(1.0, pose(0.0, 0.0, 0.0));
  }
  EXPECT_TRUE(std::filesystem::exists(dir / "trajectory.tum"));
  EXPECT_FALSE(std::filesystem::exists(dir / "estimate_gt.tum"));
  EXPECT_FALSE(std::filesystem::exists(dir / "gt.tum"));
}

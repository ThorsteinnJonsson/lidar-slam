#include "io/params.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace {

// Write `text` to a unique file under the temp dir and return its path. Each
// test gets its own name so they can run in any order.
std::filesystem::path write_yaml(const std::string& name,
                                 const std::string& text) {
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() / "lidar_slam_params_test";
  std::filesystem::create_directories(dir);
  const std::filesystem::path path = dir / (name + ".yaml");
  std::ofstream out(path);
  out << text;
  return path;
}

}  // namespace

TEST(Params, FullFileOverridesEveryDefault) {
  // Every value differs from the compiled-in default, so a field that silently
  // failed to load would show up as a mismatch.
  const auto path = write_yaml("full", R"(
lidar_time_offset_sec: -0.25
preprocess:
  scan_voxel_leaf: 0.75
init:
  imu_count: 321
  gravity_magnitude: 9.81
  max_accel_var: 0.06
  max_gyro_var: 0.02
  theta_std: 0.03
  pos_std: 0.002
  vel_std: 0.02
  bias_gyro_std: 0.002
  bias_accel_std: 0.02
  gravity_std: 0.02
  ext_rot_std: 0.007
  ext_trans_std: 0.004
imu_noise:
  gyro_noise_std: 0.11
  accel_noise_std: 0.22
  gyro_rw_std: 0.033
  accel_rw_std: 0.044
iekf:
  sigma: 0.05
  max_iterations: 9
  convergence_tol: 0.002
  reject_outliers: false
  outlier_chi2: 6.635
association:
  num_neighbors: 7
  max_neighbor_dist2: 8.0
  max_plane_dist: 0.2
  min_quality: 0.8
map:
  voxel_on_insert: false
  voxel_leaf: 0.3
  crop:
    enabled: false
    sensor_range: 100.0
    margin_factor: 1.25
    keep_factor: 3.0
estimator:
  enable_extrinsic_estimation: true
)");

  std::vector<std::string> warnings;
  const Params p = load_params(path, &warnings);

  EXPECT_TRUE(warnings.empty());
  EXPECT_DOUBLE_EQ(p.lidar_time_offset_sec, -0.25);
  EXPECT_DOUBLE_EQ(p.scan_voxel_leaf, 0.75);
  EXPECT_EQ(p.init_imu_count, 321u);
  EXPECT_TRUE(p.enable_extrinsic_estimation);

  EXPECT_DOUBLE_EQ(p.init.gravity_magnitude, 9.81);
  EXPECT_DOUBLE_EQ(p.init.max_accel_var, 0.06);
  EXPECT_DOUBLE_EQ(p.init.max_gyro_var, 0.02);
  EXPECT_DOUBLE_EQ(p.init.theta_std, 0.03);
  EXPECT_DOUBLE_EQ(p.init.pos_std, 0.002);
  EXPECT_DOUBLE_EQ(p.init.vel_std, 0.02);
  EXPECT_DOUBLE_EQ(p.init.bias_gyro_std, 0.002);
  EXPECT_DOUBLE_EQ(p.init.bias_accel_std, 0.02);
  EXPECT_DOUBLE_EQ(p.init.gravity_std, 0.02);
  EXPECT_DOUBLE_EQ(p.init.ext_rot_std, 0.007);
  EXPECT_DOUBLE_EQ(p.init.ext_trans_std, 0.004);

  EXPECT_DOUBLE_EQ(p.noise.gyro_noise_std, 0.11);
  EXPECT_DOUBLE_EQ(p.noise.accel_noise_std, 0.22);
  EXPECT_DOUBLE_EQ(p.noise.gyro_rw_std, 0.033);
  EXPECT_DOUBLE_EQ(p.noise.accel_rw_std, 0.044);

  EXPECT_DOUBLE_EQ(p.iekf.sigma, 0.05);
  EXPECT_EQ(p.iekf.max_iterations, 9);
  EXPECT_DOUBLE_EQ(p.iekf.convergence_tol, 0.002);
  EXPECT_FALSE(p.iekf.reject_outliers);
  EXPECT_DOUBLE_EQ(p.iekf.outlier_chi2, 6.635);

  EXPECT_EQ(p.assoc.num_neighbors, 7);
  EXPECT_FLOAT_EQ(p.assoc.max_neighbor_dist2, 8.0f);
  EXPECT_FLOAT_EQ(p.assoc.max_plane_dist, 0.2f);
  EXPECT_FLOAT_EQ(p.assoc.min_quality, 0.8f);

  EXPECT_FALSE(p.map.voxel_on_insert);
  EXPECT_FLOAT_EQ(p.map.voxel_leaf, 0.3f);
  EXPECT_FALSE(p.map.crop.enabled);
  EXPECT_FLOAT_EQ(p.map.crop.sensor_range, 100.0f);
  EXPECT_FLOAT_EQ(p.map.crop.margin_factor, 1.25f);
  EXPECT_FLOAT_EQ(p.map.crop.keep_factor, 3.0f);
}

TEST(Params, AbsentKeysKeepStructDefaults) {
  // Only two keys are present; everything else must fall back to the value
  // compiled into the owning struct.
  const auto path = write_yaml("partial", R"(
iekf:
  sigma: 0.05
map:
  crop:
    enabled: false
)");

  const Params defaults;
  std::vector<std::string> warnings;
  const Params p = load_params(path, &warnings);

  EXPECT_TRUE(warnings.empty());
  // Overridden.
  EXPECT_DOUBLE_EQ(p.iekf.sigma, 0.05);
  EXPECT_FALSE(p.map.crop.enabled);
  // Untouched, including siblings within the same sections.
  EXPECT_EQ(p.iekf.max_iterations, defaults.iekf.max_iterations);
  EXPECT_DOUBLE_EQ(p.iekf.outlier_chi2, defaults.iekf.outlier_chi2);
  EXPECT_FLOAT_EQ(p.map.crop.sensor_range, defaults.map.crop.sensor_range);
  EXPECT_FLOAT_EQ(p.map.voxel_leaf, defaults.map.voxel_leaf);
  EXPECT_DOUBLE_EQ(p.lidar_time_offset_sec, defaults.lidar_time_offset_sec);
  EXPECT_EQ(p.init_imu_count, defaults.init_imu_count);
  EXPECT_DOUBLE_EQ(p.noise.gyro_noise_std, defaults.noise.gyro_noise_std);
}

TEST(Params, EmptyFileYieldsAllDefaults) {
  const auto path = write_yaml("empty_map", "{}\n");
  const Params defaults;
  const Params p = load_params(path);

  EXPECT_DOUBLE_EQ(p.lidar_time_offset_sec, defaults.lidar_time_offset_sec);
  EXPECT_DOUBLE_EQ(p.iekf.sigma, defaults.iekf.sigma);
  EXPECT_FLOAT_EQ(p.assoc.min_quality, defaults.assoc.min_quality);
}

TEST(Params, UnknownKeysAreReportedNotFatal) {
  // A typo must not be silently swallowed: the surrounding valid keys still
  // load, and the bad ones come back as warnings naming their full path.
  const auto path = write_yaml("unknown", R"(
totally_bogus: 1
iekf:
  sigma: 0.05
  simga: 0.09
map:
  crop:
    enbaled: false
)");

  std::vector<std::string> warnings;
  const Params p = load_params(path, &warnings);

  EXPECT_DOUBLE_EQ(p.iekf.sigma, 0.05);  // the valid neighbor still loaded
  ASSERT_EQ(warnings.size(), 3u);

  const auto mentions = [&](const std::string& needle) {
    for (const std::string& w : warnings)
      if (w.find(needle) != std::string::npos) return true;
    return false;
  };
  EXPECT_TRUE(mentions("totally_bogus"));
  EXPECT_TRUE(mentions("iekf.simga"));
  EXPECT_TRUE(mentions("map.crop.enbaled"));
}

TEST(Params, ExtrinsicTranslationAndRotation) {
  const auto path = write_yaml("extrinsic", R"(
extrinsic:
  translation: [0.04165, 0.02326, -0.0284]
  rotation: [0.7071067811865476, 0.0, 0.0, 0.7071067811865476]
)");
  const Params p = load_params(path);
  EXPECT_NEAR(p.extrinsic.translation().x(), 0.04165, 1e-9);
  EXPECT_NEAR(p.extrinsic.translation().z(), -0.0284, 1e-9);
  // 90 deg about z: x axis maps to y.
  const Eigen::Vector3d x = p.extrinsic.so3() * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(x.y(), 1.0, 1e-6);
}

TEST(Params, ExtrinsicWrongLengthThrows) {
  const auto path = write_yaml("ext_bad", R"(
extrinsic:
  translation: [1.0, 2.0]
)");
  EXPECT_THROW(load_params(path), std::runtime_error);
}

TEST(Params, MissingFileThrows) {
  EXPECT_THROW(load_params("definitely/not/here/params.yaml"),
               std::runtime_error);
}

TEST(Params, MalformedYamlThrows) {
  const auto path = write_yaml("malformed", "iekf: [unclosed\n");
  EXPECT_THROW(load_params(path), std::runtime_error);
}

TEST(Params, NonMappingRootThrows) {
  const auto path = write_yaml("scalar_root", "just-a-string\n");
  EXPECT_THROW(load_params(path), std::runtime_error);
}

TEST(Params, WrongValueTypeThrowsNamingTheKey) {
  const auto path = write_yaml("badtype", R"(
iekf:
  max_iterations: not-an-int
)");
  try {
    load_params(path);
    FAIL() << "expected a throw";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("max_iterations"), std::string::npos)
        << "message should name the offending key: " << e.what();
  }
}

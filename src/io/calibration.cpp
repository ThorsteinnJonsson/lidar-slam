#include "io/calibration.h"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

// Read the file and strip lines that break yaml-cpp:
//   - %YAML:1.0 directives (OpenCV format, not standard YAML)
//   - Lines containing a given key prefix (used to drop duplicate keys before
//     yaml-cpp sees them; we extract those values from raw text ourselves)
std::string read_stripped(
    const std::filesystem::path& path,
    std::initializer_list<std::string_view> drop_keys = {}) {
  std::ifstream f(path);
  if (!f)
    throw std::runtime_error("Cannot open calibration file: " + path.string());

  std::string result, line;
  while (std::getline(f, line)) {
    if (line.starts_with("%YAML")) continue;
    bool drop = false;
    for (auto key : drop_keys) {
      if (line.find(key) != std::string::npos) {
        drop = true;
        break;
      }
    }
    if (!drop) result += line + '\n';
  }
  return result;
}

// Parse a !!opencv-matrix node into an Eigen::Isometry3d.
// Expects sub-keys: rows, cols, data (row-major flat array).
Eigen::Isometry3d parse_opencv_transform(const YAML::Node& node) {
  int rows = node["rows"].as<int>();
  int cols = node["cols"].as<int>();
  if (rows != 4 || cols != 4) {
    throw std::runtime_error("Expected a 4x4 opencv-matrix for a transform");
  }
  auto data = node["data"].as<std::vector<double>>();

  Eigen::Matrix4d mat;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) mat(i, j) = data[i * 4 + j];

  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = mat.topLeftCorner<3, 3>();
  iso.translation() = mat.topRightCorner<3, 1>();
  return iso;
}

// The NTU VIRAL imu_v100.yaml has a duplicate `gyro_std` key: the first is the
// noise std and the second is the random-walk std (mislabeled). yaml-cpp keeps
// only the last value, so we scan the raw text to recover both.
std::pair<double, double> extract_gyro_stds(const std::string& raw) {
  std::vector<double> vals;
  std::istringstream ss(raw);
  std::string line;
  while (std::getline(ss, line)) {
    auto first_char = line.find_first_not_of(" \t");
    if (first_char == std::string::npos || line[first_char] == '#') continue;
    if (line.find("gyro_std:") == std::string::npos) continue;

    std::string rhs = line.substr(line.find(':') + 1);
    if (auto comment = rhs.find('#'); comment != std::string::npos)
      rhs = rhs.substr(0, comment);
    try {
      vals.push_back(std::stod(rhs));
    } catch (...) {
    }
  }

  if (vals.size() >= 2) return {vals[0], vals[1]};
  if (vals.size() == 1) return {vals[0], 0.0};
  return {0.0, 0.0};
}

}  // namespace

ImuCalibration load_imu_calibration(const std::filesystem::path& path) {
  // Read raw first (for gyro_std duplicate extraction), then strip duplicate
  // keys before yaml-cpp sees them.
  std::ifstream raw_f(path);
  std::string raw((std::istreambuf_iterator<char>(raw_f)), {});

  std::string yaml_content = read_stripped(path, {"gyro_std"});
  auto root = YAML::Load(yaml_content);

  ImuCalibration cal;
  cal.topic = root["imu_topic"].as<std::string>();
  cal.T_body_imu = parse_opencv_transform(root["T_Body_Imu"]);
  cal.accel_noise_std = root["accel_std"].as<double>();
  cal.accel_rw_std = root["accel_rw"].as<double>();

  auto [gyro_noise, gyro_rw] = extract_gyro_stds(raw);
  cal.gyro_noise_std = gyro_noise;
  cal.gyro_rw_std = gyro_rw;

  return cal;
}

LidarCalibration load_lidar_calibration(const std::filesystem::path& path) {
  std::string raw = read_stripped(path);
  auto root = YAML::Load(raw);

  LidarCalibration cal;
  cal.topic = root["pointcloud_topic"].as<std::string>();
  cal.T_body_lidar = parse_opencv_transform(root["T_Body_Lidar"]);
  cal.vert_res = root["VERT_RES"].as<int>();
  cal.horz_res = root["HORZ_RES"].as<int>();

  return cal;
}

Sophus::SE3d imu_from_lidar(const ImuCalibration& imu,
                            const LidarCalibration& lidar) {
  const Eigen::Isometry3d iso = imu.T_body_imu.inverse() * lidar.T_body_lidar;
  // Normalize the rotation block before handing it to Sophus, which asserts a
  // valid SO3 element (the YAML matrices may not be exactly orthonormal).
  const Eigen::Quaterniond q(iso.linear());
  return Sophus::SE3d(q.normalized(), iso.translation());
}

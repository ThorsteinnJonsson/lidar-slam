#pragma once

#include <Eigen/Geometry>
#include <filesystem>
#include <string>

struct ImuCalibration {
  std::string topic;
  Eigen::Isometry3d T_body_imu{Eigen::Isometry3d::Identity()};
  double accel_noise_std{0.0};
  double accel_rw_std{0.0};
  double gyro_noise_std{0.0};
  double gyro_rw_std{0.0};
};

struct LidarCalibration {
  std::string topic;
  Eigen::Isometry3d T_body_lidar{Eigen::Isometry3d::Identity()};
  int vert_res{0};
  int horz_res{0};
};

ImuCalibration load_imu_calibration(const std::filesystem::path& path);
LidarCalibration load_lidar_calibration(const std::filesystem::path& path);

#pragma once

#include <cstddef>
#include <span>

#include "types.h"

// A timestamped pose (geometry_msgs/PoseStamped).
struct PoseStampedMsg {
  Timestamp stamp;
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

// Deserialize a raw sensor_msgs/Imu message byte buffer.
ImuMeasurement deserialize_imu(std::span<const std::byte> data);

// Deserialize a raw geometry_msgs/PoseStamped message byte buffer.
PoseStampedMsg deserialize_pose_stamped(std::span<const std::byte> data);

// Deserialize a raw sensor_msgs/PointCloud2 message byte buffer.
// Field offsets and datatypes are read from the embedded field descriptors,
// so this works regardless of driver-specific field ordering or naming.
PointCloud deserialize_pointcloud2(std::span<const std::byte> data);

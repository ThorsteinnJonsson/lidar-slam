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

// How a lidar encodes its per-point time field. The field name and datatype are
// read from the message, but not whether the value is an offset or an absolute
// time, so the caller (which knows the lidar) states it.
enum class PointTimeMode {
  OffsetNanos,      // integer ns from the header stamp (e.g. Ouster `t`)
  AbsoluteSeconds,  // absolute time in seconds (e.g. Hesai `timestamp`, f64)
};

// Deserialize a raw sensor_msgs/PointCloud2 message byte buffer.
// Field offsets and datatypes are read from the embedded field descriptors,
// so this works regardless of driver-specific field ordering or naming.
PointCloud deserialize_pointcloud2(
    std::span<const std::byte> data,
    PointTimeMode time_mode = PointTimeMode::OffsetNanos);

// Deserialize a raw livox_ros_driver/CustomMsg message byte buffer (Livox Avia,
// as used by FAST-LIVO2). Per-point time is the message's `timebase` plus each
// point's `offset_time`, normalized to ns from the header stamp. Reflectivity
// maps to intensity and the point's laser line to ring.
PointCloud deserialize_livox_custommsg(std::span<const std::byte> data);

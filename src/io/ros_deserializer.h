#pragma once

#include <cstddef>
#include <span>

#include "types.h"

// Deserialize a raw sensor_msgs/Imu message byte buffer.
ImuMeasurement deserialize_imu(std::span<const std::byte> data);

// Deserialize a raw sensor_msgs/PointCloud2 message byte buffer.
// Field offsets and datatypes are read from the embedded field descriptors,
// so this works regardless of driver-specific field ordering or naming.
PointCloud deserialize_pointcloud2(std::span<const std::byte> data);

#include "io/ros_deserializer.h"

#include <cstring>

#include "io/span_reader.h"

// ROS1 PointField datatype constants
namespace {
constexpr uint8_t PF_INT8 = 1;
constexpr uint8_t PF_UINT8 = 2;
constexpr uint8_t PF_INT16 = 3;
constexpr uint8_t PF_UINT16 = 4;
constexpr uint8_t PF_INT32 = 5;
constexpr uint8_t PF_UINT32 = 6;
constexpr uint8_t PF_FLOAT32 = 7;
constexpr uint8_t PF_FLOAT64 = 8;

float read_as_float(const std::byte* p, uint8_t datatype) {
  switch (datatype) {
    case PF_FLOAT32: {
      float v;
      std::memcpy(&v, p, 4);
      return v;
    }
    case PF_FLOAT64: {
      double v;
      std::memcpy(&v, p, 8);
      return static_cast<float>(v);
    }
    case PF_UINT8: {
      uint8_t v;
      std::memcpy(&v, p, 1);
      return static_cast<float>(v);
    }
    case PF_INT8: {
      int8_t v;
      std::memcpy(&v, p, 1);
      return static_cast<float>(v);
    }
    case PF_UINT16: {
      uint16_t v;
      std::memcpy(&v, p, 2);
      return static_cast<float>(v);
    }
    case PF_INT16: {
      int16_t v;
      std::memcpy(&v, p, 2);
      return static_cast<float>(v);
    }
    case PF_UINT32: {
      uint32_t v;
      std::memcpy(&v, p, 4);
      return static_cast<float>(v);
    }
    case PF_INT32: {
      int32_t v;
      std::memcpy(&v, p, 4);
      return static_cast<float>(v);
    }
    default:
      return 0.0f;
  }
}

uint16_t read_as_u16(const std::byte* p, uint8_t datatype) {
  switch (datatype) {
    case PF_UINT8: {
      uint8_t v;
      std::memcpy(&v, p, 1);
      return v;
    }
    case PF_UINT16: {
      uint16_t v;
      std::memcpy(&v, p, 2);
      return v;
    }
    case PF_UINT32: {
      uint32_t v;
      std::memcpy(&v, p, 4);
      return static_cast<uint16_t>(v);
    }
    default:
      return 0;
  }
}

uint32_t read_as_u32(const std::byte* p, uint8_t datatype) {
  switch (datatype) {
    case PF_UINT8: {
      uint8_t v;
      std::memcpy(&v, p, 1);
      return v;
    }
    case PF_UINT16: {
      uint16_t v;
      std::memcpy(&v, p, 2);
      return v;
    }
    case PF_UINT32: {
      uint32_t v;
      std::memcpy(&v, p, 4);
      return v;
    }
    case PF_FLOAT32: {
      float v;
      std::memcpy(&v, p, 4);
      return static_cast<uint32_t>(v);
    }
    default:
      return 0;
  }
}

double read_as_double(const std::byte* p, uint8_t datatype) {
  switch (datatype) {
    case PF_FLOAT64: {
      double v;
      std::memcpy(&v, p, 8);
      return v;
    }
    case PF_FLOAT32: {
      float v;
      std::memcpy(&v, p, 4);
      return v;
    }
    case PF_UINT32: {
      uint32_t v;
      std::memcpy(&v, p, 4);
      return v;
    }
    default:
      return 0.0;
  }
}

}  // namespace

// ── sensor_msgs/Imu
// ───────────────────────────────────────────────────────────
//
// Wire layout (little-endian):
//   std_msgs/Header: uint32 seq, uint32 secs, uint32 nsecs, string frame_id
//   geometry_msgs/Quaternion: float64 x, y, z, w
//   float64[9] orientation_covariance
//   geometry_msgs/Vector3: float64 x, y, z  (angular_velocity)
//   float64[9] angular_velocity_covariance
//   geometry_msgs/Vector3: float64 x, y, z  (linear_acceleration)
//   float64[9] linear_acceleration_covariance

ImuMeasurement deserialize_imu(std::span<const std::byte> data) {
  SpanReader r(data);
  ImuMeasurement imu;

  r.read<uint32_t>();  // seq
  imu.stamp.secs = r.read<uint32_t>();
  imu.stamp.nsecs = r.read<uint32_t>();
  r.read_string();  // frame_id

  double ox = r.read<double>();
  double oy = r.read<double>();
  double oz = r.read<double>();
  double ow = r.read<double>();
  imu.orientation = Eigen::Quaterniond(ow, ox, oy, oz);

  r.skip(9 * sizeof(double));  // orientation_covariance

  imu.angular_velocity.x() = r.read<double>();
  imu.angular_velocity.y() = r.read<double>();
  imu.angular_velocity.z() = r.read<double>();

  r.skip(9 * sizeof(double));  // angular_velocity_covariance

  imu.linear_acceleration.x() = r.read<double>();
  imu.linear_acceleration.y() = r.read<double>();
  imu.linear_acceleration.z() = r.read<double>();

  // linear_acceleration_covariance not needed

  return imu;
}

// ── geometry_msgs/PoseStamped
// ─────────────────────────────────────────────────
//
// Wire layout (little-endian):
//   std_msgs/Header: uint32 seq, uint32 secs, uint32 nsecs, string frame_id
//   geometry_msgs/Point: float64 x, y, z          (position)
//   geometry_msgs/Quaternion: float64 x, y, z, w  (orientation)

PoseStampedMsg deserialize_pose_stamped(std::span<const std::byte> data) {
  SpanReader r(data);
  PoseStampedMsg pose;

  r.read<uint32_t>();  // seq
  pose.stamp.secs = r.read<uint32_t>();
  pose.stamp.nsecs = r.read<uint32_t>();
  r.read_string();  // frame_id

  pose.position.x() = r.read<double>();
  pose.position.y() = r.read<double>();
  pose.position.z() = r.read<double>();

  const double ox = r.read<double>();
  const double oy = r.read<double>();
  const double oz = r.read<double>();
  const double ow = r.read<double>();
  pose.orientation = Eigen::Quaterniond(ow, ox, oy, oz);

  return pose;
}

// ── sensor_msgs/PointCloud2
// ───────────────────────────────────────────────────
//
// Wire layout:
//   std_msgs/Header
//   uint32 height, uint32 width
//   PointField[] fields: uint32 count, then per field: string name, uint32
//   offset,
//                        uint8 datatype, uint32 count
//   uint8 is_bigendian
//   uint32 point_step, uint32 row_step
//   uint8[] data: uint32 len, then bytes
//   uint8 is_dense

PointCloud deserialize_pointcloud2(std::span<const std::byte> data,
                                   PointTimeMode time_mode) {
  SpanReader r(data);
  PointCloud cloud;

  r.read<uint32_t>();  // seq
  cloud.stamp.secs = r.read<uint32_t>();
  cloud.stamp.nsecs = r.read<uint32_t>();
  cloud.frame_id = r.read_string();

  uint32_t height = r.read<uint32_t>();
  uint32_t width = r.read<uint32_t>();
  uint32_t num_points = height * width;

  struct FieldDesc {
    std::string name;
    uint32_t offset;
    uint8_t datatype;
  };

  uint32_t num_fields = r.read<uint32_t>();
  std::vector<FieldDesc> descs(num_fields);
  for (auto& d : descs) {
    d.name = r.read_string();
    d.offset = r.read<uint32_t>();
    d.datatype = r.read<uint8_t>();
    r.read<uint32_t>();  // count (always 1 for scalar fields)
  }

  r.read<uint8_t>();  // is_bigendian
  uint32_t point_step = r.read<uint32_t>();
  r.read<uint32_t>();  // row_step
  uint32_t data_len = r.read<uint32_t>();
  auto point_data = r.read_bytes(data_len);

  // Find field descriptors by name. The Ouster driver may call the intensity
  // field "intensity" or "reflectivity" depending on version.
  auto find =
      [&](std::initializer_list<const char*> names) -> const FieldDesc* {
    for (auto name : names) {
      for (auto& d : descs) {
        if (d.name == name) return &d;
      }
    }
    return nullptr;
  };

  const auto* fx = find({"x"});
  const auto* fy = find({"y"});
  const auto* fz = find({"z"});
  const auto* fi = find({"intensity", "reflectivity"});
  const auto* fr = find({"ring"});
  // Per-point time; interpreted per `time_mode` (needed for deskewing).
  const auto* ft = find({"t", "time", "timestamp"});
  const double header_sec = cloud.stamp.to_sec();

  cloud.reserve(num_points);

  for (uint32_t i = 0; i < num_points; ++i) {
    const std::byte* p = point_data.data() + i * point_step;
    cloud.x.push_back(fx ? read_as_float(p + fx->offset, fx->datatype) : 0.0f);
    cloud.y.push_back(fy ? read_as_float(p + fy->offset, fy->datatype) : 0.0f);
    cloud.z.push_back(fz ? read_as_float(p + fz->offset, fz->datatype) : 0.0f);
    cloud.intensity.push_back(fi ? read_as_float(p + fi->offset, fi->datatype)
                                 : 0.0f);
    cloud.ring.push_back(fr ? read_as_u16(p + fr->offset, fr->datatype) : 0);

    uint32_t t_offset = 0;
    if (ft) {
      if (time_mode == PointTimeMode::AbsoluteSeconds) {
        // Absolute per-point time (seconds) minus the header stamp. Clamp a
        // tiny negative first point (rounding) to 0; a full scan is well under
        // the uint32 ns range (~4.29 s).
        const double off_ns =
            (read_as_double(p + ft->offset, ft->datatype) - header_sec) * 1e9;
        t_offset = off_ns > 0.0 ? static_cast<uint32_t>(off_ns) : 0;
      } else {
        t_offset = read_as_u32(p + ft->offset, ft->datatype);
      }
    }
    cloud.t_offset_ns.push_back(t_offset);
  }

  return cloud;
}

// ── livox_ros_driver/CustomMsg
// ─────────────────────────────────────────────────
//
// Wire layout (little-endian):
//   std_msgs/Header: uint32 seq, uint32 secs, uint32 nsecs, string frame_id
//   uint64 timebase        (scan base time, ns)
//   uint32 point_num
//   uint8  lidar_id
//   uint8[3] rsvd          (fixed array: 3 raw bytes, no length prefix)
//   CustomPoint[] points   (variable array: uint32 length, then elements)
//     CustomPoint: uint32 offset_time, float32 x/y/z, uint8 reflectivity,
//                  uint8 tag, uint8 line

PointCloud deserialize_livox_custommsg(std::span<const std::byte> data) {
  SpanReader r(data);
  PointCloud cloud;

  r.read<uint32_t>();  // seq
  cloud.stamp.secs = r.read<uint32_t>();
  cloud.stamp.nsecs = r.read<uint32_t>();
  cloud.frame_id = r.read_string();

  r.read<uint64_t>();  // timebase: unreliable here (seen 160 s off the header
                       // stamp), and offset_time is already the intra-scan
                       // offset, so the header stamp alone is the scan
                       // reference
  r.read<uint32_t>();  // point_num (redundant with the array length below)
  r.read<uint8_t>();   // lidar_id
  r.skip(3);           // rsvd[3]

  const uint32_t n = r.read<uint32_t>();  // points array length
  cloud.reserve(n);

  for (uint32_t i = 0; i < n; ++i) {
    const uint32_t offset_time = r.read<uint32_t>();  // ns from scan start
    cloud.x.push_back(r.read<float>());
    cloud.y.push_back(r.read<float>());
    cloud.z.push_back(r.read<float>());
    cloud.intensity.push_back(static_cast<float>(r.read<uint8_t>()));  // refl
    r.read<uint8_t>();                                                 // tag
    cloud.ring.push_back(r.read<uint8_t>());                           // line
    cloud.t_offset_ns.push_back(offset_time);
  }

  return cloud;
}

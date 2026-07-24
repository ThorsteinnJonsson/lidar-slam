#include "io/ros_deserializer.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

namespace {

template <typename T>
void append(std::vector<std::byte>& buf, T v) {
  const auto* p = reinterpret_cast<const std::byte*>(&v);
  buf.insert(buf.end(), p, p + sizeof(T));
}

void append_string(std::vector<std::byte>& buf, const std::string& s) {
  append<uint32_t>(buf, static_cast<uint32_t>(s.size()));
  for (char c : s) buf.push_back(static_cast<std::byte>(c));
}

struct FieldSpec {
  std::string name;
  uint32_t offset;
  uint8_t datatype;  // ROS PointField enum
};

// Build a minimal sensor_msgs/PointCloud2 buffer from field specs and packed
// per-point bytes.
std::vector<std::byte> build_pc2(uint32_t secs, uint32_t nsecs,
                                 const std::vector<FieldSpec>& fields,
                                 uint32_t point_step, uint32_t num_points,
                                 const std::vector<std::byte>& points) {
  std::vector<std::byte> buf;
  append<uint32_t>(buf, 0u);  // seq
  append<uint32_t>(buf, secs);
  append<uint32_t>(buf, nsecs);
  append_string(buf, "lidar");  // frame_id
  append<uint32_t>(buf, 1u);    // height
  append<uint32_t>(buf, num_points);
  append<uint32_t>(buf, static_cast<uint32_t>(fields.size()));
  for (const FieldSpec& f : fields) {
    append_string(buf, f.name);
    append<uint32_t>(buf, f.offset);
    append<uint8_t>(buf, f.datatype);
    append<uint32_t>(buf, 1u);  // count
  }
  append<uint8_t>(buf, 0u);  // is_bigendian
  append<uint32_t>(buf, point_step);
  append<uint32_t>(buf, point_step * num_points);  // row_step
  append<uint32_t>(buf, static_cast<uint32_t>(points.size()));
  buf.insert(buf.end(), points.begin(), points.end());
  append<uint8_t>(buf, 1u);  // is_dense
  return buf;
}

constexpr uint8_t PF_UINT16 = 4;
constexpr uint8_t PF_UINT32 = 6;
constexpr uint8_t PF_FLOAT32 = 7;
constexpr uint8_t PF_FLOAT64 = 8;

}  // namespace

// Ouster layout: per-point time is a uint32 ns offset from the header stamp.
TEST(PointCloud2, OusterOffsetNanosTime) {
  const std::vector<FieldSpec> fields = {{"x", 0, PF_FLOAT32},
                                         {"y", 4, PF_FLOAT32},
                                         {"z", 8, PF_FLOAT32},
                                         {"t", 12, PF_UINT32},
                                         {"ring", 16, PF_UINT16}};
  const uint32_t step = 18;
  std::vector<std::byte> pts;
  auto add_point = [&](float x, float y, float z, uint32_t t, uint16_t ring) {
    append<float>(pts, x);
    append<float>(pts, y);
    append<float>(pts, z);
    append<uint32_t>(pts, t);
    append<uint16_t>(pts, ring);
  };
  add_point(1.0f, 2.0f, 3.0f, 100u, 0);
  add_point(4.0f, 5.0f, 6.0f, 5000u, 7);

  const auto buf = build_pc2(1000, 0, fields, step, 2, pts);
  const PointCloud c = deserialize_pointcloud2(buf, PointTimeMode::OffsetNanos);

  ASSERT_EQ(c.size(), 2u);
  EXPECT_FLOAT_EQ(c.x[0], 1.0f);
  EXPECT_FLOAT_EQ(c.z[1], 6.0f);
  EXPECT_EQ(c.ring[1], 7u);
  EXPECT_EQ(c.t_offset_ns[0], 100u);
  EXPECT_EQ(c.t_offset_ns[1], 5000u);
}

// Hesai layout: per-point time is an absolute FLOAT64 timestamp (seconds); the
// deserializer must subtract the header stamp to recover the ns offset. Reading
// it as a raw uint32 (the Ouster path) would produce garbage.
TEST(PointCloud2, HesaiAbsoluteSecondsTime) {
  const std::vector<FieldSpec> fields = {{"x", 0, PF_FLOAT32},
                                         {"y", 4, PF_FLOAT32},
                                         {"z", 8, PF_FLOAT32},
                                         {"timestamp", 12, PF_FLOAT64},
                                         {"ring", 20, PF_UINT16}};
  const uint32_t step = 22;
  const double header = 1000.0;
  std::vector<std::byte> pts;
  auto add_point = [&](float x, float y, float z, double ts, uint16_t ring) {
    append<float>(pts, x);
    append<float>(pts, y);
    append<float>(pts, z);
    append<double>(pts, ts);
    append<uint16_t>(pts, ring);
  };
  add_point(1.0f, 2.0f, 3.0f, header + 100e-9, 0);  // 100 ns after header
  add_point(4.0f, 5.0f, 6.0f, header + 0.05, 7);    // 0.05 s after header

  const auto buf = build_pc2(1000, 0, fields, step, 2, pts);
  const PointCloud c =
      deserialize_pointcloud2(buf, PointTimeMode::AbsoluteSeconds);

  ASSERT_EQ(c.size(), 2u);
  EXPECT_FLOAT_EQ(c.x[0], 1.0f);
  EXPECT_NEAR(static_cast<double>(c.t_offset_ns[0]), 100.0, 2.0);
  EXPECT_NEAR(static_cast<double>(c.t_offset_ns[1]), 50'000'000.0, 2.0);
}

// livox_ros_driver/CustomMsg: per-point time is timebase + offset_time. With
// timebase equal to the header stamp, t_offset_ns is just offset_time.
TEST(LivoxCustomMsg, ParsesPointsAndTime) {
  const uint64_t stamp_ns = 1000ull * 1'000'000'000ull;  // secs=1000, nsecs=0
  std::vector<std::byte> buf;
  append<uint32_t>(buf, 0u);        // seq
  append<uint32_t>(buf, 1000u);     // secs
  append<uint32_t>(buf, 0u);        // nsecs
  append_string(buf, "livox");      // frame_id
  append<uint64_t>(buf, stamp_ns);  // timebase == header stamp
  append<uint32_t>(buf, 2u);        // point_num
  append<uint8_t>(buf, 0u);         // lidar_id
  append<uint8_t>(buf, 0u);         // rsvd[0..2]
  append<uint8_t>(buf, 0u);
  append<uint8_t>(buf, 0u);
  append<uint32_t>(buf, 2u);  // points array length
  auto add_point = [&](uint32_t off, float x, float y, float z, uint8_t refl,
                       uint8_t line) {
    append<uint32_t>(buf, off);
    append<float>(buf, x);
    append<float>(buf, y);
    append<float>(buf, z);
    append<uint8_t>(buf, refl);
    append<uint8_t>(buf, 0u);  // tag
    append<uint8_t>(buf, line);
  };
  add_point(200u, 1.0f, 2.0f, 3.0f, 55u, 2);
  add_point(123456u, 4.0f, 5.0f, 6.0f, 77u, 5);

  const PointCloud c = deserialize_livox_custommsg(buf);

  ASSERT_EQ(c.size(), 2u);
  EXPECT_FLOAT_EQ(c.x[1], 4.0f);
  EXPECT_FLOAT_EQ(c.intensity[0], 55.0f);
  EXPECT_EQ(c.ring[1], 5u);
  EXPECT_EQ(c.t_offset_ns[0], 200u);
  EXPECT_EQ(c.t_offset_ns[1], 123456u);
}

TEST(PoseStamped, ParsesStampAndPosition) {
  std::vector<std::byte> buf;
  append<uint32_t>(buf, 7u);    // seq
  append<uint32_t>(buf, 12u);   // secs
  append<uint32_t>(buf, 345u);  // nsecs
  append_string(buf, "world");  // frame_id
  append<double>(buf, 1.5);     // position x
  append<double>(buf, -2.0);    // position y
  append<double>(buf, 3.25);    // position z
  append<double>(buf, 0.0);     // orientation x
  append<double>(buf, 0.0);     // orientation y
  append<double>(buf, 0.0);     // orientation z
  append<double>(buf, 1.0);     // orientation w

  const PoseStampedMsg pose = deserialize_pose_stamped(buf);

  EXPECT_EQ(pose.stamp.secs, 12u);
  EXPECT_EQ(pose.stamp.nsecs, 345u);
  EXPECT_DOUBLE_EQ(pose.position.x(), 1.5);
  EXPECT_DOUBLE_EQ(pose.position.y(), -2.0);
  EXPECT_DOUBLE_EQ(pose.position.z(), 3.25);
  EXPECT_DOUBLE_EQ(pose.orientation.w(), 1.0);
}

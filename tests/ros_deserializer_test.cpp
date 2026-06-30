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

}  // namespace

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

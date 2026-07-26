#include <gtest/gtest.h>

#include "preprocess/deskew.h"
#include "types.h"

namespace {

PointCloud make_cloud(uint32_t secs, uint32_t nsecs,
                      const std::vector<uint32_t>& t_offsets) {
  PointCloud c;
  c.stamp = {secs, nsecs};
  for (size_t i = 0; i < t_offsets.size(); ++i) {
    c.x.push_back(static_cast<float>(i));
    c.y.push_back(static_cast<float>(i) + 0.5f);
    c.z.push_back(-static_cast<float>(i));
    c.intensity.push_back(0.0f);
    c.ring.push_back(0);
    c.t_offset_ns.push_back(t_offsets[i]);
  }
  return c;
}

}  // namespace

TEST(PointCloud, XyzDropsNonPositionFields) {
  const PointCloud c = make_cloud(10, 0, {0, 100, 200});
  const auto pts = c.xyz();

  ASSERT_EQ(pts.size(), 3u);
  EXPECT_FLOAT_EQ(pts[1].x(), 1.0f);
  EXPECT_FLOAT_EQ(pts[1].y(), 1.5f);
  EXPECT_FLOAT_EQ(pts[2].z(), -2.0f);
}

TEST(ScanTimeWindow, SpansMinToMaxOffsetFromStamp) {
  // Offsets out of order; the window is [base + min, base + max].
  const PointCloud c = make_cloud(10, 0, {5000, 100, 300});
  const uint64_t base = c.stamp.to_nsec();

  const auto window = scan_time_window(c);
  ASSERT_TRUE(window.has_value());
  EXPECT_EQ(window->first, base + 100);
  EXPECT_EQ(window->second, base + 5000);
}

TEST(ScanTimeWindow, EmptyCloudIsNullopt) {
  const PointCloud c = make_cloud(10, 0, {});
  EXPECT_FALSE(scan_time_window(c).has_value());
}

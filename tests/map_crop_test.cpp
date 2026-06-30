#include <gtest/gtest.h>

#include <Eigen/Core>
#include <vector>

#include "map/ikd_tree.h"
#include "map/local_map.h"

namespace {

// A solid grid of integer-coordinate points spanning [-r, r] on every axis.
std::vector<Eigen::Vector3f> grid(int r) {
  std::vector<Eigen::Vector3f> pts;
  for (int x = -r; x <= r; ++x)
    for (int y = -r; y <= r; ++y)
      for (int z = -r; z <= r; ++z)
        pts.emplace_back(static_cast<float>(x), static_cast<float>(y),
                         static_cast<float>(z));
  return pts;
}

bool inside(const Eigen::Vector3f& p, const Eigen::Vector3f& lo,
            const Eigen::Vector3f& hi) {
  return (p.array() >= lo.array()).all() && (p.array() <= hi.array()).all();
}

}  // namespace

TEST(MapCrop, CropToBoxKeepsExactlyInsidePoints) {
  const std::vector<Eigen::Vector3f> pts = grid(10);  // 21^3 = 9261 points
  IkdTree tree;
  tree.build(pts);

  const Eigen::Vector3f lo(-3, -3, -3);
  const Eigen::Vector3f hi(3, 3, 3);
  crop_to_box(tree, lo, hi);

  size_t expected = 0;
  for (const Eigen::Vector3f& p : pts)
    if (inside(p, lo, hi)) ++expected;

  EXPECT_EQ(tree.size(), expected);  // 7^3 = 343
  EXPECT_TRUE(tree.validate());
}

TEST(MapCrop, CropToBoxKeepsBoundaryPoints) {
  // A point exactly on a face of the keep cube must survive (closed box).
  IkdTree tree;
  tree.build({Eigen::Vector3f(3, 0, 0), Eigen::Vector3f(4, 0, 0)});
  crop_to_box(tree, Eigen::Vector3f(-3, -3, -3), Eigen::Vector3f(3, 3, 3));
  EXPECT_EQ(tree.size(), 1u);  // (3,0,0) kept, (4,0,0) dropped
}

TEST(MapCrop, CropToBoxOnNonOverlappingMapIsNoOp) {
  const std::vector<Eigen::Vector3f> pts = grid(2);
  IkdTree tree;
  tree.build(pts);
  // Keep cube fully contains the map: nothing removed.
  crop_to_box(tree, Eigen::Vector3f(-100, -100, -100),
              Eigen::Vector3f(100, 100, 100));
  EXPECT_EQ(tree.size(), pts.size());
}

TEST(MapCrop, SlideTriggersOnlyNearAFace) {
  const Eigen::Vector3f lo(-100, -100, -100);
  const Eigen::Vector3f hi(100, 100, 100);
  const float margin = 20.0f;

  EXPECT_FALSE(box_needs_slide(Eigen::Vector3f(0, 0, 0), lo, hi, margin));
  EXPECT_FALSE(box_needs_slide(Eigen::Vector3f(75, 0, 0), lo, hi, margin));
  EXPECT_TRUE(box_needs_slide(Eigen::Vector3f(85, 0, 0), lo, hi, margin));
  EXPECT_TRUE(box_needs_slide(Eigen::Vector3f(0, -90, 0), lo, hi, margin));
  EXPECT_TRUE(box_needs_slide(Eigen::Vector3f(0, 0, -81), lo, hi, margin));
}

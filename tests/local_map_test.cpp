#include "map/local_map.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <vector>

namespace {

// A dense slab of points spaced `step` apart over [0, extent] on x and y at
// z = 0. Spacing below the map leaf so a re-inserted copy is fully redundant.
std::vector<Eigen::Vector3f> sheet(float extent, float step, float origin = 0) {
  std::vector<Eigen::Vector3f> pts;
  for (float x = 0; x <= extent; x += step)
    for (float y = 0; y <= extent; y += step)
      pts.emplace_back(origin + x, y, 0.0f);
  return pts;
}

LocalMapParams params(bool voxel, float leaf = 0.5f) {
  LocalMapParams p;
  p.voxel_on_insert = voxel;
  p.voxel_leaf = leaf;
  p.crop.enabled = false;  // isolate insertion behavior from cropping
  return p;
}

}  // namespace

TEST(LocalMap, FirstInsertSeatsAllPoints) {
  const auto pts = sheet(2.0f, 0.5f);
  LocalMap map(params(true));
  map.insert(pts);
  EXPECT_EQ(map.size(), pts.size());
}

TEST(LocalMap, ReinsertingSameScanAddsNothing) {
  const auto pts = sheet(5.0f, 0.5f);
  LocalMap map(params(true));
  map.insert(pts);
  const size_t after_first = map.size();
  map.insert(pts);  // identical points, all within leaf of existing
  EXPECT_EQ(map.size(), after_first);
}

TEST(LocalMap, SmallShiftAddsFewPoints) {
  LocalMap map(params(true, 0.5f));
  map.insert(sheet(5.0f, 0.5f));
  const size_t before = map.size();
  // Shift by well under the leaf: every point falls next to an existing one.
  std::vector<Eigen::Vector3f> shifted = sheet(5.0f, 0.5f);
  for (Eigen::Vector3f& p : shifted) p.x() += 0.1f;
  map.insert(shifted);
  EXPECT_EQ(map.size(), before);
}

TEST(LocalMap, DisjointScanAddsAllPoints) {
  LocalMap map(params(true));
  const auto first = sheet(2.0f, 0.5f);
  map.insert(first);
  const auto far = sheet(2.0f, 0.5f, /*origin=*/100.0f);  // 100 m away
  map.insert(far);
  EXPECT_EQ(map.size(), first.size() + far.size());
}

TEST(LocalMap, DisabledVoxelKeepsEveryInsert) {
  const auto pts = sheet(5.0f, 0.5f);
  LocalMap map(params(/*voxel=*/false));
  map.insert(pts);
  map.insert(pts);
  EXPECT_EQ(map.size(), 2 * pts.size());
}

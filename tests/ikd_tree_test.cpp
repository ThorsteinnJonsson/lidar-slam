#include "map/ikd_tree.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <nanoflann.hpp>
#include <random>
#include <vector>

namespace {

using Vec3 = Eigen::Vector3f;

std::vector<Vec3> random_cloud(int n, uint32_t seed, float extent = 50.0f) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> dist(-extent, extent);
  std::vector<Vec3> pts(n);
  for (auto& p : pts) p = Vec3(dist(rng), dist(rng), dist(rng));
  return pts;
}

// Ground-truth k nearest squared distances, by brute force.
std::vector<float> brute_force_dist2(const std::vector<Vec3>& pts,
                                     const Vec3& query, int k) {
  std::vector<float> d2;
  d2.reserve(pts.size());
  for (const auto& p : pts) d2.push_back((p - query).squaredNorm());
  std::sort(d2.begin(), d2.end());
  d2.resize(std::min<size_t>(k, d2.size()));
  return d2;
}

// nanoflann adaptor over a std::vector<Vec3>, used as a second oracle.
struct CloudAdaptor {
  const std::vector<Vec3>& pts;
  size_t kdtree_get_point_count() const { return pts.size(); }
  float kdtree_get_pt(size_t idx, size_t dim) const { return pts[idx][dim]; }
  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }
};

using NanoTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, CloudAdaptor>, CloudAdaptor, 3>;

std::vector<float> nanoflann_dist2(const std::vector<Vec3>& pts,
                                   const Vec3& query, int k) {
  CloudAdaptor adaptor{pts};
  NanoTree index(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  index.buildIndex();

  const size_t kk = std::min<size_t>(k, pts.size());
  std::vector<size_t> idx(kk);
  std::vector<float> d2(kk);
  nanoflann::KNNResultSet<float> result(kk);
  result.init(idx.data(), d2.data());
  const float q[3] = {query.x(), query.y(), query.z()};
  index.findNeighbors(result, q);
  std::sort(d2.begin(), d2.end());  // nanoflann returns ascending already
  return d2;
}

// Compare two ascending squared-distance sequences. Distances are computed from
// identical float points, so they agree to within float rounding.
void expect_dist2_match(const std::vector<float>& got,
                        const std::vector<float>& expected) {
  ASSERT_EQ(got.size(), expected.size());
  for (size_t i = 0; i < got.size(); ++i) {
    const float tol = 1e-3f * std::max(1.0f, expected[i]);
    EXPECT_NEAR(got[i], expected[i], tol) << "at neighbor " << i;
  }
}

}  // namespace

TEST(IkdTree, EmptyTree) {
  IkdTree tree;
  tree.build({});
  EXPECT_EQ(tree.size(), 0u);
  EXPECT_TRUE(tree.validate());

  std::vector<Vec3> pts;
  std::vector<float> d2;
  tree.knn(Vec3(0, 0, 0), 5, pts, d2);
  EXPECT_TRUE(pts.empty());
  EXPECT_TRUE(d2.empty());
}

TEST(IkdTree, SinglePoint) {
  IkdTree tree;
  tree.build({Vec3(1, 2, 3)});
  EXPECT_EQ(tree.size(), 1u);
  EXPECT_TRUE(tree.validate());

  std::vector<Vec3> pts;
  std::vector<float> d2;
  tree.knn(Vec3(0, 0, 0), 5, pts, d2);  // k > size
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_FLOAT_EQ(d2[0], 14.0f);  // 1 + 4 + 9
  EXPECT_TRUE(pts[0].isApprox(Vec3(1, 2, 3)));
}

TEST(IkdTree, ValidatesInvariantsAcrossSizes) {
  for (int n : {2, 3, 10, 100, 1000, 5000}) {
    IkdTree tree;
    tree.build(random_cloud(n, /*seed=*/n));
    EXPECT_EQ(tree.size(), static_cast<size_t>(n)) << "n=" << n;
    EXPECT_TRUE(tree.validate()) << "n=" << n;
  }
}

TEST(IkdTree, KnnMatchesBruteForce) {
  std::mt19937 rng(12345);
  std::uniform_real_distribution<float> coord(-60.0f, 60.0f);

  for (int n : {1, 5, 50, 500, 2000}) {
    const auto pts = random_cloud(n, /*seed=*/100 + n);
    IkdTree tree;
    tree.build(pts);

    for (int k : {1, 5, 10, 50}) {
      for (int q = 0; q < 20; ++q) {
        const Vec3 query(coord(rng), coord(rng), coord(rng));
        std::vector<Vec3> got_pts;
        std::vector<float> got_d2;
        tree.knn(query, k, got_pts, got_d2);

        EXPECT_EQ(got_pts.size(), got_d2.size());
        EXPECT_EQ(got_d2.size(), std::min<size_t>(k, n))
            << "n=" << n << " k=" << k;
        expect_dist2_match(got_d2, brute_force_dist2(pts, query, k));

        // Reported distances are consistent with the returned points.
        for (size_t i = 0; i < got_pts.size(); ++i) {
          EXPECT_NEAR((got_pts[i] - query).squaredNorm(), got_d2[i],
                      1e-3f * std::max(1.0f, got_d2[i]));
        }
      }
    }
  }
}

TEST(IkdTree, KnnMatchesNanoflann) {
  std::mt19937 rng(777);
  std::uniform_real_distribution<float> coord(-60.0f, 60.0f);

  for (int n : {50, 500, 3000}) {
    const auto pts = random_cloud(n, /*seed=*/200 + n);
    IkdTree tree;
    tree.build(pts);

    for (int k : {1, 5, 20}) {
      for (int q = 0; q < 20; ++q) {
        const Vec3 query(coord(rng), coord(rng), coord(rng));
        std::vector<Vec3> got_pts;
        std::vector<float> got_d2;
        tree.knn(query, k, got_pts, got_d2);
        expect_dist2_match(got_d2, nanoflann_dist2(pts, query, k));
      }
    }
  }
}

TEST(IkdTree, InsertFromEmptyMatchesBruteForce) {
  std::mt19937 rng(2024);
  std::uniform_real_distribution<float> coord(-60.0f, 60.0f);

  for (int n : {1, 5, 50, 500, 2000}) {
    const auto pts = random_cloud(n, /*seed=*/300 + n);
    IkdTree tree;
    for (const auto& p : pts) tree.insert(p);

    EXPECT_EQ(tree.size(), static_cast<size_t>(n)) << "n=" << n;
    EXPECT_TRUE(tree.validate()) << "n=" << n;

    for (int k : {1, 5, 10}) {
      for (int q = 0; q < 10; ++q) {
        const Vec3 query(coord(rng), coord(rng), coord(rng));
        std::vector<Vec3> got_pts;
        std::vector<float> got_d2;
        tree.knn(query, k, got_pts, got_d2);
        expect_dist2_match(got_d2, brute_force_dist2(pts, query, k));
      }
    }
  }
}

TEST(IkdTree, BatchInsertAfterBuild) {
  const auto pts = random_cloud(1000, /*seed=*/42);
  // Build with the first half, then add the rest as a batch insert.
  const std::vector<Vec3> head(pts.begin(), pts.begin() + 500);
  const std::vector<Vec3> tail(pts.begin() + 500, pts.end());
  IkdTree tree;
  tree.build(head);
  tree.insert(tail);

  EXPECT_EQ(tree.size(), 1000u);
  EXPECT_TRUE(tree.validate());

  std::mt19937 rng(99);
  std::uniform_real_distribution<float> coord(-60.0f, 60.0f);
  for (int q = 0; q < 50; ++q) {
    const Vec3 query(coord(rng), coord(rng), coord(rng));
    std::vector<Vec3> got_pts;
    std::vector<float> got_d2;
    tree.knn(query, 8, got_pts, got_d2);
    expect_dist2_match(got_d2, brute_force_dist2(pts, query, 8));
  }
}

TEST(IkdTree, InsertIntoEmptyTreeSinglePoint) {
  IkdTree tree;
  tree.insert(Vec3(1, 2, 3));
  EXPECT_EQ(tree.size(), 1u);
  EXPECT_TRUE(tree.validate());

  std::vector<Vec3> got_pts;
  std::vector<float> got_d2;
  tree.knn(Vec3(0, 0, 0), 5, got_pts, got_d2);
  ASSERT_EQ(got_pts.size(), 1u);
  EXPECT_FLOAT_EQ(got_d2[0], 14.0f);
}

TEST(IkdTree, DuplicatePoints) {
  std::vector<Vec3> pts(100, Vec3(2, 2, 2));
  pts.push_back(Vec3(5, 5, 5));
  IkdTree tree;
  tree.build(pts);
  EXPECT_EQ(tree.size(), 101u);
  EXPECT_TRUE(tree.validate());

  std::vector<Vec3> got_pts;
  std::vector<float> got_d2;
  tree.knn(Vec3(2, 2, 2), 5, got_pts, got_d2);
  ASSERT_EQ(got_d2.size(), 5u);
  for (float d : got_d2) EXPECT_FLOAT_EQ(d, 0.0f);  // five coincident neighbors
}

TEST(IkdTree, CollinearPoints) {
  std::vector<Vec3> pts;
  for (int i = 0; i < 200; ++i)
    pts.push_back(Vec3(static_cast<float>(i), 0, 0));
  IkdTree tree;
  tree.build(pts);
  EXPECT_TRUE(tree.validate());

  std::vector<Vec3> got_pts;
  std::vector<float> got_d2;
  tree.knn(Vec3(10.4f, 0, 0), 3, got_pts, got_d2);
  expect_dist2_match(got_d2, brute_force_dist2(pts, Vec3(10.4f, 0, 0), 3));
}

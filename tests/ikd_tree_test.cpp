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

bool in_box(const Vec3& p, const Vec3& lo, const Vec3& hi) {
  return (p.array() >= lo.array()).all() && (p.array() <= hi.array()).all();
}

// The points of `pts` that fall outside the closed box [lo, hi], the live set
// expected after a box delete.
std::vector<Vec3> outside_box(const std::vector<Vec3>& pts, const Vec3& lo,
                              const Vec3& hi) {
  std::vector<Vec3> out;
  for (const auto& p : pts)
    if (!in_box(p, lo, hi)) out.push_back(p);
  return out;
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

TEST(IkdTree, RemoveBoxMatchesBruteForce) {
  std::mt19937 rng(555);
  std::uniform_real_distribution<float> coord(-60.0f, 60.0f);
  const Vec3 lo(-20, -20, -20);
  const Vec3 hi(20, 20, 20);

  for (int n : {50, 500, 2000}) {
    const auto pts = random_cloud(n, /*seed=*/400 + n);
    IkdTree tree;
    tree.build(pts);
    tree.remove_box(lo, hi);

    const auto live = outside_box(pts, lo, hi);
    EXPECT_EQ(tree.size(), live.size()) << "n=" << n;
    EXPECT_TRUE(tree.validate()) << "n=" << n;

    for (int k : {1, 5, 10}) {
      for (int q = 0; q < 20; ++q) {
        const Vec3 query(coord(rng), coord(rng), coord(rng));
        std::vector<Vec3> got_pts;
        std::vector<float> got_d2;
        tree.knn(query, k, got_pts, got_d2);
        expect_dist2_match(got_d2, brute_force_dist2(live, query, k));
        // No returned point lies in the deleted region.
        for (const auto& p : got_pts) EXPECT_FALSE(in_box(p, lo, hi));
      }
    }
  }
}

TEST(IkdTree, RemoveBoxEverything) {
  const auto pts = random_cloud(500, /*seed=*/7);
  IkdTree tree;
  tree.build(pts);
  tree.remove_box(Vec3(-1000, -1000, -1000), Vec3(1000, 1000, 1000));

  EXPECT_EQ(tree.size(), 0u);
  EXPECT_TRUE(tree.validate());

  std::vector<Vec3> got_pts;
  std::vector<float> got_d2;
  tree.knn(Vec3(0, 0, 0), 5, got_pts, got_d2);
  EXPECT_TRUE(got_pts.empty());
  EXPECT_TRUE(got_d2.empty());
}

TEST(IkdTree, RemoveBoxThenInsert) {
  const auto base = random_cloud(1000, /*seed=*/13);
  const Vec3 lo(-20, -20, -20);
  const Vec3 hi(20, 20, 20);
  IkdTree tree;
  tree.build(base);
  tree.remove_box(lo, hi);

  // Insert fresh points. Some land back inside the deleted region and must
  // come alive again (insert revives, it doesn't inherit the deleted label).
  const auto added = random_cloud(300, /*seed=*/14, /*extent=*/15.0f);
  tree.insert(added);

  std::vector<Vec3> live = outside_box(base, lo, hi);
  live.insert(live.end(), added.begin(), added.end());
  EXPECT_EQ(tree.size(), live.size());
  EXPECT_TRUE(tree.validate());

  std::mt19937 rng(321);
  std::uniform_real_distribution<float> coord(-60.0f, 60.0f);
  for (int q = 0; q < 50; ++q) {
    const Vec3 query(coord(rng), coord(rng), coord(rng));
    std::vector<Vec3> got_pts;
    std::vector<float> got_d2;
    tree.knn(query, 8, got_pts, got_d2);
    expect_dist2_match(got_d2, brute_force_dist2(live, query, 8));
  }
}

TEST(IkdTree, SortedInsertStaysBalanced) {
  // Inserting in sorted order is the worst case for an un-rebalanced k-d tree:
  // it degenerates into a linked list of height n. The partial rebuild must
  // keep the height close to log2(n).
  IkdTree tree;
  const int n = 5000;
  for (int i = 0; i < n; ++i) {
    const float f = static_cast<float>(i);
    tree.insert(Vec3(f, f, f));
  }

  EXPECT_EQ(tree.size(), static_cast<size_t>(n));
  EXPECT_TRUE(tree.validate());

  const int log2n = static_cast<int>(std::ceil(std::log2(n)));
  EXPECT_LT(tree.height(), 4 * log2n) << "height=" << tree.height();
}

TEST(IkdTree, RebuildReclaimsDeletedGarbage) {
  const auto pts = random_cloud(4000, /*seed=*/77);
  IkdTree tree;
  tree.build(pts);
  EXPECT_EQ(tree.physical_size(), 4000u);

  // Delete a large region that partially overlaps the root box (so the deletion
  // recurses and unwinds, triggering garbage-collection rebuilds) and removes
  // the majority of points.
  const Vec3 lo(-50, -50, -50);
  const Vec3 hi(40, 40, 40);
  tree.remove_box(lo, hi);

  const auto live = outside_box(pts, lo, hi);
  EXPECT_EQ(tree.size(), live.size());
  EXPECT_TRUE(tree.validate());

  // Garbage was physically reclaimed: physical size collapsed from 4000 toward
  // the live count.
  EXPECT_LT(tree.physical_size(), pts.size());
  EXPECT_GE(tree.physical_size(), tree.size());
}

TEST(IkdTree, InterleavedInsertDeleteMatchesBruteForce) {
  std::mt19937 rng(2718);
  std::uniform_real_distribution<float> coord(-50.0f, 50.0f);

  std::vector<Vec3> model = random_cloud(1000, /*seed=*/1);
  IkdTree tree;
  tree.build(model);

  for (int round = 0; round < 20; ++round) {
    if (round % 2 == 0) {
      const auto add = random_cloud(200, /*seed=*/1000 + round);
      tree.insert(add);
      model.insert(model.end(), add.begin(), add.end());
    } else {
      const Vec3 c(coord(rng), coord(rng), coord(rng));
      const Vec3 lo = c - Vec3(15, 15, 15);
      const Vec3 hi = c + Vec3(15, 15, 15);
      tree.remove_box(lo, hi);
      std::vector<Vec3> kept;
      for (const auto& p : model)
        if (!in_box(p, lo, hi)) kept.push_back(p);
      model = std::move(kept);
    }

    EXPECT_EQ(tree.size(), model.size()) << "round=" << round;
    EXPECT_TRUE(tree.validate()) << "round=" << round;

    for (int q = 0; q < 10; ++q) {
      const Vec3 query(coord(rng), coord(rng), coord(rng));
      std::vector<Vec3> got_pts;
      std::vector<float> got_d2;
      tree.knn(query, 5, got_pts, got_d2);
      expect_dist2_match(got_d2, brute_force_dist2(model, query, 5));
    }
  }
}

TEST(IkdTree, PartialDeleteAccountingWithoutRebuild) {
  const auto pts = random_cloud(2000, /*seed=*/91);
  IkdTree tree;
  tree.build(pts);

  // Delete a tiny box around a single existing point. One point goes away, the
  // leaf subtree is below kMinRebuildSize and the garbage fraction is
  // negligible, so no rebuild fires and the deleted node stays physically
  // present. That lets us check the invalid_num accounting exactly.
  const Vec3 target = pts[0];
  const Vec3 eps(0.01f, 0.01f, 0.01f);
  tree.remove_box(target - eps, target + eps);

  const size_t deleted =
      pts.size() - outside_box(pts, target - eps, target + eps).size();
  ASSERT_EQ(deleted, 1u);

  EXPECT_TRUE(tree.validate());
  EXPECT_EQ(tree.physical_size(), pts.size());  // nothing reclaimed
  EXPECT_EQ(tree.size(), pts.size() - 1);       // one live point fewer
  EXPECT_EQ(tree.physical_size() - tree.size(), deleted);
}

TEST(IkdTree, DeleteCreatesPushdownAndValidates) {
  // A handful of points in a spatially separated cluster the tree groups into
  // one subtree, plus a dense blob near the origin.
  std::vector<Vec3> pts = random_cloud(500, /*seed=*/5, /*extent=*/10.0f);
  const Vec3 c(100, 100, 100);
  std::vector<Vec3> cluster;
  for (int i = 0; i < 5; ++i)
    cluster.push_back(c + Vec3(0.1f * i, -0.1f * i, 0.05f * i));
  pts.insert(pts.end(), cluster.begin(), cluster.end());

  IkdTree tree;
  tree.build(pts);
  EXPECT_EQ(tree.physical_size(), pts.size());

  // A box that fully contains the cluster subtree but no origin points. The
  // subtree root becomes a pending-pushdown node: tree_deleted with stale child
  // labels underneath. validate() must account for that without descending.
  tree.remove_box(c - Vec3(1, 1, 1), c + Vec3(1, 1, 1));

  EXPECT_TRUE(tree.validate());
  EXPECT_EQ(tree.size(), pts.size() - cluster.size());
  // Cluster is tiny (< kMinRebuildSize) and a small fraction, so no rebuild
  // reclaimed it; the deleted nodes are still physically present.
  EXPECT_EQ(tree.physical_size(), pts.size());
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

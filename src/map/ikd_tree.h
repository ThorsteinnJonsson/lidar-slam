#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

// Incremental k-d tree for the SLAM map (after Cai et al. 2021, "ikd-Tree").
//
// Phase 4.1 implements the static core: a balanced `build()` and a k-NN search
// with axis-aligned-bounding-box pruning. Each node caches its subtree's size
// and bounding box; the incremental insert/delete/rebalance machinery is
// layered on in 4.2 (the per-node `deleted`/`treedeleted`/etc. fields are
// reserved here but unused for now).
//
// Points are stored as float (geometry is all point-to-plane needs, and float
// halves the node footprint versus double).
class IkdTree {
 public:
  IkdTree() = default;

  // Build a balanced tree from scratch, discarding any current contents.
  // The argument is consumed (reordered in place) for an in-place median split.
  void build(std::vector<Eigen::Vector3f> points);

  // k nearest neighbors of `query`. Outputs are sorted by ascending squared
  // distance and sized to min(k, size()).
  void knn(const Eigen::Vector3f& query, size_t k,
           std::vector<Eigen::Vector3f>& out_points,
           std::vector<float>& out_dist2) const;

  // Number of points in the tree.
  size_t size() const noexcept;

  // Verify structural invariants (subtree sizes, bounding boxes, split
  // ordering). Used by the tests and as a debugging aid for the 4.2 incremental
  // ops.
  bool validate() const;

 private:
  struct Node {
    Eigen::Vector3f point;
    std::unique_ptr<Node> left;
    std::unique_ptr<Node> right;
    uint8_t axis = 0;  // split axis (0/1/2)
    Eigen::Vector3f range_min;
    Eigen::Vector3f range_max;
    int treesize = 1;
    // --- reserved for incremental ops (4.2); unused in the static tree ---
    int invalid_num = 0;
    bool deleted = false;
    bool treedeleted = false;
    bool pushdown = false;
  };

  // A k-NN search candidate; the search keeps these in a bounded max-heap.
  struct HeapItem {
    float dist2;
    Eigen::Vector3f point;
  };

  static bool heap_less(const HeapItem& a, const HeapItem& b);
  static int node_size(const Node* n);
  static void pull_up(Node* n);
  static void push_down(Node* n);

  static std::unique_ptr<Node> build_range(Eigen::Vector3f* first,
                                           Eigen::Vector3f* last);

  void search(const Node* node, const Eigen::Vector3f& query, size_t k,
              std::vector<HeapItem>& heap) const;

  bool check(const Node* n, int& out_size, Eigen::Vector3f& out_lo,
             Eigen::Vector3f& out_hi) const;

  std::unique_ptr<Node> root_;
};

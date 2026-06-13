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
// layered on in 4.2 (the per-node `deleted`/`tree_deleted`/etc. fields are
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

  // Insert points into the existing tree (cheaper than a rebuild when the map
  // changes by a small fraction each scan). Balance is kept by the partial
  // rebuild added in a later step; until then a degenerate insert order can
  // leave the tree unbalanced (k-NN stays correct regardless).
  void insert(const std::vector<Eigen::Vector3f>& points);
  void insert(const Eigen::Vector3f& point);

  // Logically delete every point inside the closed axis-aligned box
  // [box_min, box_max]. Deletion is lazy: points are flagged, not physically
  // removed (a later rebuild reclaims them). size() and k-NN ignore them.
  void remove_box(const Eigen::Vector3f& box_min,
                  const Eigen::Vector3f& box_max);

  // k nearest neighbors of `query`. Outputs are sorted by ascending squared
  // distance and sized to min(k, size()).
  void knn(const Eigen::Vector3f& query, size_t k,
           std::vector<Eigen::Vector3f>& out_points,
           std::vector<float>& out_dist2) const;

  // Number of live points in the tree (excludes lazily-deleted points still
  // physically present pending a rebuild).
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
    bool tree_deleted = false;
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

  // Insert a point into the subtree held by `slot`, operating on the slot (not
  // a raw pointer) so a future rebuild can swap the whole subtree in place.
  static void insert_at(std::unique_ptr<Node>& slot,
                        const Eigen::Vector3f& point);

  // Box-delete within the subtree held by `slot` (slot, not raw pointer, so a
  // future rebuild can swap the subtree in place).
  static void remove_box_at(std::unique_ptr<Node>& slot,
                            const Eigen::Vector3f& box_min,
                            const Eigen::Vector3f& box_max);

  void search(const Node* node, const Eigen::Vector3f& query, size_t k,
              std::vector<HeapItem>& heap) const;

  bool check(const Node* n, int& out_size, Eigen::Vector3f& out_lo,
             Eigen::Vector3f& out_hi) const;

  std::unique_ptr<Node> root_;
};

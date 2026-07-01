#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

// Incremental k-d tree for the SLAM map (after Cai et al. 2021, "ikd-Tree").
//
// Supports a balanced `build()`, k-NN search with axis-aligned-bounding-box
// pruning, incremental `insert`, and lazy box-delete (`remove_box`). Each node
// caches its subtree's size, bounding box, and deletion bookkeeping
// (invalid_num / tree_deleted / pushdown); inserts and deletes maintain these
// on unwind and trigger a partial rebuild when a subtree grows unbalanced or
// accumulates too many deleted points.
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
  // changes by a small fraction each scan). A partial rebuild keeps the tree
  // balanced as points accumulate.
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

  // All live (non-deleted) points, in unspecified order. O(n) and allocates;
  // intended for visualization and diagnostics, not the hot path.
  std::vector<Eigen::Vector3f> collect() const;

  // Number of live points in the tree (excludes lazily-deleted points still
  // physically present pending a rebuild).
  size_t size() const noexcept;

  // Physical node count including lazily-deleted nodes not yet reclaimed by a
  // rebuild. For tests and diagnostics; normal callers want size().
  size_t physical_size() const noexcept;

  // Height of the tree (longest root-to-leaf path), 0 when empty. For tests and
  // diagnostics.
  int height() const noexcept;

  // Verify structural invariants (subtree sizes, bounding boxes, split
  // ordering) and the lazy-deletion bookkeeping (invalid_num and tree_deleted,
  // accounting for pending pushdown). Used by the tests and as a debugging aid.
  bool validate() const;

 private:
  struct Node {
    Eigen::Vector3f point;
    std::unique_ptr<Node> left;
    std::unique_ptr<Node> right;
    uint8_t axis = 0;  // split axis (0/1/2)
    Eigen::Vector3f range_min;
    Eigen::Vector3f range_max;
    int treesize = 1;  // physical node count in this subtree
    // Lazy-deletion bookkeeping.
    int invalid_num = 0;        // deleted nodes in this subtree
    bool deleted = false;       // this node's own point is logically deleted
    bool tree_deleted = false;  // every point in this subtree is deleted
    bool pushdown = false;      // a subtree-wide delete is pending propagation
  };

  // A k-NN search candidate; the search keeps these in a bounded max-heap.
  struct HeapItem {
    float dist2;
    Eigen::Vector3f point;
  };

  static bool heap_less(const HeapItem& a, const HeapItem& b);
  static int node_size(const Node* n);
  static int node_height(const Node* n);
  static void pull_up(Node* n);
  static void push_down(Node* n);

  // Whether the subtree at `n` violates the balance or garbage criteria and
  // should be rebuilt.
  static bool needs_rebuild(const Node* n);

  // Collect the live (non-deleted) points of a subtree into `out`.
  static void flatten(const Node* n, std::vector<Eigen::Vector3f>& out);

  // Rebuild the subtree held by `slot` from its live points: balanced, with
  // dead nodes physically reclaimed.
  static void rebuild(std::unique_ptr<Node>& slot);

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

  bool check(const Node* n, int& out_size, int& out_invalid,
             bool& out_tree_deleted, Eigen::Vector3f& out_lo,
             Eigen::Vector3f& out_hi) const;

  std::unique_ptr<Node> root_;
};

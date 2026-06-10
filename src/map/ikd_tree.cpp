#include "map/ikd_tree.h"

#include <algorithm>

namespace {

// Squared distance from a point to an axis-aligned box [lo, hi]. Zero when the
// point is inside the box. Drives k-NN pruning: a subtree can be skipped once
// the closest its box could possibly be is no nearer than the current k-th
// best.
float point_box_dist2(const Eigen::Vector3f& q, const Eigen::Vector3f& lo,
                      const Eigen::Vector3f& hi) {
  float d2 = 0.0f;
  for (int a = 0; a < 3; ++a) {
    const float v =
        q[a] < lo[a] ? lo[a] - q[a] : (q[a] > hi[a] ? q[a] - hi[a] : 0.0f);
    d2 += v * v;
  }
  return d2;
}

}  // namespace

bool IkdTree::heap_less(const HeapItem& a, const HeapItem& b) {
  return a.dist2 < b.dist2;
}

int IkdTree::node_size(const Node* n) { return n ? n->treesize : 0; }

// Recompute a node's cached attributes from its children (bottom-up).
void IkdTree::pull_up(Node* n) {
  n->treesize = 1 + node_size(n->left.get()) + node_size(n->right.get());
  n->range_min = n->point;
  n->range_max = n->point;
  if (n->left) {
    n->range_min = n->range_min.cwiseMin(n->left->range_min);
    n->range_max = n->range_max.cwiseMax(n->left->range_max);
  }
  if (n->right) {
    n->range_min = n->range_min.cwiseMin(n->right->range_min);
    n->range_max = n->range_max.cwiseMax(n->right->range_max);
  }
}

std::unique_ptr<IkdTree::Node> IkdTree::build_range(Eigen::Vector3f* first,
                                                    Eigen::Vector3f* last) {
  if (first >= last) return nullptr;

  // Split on the axis of maximum spread — balances better than depth-cycling on
  // the anisotropic point distributions LiDAR produces (long, flat surfaces).
  Eigen::Vector3f lo = *first;
  Eigen::Vector3f hi = *first;
  for (Eigen::Vector3f* p = first + 1; p < last; ++p) {
    lo = lo.cwiseMin(*p);
    hi = hi.cwiseMax(*p);
  }
  const Eigen::Vector3f spread = hi - lo;
  int axis = 0;
  if (spread[1] > spread[axis]) axis = 1;
  if (spread[2] > spread[axis]) axis = 2;

  // Median split: partition so the middle element sits at its sorted position.
  Eigen::Vector3f* mid = first + (last - first) / 2;
  std::nth_element(first, mid, last,
                   [axis](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
                     return a[axis] < b[axis];
                   });

  auto node = std::make_unique<Node>();
  node->point = *mid;
  node->axis = static_cast<uint8_t>(axis);
  node->left = build_range(first, mid);
  node->right = build_range(mid + 1, last);
  pull_up(node.get());
  return node;
}

void IkdTree::build(std::vector<Eigen::Vector3f> points) {
  root_ = build_range(points.data(), points.data() + points.size());
}

void IkdTree::search(const Node* node, const Eigen::Vector3f& query, int k,
                     std::vector<HeapItem>& heap) const {
  if (!node) return;

  const float d2 = (node->point - query).squaredNorm();
  if (static_cast<int>(heap.size()) < k) {
    heap.push_back({d2, node->point});
    std::push_heap(heap.begin(), heap.end(), heap_less);
  } else if (d2 < heap.front().dist2) {
    // Replace the current worst (heap front) with the closer point.
    std::pop_heap(heap.begin(), heap.end(), heap_less);
    heap.back() = {d2, node->point};
    std::push_heap(heap.begin(), heap.end(), heap_less);
  }

  const uint8_t a = node->axis;
  const bool go_left = query[a] < node->point[a];
  const Node* near = go_left ? node->left.get() : node->right.get();
  const Node* far = go_left ? node->right.get() : node->left.get();

  // Always descend the side the query is on, then the far side only if its
  // bounding box could still hold a closer neighbor than the current k-th best.
  search(near, query, k, heap);
  if (far) {
    const bool heap_full = static_cast<int>(heap.size()) >= k;
    if (!heap_full || point_box_dist2(query, far->range_min, far->range_max) <
                          heap.front().dist2) {
      search(far, query, k, heap);
    }
  }
}

void IkdTree::knn(const Eigen::Vector3f& query, int k,
                  std::vector<Eigen::Vector3f>& out_points,
                  std::vector<float>& out_dist2) const {
  out_points.clear();
  out_dist2.clear();
  if (!root_ || k <= 0) return;

  std::vector<HeapItem> heap;
  heap.reserve(static_cast<size_t>(k));
  search(root_.get(), query, k, heap);

  // The heap is a max-heap by distance; emit in ascending distance order.
  std::sort(heap.begin(), heap.end(), heap_less);
  out_points.resize(heap.size());
  out_dist2.resize(heap.size());
  for (size_t i = 0; i < heap.size(); ++i) {
    out_points[i] = heap[i].point;
    out_dist2[i] = heap[i].dist2;
  }
}

size_t IkdTree::size() const noexcept {
  return root_ ? static_cast<size_t>(root_->treesize) : 0;
}

bool IkdTree::check(const Node* n, int& out_size, Eigen::Vector3f& out_lo,
                    Eigen::Vector3f& out_hi) const {
  if (!n) {
    out_size = 0;
    return true;
  }

  int ls = 0;
  int rs = 0;
  Eigen::Vector3f llo, lhi, rlo, rhi;
  bool ok = true;
  ok = check(n->left.get(), ls, llo, lhi) && ok;
  ok = check(n->right.get(), rs, rlo, rhi) && ok;

  // Subtree size matches the cached count.
  const int sz = 1 + ls + rs;
  if (sz != n->treesize) ok = false;

  // Bounding box equals the actual min/max over the subtree. The recomputation
  // here mirrors pull_up exactly, so an exact comparison is valid.
  Eigen::Vector3f lo = n->point;
  Eigen::Vector3f hi = n->point;
  if (n->left) {
    lo = lo.cwiseMin(llo);
    hi = hi.cwiseMax(lhi);
    // Median split: left subtree stays on the low side of the split axis.
    if (lhi[n->axis] > n->point[n->axis]) ok = false;
  }
  if (n->right) {
    lo = lo.cwiseMin(rlo);
    hi = hi.cwiseMax(rhi);
    if (rlo[n->axis] < n->point[n->axis]) ok = false;
  }
  if (!(lo == n->range_min) || !(hi == n->range_max)) ok = false;

  out_size = sz;
  out_lo = lo;
  out_hi = hi;
  return ok;
}

bool IkdTree::validate() const {
  if (!root_) return true;
  int size = 0;
  Eigen::Vector3f lo, hi;
  return check(root_.get(), size, lo, hi);
}

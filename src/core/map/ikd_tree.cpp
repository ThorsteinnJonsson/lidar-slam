#include "map/ikd_tree.h"

#include <algorithm>
#include <chrono>
#include <future>
#include <utility>

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

// Whether a point lies within the closed axis-aligned box [lo, hi].
bool point_in_box(const Eigen::Vector3f& p, const Eigen::Vector3f& lo,
                  const Eigen::Vector3f& hi) {
  return (p.array() >= lo.array()).all() && (p.array() <= hi.array()).all();
}

// Rebuild thresholds. A subtree is rebuilt when one child holds more than
// kAlphaBalance of it (imbalance) or more than kAlphaDelete of it is lazily
// deleted (garbage). Subtrees at or below kMinRebuildSize are left alone: the
// criteria are noisy at tiny sizes and rebalancing them churns for no benefit.
constexpr float kAlphaBalance = 0.7f;
constexpr float kAlphaDelete = 0.5f;
constexpr int kMinRebuildSize = 10;

// Subtrees at or above this size rebuild on a background thread (the paper's
// N_max); smaller ones rebuild synchronously, where thread hand-off would cost
// more than the rebuild itself.
constexpr int kParallelRebuildSize = 1500;

}  // namespace

bool IkdTree::heap_less(const HeapItem& a, const HeapItem& b) {
  return a.dist2 < b.dist2;
}

int IkdTree::node_size(const Node* n) { return n ? n->treesize : 0; }

int IkdTree::node_height(const Node* n) {
  if (!n) return 0;
  return 1 + std::max(node_height(n->left.get()), node_height(n->right.get()));
}

// Recompute a node's cached attributes from its children (bottom-up). The
// bounding box spans all physical points, deleted ones included, they stay in
// the tree until a rebuild, and a conservative box keeps k-NN pruning correct.
void IkdTree::pull_up(Node* n) {
  n->treesize = 1 + node_size(n->left.get()) + node_size(n->right.get());
  n->range_min = n->point;
  n->range_max = n->point;
  // A null child contributes 0 invalid points and is vacuously "fully deleted"
  // for the tree_deleted conjunction.
  int invalid = n->deleted ? 1 : 0;
  bool all_deleted = n->deleted;
  if (n->left) {
    n->range_min = n->range_min.cwiseMin(n->left->range_min);
    n->range_max = n->range_max.cwiseMax(n->left->range_max);
    invalid += n->left->invalid_num;
    all_deleted = all_deleted && n->left->tree_deleted;
  }
  if (n->right) {
    n->range_min = n->range_min.cwiseMin(n->right->range_min);
    n->range_max = n->range_max.cwiseMax(n->right->range_max);
    invalid += n->right->invalid_num;
    all_deleted = all_deleted && n->right->tree_deleted;
  }
  n->invalid_num = invalid;
  n->tree_deleted = all_deleted;
}

// Lazily propagate a pending subtree-wide deletion to a node's children. In 4.2
// the only lazily-set attribute is "whole subtree deleted" (from box delete),
// so a pending pushdown always means "mark both children fully deleted". Call
// this before descending into a node's children in any mutating op.
void IkdTree::push_down(Node* n) {
  if (!n->pushdown) return;
  for (Node* child : {n->left.get(), n->right.get()}) {
    if (!child) continue;
    child->deleted = true;
    child->tree_deleted = true;
    child->invalid_num = child->treesize;
    child->pushdown = true;
  }
  n->pushdown = false;
}

bool IkdTree::needs_rebuild(const Node* n) {
  if (n->treesize <= kMinRebuildSize) return false;
  const int sl = node_size(n->left.get());
  const int sr = node_size(n->right.get());
  const bool imbalanced =
      std::max(sl, sr) > kAlphaBalance * static_cast<float>(n->treesize - 1);
  const bool garbage =
      static_cast<float>(n->invalid_num) > kAlphaDelete * n->treesize;
  return imbalanced || garbage;
}

void IkdTree::flatten(const Node* n, std::vector<Eigen::Vector3f>& out) {
  if (!n || n->tree_deleted) return;
  if (!n->deleted) out.push_back(n->point);
  flatten(n->left.get(), out);
  flatten(n->right.get(), out);
}

void IkdTree::rebuild(std::unique_ptr<Node>& slot) {
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(slot->treesize);
  flatten(slot.get(), pts);
  // Assigning a fresh balanced subtree frees the old one (dead nodes included).
  // An all-deleted subtree flattens to nothing and collapses to an empty slot.
  slot = build_range(pts.data(), pts.data() + pts.size());
}

bool IkdTree::maybe_rebuild(std::unique_ptr<Node>& slot) {
  // Only one rebuild in flight at a time: while a background rebuild runs,
  // defer every other rebuild (sync or async). This keeps the frozen subtree
  // and its ancestors untouched by any restructuring until the swap. The tree
  // just stays slightly unbalanced for the (brief) window until finalize.
  if (frozen_slot_ != nullptr) return false;
  if (!needs_rebuild(slot.get())) return false;
  if (!replaying_ && slot->treesize >= kParallelRebuildSize) {
    start_async_rebuild(slot);
    return true;
  }
  rebuild(slot);
  return false;
}

void IkdTree::start_async_rebuild(std::unique_ptr<Node>& slot) {
  // Snapshot the frozen subtree's live points on this thread, then let a worker
  // build a balanced replacement from the copy alone (it touches no tree node,
  // so there is no shared mutable state with the main thread).
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(slot->treesize);
  flatten(slot.get(), pts);

  frozen_slot_ = &slot;
  slot->under_rebuild = true;
  capturing_ = true;
  frozen_ancestors_.clear();
  pending_inserts_.clear();
  ++background_rebuilds_;

  rebuild_future_ =
      std::async(std::launch::async, [pts = std::move(pts)]() mutable {
        return build_range(pts.data(), pts.data() + pts.size());
      });
}

void IkdTree::try_finalize_rebuild(bool block) {
  if (frozen_slot_ == nullptr) return;
  if (!block && rebuild_future_.wait_for(std::chrono::seconds(0)) !=
                    std::future_status::ready) {
    return;  // worker still building; the frozen subtree keeps serving queries
  }
  std::unique_ptr<Node> rebuilt = rebuild_future_.get();

  // The worker built from a stale snapshot; the frozen subtree has since taken
  // logged mutations in place. Replay them onto the rebuilt copy so it matches
  // the current live content, then swap. Clear frozen state first so replay
  // runs as ordinary (synchronous) mutations.
  std::unique_ptr<Node>* slot = frozen_slot_;
  frozen_slot_ = nullptr;
  capturing_ = false;
  replaying_ = true;
  for (const Eigen::Vector3f& p : pending_inserts_) insert_at(rebuilt, p);
  replaying_ = false;
  pending_inserts_.clear();

  *slot = std::move(rebuilt);
  // The swapped subtree has a different treesize/box/invalid_num than the
  // frozen one, so refresh the cached attributes of every ancestor up to the
  // root.
  for (Node* a : frozen_ancestors_) pull_up(a);
  frozen_ancestors_.clear();
}

void IkdTree::finish_pending_rebuild() { try_finalize_rebuild(/*block=*/true); }

std::unique_ptr<IkdTree::Node> IkdTree::build_range(Eigen::Vector3f* first,
                                                    Eigen::Vector3f* last) {
  if (first >= last) return nullptr;

  // Split on the axis of maximum spread. Balances better than depth-cycling on
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

void IkdTree::insert_at(std::unique_ptr<Node>& slot,
                        const Eigen::Vector3f& point) {
  if (!slot) {
    // Reached an empty slot. Attach the point as a fresh leaf. A leaf's axis
    // is unused until it gains children; leave it at 0 and let a later rebuild
    // assign a proper max-spread axis.
    slot = std::make_unique<Node>();
    slot->point = point;
    pull_up(slot.get());
    return;
  }
  // Entering a subtree that a worker is rebuilding: log the point for replay
  // onto the rebuilt copy, then apply it here as usual so queries stay correct.
  // Only the frozen root carries the flag, so this logs exactly once per op.
  if (slot->under_rebuild) pending_inserts_.push_back(point);
  // Propagate any pending subtree-wide deletion before descending, so the new
  // leaf lands live rather than under a stale deleted label.
  push_down(slot.get());
  if (point[slot->axis] < slot->point[slot->axis]) {
    insert_at(slot->left, point);
  } else {
    insert_at(slot->right, point);
  }
  pull_up(slot.get());
  const bool started = maybe_rebuild(slot);
  // On the unwind of the op that started a background rebuild, record each
  // ancestor above the frozen node so finalize can refresh their cached stats.
  if (capturing_ && !started) frozen_ancestors_.push_back(slot.get());
}

void IkdTree::insert(const Eigen::Vector3f& point) {
  if (!suppress_auto_finalize_) try_finalize_rebuild(/*block=*/false);
  insert_at(root_, point);
  capturing_ = false;
}

void IkdTree::insert(const std::vector<Eigen::Vector3f>& points) {
  if (!suppress_auto_finalize_) try_finalize_rebuild(/*block=*/false);
  for (const auto& p : points) {
    insert_at(root_, p);
    capturing_ = false;  // one ancestor-capture window per op
  }
}

void IkdTree::remove_box_at(std::unique_ptr<Node>& slot,
                            const Eigen::Vector3f& box_min,
                            const Eigen::Vector3f& box_max) {
  Node* n = slot.get();
  if (!n || n->tree_deleted) return;

  // No overlap between the delete box and this subtree's bounding box: skip.
  if ((n->range_max.array() < box_min.array()).any() ||
      (n->range_min.array() > box_max.array()).any()) {
    return;
  }

  // Subtree fully inside the delete box: flag the whole subtree deleted lazily,
  // deferring the per-child relabel to push_down when (if) we next descend
  // here.
  if ((n->range_min.array() >= box_min.array()).all() &&
      (n->range_max.array() <= box_max.array()).all()) {
    n->deleted = true;
    n->tree_deleted = true;
    n->invalid_num = n->treesize;
    n->pushdown = true;
    return;
  }

  // Partial overlap: push any pending deletion down, test this node's own
  // point, recurse into both children, then refresh cached attributes.
  push_down(n);
  if (!n->deleted && point_in_box(n->point, box_min, box_max))
    n->deleted = true;
  remove_box_at(n->left, box_min, box_max);
  remove_box_at(n->right, box_min, box_max);
  pull_up(n);
  const bool started = maybe_rebuild(slot);
  if (capturing_ && !started) frozen_ancestors_.push_back(n);
}

void IkdTree::remove_box(const Eigen::Vector3f& box_min,
                         const Eigen::Vector3f& box_max) {
  // Finish any in-flight rebuild first: a box-delete can lazily mark an
  // ancestor of a frozen subtree deleted without descending into it, which the
  // replay buffer (inserts only) could not reconcile. Blocking here keeps
  // deletes off the frozen path entirely. Box-deletes are rare (map slides), so
  // the wait is negligible.
  try_finalize_rebuild(/*block=*/true);
  remove_box_at(root_, box_min, box_max);
  capturing_ = false;
}

void IkdTree::search(const Node* node, const Eigen::Vector3f& query, size_t k,
                     std::vector<HeapItem>& heap) const {
  // Skip whole subtrees that are lazily deleted; the per-node `deleted` check
  // below skips individual deleted points within live subtrees.
  if (!node || node->tree_deleted) return;

  if (!node->deleted) {
    const float d2 = (node->point - query).squaredNorm();
    if (heap.size() < k) {
      heap.push_back({d2, node->point});
      std::push_heap(heap.begin(), heap.end(), heap_less);
    } else if (d2 < heap.front().dist2) {
      // Replace the current worst (heap front) with the closer point.
      std::pop_heap(heap.begin(), heap.end(), heap_less);
      heap.back() = {d2, node->point};
      std::push_heap(heap.begin(), heap.end(), heap_less);
    }
  }

  const uint8_t a = node->axis;
  const bool go_left = query[a] < node->point[a];
  const Node* near = go_left ? node->left.get() : node->right.get();
  const Node* far = go_left ? node->right.get() : node->left.get();

  // Always descend the side the query is on, then the far side only if its
  // bounding box could still hold a closer neighbor than the current k-th best.
  search(near, query, k, heap);
  if (far) {
    const bool heap_full = heap.size() >= k;
    if (!heap_full || point_box_dist2(query, far->range_min, far->range_max) <
                          heap.front().dist2) {
      search(far, query, k, heap);
    }
  }
}

void IkdTree::knn(const Eigen::Vector3f& query, size_t k,
                  std::vector<Eigen::Vector3f>& out_points,
                  std::vector<float>& out_dist2) const {
  out_points.clear();
  out_dist2.clear();
  if (!root_ || k == 0) return;

  std::vector<HeapItem> heap;
  heap.reserve(k);
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
  // Live point count: physical nodes minus those lazily deleted but not yet
  // collected by a rebuild.
  return root_ ? static_cast<size_t>(root_->treesize - root_->invalid_num) : 0;
}

size_t IkdTree::physical_size() const noexcept {
  return root_ ? static_cast<size_t>(root_->treesize) : 0;
}

std::vector<Eigen::Vector3f> IkdTree::collect() const {
  std::vector<Eigen::Vector3f> out;
  out.reserve(size());
  flatten(root_.get(), out);
  return out;
}

int IkdTree::height() const noexcept { return node_height(root_.get()); }

bool IkdTree::check(const Node* n, int& out_size, int& out_invalid,
                    bool& out_tree_deleted, Eigen::Vector3f& out_lo,
                    Eigen::Vector3f& out_hi) const {
  if (!n) {
    out_size = 0;
    out_invalid = 0;
    out_tree_deleted = true;  // an empty subtree is vacuously fully deleted
    return true;
  }

  int ls = 0;
  int rs = 0;
  int li = 0;
  int ri = 0;
  bool l_td = true;
  bool r_td = true;
  Eigen::Vector3f llo, lhi, rlo, rhi;
  bool ok = true;
  ok = check(n->left.get(), ls, li, l_td, llo, lhi) && ok;
  ok = check(n->right.get(), rs, ri, r_td, rlo, rhi) && ok;

  // Subtree size matches the cached count. treesize counts physical nodes
  // (deleted ones included), so lazy deletion does not change it.
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

  // Deletion bookkeeping. A pending pushdown marks the whole subtree deleted
  // without having relabeled the children yet, so its children's labels are
  // stale and must not be used: the node is fully deleted by definition.
  // pushdown nodes are never nested, so the children stay self-consistent and
  // are still checked by the recursion above.
  int invalid;
  bool tree_deleted;
  if (n->pushdown) {
    if (!n->deleted) ok = false;
    invalid = n->treesize;
    tree_deleted = true;
  } else {
    invalid = (n->deleted ? 1 : 0) + li + ri;
    tree_deleted = n->deleted && l_td && r_td;
  }
  if (invalid != n->invalid_num) ok = false;
  if (tree_deleted != n->tree_deleted) ok = false;

  out_size = sz;
  out_invalid = invalid;
  out_tree_deleted = tree_deleted;
  out_lo = lo;
  out_hi = hi;
  return ok;
}

bool IkdTree::validate() const {
  if (!root_) return true;
  int size = 0;
  int invalid = 0;
  bool tree_deleted = false;
  Eigen::Vector3f lo, hi;
  return check(root_.get(), size, invalid, tree_deleted, lo, hi);
}

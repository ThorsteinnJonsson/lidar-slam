#include "map/association.h"

#include <Eigen/Dense>
#include <cmath>

namespace {

// Fit a plane n_hat . x + d = 0 to `pts` by solving A n = -1 (rows of A are the
// points), then normalizing. Returns false if the fit is degenerate (e.g. a
// plane through the origin, where A n = -1 has no solution) or the points are
// too non-planar (some point farther than max_dist from the fitted plane).
bool fit_plane(const std::vector<Eigen::Vector3f>& pts, float max_dist,
               Eigen::Vector3f& n_hat, float& d) {
  const Eigen::Index m = static_cast<Eigen::Index>(pts.size());
  Eigen::MatrixXf a(m, 3);
  for (Eigen::Index i = 0; i < m; ++i) a.row(i) = pts[i].transpose();
  const Eigen::VectorXf b = Eigen::VectorXf::Constant(m, -1.0f);

  const Eigen::Vector3f n = a.colPivHouseholderQr().solve(b);
  const float norm = n.norm();
  if (norm < 1e-6f) return false;

  n_hat = n / norm;
  d = 1.0f / norm;

  for (const auto& p : pts) {
    if (std::abs(n_hat.dot(p) + d) > max_dist) return false;
  }
  return true;
}

}  // namespace

std::vector<PlaneMatch> associate_planes(
    const IkdTree& map, const std::vector<Eigen::Vector3f>& points_body,
    const Sophus::SE3f& T_world_body, const PlaneAssocParams& params) {
  std::vector<PlaneMatch> matches;
  matches.reserve(points_body.size());

  const size_t k = static_cast<size_t>(params.num_neighbors);
  std::vector<Eigen::Vector3f> neighbors;
  std::vector<float> dist2;
  for (const auto& p_body : points_body) {
    const Eigen::Vector3f p_world = T_world_body * p_body;
    map.knn(p_world, k, neighbors, dist2);

    // Need a full neighborhood, and the farthest neighbor must be close enough
    // that there is plausibly a surface here (dist2 is sorted ascending).
    if (neighbors.size() < k) continue;
    if (dist2.back() > params.max_neighbor_dist2) continue;

    Eigen::Vector3f n_hat;
    float d;
    if (!fit_plane(neighbors, params.max_plane_dist, n_hat, d)) continue;

    const float residual = n_hat.dot(p_world) + d;

    // FAST-LIO2 range-normalized quality gate (h_share_model in laserMapping):
    // drop the point if it sits too far off its plane relative to sqrt(range).
    const float range = p_body.norm();
    if (range < 1e-6f) continue;  // degenerate point at the sensor origin
    const float s = 1.0f - 0.9f * std::abs(residual) / std::sqrt(range);
    if (s <= params.min_quality) continue;

    matches.push_back({p_body, n_hat, residual});
  }
  return matches;
}

#include "io/params.h"

#include <yaml-cpp/yaml.h>

#include <set>
#include <stdexcept>

namespace {

// Overwrite `out` only when `key` is present, so an absent key keeps the struct
// default. Records the key as recognized either way, so unknown-key detection
// can tell a typo from a deliberately omitted value.
template <typename T>
void set_if(const YAML::Node& node, const char* key, T& out,
            std::set<std::string>& recognized) {
  recognized.insert(key);
  const YAML::Node value = node[key];
  if (!value) return;
  try {
    out = value.as<T>();
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("params: bad value for '" + std::string(key) +
                             "': " + e.what());
  }
}

// Report keys present in `node` that the loader never looked at. `prefix` is
// the dotted section path, e.g. "map.crop.".
void collect_unknown(const YAML::Node& node,
                     const std::set<std::string>& recognized,
                     const std::string& prefix,
                     std::vector<std::string>* warnings) {
  if (!warnings || !node || !node.IsMap()) return;
  for (const auto& entry : node) {
    const std::string key = entry.first.as<std::string>();
    if (!recognized.contains(key))
      warnings->push_back("params: unknown key '" + prefix + key + "' ignored");
  }
}

}  // namespace

Params load_params(const std::filesystem::path& path,
                   std::vector<std::string>* warnings) {
  if (!std::filesystem::exists(path))
    throw std::runtime_error("params: file not found: " + path.string());

  YAML::Node root;
  try {
    root = YAML::LoadFile(path.string());
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("params: failed to parse " + path.string() + ": " +
                             e.what());
  }
  if (!root.IsMap())
    throw std::runtime_error("params: " + path.string() +
                             " is not a YAML mapping");

  Params p;
  std::set<std::string> top;

  set_if(root, "lidar_time_offset_sec", p.lidar_time_offset_sec, top);

  // Fetch a section and mark its name recognized at the top level.
  const auto section = [&](const char* name) {
    top.insert(name);
    return root[name];
  };

  if (const YAML::Node n = section("preprocess")) {
    std::set<std::string> k;
    set_if(n, "scan_voxel_leaf", p.scan_voxel_leaf, k);
    collect_unknown(n, k, "preprocess.", warnings);
  }

  if (const YAML::Node n = section("init")) {
    std::set<std::string> k;
    set_if(n, "imu_count", p.init_imu_count, k);
    set_if(n, "gravity_magnitude", p.init.gravity_magnitude, k);
    set_if(n, "max_accel_var", p.init.max_accel_var, k);
    set_if(n, "max_gyro_var", p.init.max_gyro_var, k);
    set_if(n, "theta_std", p.init.theta_std, k);
    set_if(n, "pos_std", p.init.pos_std, k);
    set_if(n, "vel_std", p.init.vel_std, k);
    set_if(n, "bias_gyro_std", p.init.bias_gyro_std, k);
    set_if(n, "bias_accel_std", p.init.bias_accel_std, k);
    set_if(n, "gravity_std", p.init.gravity_std, k);
    set_if(n, "ext_rot_std", p.init.ext_rot_std, k);
    set_if(n, "ext_trans_std", p.init.ext_trans_std, k);
    collect_unknown(n, k, "init.", warnings);
  }

  if (const YAML::Node n = section("imu_noise")) {
    std::set<std::string> k;
    set_if(n, "gyro_noise_std", p.noise.gyro_noise_std, k);
    set_if(n, "accel_noise_std", p.noise.accel_noise_std, k);
    set_if(n, "gyro_rw_std", p.noise.gyro_rw_std, k);
    set_if(n, "accel_rw_std", p.noise.accel_rw_std, k);
    collect_unknown(n, k, "imu_noise.", warnings);
  }

  if (const YAML::Node n = section("iekf")) {
    std::set<std::string> k;
    set_if(n, "sigma", p.iekf.sigma, k);
    set_if(n, "max_iterations", p.iekf.max_iterations, k);
    set_if(n, "convergence_tol", p.iekf.convergence_tol, k);
    set_if(n, "reject_outliers", p.iekf.reject_outliers, k);
    set_if(n, "outlier_chi2", p.iekf.outlier_chi2, k);
    collect_unknown(n, k, "iekf.", warnings);
  }

  if (const YAML::Node n = section("association")) {
    std::set<std::string> k;
    set_if(n, "num_neighbors", p.assoc.num_neighbors, k);
    set_if(n, "max_neighbor_dist2", p.assoc.max_neighbor_dist2, k);
    set_if(n, "max_plane_dist", p.assoc.max_plane_dist, k);
    set_if(n, "min_quality", p.assoc.min_quality, k);
    collect_unknown(n, k, "association.", warnings);
  }

  if (const YAML::Node n = section("map")) {
    std::set<std::string> k;
    set_if(n, "voxel_on_insert", p.map.voxel_on_insert, k);
    set_if(n, "voxel_leaf", p.map.voxel_leaf, k);
    k.insert("crop");
    if (const YAML::Node c = n["crop"]) {
      std::set<std::string> ck;
      set_if(c, "enabled", p.map.crop.enabled, ck);
      set_if(c, "sensor_range", p.map.crop.sensor_range, ck);
      set_if(c, "margin_factor", p.map.crop.margin_factor, ck);
      set_if(c, "keep_factor", p.map.crop.keep_factor, ck);
      collect_unknown(c, ck, "map.crop.", warnings);
    }
    collect_unknown(n, k, "map.", warnings);
  }

  if (const YAML::Node n = section("estimator")) {
    std::set<std::string> k;
    set_if(n, "enable_extrinsic_estimation", p.enable_extrinsic_estimation, k);
    collect_unknown(n, k, "estimator.", warnings);
  }

  // LiDAR-to-IMU extrinsic: translation [x,y,z] (m) and rotation as a unit
  // quaternion [w,x,y,z]. Both optional; each defaults to identity.
  if (const YAML::Node n = section("extrinsic")) {
    std::set<std::string> k{"translation", "rotation"};
    Eigen::Vector3d t = p.extrinsic.translation();
    Eigen::Quaterniond q = p.extrinsic.unit_quaternion();
    if (const YAML::Node tn = n["translation"]) {
      const auto v = tn.as<std::vector<double>>();
      if (v.size() != 3)
        throw std::runtime_error(
            "params: extrinsic.translation needs 3 values");
      t = {v[0], v[1], v[2]};
    }
    if (const YAML::Node rn = n["rotation"]) {
      const auto v = rn.as<std::vector<double>>();
      if (v.size() != 4)
        throw std::runtime_error(
            "params: extrinsic.rotation needs 4 values [w,x,y,z]");
      q = Eigen::Quaterniond(v[0], v[1], v[2], v[3]).normalized();
    }
    p.extrinsic = Sophus::SE3d(Sophus::SO3d(q), t);
    collect_unknown(n, k, "extrinsic.", warnings);
  }

  collect_unknown(root, top, "", warnings);
  return p;
}

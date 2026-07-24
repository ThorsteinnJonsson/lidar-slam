#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <filesystem>
#include <sophus/se3.hpp>
#include <string>
#include <vector>

#include "estimator/iekf.h"
#include "imu/initializer.h"
#include "imu/propagator.h"
#include "map/association.h"
#include "map/local_map.h"

// Tunable runtime parameters, read from a YAML file (see config/README.md).
//
// Every field defaults to the value baked into its owning struct, and the
// loader overwrites only the keys actually present in the file, so the file may
// be partial and the structs stay the single source of truth for defaults.
//
// Dataset selection (root directory, sequence name) is deliberately absent: it
// becomes a command-line argument. The lidar-to-IMU time offset lives here for
// now even though it is arguably a per-dataset calibration value.
struct Params {
  double lidar_time_offset_sec{-0.1};
  size_t init_imu_count{200};   // static IMU samples used to initialize
  double scan_voxel_leaf{0.5};  // per-scan downsample leaf (m)
  bool enable_extrinsic_estimation{false};

  NoiseParams noise;
  InitParams init;
  IekfConfig iekf;
  PlaneAssocParams assoc;
  LocalMapParams map;

  // LiDAR-to-IMU extrinsic, used by dataset loaders whose sequence does not
  // ship a calibration file (HILTI, FAST-LIVO2). NTU VIRAL reads it from the
  // dataset YAMLs and ignores this. Default identity.
  Sophus::SE3d extrinsic;
};

// Load parameters from a YAML file.
//
// Throws std::runtime_error if the file is missing, is not a YAML map, fails to
// parse, or holds a value of the wrong type (the message names the offending
// key). Keys the loader does not recognize are appended to `warnings` when
// non-null rather than being silently dropped: a typo would otherwise leave the
// default quietly in place, which is exactly the bug this file exists to avoid.
Params load_params(const std::filesystem::path& path,
                   std::vector<std::string>* warnings = nullptr);

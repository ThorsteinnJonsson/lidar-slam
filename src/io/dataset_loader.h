#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <ostream>
#include <sophus/se3.hpp>
#include <span>
#include <string>

#include "types.h"

// Everything that varies between dataset formats (NTU VIRAL, HILTI, FAST-LIVO2)
// lives behind this interface, so main.cpp drives one format-agnostic pipeline.
// A loader resolves paths and calibration for one sequence, decodes that
// dataset's lidar message type, and supplies ground truth.

// A position-only ground-truth sample (the datasets here track a point, not a
// full pose).
struct GtRow {
  double t_sec;
  Eigen::Vector3d pos;
};

class DatasetLoader {
 public:
  virtual ~DatasetLoader() = default;

  virtual std::filesystem::path bag_path() const = 0;
  virtual std::string imu_topic() const = 0;
  virtual std::string lidar_topic() const = 0;
  // Ground-truth topic in the bag, or "" when GT comes from a file or is
  // absent.
  virtual std::string gt_topic() const { return {}; }

  // Lidar-to-IMU extrinsic, seeding the filter state.
  virtual Sophus::SE3d T_imu_lidar() const = 0;

  // The body point that ground truth tracks, expressed in the IMU frame. The
  // estimate is shifted by this before comparison (the lever arm). Zero when GT
  // tracks the IMU origin.
  virtual Eigen::Vector3d imu_to_gt_point() const {
    return Eigen::Vector3d::Zero();
  }

  virtual bool has_gt() const = 0;

  // Scale applied to the IMU linear acceleration on ingest. The Livox Avia
  // reports acceleration in units of g, so it needs converting to m/s²; other
  // datasets already report m/s² and use 1.
  virtual double imu_accel_scale() const { return 1.0; }

  // Decode a raw lidar message on lidar_topic() into a PointCloud whose
  // t_offset_ns is normalized to nanoseconds from the header stamp.
  virtual PointCloud decode_cloud(std::span<const std::byte> data) const = 0;

  // In-bag GT: decode one message on gt_topic() to a position row.
  virtual std::optional<GtRow> decode_gt(
      std::span<const std::byte> /*data*/) const {
    return std::nullopt;
  }

  // External-file GT: write the gt.tum rows directly (called once at startup).
  // Returns the number of rows written; 0 when GT is not from a file.
  virtual size_t write_external_gt(std::ostream& /*out*/) const { return 0; }
};

struct Params;

// Build the loader for a format. `sequence` is the sequence directory (or, for
// formats with a loose bag, the directory holding it). `params` supplies the
// extrinsic for datasets that do not ship a calibration. Throws
// std::runtime_error on an unknown format or a missing/ambiguous bag.
std::unique_ptr<DatasetLoader> make_loader(
    const std::string& format, const std::filesystem::path& sequence,
    const Params& params);

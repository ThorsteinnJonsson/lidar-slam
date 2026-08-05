#include "io/dataset_loader.h"

#include <stdexcept>
#include <vector>

#include "io/calibration.h"
#include "io/params.h"
#include "io/ros_deserializer.h"

namespace {

// The single .bag file in `dir`. Throws if there is none or more than one.
std::filesystem::path find_bag(const std::filesystem::path& dir) {
  std::vector<std::filesystem::path> bags;
  for (const auto& e : std::filesystem::directory_iterator(dir))
    if (e.path().extension() == ".bag") bags.push_back(e.path());
  if (bags.empty()) throw std::runtime_error("No .bag file in " + dir.string());
  if (bags.size() > 1)
    throw std::runtime_error("Multiple .bag files in " + dir.string() +
                             "; expected one");
  return bags.front();
}

// NTU VIRAL: one directory per sequence, calibration in per-sequence YAMLs,
// ground truth streamed from the in-bag Leica prism topic.
class NtuViralLoader : public DatasetLoader {
 public:
  explicit NtuViralLoader(const std::filesystem::path& sequence)
      : dir_(sequence), sequence_(sequence.filename().string()) {
    // rtp_03 and tnp_01 ship no imu_v100.yaml; across NTU VIRAL the IMU topic
    // and body-IMU extrinsic are the same constants, so a default covers them.
    const std::filesystem::path imu_yaml = dir_ / "imu_v100.yaml";
    imu_cal_ = std::filesystem::exists(imu_yaml)
                   ? load_imu_calibration(imu_yaml)
                   : ImuCalibration{.topic = "/imu/imu"};
    lidar_cal_ = load_lidar_calibration(dir_ / "lidar_horz.yaml");
    prism_cal_ = load_prism_calibration(dir_ / "leica_prism.yaml");
  }

  std::filesystem::path bag_path() const override {
    return dir_ / (sequence_ + ".bag");
  }
  std::string imu_topic() const override { return imu_cal_.topic; }
  std::string lidar_topic() const override { return lidar_cal_.topic; }
  std::string gt_topic() const override { return prism_cal_.topic; }

  Sophus::SE3d T_imu_lidar() const override {
    return imu_from_lidar(imu_cal_, lidar_cal_);
  }
  Eigen::Vector3d imu_to_gt_point() const override {
    return (imu_cal_.T_body_imu.inverse() * prism_cal_.T_body_prism)
        .translation();
  }
  bool has_gt() const override { return true; }

  PointCloud decode_cloud(std::span<const std::byte> data) const override {
    return deserialize_pointcloud2(data);
  }

  std::optional<GtRow> decode_gt(
      std::span<const std::byte> data) const override {
    const PoseStampedMsg gt = deserialize_pose_stamped(data);
    return GtRow{gt.stamp.to_sec(), gt.position};
  }

 private:
  std::filesystem::path dir_;
  std::string sequence_;
  ImuCalibration imu_cal_;
  LidarCalibration lidar_cal_;
  PrismCalibration prism_cal_;
};

// HILTI 2022: Hesai PandarXT-32 (PointCloud2 with absolute-second per-point
// time) + Alphasense IMU. The shipped ground truth is just a handful of sparse
// survey control points, not a trajectory, so we do not emit it. The extrinsic
// is not in the sequence, so it comes from params.
class HiltiLoader : public DatasetLoader {
 public:
  HiltiLoader(const std::filesystem::path& sequence, const Params& params)
      : bag_(find_bag(sequence)), extrinsic_(params.extrinsic) {}

  std::filesystem::path bag_path() const override { return bag_; }
  std::string imu_topic() const override { return "/alphasense/imu"; }
  std::string lidar_topic() const override { return "/hesai/pandar"; }
  Sophus::SE3d T_imu_lidar() const override { return extrinsic_; }
  bool has_gt() const override { return false; }

  PointCloud decode_cloud(std::span<const std::byte> data) const override {
    return deserialize_pointcloud2(data, PointTimeMode::AbsoluteSeconds);
  }

 private:
  std::filesystem::path bag_;
  Sophus::SE3d extrinsic_;
};

// FAST-LIVO2: Livox Avia (CustomMsg) + built-in IMU. No ground truth ships with
// this sequence. Extrinsic comes from params.
class FastLivo2Loader : public DatasetLoader {
 public:
  FastLivo2Loader(const std::filesystem::path& sequence, const Params& params)
      : bag_(find_bag(sequence)), extrinsic_(params.extrinsic) {}

  std::filesystem::path bag_path() const override { return bag_; }
  std::string imu_topic() const override { return "/livox/imu"; }
  std::string lidar_topic() const override { return "/livox/lidar"; }
  Sophus::SE3d T_imu_lidar() const override { return extrinsic_; }
  bool has_gt() const override { return false; }
  double imu_accel_scale() const override { return 9.80665; }  // g -> m/s^2

  PointCloud decode_cloud(std::span<const std::byte> data) const override {
    return deserialize_livox_custommsg(data);
  }

 private:
  std::filesystem::path bag_;
  Sophus::SE3d extrinsic_;
};

}  // namespace

std::unique_ptr<DatasetLoader> make_loader(
    const std::string& format, const std::filesystem::path& sequence,
    const Params& params) {
  if (format == "NTU_VIRAL") return std::make_unique<NtuViralLoader>(sequence);
  if (format == "HILTI_22")
    return std::make_unique<HiltiLoader>(sequence, params);
  if (format == "FAST_LIVO2")
    return std::make_unique<FastLivo2Loader>(sequence, params);
  throw std::runtime_error("Unsupported dataset format: " + format);
}

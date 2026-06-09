#include <spdlog/spdlog.h>

#include <cmath>
#include <optional>
#include <print>

#include "imu/buffer.h"
#include "io/bag_reader.h"
#include "io/calibration.h"
#include "io/ros_deserializer.h"
#include "preprocess/deskew.h"
#include "preprocess/voxel_grid.h"
#include "types.h"

int main() {
  spdlog::info("lidar-slam initializing...");

  const std::string dataset = "datasets/ntu_viral/eee_03";

  auto imu_cal = load_imu_calibration(dataset + "/imu_v100.yaml");
  auto lidar_cal = load_lidar_calibration(dataset + "/lidar_horz.yaml");

  spdlog::info("IMU   topic : {}", imu_cal.topic);
  spdlog::info("IMU   accel noise/rw : {:.4e} / {:.4e}",
               imu_cal.accel_noise_std, imu_cal.accel_rw_std);
  spdlog::info("IMU   gyro  noise/rw : {:.4e} / {:.4e}", imu_cal.gyro_noise_std,
               imu_cal.gyro_rw_std);
  spdlog::info("T_body_imu  translation: [{:.3f}, {:.3f}, {:.3f}]",
               imu_cal.T_body_imu.translation().x(),
               imu_cal.T_body_imu.translation().y(),
               imu_cal.T_body_imu.translation().z());

  spdlog::info("LiDAR topic : {}", lidar_cal.topic);
  spdlog::info("LiDAR res   : {}v x {}h", lidar_cal.vert_res,
               lidar_cal.horz_res);
  spdlog::info("T_body_lidar translation: [{:.3f}, {:.3f}, {:.3f}]",
               lidar_cal.T_body_lidar.translation().x(),
               lidar_cal.T_body_lidar.translation().y(),
               lidar_cal.T_body_lidar.translation().z());

  const Sophus::SE3d T_imu_lidar = imu_from_lidar(imu_cal, lidar_cal);

  BagReader reader(dataset + "/eee_03.bag");

  // Single pass: buffer all IMU, grab the first point cloud for a deskew demo.
  ImuBuffer imu_buffer;
  std::optional<PointCloud> first_cloud;
  size_t imu_count = 0;
  size_t cloud_count = 0;

  reader.read_messages({imu_cal.topic, lidar_cal.topic},
                       [&](const std::string& topic, uint64_t /*stamp_ns*/,
                           std::span<const std::byte> data) {
                         if (topic == imu_cal.topic) {
                           ++imu_count;
                           imu_buffer.push(deserialize_imu(data));
                         } else {
                           ++cloud_count;
                           if (!first_cloud) {
                             first_cloud = deserialize_pointcloud2(data);
                           }
                         }
                       });

  spdlog::info("IMU measurements : {}", imu_count);
  spdlog::info("Point clouds     : {}", cloud_count);

  if (first_cloud && first_cloud->size() > 0) {
    const PointCloud& cloud = *first_cloud;

    // Scan window from per-point time offsets.
    const auto [min_off, max_off] =
        std::minmax_element(cloud.t_offset_ns.begin(), cloud.t_offset_ns.end());
    const uint64_t base_ns = cloud.stamp.to_nsec();
    const uint64_t t_start = base_ns + *min_off;
    const uint64_t t_end = base_ns + *max_off;
    spdlog::info("First scan: {} points, span {:.1f} ms", cloud.size(),
                 (t_end - t_start) * 1e-6);

    if (imu_buffer.covers(t_start, t_end)) {
      const auto traj = build_scan_trajectory(imu_buffer, t_start, t_end);
      const PointCloud undist = deskew(cloud, traj, T_imu_lidar, t_end);

      double max_shift = 0.0;
      for (size_t i = 0; i < cloud.size(); ++i) {
        const double dx = undist.x[i] - cloud.x[i];
        const double dy = undist.y[i] - cloud.y[i];
        const double dz = undist.z[i] - cloud.z[i];
        max_shift = std::max(max_shift, std::sqrt(dx * dx + dy * dy + dz * dz));
      }
      spdlog::info("Deskew max point shift : {:.4f} m", max_shift);

      const PointCloud filtered = voxel_downsample(undist, 0.5);
      spdlog::info("Voxel downsample: {} -> {} points ({:.1f}% kept)",
                   undist.size(), filtered.size(),
                   100.0 * filtered.size() / undist.size());
    } else {
      spdlog::warn("IMU buffer does not cover the first scan window");
    }
  }

  return 0;
}

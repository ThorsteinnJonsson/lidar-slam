#include <spdlog/spdlog.h>

#include <print>

#include "io/bag_reader.h"
#include "io/calibration.h"
#include "io/ros_deserializer.h"
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

  BagReader reader(dataset + "/eee_03.bag");

  size_t imu_count = 0;
  size_t cloud_count = 0;

  reader.read_messages({imu_cal.topic, lidar_cal.topic},
                       [&](const std::string& topic, uint64_t /*stamp_ns*/,
                           std::span<const std::byte> data) {
                         if (topic == imu_cal.topic) {
                           ++imu_count;
                           (void)deserialize_imu(data);
                         } else {
                           ++cloud_count;
                           (void)deserialize_pointcloud2(data);
                         }
                       });

  spdlog::info("IMU measurements : {}", imu_count);
  spdlog::info("Point clouds     : {}", cloud_count);

  return 0;
}

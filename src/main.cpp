#include <spdlog/spdlog.h>

#include <print>

#include "io/bag_reader.h"
#include "io/ros_deserializer.h"
#include "types.h"

int main() {
  spdlog::info("lidar-slam initializing...");

  BagReader reader("datasets/ntu_viral/eee_03/eee_03.bag");

  size_t imu_count = 0;
  size_t cloud_count = 0;

  reader.read_messages({"/imu/imu", "/os1_cloud_node1/points"},
                       [&](const std::string& topic, uint64_t /*stamp_ns*/,
                           std::span<const std::byte> data) {
                         if (topic == "/imu/imu") {
                           auto imu = deserialize_imu(data);
                           (void)imu;
                           ++imu_count;
                         } else {
                           auto cloud = deserialize_pointcloud2(data);
                           (void)cloud;
                           ++cloud_count;
                         }
                       });

  spdlog::info("IMU measurements : {}", imu_count);
  spdlog::info("Point clouds     : {}", cloud_count);

  return 0;
}

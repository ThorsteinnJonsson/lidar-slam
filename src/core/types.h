#pragma once

#include <Eigen/Geometry>
#include <cstdint>
#include <string>
#include <vector>

struct Timestamp {
  uint32_t secs{0};
  uint32_t nsecs{0};

  uint64_t to_nsec() const noexcept {
    return static_cast<uint64_t>(secs) * 1'000'000'000ULL + nsecs;
  }

  double to_sec() const noexcept { return secs + nsecs * 1e-9; }

  static Timestamp from_nsec(uint64_t ns) {
    return {static_cast<uint32_t>(ns / 1'000'000'000ULL),
            static_cast<uint32_t>(ns % 1'000'000'000ULL)};
  }
};

struct ImuMeasurement {
  Timestamp stamp;
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
};

// Struct-of-arrays layout for cache-friendly access in SLAM inner loops
struct PointCloud {
  Timestamp stamp;  // header stamp; per-point time = stamp + t_offset_ns[i]
  std::string frame_id;
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> z;
  std::vector<float> intensity;
  std::vector<uint16_t> ring;
  std::vector<uint32_t> t_offset_ns;  // per-point time offset from stamp (ns)

  size_t size() const noexcept { return x.size(); }

  void reserve(size_t n) {
    x.reserve(n);
    y.reserve(n);
    z.reserve(n);
    intensity.reserve(n);
    ring.reserve(n);
    t_offset_ns.reserve(n);
  }
};

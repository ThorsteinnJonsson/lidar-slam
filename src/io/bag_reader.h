#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <span>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Reads a ROS bag V2.0 file without requiring a ROS installation.
// Supports lz4 and uncompressed chunks. bz2 is not supported.
//
// Usage:
//   BagReader reader("path/to/file.bag");
//   reader.read_messages({"/imu/imu", "/points"}, [](auto& topic, auto stamp_ns, auto data) {
//       ...
//   });
class BagReader {
 public:
  using Callback = std::function<void(const std::string& topic, uint64_t stamp_ns,
                                      std::span<const std::byte> data)>;

  explicit BagReader(std::filesystem::path path);

  // Iterates through all messages on the given topics in chronological order
  // (within each chunk; chunk order is file order). Calls cb for each message.
  void read_messages(const std::vector<std::string>& topics, Callback cb);

 private:
  std::filesystem::path path_;
  std::unordered_map<uint32_t, std::string> conn_topics_;  // conn_id → topic

  // Pass 1: seek to the index section and register all CONNECTION records.
  void scan_connections();

  // Pass 2: decompress each CHUNK and dispatch MESSAGE_DATA records.
  void process_chunk(std::span<const std::byte> data,
                     const std::unordered_set<uint32_t>& filter,
                     const Callback& cb) const;
};

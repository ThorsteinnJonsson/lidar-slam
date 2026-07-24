// Dev tool: dump a ROS bag's topics/types, and for one PointCloud2 lidar topic,
// the per-point field layout plus the header vs first/last point timestamps so
// the per-point time semantics (ns offset vs absolute seconds) are unambiguous.
//
//   bag_inspect <bag> [lidar_topic]

#include <CLI/CLI.hpp>

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <optional>
#include <span>
#include <string>

#include "io/bag_reader.h"

namespace {

// Minimal PointCloud2 field dump, independent of the main deserializer so it can
// report raw field metadata (name, datatype, offset) for any layout.
struct Field {
  std::string name;
  uint32_t offset;
  uint8_t datatype;  // ROS PointField enum
};

const char* datatype_name(uint8_t d) {
  switch (d) {
    case 1: return "INT8";
    case 2: return "UINT8";
    case 3: return "INT16";
    case 4: return "UINT16";
    case 5: return "INT32";
    case 6: return "UINT32";
    case 7: return "FLOAT32";
    case 8: return "FLOAT64";
    default: return "?";
  }
}

template <typename T>
T read(const std::byte* p) {
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

double field_as_double(const std::byte* p, uint8_t datatype) {
  switch (datatype) {
    case 6: return read<uint32_t>(p);
    case 7: return read<float>(p);
    case 8: return read<double>(p);
    case 5: return read<int32_t>(p);
    case 4: return read<uint16_t>(p);
    default: return 0.0;
  }
}

void dump_pointcloud2(std::span<const std::byte> data) {
  const std::byte* p = data.data();
  auto u32 = [&]() {
    uint32_t v = read<uint32_t>(p);
    p += 4;
    return v;
  };
  u32();                             // seq
  const uint32_t secs = u32();
  const uint32_t nsecs = u32();
  const uint32_t frame_len = u32();  // frame_id string
  p += frame_len;
  const uint32_t height = u32();
  const uint32_t width = u32();
  const uint32_t num_fields = u32();

  std::vector<Field> fields;
  for (uint32_t i = 0; i < num_fields; ++i) {
    const uint32_t name_len = u32();
    std::string name(reinterpret_cast<const char*>(p), name_len);
    p += name_len;
    const uint32_t offset = u32();
    const uint8_t dt = read<uint8_t>(p);
    p += 1;
    u32();  // count
    fields.push_back({name, offset, dt});
  }
  p += 1;                    // is_bigendian
  const uint32_t point_step = u32();
  u32();                     // row_step
  const uint32_t data_len = u32();
  const std::byte* points = p;

  const double header_stamp = secs + nsecs * 1e-9;
  std::cout << "  header stamp: " << secs << "." << nsecs << " s  (" << width
            << "x" << height << " pts, point_step " << point_step << ")\n";
  std::cout << "  fields:\n";
  const Field* time_field = nullptr;
  for (const Field& f : fields) {
    std::cout << "    " << f.name << "  offset=" << f.offset << "  "
              << datatype_name(f.datatype) << "\n";
    if (f.name == "t" || f.name == "time" || f.name == "timestamp")
      time_field = &f;
  }
  if (!time_field) {
    std::cout << "  (no t/time/timestamp field)\n";
    return;
  }
  const uint32_t n = width * height;
  if (n == 0 || data_len < point_step) return;
  const double t_first =
      field_as_double(points + time_field->offset, time_field->datatype);
  const double t_last = field_as_double(
      points + (n - 1) * point_step + time_field->offset, time_field->datatype);
  std::cout << "  time field '" << time_field->name << "' ("
            << datatype_name(time_field->datatype) << "): first=" << t_first
            << " last=" << t_last << "\n";
  std::cout << "  -> interpreted as offset-from-header, span = "
            << (t_last - t_first) << " (ns if integer)\n";
  std::cout << "  -> interpreted as absolute seconds, first - header = "
            << (t_first - header_stamp) << " s\n";
}

}  // namespace

int main(int argc, char** argv) {
  CLI::App app{"Inspect a ROS bag: topics, types, and lidar point layout"};
  std::string bag_path;
  std::string lidar_topic;
  app.add_option("bag", bag_path, "Path to .bag")->required();
  app.add_option("lidar_topic", lidar_topic,
                 "PointCloud2 topic to dump field layout for");
  CLI11_PARSE(app, argc, argv);

  BagReader reader(bag_path);
  std::cout << "Topics:\n";
  for (const auto& [topic, type] : reader.topics())
    std::cout << "  " << topic << "  [" << type << "]\n";

  if (lidar_topic.empty()) return 0;

  std::cout << "\nFirst message on " << lidar_topic << ":\n";
  bool done = false;
  try {
    reader.read_messages(
        {lidar_topic}, [&](const std::string&, uint64_t,
                           std::span<const std::byte> data) {
          if (done) return;
          // Hex dump of the first bytes, useful for non-PointCloud2 layouts.
          std::cout << "  first " << std::min<size_t>(96, data.size())
                    << " bytes:\n    ";
          for (size_t i = 0; i < std::min<size_t>(96, data.size()); ++i) {
            printf("%02x ", static_cast<unsigned>(data[i]));
            if ((i + 1) % 16 == 0) std::cout << "\n    ";
          }
          std::cout << "\n";
          done = true;
          throw std::runtime_error("__stop__");  // early-out of the bag scan
        });
  } catch (const std::runtime_error& e) {
    if (std::string(e.what()) != "__stop__") throw;
  }
  return 0;
}

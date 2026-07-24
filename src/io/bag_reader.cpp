#include "io/bag_reader.h"

#include <lz4frame.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <ranges>
#include <stdexcept>
#include <unordered_set>

#include "io/span_reader.h"

namespace {

constexpr uint8_t OP_MESSAGE_DATA = 0x02;
constexpr uint8_t OP_CHUNK = 0x05;
constexpr uint8_t OP_CONNECTION = 0x07;

// ── File-level binary helpers
// ─────────────────────────────────────────────────

void read_exact(std::ifstream& f, void* buf, size_t n) {
  f.read(static_cast<char*>(buf), static_cast<std::streamsize>(n));
  if (static_cast<size_t>(f.gcount()) != n) {
    throw std::runtime_error("Unexpected end of bag file");
  }
}

uint32_t read_u32(std::ifstream& f) {
  uint32_t v{};
  read_exact(f, &v, 4);
  return v;
}

std::vector<std::byte> read_vec(std::ifstream& f, size_t n) {
  std::vector<std::byte> buf(n);
  read_exact(f, buf.data(), n);
  return buf;
}

// ── Record header parsing
// ───────────────────────────────────────────────────── A ROS bag record header
// is: [uint32 total_len] followed by repeated [uint32 field_len][field_name '='
// field_value] entries.

using FieldMap = std::unordered_map<std::string, std::vector<std::byte>>;

FieldMap parse_fields(std::span<const std::byte> buf) {
  FieldMap fields;
  size_t pos = 0;
  while (pos + 4 <= buf.size()) {
    uint32_t flen{};
    std::memcpy(&flen, buf.data() + pos, 4);
    pos += 4;
    if (pos + flen > buf.size()) break;

    size_t eq = pos;
    while (eq < pos + flen && buf[eq] != std::byte{'='}) ++eq;

    std::string key(reinterpret_cast<const char*>(buf.data() + pos), eq - pos);
    std::vector<std::byte> val(buf.data() + eq + 1, buf.data() + pos + flen);
    fields.emplace(std::move(key), std::move(val));
    pos += flen;
  }
  return fields;
}

FieldMap read_header(std::ifstream& f) {
  uint32_t len = read_u32(f);
  auto buf = read_vec(f, len);
  return parse_fields(buf);
}

FieldMap read_span_header(SpanReader& r) {
  uint32_t len = r.read<uint32_t>();
  return parse_fields(r.read_bytes(len));
}

template <typename T>
T field_val(const FieldMap& m, const std::string& k) {
  T result{};
  std::memcpy(&result, m.at(k).data(), sizeof(T));
  return result;
}

std::string field_str(const FieldMap& m, const std::string& k) {
  const auto& v = m.at(k);
  return std::string(reinterpret_cast<const char*>(v.data()), v.size());
}

// ── LZ4 decompression
// ─────────────────────────────────────────────────────────

std::vector<std::byte> decompress_lz4(std::span<const std::byte> src,
                                      uint32_t uncompressed_size) {
  std::vector<std::byte> dst(uncompressed_size);

  LZ4F_decompressionContext_t ctx{};
  auto rc = LZ4F_createDecompressionContext(&ctx, LZ4F_VERSION);
  if (LZ4F_isError(rc)) {
    throw std::runtime_error(std::string("LZ4F_createDecompressionContext: ") +
                             LZ4F_getErrorName(rc));
  }

  size_t dst_size = dst.size();
  size_t src_size = src.size();
  rc = LZ4F_decompress(ctx, dst.data(), &dst_size, src.data(), &src_size,
                       nullptr);
  LZ4F_freeDecompressionContext(ctx);

  if (LZ4F_isError(rc)) {
    throw std::runtime_error(std::string("LZ4F_decompress: ") +
                             LZ4F_getErrorName(rc));
  }
  return dst;
}

}  // namespace

// ── BagReader
// ─────────────────────────────────────────────────────────────────

BagReader::BagReader(std::filesystem::path path) : path_(std::move(path)) {
  scan_connections();
}

void BagReader::scan_connections() {
  std::ifstream f(path_, std::ios::binary);
  if (!f) throw std::runtime_error("Cannot open bag: " + path_.string());

  std::string version;
  std::getline(f, version);
  if (version != "#ROSBAG V2.0") {
    throw std::runtime_error("Not a ROS bag V2.0 file: " + path_.string());
  }

  // Read bag header to find where the connection records start.
  auto bag_hdr = read_header(f);
  uint32_t bag_data_len = read_u32(f);
  uint64_t index_pos = field_val<uint64_t>(bag_hdr, "index_pos");
  f.seekg(bag_data_len, std::ios::cur);

  // CONNECTION records live in the index section at the end of the file.
  f.seekg(static_cast<std::streamoff>(index_pos));

  while (f.good() && f.peek() != EOF) {
    auto hdr = read_header(f);
    if (!f) break;
    uint32_t data_len = read_u32(f);

    if (field_val<uint8_t>(hdr, "op") == OP_CONNECTION) {
      uint32_t conn_id = field_val<uint32_t>(hdr, "conn");
      conn_topics_[conn_id] = field_str(hdr, "topic");
      // The connection data is itself a header-format blob carrying the message
      // type (and definition, md5sum, ...). Parse it for the type.
      auto data = read_vec(f, data_len);
      conn_types_[conn_id] = field_str(parse_fields(data), "type");
      continue;
    }

    f.seekg(data_len, std::ios::cur);
  }
}

std::vector<std::pair<std::string, std::string>> BagReader::topics() const {
  std::vector<std::pair<std::string, std::string>> out;
  for (const auto& [id, topic] : conn_topics_) {
    auto it = conn_types_.find(id);
    out.emplace_back(topic, it != conn_types_.end() ? it->second : "");
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

void BagReader::read_messages(const std::vector<std::string>& topics,
                              Callback cb) {
  std::unordered_set<uint32_t> filter;
  for (auto& [id, topic] : conn_topics_) {
    if (std::ranges::find(topics, topic) != topics.end()) {
      filter.insert(id);
    }
  }

  std::ifstream f(path_, std::ios::binary);
  std::string version;
  std::getline(f, version);

  auto bag_hdr = read_header(f);
  uint32_t bag_data_len = read_u32(f);
  uint64_t index_pos = field_val<uint64_t>(bag_hdr, "index_pos");
  f.seekg(bag_data_len, std::ios::cur);

  while (f.good() && static_cast<uint64_t>(f.tellg()) < index_pos) {
    auto hdr = read_header(f);
    if (!f) break;
    uint32_t data_len = read_u32(f);
    uint8_t op = field_val<uint8_t>(hdr, "op");

    if (op == OP_CHUNK) {
      std::string compression = field_str(hdr, "compression");
      uint32_t uncompressed_size = field_val<uint32_t>(hdr, "size");
      auto chunk_data = read_vec(f, data_len);

      std::vector<std::byte> decompressed;
      if (compression == "lz4") {
        decompressed = decompress_lz4(chunk_data, uncompressed_size);
      } else if (compression == "none") {
        decompressed = std::move(chunk_data);
      } else {
        throw std::runtime_error("Unsupported bag compression: " + compression);
      }

      process_chunk(decompressed, filter, cb);
    } else {
      f.seekg(data_len, std::ios::cur);
    }
  }
}

void BagReader::process_chunk(std::span<const std::byte> data,
                              const std::unordered_set<uint32_t>& filter,
                              const Callback& cb) const {
  SpanReader r(data);
  while (!r.empty()) {
    auto hdr = read_span_header(r);
    uint32_t data_len = r.read<uint32_t>();
    uint8_t op = field_val<uint8_t>(hdr, "op");

    if (op == OP_MESSAGE_DATA) {
      uint32_t conn_id = field_val<uint32_t>(hdr, "conn");
      if (filter.contains(conn_id)) {
        const auto& time_bytes = hdr.at("time");
        uint32_t secs{}, nsecs{};
        std::memcpy(&secs, time_bytes.data(), 4);
        std::memcpy(&nsecs, time_bytes.data() + 4, 4);
        uint64_t stamp_ns =
            static_cast<uint64_t>(secs) * 1'000'000'000ULL + nsecs;

        cb(conn_topics_.at(conn_id), stamp_ns, r.read_bytes(data_len));
        continue;
      }
    }

    r.skip(data_len);
  }
}

#pragma once

#include <cstddef>
#include <cstring>
#include <span>
#include <stdexcept>
#include <string>

// Lightweight cursor over a read-only byte span. Used for parsing ROS messages
// and decompressed chunk data where we already have the full buffer in memory.
class SpanReader {
 public:
  explicit SpanReader(std::span<const std::byte> data) : data_(data) {}

  template <typename T>
  T read() {
    if (pos_ + sizeof(T) > data_.size()) {
      throw std::runtime_error("SpanReader: read past end of buffer");
    }
    T v{};
    std::memcpy(&v, data_.data() + pos_, sizeof(T));
    pos_ += sizeof(T);
    return v;
  }

  std::string read_string() {
    uint32_t len = read<uint32_t>();
    if (pos_ + len > data_.size()) {
      throw std::runtime_error("SpanReader: string length past end of buffer");
    }
    std::string s(reinterpret_cast<const char*>(data_.data() + pos_), len);
    pos_ += len;
    return s;
  }

  std::span<const std::byte> read_bytes(size_t n) {
    if (pos_ + n > data_.size()) {
      throw std::runtime_error("SpanReader: read_bytes past end of buffer");
    }
    auto s = data_.subspan(pos_, n);
    pos_ += n;
    return s;
  }

  void skip(size_t n) { read_bytes(n); }

  bool empty() const noexcept { return pos_ >= data_.size(); }
  size_t remaining() const noexcept { return data_.size() - pos_; }

 private:
  std::span<const std::byte> data_;
  size_t pos_{0};
};

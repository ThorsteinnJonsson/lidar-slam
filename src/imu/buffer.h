#pragma once

#include <cstdint>
#include <deque>
#include <vector>

#include "types.h"

// Chronological store of IMU measurements supporting time-range queries.
// get_between() always returns a sequence that starts exactly at t0_ns and
// ends exactly at t1_ns, interpolating at the boundaries when no exact sample
// exists at those timestamps.
class ImuBuffer {
 public:
  void push(const ImuMeasurement& meas);

  // True if the buffer has data covering [t0_ns, t1_ns].
  bool covers(uint64_t t0_ns, uint64_t t1_ns) const;

  // Returns measurements spanning [t0_ns, t1_ns] with interpolated endpoints.
  // Throws if the range is not covered.
  std::vector<ImuMeasurement> get_between(uint64_t t0_ns, uint64_t t1_ns) const;

  bool empty() const { return data_.empty(); }
  size_t size() const { return data_.size(); }

 private:
  std::deque<ImuMeasurement> data_;

  static ImuMeasurement interpolate(const ImuMeasurement& a,
                                    const ImuMeasurement& b, uint64_t t_ns);
};

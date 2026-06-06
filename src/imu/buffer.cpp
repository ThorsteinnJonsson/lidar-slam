#include "imu/buffer.h"

#include <algorithm>
#include <stdexcept>

void ImuBuffer::push(const ImuMeasurement& meas) { data_.push_back(meas); }

bool ImuBuffer::covers(uint64_t t0_ns, uint64_t t1_ns) const {
  if (data_.empty()) return false;
  return data_.front().stamp.to_nsec() <= t0_ns &&
         data_.back().stamp.to_nsec() >= t1_ns;
}

std::vector<ImuMeasurement> ImuBuffer::get_between(uint64_t t0_ns,
                                                   uint64_t t1_ns) const {
  if (!covers(t0_ns, t1_ns)) {
    throw std::runtime_error("ImuBuffer::get_between: range not covered");
  }

  // Find first sample strictly after t0_ns and last sample strictly before
  // t1_ns.
  auto after_t0 = std::lower_bound(data_.begin(), data_.end(), t0_ns,
                                   [](const ImuMeasurement& m, uint64_t t) {
                                     return m.stamp.to_nsec() < t;
                                   });

  auto after_t1 = std::upper_bound(data_.begin(), data_.end(), t1_ns,
                                   [](uint64_t t, const ImuMeasurement& m) {
                                     return t < m.stamp.to_nsec();
                                   });

  std::vector<ImuMeasurement> result;
  result.reserve(std::distance(after_t0, after_t1) + 2);

  // Prepend interpolated sample at exactly t0_ns (if the first sample is after
  // t0).
  if (after_t0 == data_.begin() || after_t0->stamp.to_nsec() == t0_ns) {
    // Exact match at the boundary — no interpolation needed.
    result.push_back(*after_t0);
  } else {
    auto prev = std::prev(after_t0);
    result.push_back(interpolate(*prev, *after_t0, t0_ns));
  }

  // Interior samples strictly between t0_ns and t1_ns.
  for (auto it = after_t0; it != after_t1; ++it) {
    if (it->stamp.to_nsec() > t0_ns && it->stamp.to_nsec() < t1_ns) {
      result.push_back(*it);
    }
  }

  // Append interpolated sample at exactly t1_ns.
  auto at_or_after_t1 = after_t1;
  if (at_or_after_t1 != data_.end() &&
      (at_or_after_t1 == data_.begin() ||
       std::prev(at_or_after_t1)->stamp.to_nsec() < t1_ns)) {
    auto prev = std::prev(at_or_after_t1);
    if (prev->stamp.to_nsec() == t1_ns) {
      result.push_back(*prev);
    } else {
      result.push_back(interpolate(*prev, *at_or_after_t1, t1_ns));
    }
  }

  return result;
}

ImuMeasurement ImuBuffer::interpolate(const ImuMeasurement& a,
                                      const ImuMeasurement& b, uint64_t t_ns) {
  const uint64_t t_a = a.stamp.to_nsec();
  const uint64_t t_b = b.stamp.to_nsec();
  const double alpha = static_cast<double>(t_ns - t_a) / (t_b - t_a);

  ImuMeasurement out;
  out.stamp = Timestamp::from_nsec(t_ns);
  out.angular_velocity =
      a.angular_velocity + alpha * (b.angular_velocity - a.angular_velocity);
  out.linear_acceleration =
      a.linear_acceleration +
      alpha * (b.linear_acceleration - a.linear_acceleration);
  // Orientation is not used by the propagator; skip slerp for simplicity.
  out.orientation = a.orientation.slerp(alpha, b.orientation);
  return out;
}

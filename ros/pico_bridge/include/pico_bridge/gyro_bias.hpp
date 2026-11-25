#pragma once

#include <cstddef>

#include "pico_bridge/vector3.hpp"

namespace pico_bridge {

// The gyro's offset, averaged over a window while the robot sits still.
class GyroBias {
 public:
  enum class Progress { kMeasuring, kMovedSoRestarted, kReady };

  GyroBias(size_t window_size, double max_spread_radps)
      : window_size_(window_size), max_spread_radps_(max_spread_radps) {}

  Progress add(const Vector3& gyro) {
    if (is_ready()) {
      return Progress::kReady;
    }
    if (sample_count_ == 0) {
      min_ = max_ = sum_ = gyro;
    } else {
      min_ = min(min_, gyro);
      max_ = max(max_, gyro);
      sum_ = sum_ + gyro;
    }
    ++sample_count_;
    if (largest_component(max_ - min_) > max_spread_radps_) {
      sample_count_ = 0;
      return Progress::kMovedSoRestarted;
    }
    return is_ready() ? Progress::kReady : Progress::kMeasuring;
  }

  bool is_ready() const { return sample_count_ >= window_size_; }

  // Valid once ready.
  Vector3 bias() const { return sum_ / static_cast<double>(sample_count_); }

  Vector3 remove_from(const Vector3& gyro) const { return gyro - bias(); }

 private:
  size_t window_size_;
  double max_spread_radps_;
  size_t sample_count_ = 0;
  Vector3 min_{};
  Vector3 max_{};
  Vector3 sum_{};
};

}  // namespace pico_bridge

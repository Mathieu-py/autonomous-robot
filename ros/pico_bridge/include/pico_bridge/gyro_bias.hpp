#pragma once

#include <cstddef>

#include "pico_bridge/vector3.hpp"

namespace pico_bridge {

// The gyro's offset, averaged over a window while the robot sits still.
class GyroBias {
 public:
  enum class Progress { kMeasuring, kMovedSoRestarted, kReady };

  GyroBias(size_t sample_count, double max_spread_radps)
      : sample_count_(sample_count), max_spread_radps_(max_spread_radps) {}

  Progress add(const Vector3& gyro) {
    if (is_ready()) {
      return Progress::kReady;
    }
    if (samples_ == 0) {
      min_ = max_ = sum_ = gyro;
    } else {
      min_ = min(min_, gyro);
      max_ = max(max_, gyro);
      sum_ = sum_ + gyro;
    }
    ++samples_;
    if (largest_axis(max_ - min_) > max_spread_radps_) {
      samples_ = 0;
      return Progress::kMovedSoRestarted;
    }
    if (samples_ < sample_count_) {
      return Progress::kMeasuring;
    }
    bias_ = sum_ / static_cast<double>(samples_);
    return Progress::kReady;
  }

  bool is_ready() const { return samples_ >= sample_count_; }

  // Valid once ready.
  const Vector3& bias() const { return bias_; }

  Vector3 remove_from(const Vector3& gyro) const { return gyro - bias_; }

 private:
  size_t sample_count_;
  double max_spread_radps_;
  size_t samples_ = 0;
  Vector3 min_{};
  Vector3 max_{};
  Vector3 sum_{};
  Vector3 bias_{};
};

}  // namespace pico_bridge

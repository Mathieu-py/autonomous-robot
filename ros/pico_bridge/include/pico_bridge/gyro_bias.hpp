#pragma once

#include <algorithm>
#include <cstddef>

#include "pico_bridge/drive_protocol.hpp"

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
      min_ = {std::min(min_.x, gyro.x), std::min(min_.y, gyro.y),
              std::min(min_.z, gyro.z)};
      max_ = {std::max(max_.x, gyro.x), std::max(max_.y, gyro.y),
              std::max(max_.z, gyro.z)};
      sum_ = {sum_.x + gyro.x, sum_.y + gyro.y, sum_.z + gyro.z};
    }
    ++samples_;
    if (largest_spread() > max_spread_radps_) {
      samples_ = 0;
      return Progress::kMovedSoRestarted;
    }
    if (samples_ < sample_count_) {
      return Progress::kMeasuring;
    }
    const double n = static_cast<double>(samples_);
    bias_ = {sum_.x / n, sum_.y / n, sum_.z / n};
    return Progress::kReady;
  }

  bool is_ready() const { return samples_ >= sample_count_; }

  // Valid once ready.
  const Vector3& bias() const { return bias_; }

  Vector3 remove_from(const Vector3& gyro) const {
    return {gyro.x - bias_.x, gyro.y - bias_.y, gyro.z - bias_.z};
  }

 private:
  double largest_spread() const {
    return std::max({max_.x - min_.x, max_.y - min_.y, max_.z - min_.z});
  }

  size_t sample_count_;
  double max_spread_radps_;
  size_t samples_ = 0;
  Vector3 min_{};
  Vector3 max_{};
  Vector3 sum_{};
  Vector3 bias_{};
};

}  // namespace pico_bridge

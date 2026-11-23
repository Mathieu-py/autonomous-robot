#include "imu_reading.h"

#include <numbers>

namespace {

constexpr float kStandardGravity = 9.80665f;
constexpr float kCountsPerG = 8192.0f;         // ±4 g range
constexpr float kCountsPerDegreePerS = 65.5f;  // ±500 °/s range

// Registers are big-endian 16-bit two's complement.
float signed_counts_at(const std::array<uint8_t, 14>& bytes, int offset) {
  return static_cast<int16_t>((bytes[offset] << 8) | bytes[offset + 1]);
}

Vector3 vector_at(const std::array<uint8_t, 14>& bytes, int offset,
                  float units_per_count) {
  return {signed_counts_at(bytes, offset) * units_per_count,
          signed_counts_at(bytes, offset + 2) * units_per_count,
          signed_counts_at(bytes, offset + 4) * units_per_count};
}

}  // namespace

ImuReading imu_reading_from_bytes(const std::array<uint8_t, 14>& bytes) {
  constexpr float kMps2PerCount = kStandardGravity / kCountsPerG;
  constexpr float kRadpsPerCount =
      std::numbers::pi_v<float> / 180.0f / kCountsPerDegreePerS;
  // Bytes 6-7 are the temperature, unused.
  return {.accel_mps2 = vector_at(bytes, 0, kMps2PerCount),
          .gyro_radps = vector_at(bytes, 8, kRadpsPerCount)};
}

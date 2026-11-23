#include "imu_reading.h"

#include <array>
#include <numbers>

#include "check.h"

namespace {

constexpr float kOneDegreeInRadians = std::numbers::pi_v<float> / 180.0f;

std::array<uint8_t, 14> bytes_with(int offset, uint8_t high, uint8_t low) {
  std::array<uint8_t, 14> bytes{};
  bytes[offset] = high;
  bytes[offset + 1] = low;
  return bytes;
}

void test_one_g_up_reads_standard_gravity() {
  const auto reading = imu_reading_from_bytes(bytes_with(4, 0x20, 0x00));
  CHECK_NEAR(reading.accel_mps2.z, 9.80665, 1e-4);
  CHECK_NEAR(reading.accel_mps2.x, 0.0, 1e-6);
}

void test_negative_counts_read_negative() {
  const auto reading = imu_reading_from_bytes(bytes_with(0, 0xE0, 0x00));
  CHECK_NEAR(reading.accel_mps2.x, -9.80665, 1e-4);
}

void test_gyro_reads_in_radians_per_second() {
  // 6550 counts = 100 °/s at ±500 °/s.
  const auto reading = imu_reading_from_bytes(bytes_with(12, 0x19, 0x96));
  CHECK_NEAR(reading.gyro_radps.z, 100.0 * kOneDegreeInRadians, 1e-4);
}

void test_temperature_bytes_are_ignored() {
  const auto reading = imu_reading_from_bytes(bytes_with(6, 0x7F, 0xFF));
  CHECK_NEAR(reading.accel_mps2.z, 0.0, 1e-6);
  CHECK_NEAR(reading.gyro_radps.x, 0.0, 1e-6);
}

}  // namespace

int main() {
  test_one_g_up_reads_standard_gravity();
  test_negative_counts_read_negative();
  test_gyro_reads_in_radians_per_second();
  test_temperature_bytes_are_ignored();
  return report("imu_reading");
}

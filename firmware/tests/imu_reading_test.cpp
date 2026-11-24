#include "devices/imu_reading.h"

#include <numbers>

#include "check.h"

static_assert(kAccelRangeSetting == 1 && kGyroRangeSetting == 1,
              "these counts are for ±4 g and ±500 °/s");

namespace {

constexpr float kOneDegreeInRadians = std::numbers::pi_v<float> / 180.0f;

ImuBytes bytes_with(int offset, uint8_t high, uint8_t low) {
  ImuBytes bytes{};
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

void test_each_axis_reads_its_own_bytes() {
  // Accel 1, 2 and -1 g; temperature; gyro 100, 200 and -100 °/s.
  const ImuBytes bytes = {0x20, 0x00, 0x40, 0x00, 0xE0, 0x00, 0x7F,
                          0xFF, 0x19, 0x96, 0x33, 0x2C, 0xE6, 0x6A};
  const auto reading = imu_reading_from_bytes(bytes);
  CHECK_NEAR(reading.accel_mps2.x, 9.80665, 1e-4);
  CHECK_NEAR(reading.accel_mps2.y, 2 * 9.80665, 1e-4);
  CHECK_NEAR(reading.accel_mps2.z, -9.80665, 1e-4);
  CHECK_NEAR(reading.gyro_radps.x, 100.0 * kOneDegreeInRadians, 1e-4);
  CHECK_NEAR(reading.gyro_radps.y, 200.0 * kOneDegreeInRadians, 1e-4);
  CHECK_NEAR(reading.gyro_radps.z, -100.0 * kOneDegreeInRadians, 1e-4);
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
  test_each_axis_reads_its_own_bytes();
  test_temperature_bytes_are_ignored();
  return report("imu_reading");
}

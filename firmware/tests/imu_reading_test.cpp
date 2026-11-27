#include "devices/imu_reading.h"

#include <gtest/gtest.h>

#include <numbers>

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

TEST(ImuReadingFromBytes, EachAxisReadsItsOwnBytes) {
  // Accel 1, 2 and -1 g; temperature; gyro 100, 200 and -100 °/s.
  const ImuBytes bytes = {0x20, 0x00, 0x40, 0x00, 0xE0, 0x00, 0x7F,
                          0xFF, 0x19, 0x96, 0x33, 0x2C, 0xE6, 0x6A};
  const auto reading = imu_reading_from_bytes(bytes);
  EXPECT_NEAR(reading.accel_mps2.x, 9.80665, 1e-4);
  EXPECT_NEAR(reading.accel_mps2.y, 2 * 9.80665, 1e-4);
  EXPECT_NEAR(reading.accel_mps2.z, -9.80665, 1e-4);
  EXPECT_NEAR(reading.gyro_radps.x, 100.0 * kOneDegreeInRadians, 1e-4);
  EXPECT_NEAR(reading.gyro_radps.y, 200.0 * kOneDegreeInRadians, 1e-4);
  EXPECT_NEAR(reading.gyro_radps.z, -100.0 * kOneDegreeInRadians, 1e-4);
}

TEST(ImuReadingFromBytes, FullScaleReadsFourG) {
  const auto low = imu_reading_from_bytes(bytes_with(0, 0x80, 0x00));
  EXPECT_NEAR(low.accel_mps2.x, -4 * 9.80665, 1e-4);
  const auto high = imu_reading_from_bytes(bytes_with(0, 0x7F, 0xFF));
  EXPECT_NEAR(high.accel_mps2.x, 32767.0 / 8192.0 * 9.80665, 1e-4);
}

TEST(ImuReadingFromBytes, TemperatureBytesAreIgnored) {
  const auto reading = imu_reading_from_bytes(bytes_with(6, 0x7F, 0xFF));
  EXPECT_NEAR(reading.accel_mps2.z, 0.0, 1e-6);
  EXPECT_NEAR(reading.gyro_radps.x, 0.0, 1e-6);
}

}  // namespace

#pragma once

#include <array>
#include <cstdint>

// The MPU-6050's 0-3 range settings; each step doubles the range.
// Imu::start() writes them and imu_reading_from_bytes() scales by them.
constexpr uint8_t kAccelRangeSetting = 1;  // ±4 g
constexpr uint8_t kGyroRangeSetting = 1;   // ±500 °/s

// The 14 bytes from ACCEL_XOUT_H: accel, temperature, gyro.
using ImuBytes = std::array<uint8_t, 14>;

struct Vector3 {
  float x;
  float y;
  float z;
};

struct ImuReading {
  Vector3 accel_mps2;
  Vector3 gyro_radps;
};

ImuReading imu_reading_from_bytes(const ImuBytes& bytes);

#pragma once

#include <array>
#include <cstdint>

struct Vector3 {
  float x;
  float y;
  float z;
};

struct ImuReading {
  Vector3 accel_mps2;
  Vector3 gyro_radps;
};

// The MPU-6050's 14 bytes from ACCEL_XOUT_H, at ±4 g and ±500 °/s.
ImuReading imu_reading_from_bytes(const std::array<uint8_t, 14>& bytes);

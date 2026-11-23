#include "imu_test.h"

#include <cstdio>
#include <numbers>

#include "pico/stdio_usb.h"
#include "pico/time.h"

namespace {

constexpr uint32_t kStartDelayMs = 3000;
constexpr uint32_t kPrintIntervalMs = 100;
constexpr uint8_t kMpu6050Id = 0x68;
constexpr float kDegreesPerRadian = 180.0f / std::numbers::pi_v<float>;

void wait_for_serial_then_count_down() {
  while (!stdio_usb_connected()) {
    sleep_ms(10);
  }
  printf("Serial connected: IMU test starts in 3 s\n");
  sleep_ms(kStartDelayMs);
}

void print_chip_id(const Imu& imu) {
  const auto id = imu.chip_id();
  if (!id) {
    printf("Couldn't read the IMU's chip ID\n");
  } else if (*id == kMpu6050Id) {
    printf("IMU chip ID: 0x%02X (MPU-6050)\n", *id);
  } else {
    printf("IMU chip ID: 0x%02X, expected 0x%02X\n", *id, kMpu6050Id);
  }
}

void print_reading(const ImuReading& reading) {
  const Vector3& a = reading.accel_mps2;
  const Vector3& g = reading.gyro_radps;
  printf("accel %7.2f %7.2f %7.2f   gyro %8.2f %8.2f %8.2f\n", a.x, a.y, a.z,
         g.x * kDegreesPerRadian, g.y * kDegreesPerRadian,
         g.z * kDegreesPerRadian);
}

}  // namespace

void run_imu_test(const Imu& imu) {
  wait_for_serial_then_count_down();
  print_chip_id(imu);
  printf("accel X Y Z in m/s^2, gyro X Y Z in deg/s\n");
  while (true) {
    if (const auto reading = imu.read()) {
      print_reading(*reading);
    } else {
      printf("IMU read failed\n");
    }
    sleep_ms(kPrintIntervalMs);
  }
}

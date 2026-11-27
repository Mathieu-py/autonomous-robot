#include "devices/imu.h"

#include "hardware/gpio.h"
#include "pico/time.h"

namespace {

constexpr uint8_t kAddress = 0x68;
constexpr uint kI2cClockHz = 400'000;
constexpr uint kTimeoutUs = 2000;  // two per read still fit the 20 ms period

constexpr uint8_t kSampleRateDivider = 0x19;
constexpr uint8_t kConfig = 0x1A;
constexpr uint8_t kGyroConfig = 0x1B;
constexpr uint8_t kAccelConfig = 0x1C;
constexpr uint8_t kAccelXoutH = 0x3B;
constexpr uint8_t kPowerManagement = 0x6B;
constexpr uint8_t kWhoAmI = 0x75;

// Both config registers hold the range setting in bits 3-4.
constexpr uint8_t range_bits(uint8_t setting) { return setting << 3; }

constexpr int kBusReleaseClocks = 9;  // enough to finish any byte
constexpr uint kBusReleaseHalfClockUs = 5;

// A reset mid-read can leave the IMU holding SDA low; clocking frees it.
void release_stuck_bus(uint sda_pin, uint scl_pin) {
  gpio_init(sda_pin);
  gpio_init(scl_pin);  // output level 0: switching to output pulls SCL low
  for (int i = 0; i < kBusReleaseClocks && !gpio_get(sda_pin); ++i) {
    gpio_set_dir(scl_pin, GPIO_OUT);
    sleep_us(kBusReleaseHalfClockUs);
    gpio_set_dir(scl_pin, GPIO_IN);
    sleep_us(kBusReleaseHalfClockUs);
  }
}

}  // namespace

const char* Imu::start(i2c_inst_t* i2c, uint sda_pin, uint scl_pin) {
  i2c_ = i2c;
  release_stuck_bus(sda_pin, scl_pin);
  i2c_init(i2c_, kI2cClockHz);
  gpio_set_function(sda_pin, GPIO_FUNC_I2C);
  gpio_set_function(scl_pin, GPIO_FUNC_I2C);

  // Wake, clocked from the X gyro, which is steadier than the internal one.
  if (!write_register(kPowerManagement, 0x01)) {
    return "the IMU didn't answer on I2C: check SDA, SCL, VIN and GND";
  }
  sleep_ms(10);
  const bool is_configured =
      // ~10 Hz filter, under half the 50 Hz read rate: little aliasing.
      write_register(kConfig, 0x05) &&
      write_register(kSampleRateDivider, 9) &&  // 100 Hz
      write_register(kGyroConfig, range_bits(kGyroRangeSetting)) &&
      write_register(kAccelConfig, range_bits(kAccelRangeSetting));
  return is_configured ? nullptr : "the IMU stopped answering during setup";
}

std::optional<uint8_t> Imu::chip_id() const {
  uint8_t id;
  if (!read_registers(kWhoAmI, &id, 1)) {
    return std::nullopt;
  }
  return id;
}

std::optional<ImuReading> Imu::read() const {
  ImuBytes bytes;
  if (!read_registers(kAccelXoutH, bytes.data(), bytes.size())) {
    return std::nullopt;
  }
  return imu_reading_from_bytes(bytes);
}

bool Imu::write_register(uint8_t reg, uint8_t value) const {
  const uint8_t message[2] = {reg, value};
  return i2c_write_timeout_us(i2c_, kAddress, message, 2, false, kTimeoutUs) ==
         2;
}

bool Imu::read_registers(uint8_t first_reg, uint8_t* out, size_t count) const {
  if (i2c_write_timeout_us(i2c_, kAddress, &first_reg, 1, true, kTimeoutUs) !=
      1) {
    return false;
  }
  return i2c_read_timeout_us(i2c_, kAddress, out, count, false, kTimeoutUs) ==
         static_cast<int>(count);
}

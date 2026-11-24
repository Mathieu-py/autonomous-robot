#pragma once

#include <cstdint>
#include <optional>

#include "devices/imu_reading.h"
#include "hardware/i2c.h"

// The MPU-6050 on I2C.
class Imu {
 public:
  // What chip_id() reads on a genuine MPU-6050.
  static constexpr uint8_t kChipId = 0x68;

  // Returns nullptr or the error.
  const char* start(i2c_inst_t* i2c, uint sda_pin, uint scl_pin);

  std::optional<uint8_t> chip_id() const;  // WHO_AM_I

  std::optional<ImuReading> read() const;

 private:
  bool write_register(uint8_t reg, uint8_t value) const;
  bool read_registers(uint8_t first_reg, uint8_t* out, size_t count) const;

  i2c_inst_t* i2c_ = nullptr;
};

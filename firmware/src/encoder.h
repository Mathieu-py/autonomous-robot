#pragma once

#include <cstdint>

#include "hardware/pio.h"

// A quadrature encoder counted in hardware by a PIO state machine.
class Encoder {
 public:
  int32_t count() const;

  // After cyw43_arch_init(); B pin = A + 1. Returns nullptr or the error.
  static const char* start_pair(uint first_pin_a, uint second_pin_a,
                                Encoder& first, Encoder& second);

 private:
  PIO pio_ = nullptr;
  uint state_machine_ = 0;
};

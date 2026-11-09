#pragma once

#include <cstdint>

#include "hardware/pio.h"

// A quadrature encoder counted in hardware by a PIO state machine.
class Encoder {
 public:
  int32_t count() const;

  // Starts both encoders on one PIO block (B pin = A + 1), after
  // cyw43_arch_init(). Returns nullptr on success, or what went wrong.
  static const char* start_pair(uint first_pin_a, uint second_pin_a,
                                Encoder& first, Encoder& second);

 private:
  PIO pio_ = nullptr;
  uint state_machine_ = 0;
};

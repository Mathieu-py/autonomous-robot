// Encoder bring-up: counts both wheel encoders with PIO and prints the
// counts over USB serial every 200 ms. The LED blinks to show it's running.
//
// Wiring (encoders powered from 3V3(OUT) for this test):
//   First motor:  C1 -> GP2, C2 -> GP3
//   Second motor: C1 -> GP6, C2 -> GP7
// GP4/GP5 are left free for the MPU-6050's I2C.

#include <cstdio>

#include "hardware/pio.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "quadrature_encoder.pio.h"

namespace {

// Each encoder uses its A pin and the pin after it (A + 1) for B.
constexpr uint kFirstEncoderPinA = 2;
constexpr uint kSecondEncoderPinA = 6;

// 0 runs the state machines at full speed. At 178 RPM and about 2473
// ticks per turn, a wheel produces about 7,300 ticks per second, far
// below what PIO can count.
constexpr int kMaxStepRate = 0;

constexpr uint32_t kPrintIntervalMs = 200;
constexpr uint32_t kLedIntervalMs = 500;

[[noreturn]] void halt(const char* reason) {
  while (true) {
    printf("ERROR: %s\n", reason);
    sleep_ms(1000);
  }
}

}  // namespace

int main() {
  stdio_init_all();

  // Start the wireless chip first. It drives the LED and claims a PIO state
  // machine of its own, so the encoders must pick from what's left.
  if (cyw43_arch_init() != 0) {
    halt("failed to start the wireless chip, which drives the LED");
  }

  PIO encoder_pio;
  uint first_sm;
  uint program_offset;
  if (!pio_claim_free_sm_and_add_program_for_gpio_range(
          &quadrature_encoder_program, &encoder_pio, &first_sm,
          &program_offset, kFirstEncoderPinA, 2, true)) {
    halt("no free PIO state machine for the first encoder");
  }

  const int second_sm = pio_claim_unused_sm(encoder_pio, false);
  if (second_sm < 0) {
    halt("no free PIO state machine for the second encoder");
  }

  quadrature_encoder_program_init(encoder_pio, first_sm, kFirstEncoderPinA,
                                  kMaxStepRate);
  quadrature_encoder_program_init(encoder_pio, second_sm, kSecondEncoderPinA,
                                  kMaxStepRate);

  printf("Encoders started on PIO %u, state machines %u and %d\n",
         pio_get_index(encoder_pio), first_sm, second_sm);

  bool led_on = false;
  absolute_time_t next_print = get_absolute_time();
  absolute_time_t next_led = get_absolute_time();

  while (true) {
    if (time_reached(next_print)) {
      next_print = delayed_by_ms(next_print, kPrintIntervalMs);
      const int32_t first =
          quadrature_encoder_get_count(encoder_pio, first_sm);
      const int32_t second =
          quadrature_encoder_get_count(encoder_pio, second_sm);
      printf("first: %8ld  second: %8ld\n", static_cast<long>(first),
             static_cast<long>(second));
    }

    if (time_reached(next_led)) {
      next_led = delayed_by_ms(next_led, kLedIntervalMs);
      led_on = !led_on;
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
    }
  }
}

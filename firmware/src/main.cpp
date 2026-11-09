// Pins: encoders on GP2/GP3 and GP6/GP7, MPU-6050 I2C on GP4/GP5,
// first motor's MDD3A inputs on GP10/GP11.

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "quadrature_encoder.pio.h"

namespace {

// Each encoder's B pin is A + 1.
constexpr uint kFirstEncoderPinA = 2;
constexpr uint kSecondEncoderPinA = 6;

constexpr int kMaxStepRate = 0;  // 0 = sample at full speed

constexpr uint32_t kPrintIntervalMs = 200;
constexpr uint32_t kLedIntervalMs = 500;

constexpr uint kFirstMotorPinA = 10;

// 7500 steps at 150 MHz = 20 kHz PWM.
constexpr uint16_t kPwmWrap = 7499;

// 12 V motors on a 12.6 V battery: 12 / 12.6.
constexpr float kMaxSpeed = 0.95f;

// Motor test: runs once after power-up, then stays stopped.
constexpr float kTestSpeed = 0.3f;
constexpr uint32_t kStartDelayMs = 3000;
constexpr uint32_t kRunMs = 2000;
constexpr uint32_t kPauseMs = 1000;

enum class TestStep { kWaiting, kForward, kPause, kBackward, kDone };

[[noreturn]] void halt(const char* reason) {
  while (true) {
    printf("ERROR: %s\n", reason);
    sleep_ms(1000);
  }
}

// pin_a must be even, so pin_a and pin_a + 1 share a PWM slice.
uint setup_motor(uint pin_a) {
  gpio_set_function(pin_a, GPIO_FUNC_PWM);
  gpio_set_function(pin_a + 1, GPIO_FUNC_PWM);

  const uint slice = pwm_gpio_to_slice_num(pin_a);

  pwm_set_clkdiv(slice, 1.0f);
  pwm_set_wrap(slice, kPwmWrap);

  pwm_set_chan_level(slice, PWM_CHAN_A, 0);
  pwm_set_chan_level(slice, PWM_CHAN_B, 0);

  pwm_set_enabled(slice, true);

  return slice;
}

// speed: -1.0 (full backward) to +1.0 (full forward).
void set_motor_speed(uint slice, float speed) {
  if (std::isnan(speed)) {
    speed = 0.0f;
  }

  speed = std::clamp(speed, -kMaxSpeed, kMaxSpeed);

  const auto level =
      static_cast<uint16_t>(std::fabs(speed) * (kPwmWrap + 1) + 0.5f);

  // Set both at once so A and B are never on together. Zero brakes.
  if (speed > 0.0f) {
    pwm_set_both_levels(slice, level, 0);
  } else {
    pwm_set_both_levels(slice, 0, level);
  }
}

}  // namespace

int main() {
  stdio_init_all();

  const uint first_motor = setup_motor(kFirstMotorPinA);

  // Before the encoders: the wireless chip (LED) also claims a PIO state machine.
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

  TestStep step = TestStep::kWaiting;
  absolute_time_t next_step = at_the_end_of_time;
  bool countdown_started = false;

  while (true) {
    // Test only: start the countdown once a serial viewer is connected.
    if (!countdown_started && stdio_usb_connected()) {
      countdown_started = true;
      next_step = make_timeout_time_ms(kStartDelayMs);
      printf("Serial connected: motor test starts in 3 s\n");
    }

    if (step != TestStep::kDone && time_reached(next_step)) {
      switch (step) {
        case TestStep::kWaiting:
          set_motor_speed(first_motor, kTestSpeed);
          step = TestStep::kForward;
          next_step = make_timeout_time_ms(kRunMs);
          printf("Forward\n");
          break;
        case TestStep::kForward:
          set_motor_speed(first_motor, 0.0f);
          step = TestStep::kPause;
          next_step = make_timeout_time_ms(kPauseMs);
          printf("Stop\n");
          break;
        case TestStep::kPause:
          set_motor_speed(first_motor, -kTestSpeed);
          step = TestStep::kBackward;
          next_step = make_timeout_time_ms(kRunMs);
          printf("Backward\n");
          break;
        case TestStep::kBackward:
        case TestStep::kDone:
          set_motor_speed(first_motor, 0.0f);
          step = TestStep::kDone;
          printf("Done\n");
          break;
      }
    }

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

// Pins: encoders on GP2/GP3 and GP6/GP7, MPU-6050 I2C on GP4/GP5,
// MDD3A inputs: first motor on GP10/GP11, second on GP12/GP13.

#include <cstdio>

#include "encoder.h"
#include "motor.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "speed_controller.h"
#include "test_sequences.h"
#include "wheel_speed.h"

namespace {

constexpr uint kFirstEncoderPinA = 2;
constexpr uint kSecondEncoderPinA = 6;
constexpr uint kFirstMotorPinA = 10;
constexpr uint kSecondMotorPinA = 12;

constexpr float kTicksPerRev = 2473.0f;
constexpr uint32_t kControlPeriodMs = 20;

// From results/sysid/2026-11-09-run01 (see results/README.md).
constexpr MotorGains kFirstMotorGains = {
    .k_s = 0.012f, .k_v = 0.345f, .k_a = 0.0174f, .k_p = 0.277f, .k_i = 5.48f};
constexpr MotorGains kSecondMotorGains = {
    .k_s = 0.010f, .k_v = 0.349f, .k_a = 0.0171f, .k_p = 0.272f, .k_i = 5.54f};

constexpr float kMaxAccel = 5.0f;  // rev/s², so 0 to 1.0 rev/s in 0.2 s

enum class TestMode { kSpeed, kSysId };
constexpr TestMode kTestMode = TestMode::kSpeed;
constexpr float kTestTargetRps = 1.0f;
constexpr uint32_t kTestStartDelayMs = 3000;

constexpr uint32_t kStatusPrintIntervalMs = 200;
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

  // First, so the motor pins are held low (stopped) during startup.
  const Motor first_motor(kFirstMotorPinA);
  const Motor second_motor(kSecondMotorPinA);

  // Before the encoders: the wireless chip (LED) also claims a PIO state
  // machine.
  if (cyw43_arch_init() != 0) {
    halt("failed to start the wireless chip, which drives the LED");
  }

  Encoder first_encoder;
  Encoder second_encoder;
  if (const char* error =
          Encoder::start_pair(kFirstEncoderPinA, kSecondEncoderPinA,
                              first_encoder, second_encoder)) {
    halt(error);
  }

  const absolute_time_t boot_time = get_absolute_time();
  WheelSpeed first_wheel(kTicksPerRev, first_encoder.count(), boot_time);
  WheelSpeed second_wheel(kTicksPerRev, second_encoder.count(), boot_time);

  SetpointRamp ramp(kMaxAccel);
  SpeedController first_controller(kFirstMotorGains, Motor::kMaxPower);
  SpeedController second_controller(kSecondMotorGains, Motor::kMaxPower);

  SpeedTest speed_test(kTestTargetRps);
  SysIdTest sys_id_test;
  absolute_time_t test_start_time = at_the_end_of_time;  // set on connect
  bool is_test_started = false;
  bool is_test_finished = false;

  bool is_led_on = false;
  absolute_time_t next_status_print = boot_time;
  absolute_time_t next_led_toggle = boot_time;
  absolute_time_t next_control = boot_time;

  while (true) {
    // Test only: start the countdown once a serial viewer is connected.
    if (is_at_the_end_of_time(test_start_time) && stdio_usb_connected()) {
      test_start_time = make_timeout_time_ms(kTestStartDelayMs);
      printf(
          "Serial connected: %s starts in 3 s\n",
          kTestMode == TestMode::kSysId ? "identification test" : "speed test");
    }

    if (!is_test_started && time_reached(test_start_time)) {
      is_test_started = true;
      const absolute_time_t now = get_absolute_time();
      if (kTestMode == TestMode::kSysId) {
        sys_id_test.start(now);
        printf("t_ms,phase,power,first_speed,second_speed\n");
      } else {
        speed_test.start(now);
        printf(
            "t_ms,setpoint,first_speed,first_power,second_speed,"
            "second_power\n");
      }
    }

    if (time_reached(next_control)) {
      next_control = delayed_by_ms(next_control, kControlPeriodMs);
      if (time_reached(next_control)) {
        // Fell behind: skip ahead, since back-to-back updates with tiny
        // gaps would make the speed readings jumpy.
        next_control = make_timeout_time_ms(kControlPeriodMs);
      }

      const absolute_time_t now = get_absolute_time();
      first_wheel.update(first_encoder.count(), now);
      second_wheel.update(second_encoder.count(), now);
      const float period_seconds = first_wheel.last_period_seconds();
      const int64_t test_time_ms =
          absolute_time_diff_us(test_start_time, now) / 1000;

      bool is_test_running = false;
      if (kTestMode == TestMode::kSysId) {
        is_test_running = sys_id_test.is_running(now);
        const float power = sys_id_test.power(now);
        first_motor.set_power(power);
        second_motor.set_power(power);
        if (is_test_running) {
          printf("%lld,%u,%.4f,%.3f,%.3f\n", test_time_ms,
                 static_cast<unsigned>(sys_id_test.phase(now)), power,
                 first_wheel.speed_rps(), second_wheel.speed_rps());
        }
      } else {
        is_test_running = speed_test.is_running(now);
        const RampedSetpoint setpoint =
            ramp.advance(speed_test.target_rps(now), period_seconds);
        const float first_power = first_controller.power_for_next_period(
            setpoint, first_wheel.speed_rps(), period_seconds);
        const float second_power = second_controller.power_for_next_period(
            setpoint, second_wheel.speed_rps(), period_seconds);
        first_motor.set_power(first_power);
        second_motor.set_power(second_power);
        if (is_test_running) {
          printf("%lld,%.3f,%.3f,%.3f,%.3f,%.3f\n", test_time_ms,
                 setpoint.at_last_reading, first_wheel.speed_rps(), first_power,
                 second_wheel.speed_rps(), second_power);
        }
      }

      if (is_test_started && !is_test_running && !is_test_finished) {
        is_test_finished = true;
        printf("Done\n");
      }
    }

    if (time_reached(next_status_print)) {
      next_status_print =
          delayed_by_ms(next_status_print, kStatusPrintIntervalMs);
      if (!is_test_started || is_test_finished) {
        printf("first: %8ld (%5.2f rev/s)  second: %8ld (%5.2f rev/s)\n",
               static_cast<long>(first_wheel.count()), first_wheel.speed_rps(),
               static_cast<long>(second_wheel.count()),
               second_wheel.speed_rps());
      }
    }

    if (time_reached(next_led_toggle)) {
      next_led_toggle = delayed_by_ms(next_led_toggle, kLedIntervalMs);
      is_led_on = !is_led_on;
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, is_led_on);
    }
  }
}

// Pins: encoders on GP2/GP3 and GP6/GP7, MPU-6050 I2C on GP4/GP5,
// MDD3A inputs: first motor on GP10/GP11, second on GP12/GP13,
// bumpers on GP14/GP15, battery divider on GP28 (ADC2).

#include <cstdio>

#include "encoder.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "motor.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "safety.h"
#include "speed_controller.h"
#include "test_sequences.h"
#include "wheel_speed.h"

namespace {

constexpr uint kFirstEncoderPinA = 2;
constexpr uint kSecondEncoderPinA = 6;
constexpr uint kFirstMotorPinA = 10;
constexpr uint kSecondMotorPinA = 12;
constexpr uint kFirstBumperPin = 14;
constexpr uint kSecondBumperPin = 15;
constexpr uint kBatteryPin = 28;
constexpr uint kBatteryAdcInput = 2;  // GP28

// 33k/10k divider; 12-bit ADC on a 3.3 V reference.
constexpr float kBatteryVoltsPerCount =
    3.3f / 4095.0f * (33.0f + 10.0f) / 10.0f;

constexpr float kTicksPerRev = 2473.0f;
constexpr uint32_t kControlPeriodMs = 20;
constexpr uint32_t kWatchdogTimeoutMs = 100;

// From results/sysid/2026-11-09-run01 (see results/README.md).
constexpr MotorGains kFirstMotorGains = {
    .k_s = 0.012f, .k_v = 0.345f, .k_a = 0.0174f, .k_p = 0.277f, .k_i = 5.48f};
constexpr MotorGains kSecondMotorGains = {
    .k_s = 0.010f, .k_v = 0.349f, .k_a = 0.0171f, .k_p = 0.272f, .k_i = 5.54f};

constexpr float kMaxAccel = 5.0f;  // rev/s², so 0 to 1.0 rev/s in 0.2 s

enum class TestMode { kSpeed, kSafety, kSysId };
constexpr TestMode kTestMode = TestMode::kSafety;
constexpr float kTestTargetRps = 1.0f;
constexpr int64_t kSafetyTestMs = 60000;
constexpr uint32_t kTestStartDelayMs = 3000;

constexpr uint32_t kStatusPrintIntervalMs = 200;
constexpr uint32_t kLedIntervalMs = 500;

[[noreturn]] void halt(const char* reason) {
  while (true) {
    printf("ERROR: %s\n", reason);
    sleep_ms(1000);
  }
}

const char* test_name(TestMode mode) {
  switch (mode) {
    case TestMode::kSpeed:
      return "speed test";
    case TestMode::kSafety:
      return "safety test";
    case TestMode::kSysId:
      return "identification test";
  }
  return "test";
}

void setup_bumper(uint pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_IN);
  gpio_pull_up(pin);
}

// Switches connect to ground, so a pressed switch reads low.
bool is_bumper_pressed() {
  return !gpio_get(kFirstBumperPin) || !gpio_get(kSecondBumperPin);
}

float read_battery_volts() {
  return static_cast<float>(adc_read()) * kBatteryVoltsPerCount;
}

}  // namespace

int main() {
  stdio_init_all();

  // First, so the motor pins are held low (stopped) during startup.
  const Motor first_motor(kFirstMotorPinA);
  const Motor second_motor(kSecondMotorPinA);

  setup_bumper(kFirstBumperPin);
  setup_bumper(kSecondBumperPin);
  adc_init();
  adc_gpio_init(kBatteryPin);
  adc_select_input(kBatteryAdcInput);

  // Before the encoders: the wireless chip (LED) also claims a PIO state
  // machine.
  if (cyw43_arch_init() != 0) {
    halt("failed to start the wireless chip, which drives the LED");
  }
  // The first LED write takes about 0.5 s (the wireless chip finishes starting
  // up), so do it here rather than in the watchdog-guarded loop.
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);

  Encoder first_encoder;
  Encoder second_encoder;
  if (const char* error =
          Encoder::start_pair(kFirstEncoderPinA, kSecondEncoderPinA,
                              first_encoder, second_encoder)) {
    halt(error);
  }

  // Last, once startup is done: a hang of more than 100 ms reboots the Pico,
  // which comes back with the motors stopped.
  const bool was_rebooted_by_watchdog = watchdog_caused_reboot();
  watchdog_enable(kWatchdogTimeoutMs, true);

  const absolute_time_t boot_time = get_absolute_time();
  WheelSpeed first_wheel(kTicksPerRev, first_encoder.count(), boot_time);
  WheelSpeed second_wheel(kTicksPerRev, second_encoder.count(), boot_time);

  SetpointRamp ramp(kMaxAccel);
  SpeedController first_controller(kFirstMotorGains, Motor::kMaxPower);
  SpeedController second_controller(kSecondMotorGains, Motor::kMaxPower);
  SafetyMonitor safety;
  StopReason stop_reason = StopReason::kNone;
  bool was_pi_shutdown_requested = false;
  float first_power = 0.0f;
  float second_power = 0.0f;

  SpeedTest speed_test(kTestTargetRps);
  CyclingSpeedTest safety_test(kTestTargetRps, kSafetyTestMs);
  SysIdTest sys_id_test;
  absolute_time_t test_start_time = at_the_end_of_time;  // set on connect
  bool is_test_started = false;
  bool is_test_finished = false;

  bool is_led_on = false;
  absolute_time_t next_status_print = boot_time;
  absolute_time_t next_led_toggle = boot_time;
  absolute_time_t next_control = boot_time;

  while (true) {
    watchdog_update();

    // Test only: start the countdown once a serial viewer is connected.
    if (is_at_the_end_of_time(test_start_time) && stdio_usb_connected()) {
      test_start_time = make_timeout_time_ms(kTestStartDelayMs);
      if (was_rebooted_by_watchdog) {
        printf("Rebooted by the watchdog: the firmware had hung\n");
      }
      printf("Serial connected: %s starts in 3 s\n", test_name(kTestMode));
    }

    if (!is_test_started && time_reached(test_start_time)) {
      is_test_started = true;
      const absolute_time_t now = get_absolute_time();
      switch (kTestMode) {
        case TestMode::kSpeed:
          speed_test.start(now);
          printf(
              "t_ms,setpoint,first_speed,first_power,second_speed,"
              "second_power\n");
          break;
        case TestMode::kSafety:
          safety_test.start(now);
          printf(
              "t_ms,setpoint,first_speed,first_power,second_speed,"
              "second_power,battery_volts,stop\n");
          break;
        case TestMode::kSysId:
          sys_id_test.start(now);
          printf("t_ms,phase,power,first_speed,second_speed\n");
          break;
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

      const bool is_sys_id = kTestMode == TestMode::kSysId;
      const float sys_id_power = sys_id_test.power(now);
      const float target_rps = kTestMode == TestMode::kSafety
                                   ? safety_test.target_rps(now)
                                   : speed_test.target_rps(now);
      const bool is_test_running = is_sys_id ? sys_id_test.is_running(now)
                                   : kTestMode == TestMode::kSafety
                                       ? safety_test.is_running(now)
                                       : speed_test.is_running(now);

      const SafetyInputs safety_inputs = {
          .is_bumper_pressed = is_bumper_pressed(),
          .battery_volts = read_battery_volts(),
          .first_power = first_power,
          .first_speed_rps = first_wheel.speed_rps(),
          .second_power = second_power,
          .second_speed_rps = second_wheel.speed_rps(),
          .is_stop_commanded =
              is_sys_id ? sys_id_power == 0.0f : target_rps == 0.0f,
          .period_seconds = period_seconds,
      };
      const StopReason new_stop_reason = safety.update(safety_inputs);
      if (new_stop_reason != stop_reason) {
        stop_reason = new_stop_reason;
        if (stop_reason == StopReason::kNone) {
          printf("Stop cleared\n");
        } else {
          printf("STOP: %s\n", stop_reason_name(stop_reason));
        }
      }
      if (safety.should_pi_shut_down() && !was_pi_shutdown_requested) {
        was_pi_shutdown_requested = true;
        printf("SHUTDOWN: battery critically low\n");
      }

      float setpoint_rps = 0.0f;
      if (stop_reason != StopReason::kNone) {
        ramp.reset();
        first_controller.reset();
        second_controller.reset();
        first_power = second_power = 0.0f;
      } else if (is_sys_id) {
        first_power = second_power = sys_id_power;
      } else {
        const RampedSetpoint setpoint =
            ramp.advance(target_rps, period_seconds);
        setpoint_rps = setpoint.at_last_reading;
        first_power = first_controller.power_for_next_period(
            setpoint, first_wheel.speed_rps(), period_seconds);
        second_power = second_controller.power_for_next_period(
            setpoint, second_wheel.speed_rps(), period_seconds);
      }
      first_motor.set_power(first_power);
      second_motor.set_power(second_power);

      if (is_test_running) {
        switch (kTestMode) {
          case TestMode::kSpeed:
            printf("%lld,%.3f,%.3f,%.3f,%.3f,%.3f\n", test_time_ms,
                   setpoint_rps, first_wheel.speed_rps(), first_power,
                   second_wheel.speed_rps(), second_power);
            break;
          case TestMode::kSafety:
            printf("%lld,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%s\n", test_time_ms,
                   setpoint_rps, first_wheel.speed_rps(), first_power,
                   second_wheel.speed_rps(), second_power,
                   safety.filtered_battery_volts(),
                   stop_reason_name(stop_reason));
            break;
          case TestMode::kSysId:
            printf("%lld,%u,%.4f,%.3f,%.3f\n", test_time_ms,
                   static_cast<unsigned>(sys_id_test.phase(now)), sys_id_power,
                   first_wheel.speed_rps(), second_wheel.speed_rps());
            break;
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
        printf(
            "first: %8ld (%5.2f rev/s)  second: %8ld (%5.2f rev/s)  "
            "battery: %.2f V\n",
            static_cast<long>(first_wheel.count()), first_wheel.speed_rps(),
            static_cast<long>(second_wheel.count()), second_wheel.speed_rps(),
            safety.filtered_battery_volts());
      }
    }

    if (time_reached(next_led_toggle)) {
      next_led_toggle = delayed_by_ms(next_led_toggle, kLedIntervalMs);
      is_led_on = !is_led_on;
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, is_led_on);
    }
  }
}

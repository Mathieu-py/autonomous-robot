// Pins: encoders on GP2/GP3 and GP6/GP7, MPU-6050 I2C on GP4/GP5,
// MDD3A inputs: first motor on GP10/GP11, second on GP12/GP13,
// bumpers on GP14/GP15, battery divider on GP28 (ADC2).

#include <cstdio>
#include <optional>

#include "bench_test.h"
#include "drive_command.h"
#include "drivetrain.h"
#include "encoder.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "motor.h"
#include "periodic_timer.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "wheel.h"

namespace {

constexpr Mode kMode = Mode::kDrive;

constexpr uint kFirstEncoderPinA = 2;
constexpr uint kSecondEncoderPinA = 6;
constexpr uint kFirstMotorPinA = 10;
constexpr uint kSecondMotorPinA = 12;
constexpr uint kFirstBumperPin = 14;
constexpr uint kSecondBumperPin = 15;
constexpr uint kBatteryPin = 28;
constexpr uint kBatteryAdcInput = kBatteryPin - 26;  // GP26-29 are ADC 0-3

// 33k/10k divider; 12-bit ADC on a 3.3 V reference.
constexpr float kBatteryVoltsPerCount =
    3.3f / 4095.0f * (33.0f + 10.0f) / 10.0f;

constexpr uint32_t kControlPeriodMs = 20;
constexpr uint32_t kWatchdogTimeoutMs = 100;
constexpr int64_t kCommandTimeoutMs = 250;
constexpr uint32_t kStatusPrintIntervalMs = 200;
constexpr uint32_t kLedIntervalMs = 500;

// Gains from results/sysid/2026-11-09-run01 (see results/README.md).
constexpr Wheel::Config kFirstWheelConfig = {
    .ticks_per_rev = 2473.0f,
    .max_accel = 5.0f,  // 0 to 1.0 rev/s in 0.2 s
    .max_power = Motor::kMaxPower,
    .gains = {.k_s = 0.012f,
              .k_v = 0.345f,
              .k_a = 0.0174f,
              .k_p = 0.277f,
              .k_i = 5.48f},
};
constexpr Wheel::Config kSecondWheelConfig = {
    .ticks_per_rev = 2473.0f,
    .max_accel = 5.0f,
    .max_power = Motor::kMaxPower,
    .gains = {.k_s = 0.010f,
              .k_v = 0.349f,
              .k_a = 0.0171f,
              .k_p = 0.272f,
              .k_i = 5.54f},
};

[[noreturn]] void halt(const char* reason) {
  while (true) {
    printf("ERROR: %s\n", reason);
    sleep_ms(1000);
  }
}

void setup_bumper(uint pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_IN);
  gpio_pull_up(pin);
}

void setup_battery_adc() {
  adc_init();
  adc_gpio_init(kBatteryPin);
  adc_select_input(kBatteryAdcInput);
}

Readings read_hardware(const Encoder& first, const Encoder& second) {
  return {
      .first_count = first.count(),
      .second_count = second.count(),
      // Switches connect to ground, so a pressed switch reads low.
      .is_bumper_pressed =
          !gpio_get(kFirstBumperPin) || !gpio_get(kSecondBumperPin),
      .battery_volts = static_cast<float>(adc_read()) * kBatteryVoltsPerCount,
  };
}

// Reads whatever serial input has arrived, without waiting.
void read_commands(DriveCommandReader& reader, LatestCommand& latest,
                   int64_t now_ms) {
  int c;
  while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
    if (const auto command = reader.feed(static_cast<char>(c))) {
      latest.record(*command, now_ms);
    }
  }
}

// The status line pico_bridge reads, every control period.
void print_status_for_pi(int64_t uptime_ms, const PeriodReport& report) {
  printf("S,%lld,%ld,%ld,%.3f,%.3f,%.2f,%s\n", uptime_ms,
         static_cast<long>(report.first_count),
         static_cast<long>(report.second_count), report.first_rps,
         report.second_rps, report.battery_volts,
         stop_reason_name(report.stop_reason));
}

}  // namespace

int main() {
  stdio_init_all();

  // First, so the motor pins are held low (stopped) during startup.
  const Motor first_motor(kFirstMotorPinA);
  const Motor second_motor(kSecondMotorPinA);

  setup_bumper(kFirstBumperPin);
  setup_bumper(kSecondBumperPin);
  setup_battery_adc();

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

  Drivetrain drivetrain(Wheel(kFirstWheelConfig, first_encoder.count()),
                        Wheel(kSecondWheelConfig, second_encoder.count()));
  PeriodReport last_report{};
  bool was_pi_shutdown_requested = false;

  DriveCommandReader command_reader;
  LatestCommand latest_command(kCommandTimeoutMs);
  std::optional<BenchTest> bench_test;
  if (kMode != Mode::kDrive) {
    bench_test.emplace(kMode);
  }
  bool has_serial_connected = false;

  const absolute_time_t boot_time = get_absolute_time();
  PeriodicTimer control_timer(boot_time, kControlPeriodMs);
  PeriodicTimer status_timer(boot_time, kStatusPrintIntervalMs);
  PeriodicTimer led_timer(boot_time, kLedIntervalMs);
  bool is_led_on = false;

  while (true) {
    watchdog_update();
    const absolute_time_t now = get_absolute_time();
    const int64_t uptime_ms = absolute_time_diff_us(boot_time, now) / 1000;

    // On first connection: report a watchdog reboot, and start a test's
    // countdown so the log captures it from the start.
    if (!has_serial_connected && stdio_usb_connected()) {
      has_serial_connected = true;
      if (was_rebooted_by_watchdog) {
        printf("Rebooted by the watchdog: the firmware had hung\n");
      }
      if (bench_test) {
        bench_test->start_countdown(now);
      }
    }

    if (!bench_test) {
      read_commands(command_reader, latest_command, uptime_ms);
    }

    if (const auto period_seconds = control_timer.tick(now)) {
      const DriveCommand command = latest_command.current(uptime_ms);
      const Target target = bench_test ? bench_test->target(now)
                                       : Target{.is_open_loop_power = false,
                                                .first = command.first_rps,
                                                .second = command.second_rps};
      last_report = drivetrain.run_period(
          target, read_hardware(first_encoder, second_encoder),
          *period_seconds);
      first_motor.set_power(last_report.first_power);
      second_motor.set_power(last_report.second_power);

      if (last_report.should_pi_shut_down && !was_pi_shutdown_requested) {
        was_pi_shutdown_requested = true;
        printf("SHUTDOWN: battery critically low\n");
      }
      if (bench_test) {
        bench_test->log(now, last_report);
      } else {
        print_status_for_pi(uptime_ms, last_report);
      }
    }

    if (bench_test && status_timer.tick(now)) {
      bench_test->print_idle_status(last_report);
    }

    if (led_timer.tick(now)) {
      is_led_on = !is_led_on;
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, is_led_on);
    }
  }
}

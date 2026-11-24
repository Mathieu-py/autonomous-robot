#include <cstdio>
#include <optional>

#include "bench/imu_test.h"
#include "bench/mode.h"
#include "bench/motor_tests.h"
#include "devices/encoder.h"
#include "devices/imu.h"
#include "devices/motor.h"
#include "drive/drive_command.h"
#include "drive/drivetrain.h"
#include "drive/wheel.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "periodic_timer.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

namespace {

constexpr Mode kMode = Mode::kDrive;

constexpr uint kFirstEncoderPinA = 2;
constexpr uint kSecondEncoderPinA = 6;
constexpr uint kFirstMotorPinA = 10;
constexpr uint kSecondMotorPinA = 12;
constexpr uint kFirstBumperPin = 14;
constexpr uint kSecondBumperPin = 15;
constexpr uint kBatteryPin = 28;
constexpr uint kImuSdaPin = 4;  // I2C0
constexpr uint kImuSclPin = 5;
constexpr uint kBatteryAdcInput = kBatteryPin - 26;  // GP26-29 are ADC 0-3

// 33k/10k divider; 12-bit ADC on a 3.3 V reference.
constexpr float kBatteryVoltsPerCount =
    3.3f / 4095.0f * (33.0f + 10.0f) / 10.0f;

constexpr uint32_t kControlPeriodMs = 20;
constexpr uint32_t kWatchdogTimeoutMs = 100;
constexpr int64_t kCommandTimeoutMs = 250;
constexpr uint32_t kLedIntervalMs = 500;

// Same for both wheels; CMakeLists.txt reads it from robot.yaml.
constexpr float kTicksPerRev = ROBOT_TICKS_PER_REV;

constexpr Wheel::Config wheel_config(const MotorGains& gains) {
  return {.ticks_per_rev = kTicksPerRev,
          .max_accel = 5.0f,  // 0 to 1.0 rev/s in 0.2 s
          .max_power = Motor::kMaxPower,
          .gains = gains};
}

// Gains from results/sysid/2026-11-09-run01 (see results/README.md).
constexpr Wheel::Config kFirstWheelConfig = wheel_config({.k_s = 0.012f,
                                                          .k_v = 0.345f,
                                                          .k_a = 0.0174f,
                                                          .k_p = 0.277f,
                                                          .k_i = 5.48f});
constexpr Wheel::Config kSecondWheelConfig = wheel_config({.k_s = 0.010f,
                                                           .k_v = 0.349f,
                                                           .k_a = 0.0171f,
                                                           .k_p = 0.272f,
                                                           .k_i = 5.54f});

[[noreturn]] void halt(const char* reason) {
  while (true) {
    printf("ERROR: %s\n", reason);
    sleep_ms(1000);
  }
}

void start_wireless_chip() {
  if (cyw43_arch_init() != 0) {
    halt("failed to start the wireless chip, which drives the LED");
  }
  // The first write takes ~0.5 s, too long for the watchdog.
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
}

void toggle_led(bool& is_led_on) {
  is_led_on = !is_led_on;
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, is_led_on);
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
      // Pressed switches read low.
      .is_bumper_pressed =
          !gpio_get(kFirstBumperPin) || !gpio_get(kSecondBumperPin),
      .battery_volts = static_cast<float>(adc_read()) * kBatteryVoltsPerCount,
  };
}

void read_commands(DriveCommandReader& reader, LatestCommand& latest,
                   int64_t now_ms) {
  int c;
  while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
    if (const auto command = reader.feed(static_cast<char>(c))) {
      latest.record(*command, now_ms);
    }
  }
}

// Chip axes: X robot right, Y forward, Z up.
void print_imu_for_pi(int64_t uptime_ms, const ImuReading& reading) {
  const Vector3& a = reading.accel_mps2;
  const Vector3& g = reading.gyro_radps;
  printf("I,%lld,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f\n", uptime_ms, a.x, a.y, a.z,
         g.x, g.y, g.z);
}

Target speed_target(const DriveCommand& command) {
  return {.is_open_loop_power = false,
          .first = command.first_rps,
          .second = command.second_rps};
}

void print_connection_notes(bool was_rebooted_by_watchdog,
                            const char* imu_error) {
  if (was_rebooted_by_watchdog) {
    printf("Rebooted by the watchdog: the firmware had hung\n");
  }
  // The robot drives without the IMU.
  if (imu_error) {
    printf("WARNING: %s\n", imu_error);
  }
}

void print_status_for_pi(int64_t uptime_ms, const PeriodReport& report) {
  printf("S,%lld,%ld,%ld,%.3f,%.3f,%.2f,%s\n", uptime_ms,
         static_cast<long>(report.first.count),
         static_cast<long>(report.second.count), report.first.speed_rps,
         report.second.speed_rps, report.filtered_battery_volts,
         stop_reason_name(report.stop_reason));
}

void print_for_pi(int64_t uptime_ms, const PeriodReport& report,
                  const std::optional<ImuReading>& imu_reading) {
  print_status_for_pi(uptime_ms, report);
  if (imu_reading) {
    print_imu_for_pi(uptime_ms, *imu_reading);
  }
}

}  // namespace

int main() {
  stdio_init_all();

  // First, so the motor pins stay low during startup.
  const Motor first_motor(kFirstMotorPinA);
  const Motor second_motor(kSecondMotorPinA);

  setup_bumper(kFirstBumperPin);
  setup_bumper(kSecondBumperPin);
  setup_battery_adc();

  // Before the encoders: the wireless chip also claims a PIO state machine.
  start_wireless_chip();

  Encoder first_encoder;
  Encoder second_encoder;
  if (const char* error =
          Encoder::start_pair(kFirstEncoderPinA, kSecondEncoderPinA,
                              first_encoder, second_encoder)) {
    halt(error);
  }

  Imu imu;
  const char* imu_error = imu.start(i2c0, kImuSdaPin, kImuSclPin);
  const bool has_imu = imu_error == nullptr;
  if (kMode == Mode::kImuTest) {
    if (!has_imu) {
      halt(imu_error);
    }
    run_imu_test(imu);
  }

  // Last: a hang now reboots the Pico, motors stopped.
  const bool was_rebooted_by_watchdog = watchdog_caused_reboot();
  watchdog_enable(kWatchdogTimeoutMs, true);

  Drivetrain drivetrain(Wheel(kFirstWheelConfig, first_encoder.count()),
                        Wheel(kSecondWheelConfig, second_encoder.count()));
  DriveCommandReader command_reader;
  LatestCommand latest_command(kCommandTimeoutMs);
  bool has_serial_connected = false;

  const absolute_time_t loop_start = get_absolute_time();
  PeriodicTimer control_timer(to_us_since_boot(loop_start), kControlPeriodMs);
  PeriodicTimer led_timer(to_us_since_boot(loop_start), kLedIntervalMs);
  bool is_led_on = false;
  std::optional<MotorTest> motor_test;
  if (kMode != Mode::kDrive) {
    motor_test.emplace(kMode, loop_start);
  }

  while (true) {
    watchdog_update();
    const absolute_time_t now = get_absolute_time();
    const uint64_t now_us = to_us_since_boot(now);
    const auto uptime_ms = static_cast<int64_t>(now_us / 1000);

    if (!has_serial_connected && stdio_usb_connected()) {
      has_serial_connected = true;
      print_connection_notes(was_rebooted_by_watchdog, imu_error);
      if (motor_test) {
        motor_test->start_countdown(now);
      }
    }

    if (!motor_test) {
      read_commands(command_reader, latest_command, uptime_ms);
    }

    if (const auto period_seconds = control_timer.tick(now_us)) {
      const Target target =
          motor_test ? motor_test->target(now)
                     : speed_target(latest_command.current(uptime_ms));
      const PeriodReport report = drivetrain.run_period(
          target, read_hardware(first_encoder, second_encoder),
          *period_seconds);
      first_motor.set_power(report.first.power);
      second_motor.set_power(report.second.power);

      // Repeated, so a bridge that connects late still sees it.
      if (report.should_pi_shut_down) {
        printf("SHUTDOWN: battery critically low\n");
      }
      if (motor_test) {
        motor_test->log(now, report);
      } else {
        const auto imu_reading = has_imu ? imu.read() : std::nullopt;
        print_for_pi(uptime_ms, report, imu_reading);
      }
    }

    if (led_timer.tick(now_us)) {
      toggle_led(is_led_on);
    }
  }
}

#include "bench/motor_tests.h"

#include <array>
#include <cstdio>
#include <span>

namespace {

constexpr float kTestTargetRps = 1.0f;
constexpr uint32_t kIdleStatusIntervalMs = 200;

// PID step test, in rev/s: forward, stop, then backward.
constexpr Phase kSpeedTestPhases[] = {
    {kTestTargetRps, kTestTargetRps, 3000},
    {0.0f, 0.0f, 1000},
    {-kTestTargetRps, -kTestTargetRps, 3000},
};

// Safety test, in rev/s: 8 s on, 2 s off, six times, so each stop can clear.
constexpr auto kSafetyTestPhases = [] {
  std::array<Phase, 12> phases{};
  for (size_t i = 0; i < phases.size(); i += 2) {
    phases[i] = {kTestTargetRps, kTestTargetRps, 8000};
    phases[i + 1] = {0.0f, 0.0f, 2000};
  }
  return phases;
}();

// Motor identification, open-loop power: ramps (k_s, k_v), steps (k_a).
constexpr Phase kSysIdPhases[] = {
    {0.0f, 0.6f, 10000},   // 0: slow ramp forward
    {0.0f, 0.0f, 2000},    // 1: rest
    {0.5f, 0.5f, 1500},    // 2: step forward
    {0.0f, 0.0f, 2000},    // 3: rest
    {0.0f, -0.6f, 10000},  // 4: slow ramp backward
    {0.0f, 0.0f, 2000},    // 5: rest
    {-0.5f, -0.5f, 1500},  // 6: step backward
    {0.0f, 0.0f, 2000},    // 7: rest
};

// What each motor test runs and logs; print_row matches csv_header.
struct TestSpec {
  std::span<const Phase> phases = {};
  bool is_open_loop_power = false;
  const char* csv_header = "";
  void (*print_row)(int64_t t_ms, size_t phase,
                    const PeriodReport& report) = nullptr;
};

TestSpec spec_for(Mode mode) {
  switch (mode) {
    case Mode::kSpeedTest:
      return {
          .phases = kSpeedTestPhases,
          .is_open_loop_power = false,
          .csv_header =
              "t_ms,setpoint,first_speed,first_power,"
              "second_speed,second_power",
          .print_row = [](int64_t t_ms, size_t, const PeriodReport& report) {
            printf("%lld,%.3f,%.3f,%.3f,%.3f,%.3f\n", t_ms, report.setpoint_rps,
                   report.first.speed_rps, report.first.power,
                   report.second.speed_rps, report.second.power);
          }};
    case Mode::kSafetyTest:
      return {
          .phases = kSafetyTestPhases,
          .is_open_loop_power = false,
          .csv_header =
              "t_ms,setpoint,first_speed,first_power,"
              "second_speed,second_power,battery_volts,stop",
          .print_row = [](int64_t t_ms, size_t, const PeriodReport& report) {
            printf("%lld,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%s\n", t_ms,
                   report.setpoint_rps, report.first.speed_rps,
                   report.first.power, report.second.speed_rps,
                   report.second.power, report.filtered_battery_volts,
                   stop_reason_name(report.stop_reason));
          }};
    case Mode::kSysIdTest:
      return {.phases = kSysIdPhases,
              .is_open_loop_power = true,
              .csv_header = "t_ms,phase,power,first_speed,second_speed",
              .print_row = [](int64_t t_ms, size_t phase,
                              const PeriodReport& report) {
                printf("%lld,%u,%.4f,%.3f,%.3f\n", t_ms,
                       static_cast<unsigned>(phase), report.first.power,
                       report.first.speed_rps, report.second.speed_rps);
              }};
    case Mode::kDrive:
    case Mode::kImuTest:
      break;
  }
  return {};
}

}  // namespace

MotorTest::MotorTest(Mode mode, absolute_time_t now)
    : mode_(mode),
      sequence_(spec_for(mode).phases),
      idle_status_timer_(to_us_since_boot(now), kIdleStatusIntervalMs) {}

void MotorTest::start_countdown(absolute_time_t now) {
  if (is_stopped_by_input_) {
    return;
  }
  start_time_ = delayed_by_ms(now, kTestStartDelayMs);
  announce_test_start(mode_);
}

void MotorTest::stop_on_input() {
  if (is_stopped_by_input_) {
    return;
  }
  is_stopped_by_input_ = true;
  start_time_ = at_the_end_of_time;
  printf("Test stopped by serial input (is the bridge running?)\n");
}

Target MotorTest::target(absolute_time_t now) const {
  const float value = sequence_.value(elapsed_ms(now));
  return {.is_open_loop_power = spec_for(mode_).is_open_loop_power,
          .first = value,
          .second = value};
}

void MotorTest::log(absolute_time_t now, const PeriodReport& report) {
  print_stop_change(report.stop_reason);

  const int64_t t_ms = elapsed_ms(now);
  const bool is_running = sequence_.is_running(t_ms);
  if (is_running) {
    if (!was_running_) {
      printf("%s\n", spec_for(mode_).csv_header);
    }
    spec_for(mode_).print_row(t_ms, sequence_.phase(t_ms), report);
  } else if (was_running_) {
    printf("Done\n");
  } else if (idle_status_timer_.tick(to_us_since_boot(now))) {
    print_idle_status(report);
  }
  was_running_ = is_running;
}

int64_t MotorTest::elapsed_ms(absolute_time_t now) const {
  return absolute_time_diff_us(start_time_, now) / 1000;
}

void MotorTest::print_stop_change(StopReason stop_reason) {
  if (stop_reason == last_stop_reason_) {
    return;
  }
  if (stop_reason == StopReason::kNone) {
    printf("Stop cleared\n");
  } else {
    printf("STOP: %s\n", stop_reason_name(stop_reason));
  }
  last_stop_reason_ = stop_reason;
}

void MotorTest::print_idle_status(const PeriodReport& report) const {
  printf(
      "first: %8ld (%5.2f rev/s)  second: %8ld (%5.2f rev/s)  "
      "battery: %.2f V\n",
      static_cast<long>(report.first.count), report.first.speed_rps,
      static_cast<long>(report.second.count), report.second.speed_rps,
      report.filtered_battery_volts);
}

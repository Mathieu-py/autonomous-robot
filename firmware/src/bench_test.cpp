#include "bench_test.h"

#include <array>
#include <cstdio>
#include <span>

namespace {

constexpr float kTestTargetRps = 1.0f;
constexpr uint32_t kTestStartDelayMs = 3000;

// PID step test, in rev/s: forward, stop, then backward.
constexpr Phase kSpeedTestPhases[] = {
    {kTestTargetRps, kTestTargetRps, 3000},
    {0.0f, 0.0f, 1000},
    {-kTestTargetRps, -kTestTargetRps, 3000},
};

// Safety test, in rev/s: runs 8 s then stops 2 s, six times, so each stop's
// latch can clear before the next check (bumper, battery, stall).
constexpr auto kSafetyTestPhases = [] {
  std::array<Phase, 12> phases{};
  for (size_t i = 0; i < phases.size(); i += 2) {
    phases[i] = {kTestTargetRps, kTestTargetRps, 8000};
    phases[i + 1] = {0.0f, 0.0f, 2000};
  }
  return phases;
}();

// Motor identification, in open-loop power: slow ramps (for k_s, k_v) and
// steps (for k_a) in each direction, with rests between.
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

std::span<const Phase> phases_for(Mode mode) {
  switch (mode) {
    case Mode::kSpeedTest:
      return kSpeedTestPhases;
    case Mode::kSafetyTest:
      return kSafetyTestPhases;
    case Mode::kSysIdTest:
      return kSysIdPhases;
    case Mode::kDrive:
      break;
  }
  return {};
}

}  // namespace

const char* mode_name(Mode mode) {
  switch (mode) {
    case Mode::kSpeedTest:
      return "speed test";
    case Mode::kSafetyTest:
      return "safety test";
    case Mode::kSysIdTest:
      return "identification test";
    case Mode::kDrive:
      return "drive";
  }
  return "unknown";
}

BenchTest::BenchTest(Mode mode) : mode_(mode), sequence_(phases_for(mode)) {}

void BenchTest::start_countdown(absolute_time_t now) {
  start_time_ = delayed_by_ms(now, kTestStartDelayMs);
  printf("Serial connected: %s starts in 3 s\n", mode_name(mode_));
}

Target BenchTest::target(absolute_time_t now) const {
  const float value = sequence_.value(elapsed_ms(now));
  return {.is_open_loop_power = mode_ == Mode::kSysIdTest,
          .first = value,
          .second = value};
}

void BenchTest::log(absolute_time_t now, const PeriodReport& report) {
  if (report.stop_reason != last_stop_reason_) {
    if (report.stop_reason == StopReason::kNone) {
      printf("Stop cleared\n");
    } else {
      printf("STOP: %s\n", stop_reason_name(report.stop_reason));
    }
    last_stop_reason_ = report.stop_reason;
  }

  const int64_t t_ms = elapsed_ms(now);
  if (sequence_.is_running(t_ms)) {
    if (!has_started_) {
      has_started_ = true;
      print_csv_header();
    }
    print_csv_row(t_ms, report);
  } else if (has_started_ && !has_finished_) {
    has_finished_ = true;
    printf("Done\n");
  }
}

void BenchTest::print_idle_status(const PeriodReport& report) const {
  if (has_started_ && !has_finished_) {
    return;
  }
  printf(
      "first: %8ld (%5.2f rev/s)  second: %8ld (%5.2f rev/s)  "
      "battery: %.2f V\n",
      static_cast<long>(report.first_count), report.first_rps,
      static_cast<long>(report.second_count), report.second_rps,
      report.battery_volts);
}

int64_t BenchTest::elapsed_ms(absolute_time_t now) const {
  return absolute_time_diff_us(start_time_, now) / 1000;
}

void BenchTest::print_csv_header() const {
  switch (mode_) {
    case Mode::kSpeedTest:
      printf(
          "t_ms,setpoint,first_speed,first_power,second_speed,"
          "second_power\n");
      break;
    case Mode::kSafetyTest:
      printf(
          "t_ms,setpoint,first_speed,first_power,second_speed,"
          "second_power,battery_volts,stop\n");
      break;
    case Mode::kSysIdTest:
      printf("t_ms,phase,power,first_speed,second_speed\n");
      break;
    case Mode::kDrive:
      break;
  }
}

void BenchTest::print_csv_row(int64_t t_ms, const PeriodReport& report) const {
  switch (mode_) {
    case Mode::kSpeedTest:
      printf("%lld,%.3f,%.3f,%.3f,%.3f,%.3f\n", t_ms, report.setpoint_rps,
             report.first_rps, report.first_power, report.second_rps,
             report.second_power);
      break;
    case Mode::kSafetyTest:
      printf("%lld,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%s\n", t_ms,
             report.setpoint_rps, report.first_rps, report.first_power,
             report.second_rps, report.second_power, report.battery_volts,
             stop_reason_name(report.stop_reason));
      break;
    case Mode::kSysIdTest:
      printf("%lld,%u,%.4f,%.3f,%.3f\n", t_ms,
             static_cast<unsigned>(sequence_.phase(t_ms)),
             sequence_.value(t_ms), report.first_rps, report.second_rps);
      break;
    case Mode::kDrive:
      break;
  }
}

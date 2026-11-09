#include "test_sequences.h"

#include <cstdint>

namespace {

constexpr int64_t kRunMs = 3000;
constexpr int64_t kPauseMs = 1000;
constexpr int64_t kSpeedTestMs = kRunMs + kPauseMs + kRunMs;
constexpr int64_t kCycleRunMs = 8000;
constexpr int64_t kCycleStopMs = 2000;

struct SysIdPhase {
  float start_power;
  float end_power;
  int64_t duration_ms;
};

constexpr SysIdPhase kSysIdPhases[] = {
    {0.0f, 0.6f, 10000},   // 0: slow ramp forward
    {0.0f, 0.0f, 2000},    // 1: rest
    {0.5f, 0.5f, 1500},    // 2: step forward
    {0.0f, 0.0f, 2000},    // 3: rest
    {0.0f, -0.6f, 10000},  // 4: slow ramp backward
    {0.0f, 0.0f, 2000},    // 5: rest
    {-0.5f, -0.5f, 1500},  // 6: step backward
    {0.0f, 0.0f, 2000},    // 7: rest
};
constexpr size_t kSysIdPhaseCount =
    sizeof(kSysIdPhases) / sizeof(kSysIdPhases[0]);

int64_t elapsed_ms(absolute_time_t start, absolute_time_t now) {
  return absolute_time_diff_us(start, now) / 1000;
}

}  // namespace

void SpeedTest::start(absolute_time_t now) {
  started_ = true;
  start_time_ = now;
}

bool SpeedTest::is_running(absolute_time_t now) const {
  return started_ && elapsed_ms(start_time_, now) < kSpeedTestMs;
}

float SpeedTest::target_rps(absolute_time_t now) const {
  if (!is_running(now)) {
    return 0.0f;
  }
  const int64_t t = elapsed_ms(start_time_, now);
  if (t < kRunMs) {
    return target_rps_;
  }
  if (t < kRunMs + kPauseMs) {
    return 0.0f;
  }
  return -target_rps_;
}

void CyclingSpeedTest::start(absolute_time_t now) {
  started_ = true;
  start_time_ = now;
}

bool CyclingSpeedTest::is_running(absolute_time_t now) const {
  return started_ && elapsed_ms(start_time_, now) < duration_ms_;
}

float CyclingSpeedTest::target_rps(absolute_time_t now) const {
  const bool is_running_phase =
      elapsed_ms(start_time_, now) % (kCycleRunMs + kCycleStopMs) < kCycleRunMs;
  return is_running(now) && is_running_phase ? target_rps_ : 0.0f;
}

void SysIdTest::start(absolute_time_t now) {
  started_ = true;
  start_time_ = now;
}

bool SysIdTest::is_running(absolute_time_t now) const {
  return started_ && phase(now) < kSysIdPhaseCount;
}

size_t SysIdTest::phase(absolute_time_t now) const {
  int64_t t = elapsed_ms(start_time_, now);
  for (size_t i = 0; i < kSysIdPhaseCount; ++i) {
    if (t < kSysIdPhases[i].duration_ms) {
      return i;
    }
    t -= kSysIdPhases[i].duration_ms;
  }
  return kSysIdPhaseCount;
}

float SysIdTest::power(absolute_time_t now) const {
  if (!is_running(now)) {
    return 0.0f;
  }
  int64_t t = elapsed_ms(start_time_, now);
  const size_t current = phase(now);
  for (size_t i = 0; i < current; ++i) {
    t -= kSysIdPhases[i].duration_ms;
  }
  const SysIdPhase& p = kSysIdPhases[current];
  const float fraction =
      static_cast<float>(t) / static_cast<float>(p.duration_ms);
  return p.start_power + (p.end_power - p.start_power) * fraction;
}

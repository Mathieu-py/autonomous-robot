#include "bench/mode.h"

#include <cstdio>

const char* mode_name(Mode mode) {
  switch (mode) {
    case Mode::kDrive:
      return "drive";
    case Mode::kSpeedTest:
      return "speed test";
    case Mode::kSafetyTest:
      return "safety test";
    case Mode::kSysIdTest:
      return "identification test";
    case Mode::kImuTest:
      return "IMU test";
  }
  return "unknown";
}

void announce_test_start(Mode mode) {
  printf("Serial connected: %s starts in %lu s\n", mode_name(mode),
         static_cast<unsigned long>(kTestStartDelayMs / 1000));
}

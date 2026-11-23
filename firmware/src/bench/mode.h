#pragma once

#include <cstdint>

// What the firmware runs: kDrive takes commands from the Pi; the others test
// the robot on the bench.
enum class Mode { kDrive, kSpeedTest, kSafetyTest, kSysIdTest, kImuTest };

const char* mode_name(Mode mode);

// Every bench mode starts this long after a serial viewer connects.
constexpr uint32_t kTestStartDelayMs = 3000;

// "Serial connected: <test> starts in 3 s"
void announce_test_start(Mode mode);

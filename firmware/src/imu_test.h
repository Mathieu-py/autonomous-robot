#pragma once

#include "imu.h"

// Waits for a serial viewer, counts down 3 s, then prints readings at 10 Hz.
[[noreturn]] void run_imu_test(const Imu& imu);

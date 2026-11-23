#pragma once

#include "devices/imu.h"

// Waits for a serial viewer, counts down, then prints readings at 10 Hz.
[[noreturn]] void run_imu_test(const Imu& imu);

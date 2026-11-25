#pragma once

#include <array>

#include "devices/imu_reading.h"
#include "drive/drivetrain.h"

// A line for the Pi, without the newline; ros/pico_bridge parses it.
using PiLine = std::array<char, 128>;

// "S,<first_count>,<second_count>,<first_rps>,<second_rps>,<volts>,<stop>"
PiLine format_status_line(const PeriodReport& report);

// "I,<ax>,<ay>,<az>,<gx>,<gy>,<gz>" in m/s^2 and rad/s, chip axes.
PiLine format_imu_line(const ImuReading& reading);

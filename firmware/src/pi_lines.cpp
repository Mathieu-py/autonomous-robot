#include "pi_lines.h"

#include <cstdio>

PiLine format_status_line(const PeriodReport& report) {
  PiLine line;
  std::snprintf(line.data(), line.size(), "S,%ld,%ld,%.3f,%.3f,%.2f,%s",
                static_cast<long>(report.first.count),
                static_cast<long>(report.second.count), report.first.speed_rps,
                report.second.speed_rps, report.filtered_battery_volts,
                stop_reason_name(report.stop_reason));
  return line;
}

// Chip axes: X robot right, Y forward, Z up.
PiLine format_imu_line(const ImuReading& reading) {
  const Vector3& a = reading.accel_mps2;
  const Vector3& g = reading.gyro_radps;
  PiLine line;
  std::snprintf(line.data(), line.size(), "I,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f",
                a.x, a.y, a.z, g.x, g.y, g.z);
  return line;
}

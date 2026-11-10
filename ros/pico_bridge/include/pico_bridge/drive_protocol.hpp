#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <sstream>
#include <string>

// Pure helpers for the Pico link: differential-drive maths and the text
// protocol in firmware/src/drive_command.h and main.cpp (drive mode).
namespace pico_bridge {

struct DriveGeometry {
  double wheel_diameter_m;
  double wheel_separation_m;
};

struct WheelRps {
  double left;
  double right;
};

// Forward speed and turn rate (counterclockwise positive) to wheel speeds.
inline WheelRps wheel_rps_from_twist(double linear_mps, double angular_radps,
                                     const DriveGeometry& geometry) {
  const double half_track = geometry.wheel_separation_m / 2.0;
  const double metres_per_rev = M_PI * geometry.wheel_diameter_m;
  return {(linear_mps - angular_radps * half_track) / metres_per_rev,
          (linear_mps + angular_radps * half_track) / metres_per_rev};
}

inline std::string format_command(double first_rps, double second_rps) {
  char line[48];
  std::snprintf(line, sizeof(line), "V %.3f %.3f\n", first_rps, second_rps);
  return line;
}

// "S,<uptime_ms>,<first_count>,<second_count>,<first_rps>,<second_rps>,
// <battery_volts>,<stop reason>"
struct PicoStatus {
  int64_t uptime_ms;
  int64_t first_count;
  int64_t second_count;
  double first_rps;
  double second_rps;
  double battery_volts;
  std::string stop_reason;
};

inline std::optional<PicoStatus> parse_status_line(const std::string& line) {
  if (line.rfind("S,", 0) != 0) {
    return std::nullopt;
  }
  std::istringstream fields(line.substr(2));
  PicoStatus status;
  char comma = 0;
  fields >> status.uptime_ms >> comma >> status.first_count >> comma >>
      status.second_count >> comma >> status.first_rps >> comma >>
      status.second_rps >> comma >> status.battery_volts >> comma;
  if (!fields || comma != ',') {
    return std::nullopt;
  }
  std::getline(fields, status.stop_reason);
  if (!status.stop_reason.empty() && status.stop_reason.back() == '\r') {
    status.stop_reason.pop_back();
  }
  if (status.stop_reason.empty()) {
    return std::nullopt;
  }
  return status;
}

}  // namespace pico_bridge

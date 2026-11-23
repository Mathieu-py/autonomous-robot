#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <sstream>
#include <string>

// Pure helpers: drive maths and the Pico's text protocol.
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

struct Vector3 {
  double x;
  double y;
  double z;
};

// "I,<uptime_ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>" in m/s^2 and rad/s, chip axes.
struct ImuSample {
  int64_t uptime_ms;
  Vector3 accel_mps2;
  Vector3 gyro_radps;
};

inline std::optional<ImuSample> parse_imu_line(const std::string& line) {
  if (line.rfind("I,", 0) != 0) {
    return std::nullopt;
  }
  std::istringstream fields(line.substr(2));
  ImuSample sample;
  char c1 = 0, c2 = 0, c3 = 0, c4 = 0, c5 = 0, c6 = 0;
  fields >> sample.uptime_ms >> c1 >> sample.accel_mps2.x >> c2 >>
      sample.accel_mps2.y >> c3 >> sample.accel_mps2.z >> c4 >>
      sample.gyro_radps.x >> c5 >> sample.gyro_radps.y >> c6 >>
      sample.gyro_radps.z;
  const bool are_commas = c1 == ',' && c2 == ',' && c3 == ',' && c4 == ',' &&
                          c5 == ',' && c6 == ',';
  if (!fields || !are_commas) {
    return std::nullopt;
  }
  return sample;
}

// Repeated by the Pico while the battery is critically low.
inline bool is_shutdown_request(const std::string& line) {
  return line.rfind("SHUTDOWN", 0) == 0;
}

}  // namespace pico_bridge

#pragma once

#include <charconv>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "pico_bridge/vector3.hpp"

// Pure helpers: the text lines exchanged with the Pico over USB serial.
namespace pico_bridge {

// "V <first_rps> <second_rps>"
inline std::string format_command(double first_rps, double second_rps) {
  char line[48];
  std::snprintf(line, sizeof(line), "V %.3f %.3f\n", first_rps, second_rps);
  return line;
}

// A line's comma-separated fields, ignoring a trailing \r.
inline std::vector<std::string> split_fields(std::string line) {
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  std::vector<std::string> fields;
  size_t start = 0;
  size_t comma;
  while ((comma = line.find(',', start)) != std::string::npos) {
    fields.push_back(line.substr(start, comma - start));
    start = comma + 1;
  }
  fields.push_back(line.substr(start));
  return fields;
}

// The whole field must be the number.
template <typename T>
std::optional<T> parse_number(const std::string& field) {
  T value{};
  const char* end = field.data() + field.size();
  const auto [stopped_at, error] = std::from_chars(field.data(), end, value);
  if (error != std::errc() || stopped_at != end) {
    return std::nullopt;
  }
  return value;
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
  const auto fields = split_fields(line);
  if (fields.size() != 8 || fields[0] != "S" || fields[7].empty()) {
    return std::nullopt;
  }
  const auto uptime_ms = parse_number<int64_t>(fields[1]);
  const auto first_count = parse_number<int64_t>(fields[2]);
  const auto second_count = parse_number<int64_t>(fields[3]);
  const auto first_rps = parse_number<double>(fields[4]);
  const auto second_rps = parse_number<double>(fields[5]);
  const auto battery_volts = parse_number<double>(fields[6]);
  if (!uptime_ms || !first_count || !second_count || !first_rps ||
      !second_rps || !battery_volts) {
    return std::nullopt;
  }
  return PicoStatus{*uptime_ms,  *first_count,   *second_count, *first_rps,
                    *second_rps, *battery_volts, fields[7]};
}

// "I,<uptime_ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>" in m/s^2 and rad/s, chip axes.
struct ImuSample {
  int64_t uptime_ms;
  Vector3 accel_mps2;
  Vector3 gyro_radps;
};

inline std::optional<ImuSample> parse_imu_line(const std::string& line) {
  const auto fields = split_fields(line);
  if (fields.size() != 8 || fields[0] != "I") {
    return std::nullopt;
  }
  const auto uptime_ms = parse_number<int64_t>(fields[1]);
  if (!uptime_ms) {
    return std::nullopt;
  }
  double values[6];  // ax, ay, az, gx, gy, gz
  for (int i = 0; i < 6; ++i) {
    const auto value = parse_number<double>(fields[2 + i]);
    if (!value) {
      return std::nullopt;
    }
    values[i] = *value;
  }
  return ImuSample{*uptime_ms,
                   {values[0], values[1], values[2]},
                   {values[3], values[4], values[5]}};
}

// Repeated by the Pico while the battery is critically low.
inline bool is_shutdown_request(const std::string& line) {
  return line.rfind("SHUTDOWN", 0) == 0;
}

}  // namespace pico_bridge

#include "drive_command.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace {

// Parses one float and advances text past it; false if there isn't one.
bool parse_float(const char*& text, float& value) {
  char* end = nullptr;
  value = std::strtof(text, &end);
  if (end == text || !std::isfinite(value)) {
    return false;
  }
  text = end;
  return true;
}

std::optional<DriveCommand> parse_line(const char* line) {
  if (line[0] != 'V' || line[1] != ' ') {
    return std::nullopt;
  }
  const char* text = line + 2;
  float first = 0.0f;
  float second = 0.0f;
  if (!parse_float(text, first) || !parse_float(text, second)) {
    return std::nullopt;
  }
  while (*text == ' ' || *text == '\r') {
    ++text;
  }
  if (*text != '\0') {
    return std::nullopt;
  }
  constexpr float kMax = DriveCommandReader::kMaxRps;
  return DriveCommand{std::clamp(first, -kMax, kMax),
                      std::clamp(second, -kMax, kMax)};
}

}  // namespace

std::optional<DriveCommand> DriveCommandReader::feed(char c) {
  if (c != '\n') {
    if (length_ < kMaxLineLength) {
      line_[length_++] = c;
    } else {
      is_overflowed_ = true;
    }
    return std::nullopt;
  }

  line_[length_] = '\0';
  const std::optional<DriveCommand> command =
      is_overflowed_ ? std::nullopt : parse_line(line_);
  length_ = 0;
  is_overflowed_ = false;
  return command;
}

void LatestCommand::record(const DriveCommand& command, int64_t now_ms) {
  last_ = command;
  last_ms_ = now_ms;
  has_command_ = true;
}

DriveCommand LatestCommand::current(int64_t now_ms) const {
  const bool is_fresh = has_command_ && now_ms - last_ms_ <= timeout_ms_;
  return is_fresh ? last_ : DriveCommand{0.0f, 0.0f};
}

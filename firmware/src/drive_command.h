#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

// Wheel speed commands from the Pi over USB serial, as text lines:
// "V <first_rps> <second_rps>\n".

struct DriveCommand {
  float first_rps;
  float second_rps;
};

// Assembles characters into lines and parses each complete one.
class DriveCommandReader {
 public:
  static constexpr float kMaxRps = 2.5f;

  // Returns the command once c completes a valid line.
  std::optional<DriveCommand> feed(char c);

 private:
  static constexpr size_t kMaxLineLength = 40;

  char line_[kMaxLineLength + 1] = {};
  size_t length_ = 0;
  bool is_overflowed_ = false;
};

// The last command received, dropped to zero when commands stop arriving.
class LatestCommand {
 public:
  explicit LatestCommand(int64_t timeout_ms) : timeout_ms_(timeout_ms) {}

  void record(const DriveCommand& command, int64_t now_ms);
  DriveCommand current(int64_t now_ms) const;  // zero once timed out

 private:
  int64_t timeout_ms_;
  DriveCommand last_ = {0.0f, 0.0f};
  int64_t last_ms_ = 0;
  bool has_command_ = false;
};

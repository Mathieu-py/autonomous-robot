#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

// Wheel speed commands from the Pi: "V <first_rps> <second_rps>\n".

struct DriveCommand {
  float first_rps;
  float second_rps;
};

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

// The last command received, until commands stop arriving.
class LatestCommand {
 public:
  explicit LatestCommand(int64_t timeout_ms) : timeout_ms_(timeout_ms) {}

  void record(const DriveCommand& command, int64_t now_ms);
  std::optional<DriveCommand> fresh(int64_t now_ms) const;

 private:
  int64_t timeout_ms_;
  std::optional<DriveCommand> last_;
  int64_t last_ms_ = 0;
};

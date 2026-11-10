#include "drive_command.h"

#include <optional>
#include <string>

#include "check.h"

namespace {

// Feeds a whole string; returns the last command it completed.
std::optional<DriveCommand> feed_all(DriveCommandReader& reader,
                                     const std::string& text) {
  std::optional<DriveCommand> last;
  for (const char c : text) {
    if (const auto command = reader.feed(c)) {
      last = command;
    }
  }
  return last;
}

void test_parses_a_command() {
  DriveCommandReader reader;
  const auto command = feed_all(reader, "V 0.50 -0.25\n");
  CHECK(command.has_value());
  CHECK_NEAR(command->first_rps, 0.5f, 1e-6);
  CHECK_NEAR(command->second_rps, -0.25f, 1e-6);
}

void test_accepts_windows_line_endings() {
  DriveCommandReader reader;
  CHECK(feed_all(reader, "V 1 1\r\n"));
}

void test_rejects_malformed_lines() {
  DriveCommandReader reader;
  CHECK(!feed_all(reader, "V 0.5\n"));
  CHECK(!feed_all(reader, "X 0.5 0.5\n"));
  CHECK(!feed_all(reader, "V 0.5 0.5 extra\n"));
  CHECK(!feed_all(reader, "V nan 0.5\n"));
}

void test_recovers_after_an_overlong_line() {
  DriveCommandReader reader;
  CHECK(!feed_all(reader, std::string(100, 'x') + "\n"));
  CHECK(feed_all(reader, "V 0.1 0.1\n"));
}

void test_clamps_speeds() {
  DriveCommandReader reader;
  const auto command = feed_all(reader, "V 9 -9\n");
  CHECK(command.has_value());
  CHECK_NEAR(command->first_rps, DriveCommandReader::kMaxRps, 1e-6);
  CHECK_NEAR(command->second_rps, -DriveCommandReader::kMaxRps, 1e-6);
}

void test_timeout_zeroes_a_stale_command() {
  LatestCommand latest(250);
  CHECK_NEAR(latest.current(0).first_rps, 0.0f, 1e-6);

  latest.record({0.5f, 0.5f}, 1000);
  CHECK_NEAR(latest.current(1250).first_rps, 0.5f, 1e-6);
  CHECK_NEAR(latest.current(1251).first_rps, 0.0f, 1e-6);
}

}  // namespace

int main() {
  test_parses_a_command();
  test_accepts_windows_line_endings();
  test_rejects_malformed_lines();
  test_recovers_after_an_overlong_line();
  test_clamps_speeds();
  test_timeout_zeroes_a_stale_command();
  return report("drive_command");
}

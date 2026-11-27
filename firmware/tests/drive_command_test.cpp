#include "drive/drive_command.h"

#include <gtest/gtest.h>

#include <optional>
#include <string>

#include "pico_line_examples.hpp"

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

TEST(DriveCommandReader, ParsesTheBridgesExample) {
  DriveCommandReader reader;
  const auto command =
      feed_all(reader, std::string(kExampleCommandLine) + "\n");
  ASSERT_TRUE(command.has_value());
  EXPECT_NEAR(command->first_rps, 0.5f, 1e-6);
  EXPECT_NEAR(command->second_rps, -0.25f, 1e-6);
}

TEST(DriveCommandReader, AcceptsWindowsLineEndings) {
  DriveCommandReader reader;
  EXPECT_TRUE(feed_all(reader, "V 1 1\r\n"));
}

TEST(DriveCommandReader, RejectsMalformedLines) {
  DriveCommandReader reader;
  EXPECT_FALSE(feed_all(reader, "V 0.5\n"));
  EXPECT_FALSE(feed_all(reader, "X 0.5 0.5\n"));
  EXPECT_FALSE(feed_all(reader, "V 0.5 0.5 extra\n"));
  EXPECT_FALSE(feed_all(reader, "V nan 0.5\n"));
}

TEST(DriveCommandReader, RecoversAfterAnOverlongLine) {
  DriveCommandReader reader;
  EXPECT_FALSE(feed_all(reader, std::string(100, 'x') + "\n"));
  EXPECT_TRUE(feed_all(reader, "V 0.1 0.1\n"));
}

TEST(DriveCommandReader, LimitsSpeeds) {
  DriveCommandReader reader;
  const auto command = feed_all(reader, "V 9 -9\n");
  ASSERT_TRUE(command.has_value());
  EXPECT_NEAR(command->first_rps, DriveCommandReader::kMaxRps, 1e-6);
  EXPECT_NEAR(command->second_rps, -DriveCommandReader::kMaxRps, 1e-6);
}

TEST(DriveCommandReader, LimitingKeepsTheTurn) {
  DriveCommandReader reader;
  const auto command = feed_all(reader, "V 5 2.5\n");
  ASSERT_TRUE(command.has_value());
  EXPECT_NEAR(command->first_rps, 2.5f, 1e-6);
  EXPECT_NEAR(command->second_rps, 1.25f, 1e-6);
}

TEST(LatestCommand, HasNoneBeforeTheFirstCommand) {
  const LatestCommand latest(250);
  EXPECT_FALSE(latest.fresh(0));
}

TEST(LatestCommand, DropsAStaleCommand) {
  LatestCommand latest(250);
  latest.record({0.5f, 0.5f}, 1000);
  ASSERT_TRUE(latest.fresh(1250));
  EXPECT_NEAR(latest.fresh(1250)->first_rps, 0.5f, 1e-6);
  EXPECT_FALSE(latest.fresh(1251));
}

}  // namespace

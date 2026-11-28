#include "robot_ekf/reorder_buffer.hpp"

#include <gtest/gtest.h>

#include <vector>

using robot_ekf::ReorderBuffer;

namespace {

constexpr double kDelayS = 0.03;
constexpr size_t kWheels = 0;
constexpr size_t kGyro = 1;

using Buffer = ReorderBuffer<int, 2>;

std::vector<int> released(Buffer& buffer) {
  std::vector<int> readings;
  buffer.release([&](double, int reading) { readings.push_back(reading); });
  return readings;
}

}  // namespace

TEST(ReorderBuffer, ReleasesOnceBothSourcesHaveCaughtUp) {
  Buffer buffer(kDelayS);
  buffer.add(kGyro, 1.0001, 1);  // the gyro comes straight from the bridge
  EXPECT_TRUE(released(buffer).empty());
  buffer.add(kWheels, 1.0000, 0);  // the wheels, through wheel_odometry
  EXPECT_EQ(released(buffer), (std::vector<int>{0}));
  buffer.add(kGyro, 1.0201, 3);
  buffer.add(kWheels, 1.0200, 2);
  EXPECT_EQ(released(buffer), (std::vector<int>{1, 2}));
}

TEST(ReorderBuffer, ReleasesInTimeOrder) {
  Buffer buffer(kDelayS);
  buffer.add(kWheels, 1.02, 2);
  buffer.add(kWheels, 1.04, 4);
  buffer.add(kGyro, 1.01, 1);
  buffer.add(kGyro, 1.03, 3);
  EXPECT_EQ(released(buffer), (std::vector<int>{1, 2, 3}));
}

TEST(ReorderBuffer, FallsBackToTheDelayWhileASourceIsQuiet) {
  Buffer buffer(kDelayS);
  buffer.add(kWheels, 1.00, 0);  // no gyro yet, as while the bridge measures
  buffer.add(kWheels, 1.02, 1);  // its offset
  EXPECT_TRUE(released(buffer).empty());
  buffer.add(kWheels, 1.04, 2);
  EXPECT_EQ(released(buffer), (std::vector<int>{0}));
}

TEST(ReorderBuffer, DropsAReadingOlderThanOneAlreadyReleased) {
  Buffer buffer(kDelayS);
  buffer.add(kWheels, 1.000, 1);
  buffer.add(kGyro, 1.050, 2);
  released(buffer);
  EXPECT_FALSE(buffer.add(kWheels, 0.995, 0));
  EXPECT_TRUE(buffer.add(kWheels, 1.200, 3));
  EXPECT_EQ(released(buffer), (std::vector<int>{2}));
}

TEST(ReorderBuffer, StartsAfreshOnceCleared) {
  Buffer buffer(kDelayS);
  buffer.add(kWheels, 10.0, 1);
  buffer.add(kGyro, 10.1, 2);
  released(buffer);
  buffer.clear();
  buffer.add(kWheels, 1.000, 3);  // time went back
  buffer.add(kGyro, 1.050, 4);
  EXPECT_EQ(released(buffer), (std::vector<int>{3}));
}

#include "robot_ekf/reorder_buffer.hpp"

#include <gtest/gtest.h>

#include <vector>

using robot_ekf::ReorderBuffer;

namespace {

constexpr double kDelayS = 0.03;

std::vector<int> released(ReorderBuffer<int>& buffer) {
  std::vector<int> readings;
  buffer.release([&](double, int reading) { readings.push_back(reading); });
  return readings;
}

}  // namespace

TEST(ReorderBuffer, ReleasesInTimeOrderOnceOldEnough) {
  ReorderBuffer<int> buffer(kDelayS);
  buffer.add(1.000, 1);
  buffer.add(0.990, 0);  // arrived late, still in time
  buffer.add(1.050, 2);
  EXPECT_EQ(released(buffer), (std::vector<int>{0, 1}));
  buffer.add(1.100, 3);
  EXPECT_EQ(released(buffer), (std::vector<int>{2}));
}

TEST(ReorderBuffer, DropsAReadingOlderThanOneAlreadyReleased) {
  ReorderBuffer<int> buffer(kDelayS);
  buffer.add(1.000, 1);
  buffer.add(1.050, 2);
  released(buffer);
  EXPECT_FALSE(buffer.add(0.995, 0));
  EXPECT_TRUE(buffer.add(1.200, 3));
  EXPECT_EQ(released(buffer), (std::vector<int>{2}));
}

TEST(ReorderBuffer, StartsAfreshOnceCleared) {
  ReorderBuffer<int> buffer(kDelayS);
  buffer.add(10.0, 1);
  buffer.add(10.1, 2);
  released(buffer);
  buffer.clear();
  buffer.add(1.000, 3);  // time went back
  buffer.add(1.050, 4);
  EXPECT_EQ(released(buffer), (std::vector<int>{3}));
}

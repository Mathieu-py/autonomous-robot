#pragma once

#include <chrono>

namespace pico_bridge {

// Forward speed and turn rate (counterclockwise positive), as in /cmd_vel.
struct RobotSpeed {
  double linear_mps;
  double angular_radps;
};

// The last /cmd_vel, zero once older than the timeout; on the steady clock.
class LatestCmdVel {
 public:
  using TimePoint = std::chrono::steady_clock::time_point;

  explicit LatestCmdVel(std::chrono::steady_clock::duration timeout)
      : timeout_(timeout) {}

  void record(const RobotSpeed& speed, TimePoint now) {
    last_ = speed;
    last_time_ = now;
  }

  RobotSpeed current(TimePoint now) const {
    const bool is_fresh = now - last_time_ <= timeout_;
    return is_fresh ? last_ : RobotSpeed{0.0, 0.0};
  }

 private:
  std::chrono::steady_clock::duration timeout_;
  RobotSpeed last_{0.0, 0.0};
  TimePoint last_time_{};
};

}  // namespace pico_bridge

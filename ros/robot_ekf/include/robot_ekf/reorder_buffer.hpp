#pragma once

#include <algorithm>
#include <limits>
#include <map>

// Holds readings from several sources and releases them in time order once
// they are delay_s older than the newest. No ROS.
namespace robot_ekf {

template <typename Reading>
class ReorderBuffer {
 public:
  explicit ReorderBuffer(double delay_s) : delay_s_(delay_s) {}

  // False for a reading older than the last one released: too late, dropped.
  bool add(double t_s, const Reading& reading) {
    if (t_s < released_s_) {
      return false;
    }
    pending_.emplace(t_s, reading);
    newest_s_ = std::max(newest_s_, t_s);
    return true;
  }

  // Calls handle(t_s, reading) for each reading old enough, oldest first.
  template <typename Handler>
  void release(Handler handle) {
    while (!pending_.empty() &&
           pending_.begin()->first <= newest_s_ - delay_s_) {
      released_s_ = pending_.begin()->first;
      handle(pending_.begin()->first, pending_.begin()->second);
      pending_.erase(pending_.begin());
    }
  }

  double newest_s() const { return newest_s_; }

  void clear() {
    pending_.clear();
    newest_s_ = released_s_ = -std::numeric_limits<double>::infinity();
  }

 private:
  double delay_s_;
  std::multimap<double, Reading> pending_;
  double newest_s_ = -std::numeric_limits<double>::infinity();
  double released_s_ = -std::numeric_limits<double>::infinity();
};

}  // namespace robot_ekf

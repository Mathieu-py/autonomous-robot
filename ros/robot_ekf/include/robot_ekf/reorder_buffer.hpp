#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <map>

// Holds readings from several sources and releases them in time order once
// every source has caught up to them, or once they are delay_s older than the
// newest (a source has gone quiet). No ROS.
namespace robot_ekf {

template <typename Reading, size_t kSourceCount>
class ReorderBuffer {
 public:
  explicit ReorderBuffer(double delay_s) : delay_s_(delay_s) { clear(); }

  // False for a reading older than the last one released: too late, dropped.
  bool add(size_t source, double t_s, const Reading& reading) {
    if (t_s < released_s_) {
      return false;
    }
    pending_.emplace(t_s, reading);
    newest_by_source_s_[source] = std::max(newest_by_source_s_[source], t_s);
    return true;
  }

  // Calls handle(t_s, reading) for each reading ready, oldest first.
  template <typename Handler>
  void release(Handler handle) {
    while (!pending_.empty() && is_ready(pending_.begin()->first)) {
      released_s_ = pending_.begin()->first;
      handle(pending_.begin()->first, pending_.begin()->second);
      pending_.erase(pending_.begin());
    }
  }

  double newest_s() const {
    return *std::max_element(newest_by_source_s_.begin(),
                             newest_by_source_s_.end());
  }

  void clear() {
    pending_.clear();
    newest_by_source_s_.fill(-std::numeric_limits<double>::infinity());
    released_s_ = -std::numeric_limits<double>::infinity();
  }

 private:
  bool is_ready(double t_s) const {
    const double all_caught_up_s = *std::min_element(
        newest_by_source_s_.begin(), newest_by_source_s_.end());
    return t_s <= all_caught_up_s || t_s <= newest_s() - delay_s_;
  }

  double delay_s_;
  std::multimap<double, Reading> pending_;
  std::array<double, kSourceCount> newest_by_source_s_;
  double released_s_;
};

}  // namespace robot_ekf

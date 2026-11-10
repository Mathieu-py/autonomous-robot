#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

// A bench-test sequence: timed phases, each ramping linearly from start_value
// to end_value. Times are ms since the sequence started.

struct Phase {
  float start_value;
  float end_value;
  int64_t duration_ms;
};

class PhaseSequence {
 public:
  constexpr explicit PhaseSequence(std::span<const Phase> phases)
      : phases_(phases) {}

  bool is_running(int64_t elapsed_ms) const;
  size_t phase(int64_t elapsed_ms) const;  // the phase count once finished
  float value(int64_t elapsed_ms) const;   // 0 when not running

 private:
  struct Position {
    size_t phase;
    int64_t ms_into_phase;
  };
  Position locate(int64_t elapsed_ms) const;

  std::span<const Phase> phases_;
};

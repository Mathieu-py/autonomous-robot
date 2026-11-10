#include "phase_sequence.h"

bool PhaseSequence::is_running(int64_t elapsed_ms) const {
  return elapsed_ms >= 0 && phase(elapsed_ms) < phases_.size();
}

size_t PhaseSequence::phase(int64_t elapsed_ms) const {
  return locate(elapsed_ms).phase;
}

float PhaseSequence::value(int64_t elapsed_ms) const {
  if (!is_running(elapsed_ms)) {
    return 0.0f;
  }
  const Position position = locate(elapsed_ms);
  const Phase& phase = phases_[position.phase];
  const float fraction = static_cast<float>(position.ms_into_phase) /
                         static_cast<float>(phase.duration_ms);
  return phase.start_value + (phase.end_value - phase.start_value) * fraction;
}

PhaseSequence::Position PhaseSequence::locate(int64_t elapsed_ms) const {
  for (size_t i = 0; i < phases_.size(); ++i) {
    if (elapsed_ms < phases_[i].duration_ms) {
      return {i, elapsed_ms};
    }
    elapsed_ms -= phases_[i].duration_ms;
  }
  return {phases_.size(), 0};
}

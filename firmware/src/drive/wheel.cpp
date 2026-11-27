#include "drive/wheel.h"

#include <cmath>

namespace {

// Power not spent on speed tracks motor current: 0.5 is about half of stall.
constexpr float kStallExcessPower = 0.5f;

}  // namespace

void Wheel::measure(int32_t count, float period_seconds) {
  // Unsigned, so a count that wraps past ±2^31 still gives the right step.
  const auto ticks = static_cast<int32_t>(static_cast<uint32_t>(count) -
                                          static_cast<uint32_t>(state_.count));
  state_.speed_rps =
      static_cast<float>(ticks) / period_seconds / ticks_per_rev_;
  state_.count = count;
}

float Wheel::follow(float target_rps, float share, float period_seconds) {
  const RampedSetpoint setpoint =
      ramp_.advance(ramp_.partway_to(target_rps, share), period_seconds);
  state_.power = controller_.power_for_next_period(setpoint, state_.speed_rps,
                                                   period_seconds);
  return setpoint.at_last_reading;
}

bool Wheel::is_stalling() const {
  const float speed_power = controller_.gains().k_v * state_.speed_rps;
  const bool is_driven = state_.power != 0.0f;
  return is_driven &&
         std::fabs(state_.power - speed_power) >= kStallExcessPower;
}

void Wheel::stop() {
  ramp_.reset();
  controller_.reset();
  state_.power = 0.0f;
}

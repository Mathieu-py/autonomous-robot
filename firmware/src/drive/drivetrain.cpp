#include "drive/drivetrain.h"

#include <algorithm>

PeriodReport Drivetrain::run_period(const Target& target,
                                    const Readings& readings,
                                    float period_seconds) {
  first_.measure(readings.first_count, period_seconds);
  second_.measure(readings.second_count, period_seconds);

  const StopReason stop_reason = safety_.update({
      .is_bumper_pressed = readings.is_bumper_pressed,
      .battery_volts = readings.battery_volts,
      .is_first_stalling = first_.is_stalling(),
      .is_second_stalling = second_.is_stalling(),
      .is_stop_commanded = target.is_stop(),
      .period_seconds = period_seconds,
  });

  float setpoint_rps = 0.0f;
  if (stop_reason != StopReason::kNone) {
    first_.stop();
    second_.stop();
  } else if (target.is_open_loop_power) {
    first_.set_power(target.first);
    second_.set_power(target.second);
  } else {
    // One share for both wheels, so an arc keeps its curve while ramping.
    const float share =
        std::min(first_.allowed_share(target.first, period_seconds),
                 second_.allowed_share(target.second, period_seconds));
    setpoint_rps = first_.follow(target.first, share, period_seconds);
    second_.follow(target.second, share, period_seconds);
  }

  return {
      .setpoint_rps = setpoint_rps,
      .first = first_.state(),
      .second = second_.state(),
      .filtered_battery_volts = safety_.filtered_battery_volts(),
      .stop_reason = stop_reason,
      .should_pi_shut_down = safety_.should_pi_shut_down(),
  };
}

#include "drivetrain.h"

PeriodReport Drivetrain::run_period(const Target& target,
                                    const Readings& readings,
                                    float period_seconds) {
  first_.measure(readings.first_count, period_seconds);
  second_.measure(readings.second_count, period_seconds);

  const StopReason stop_reason = safety_.update({
      .is_bumper_pressed = readings.is_bumper_pressed,
      .battery_volts = readings.battery_volts,
      .first_power = first_.power(),
      .first_speed_rps = first_.speed_rps(),
      .second_power = second_.power(),
      .second_speed_rps = second_.speed_rps(),
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
    setpoint_rps = first_.follow(target.first, period_seconds);
    second_.follow(target.second, period_seconds);
  }

  return {
      .setpoint_rps = setpoint_rps,
      .first_count = first_.count(),
      .first_rps = first_.speed_rps(),
      .first_power = first_.power(),
      .second_count = second_.count(),
      .second_rps = second_.speed_rps(),
      .second_power = second_.power(),
      .battery_volts = safety_.filtered_battery_volts(),
      .stop_reason = stop_reason,
      .should_pi_shut_down = safety_.should_pi_shut_down(),
  };
}

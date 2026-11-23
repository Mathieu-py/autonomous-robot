#pragma once

#include <numbers>

// Pure helpers: robot speeds to wheel speeds, and wheels to the Pico's motors.
namespace pico_bridge {

struct DriveGeometry {
  double wheel_diameter_m;
  double wheel_separation_m;
};

// Forward positive on both wheels.
struct WheelPair {
  double left;
  double right;
};

// In the Pico's order.
struct MotorPair {
  double first;
  double second;
};

// Forward speed and turn rate (counterclockwise positive) to wheel rev/s.
inline WheelPair wheel_rps_from_twist(double linear_mps, double angular_radps,
                                      const DriveGeometry& geometry) {
  const double half_track = geometry.wheel_separation_m / 2.0;
  const double metres_per_rev = std::numbers::pi * geometry.wheel_diameter_m;
  return {(linear_mps - angular_radps * half_track) / metres_per_rev,
          (linear_mps + angular_radps * half_track) / metres_per_rev};
}

// Which Pico motor turns each wheel, and whether it runs backwards.
struct MotorMapping {
  bool is_left_first_motor;
  bool is_left_reversed;
  bool is_right_reversed;

  MotorPair to_motors(const WheelPair& wheels) const {
    const double left = wheels.left * sign(is_left_reversed);
    const double right = wheels.right * sign(is_right_reversed);
    return is_left_first_motor ? MotorPair{left, right}
                               : MotorPair{right, left};
  }

  WheelPair to_wheels(const MotorPair& motors) const {
    const WheelPair wheels = is_left_first_motor
                                 ? WheelPair{motors.first, motors.second}
                                 : WheelPair{motors.second, motors.first};
    return {wheels.left * sign(is_left_reversed),
            wheels.right * sign(is_right_reversed)};
  }

 private:
  static double sign(bool is_reversed) { return is_reversed ? -1.0 : 1.0; }
};

}  // namespace pico_bridge

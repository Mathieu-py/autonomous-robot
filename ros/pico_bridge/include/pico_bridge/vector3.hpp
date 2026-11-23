#pragma once

#include <algorithm>

namespace pico_bridge {

struct Vector3 {
  double x;
  double y;
  double z;
};

inline Vector3 operator+(const Vector3& a, const Vector3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vector3 operator-(const Vector3& a, const Vector3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vector3 operator/(const Vector3& v, double divisor) {
  return {v.x / divisor, v.y / divisor, v.z / divisor};
}

// Per axis.
inline Vector3 min(const Vector3& a, const Vector3& b) {
  return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}

inline Vector3 max(const Vector3& a, const Vector3& b) {
  return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}

inline double largest_axis(const Vector3& v) {
  return std::max({v.x, v.y, v.z});
}

}  // namespace pico_bridge

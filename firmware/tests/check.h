#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>

// Minimal test helpers: each failed check prints its line and fails the run.

inline int& check_failures() {
  static int failures = 0;
  return failures;
}

#define CHECK(condition)                                           \
  do {                                                             \
    if (!(condition)) {                                            \
      std::printf("%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, \
                  #condition);                                     \
      ++check_failures();                                          \
    }                                                              \
  } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                          \
  do {                                                                   \
    const double check_actual = (actual);                                \
    if (std::fabs(check_actual - (expected)) > (tolerance)) {            \
      std::printf("%s:%d: %s = %g, expected %g\n", __FILE__, __LINE__,   \
                  #actual, check_actual, static_cast<double>(expected)); \
      ++check_failures();                                                \
    }                                                                    \
  } while (0)

inline int report(const char* suite) {
  std::printf("%s: %s\n", suite, check_failures() == 0 ? "passed" : "FAILED");
  return check_failures() == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

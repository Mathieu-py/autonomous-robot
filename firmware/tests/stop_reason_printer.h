#pragma once

#include <ostream>

#include "drive/safety.h"

// Shows the reason's name when a check fails.
inline void PrintTo(StopReason reason, std::ostream* os) {
  *os << stop_reason_name(reason);
}

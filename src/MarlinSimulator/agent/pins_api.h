#pragma once

/**
 * Pin table shared with the agent interface (GET/POST /pins).
 * Marlin's pin_array lives in application.cpp, so the agent reads it through here.
 */

#include <cstddef>

namespace agent {

struct PinEntry { const char *name; int pin; bool digital; };

size_t pin_table_size();
PinEntry pin_table_at(const size_t i);

} // namespace agent

#pragma once

/**
 * Blink the heater Marlin stopped for a thermal fault, so it's obvious in the
 * viewport which hotend (or the bed) threw MINTEMP, MAXTEMP, Heating Failed
 * (thermal watchdog), Thermal Runaway or Thermal Malfunction.
 *
 * Marlin reports every one of these through Temperature::_temp_error() as
 *   "Error:<reason>, system stopped! Heater_ID: <E<n> | bed | ...>"
 * so the fault is read from the serial output, the same thing a host sees.
 * All of them kill the machine, and a reset prints "start", which ends the blink.
 * To be kind to anyone leaving the sim open, the blink also stops on its own.
 *
 * The blink alternates the part's normal color with a 50% darkened version of it.
 *
 * THREADING: observe() runs on the simulation thread (a SerialLog observer);
 * dim() and serialize() run on the UI and agent server threads.
 */

#include <cstdint>
#include <string>

namespace agent { class JsonWriter; }

namespace thermal_fault_blink {

constexpr int bed = -1;               // Heater index for the bed; hotends are 0..max_hotends-1
constexpr int max_hotends = 8;
constexpr uint32_t interval_ms = 500; // Each half of the blink
constexpr uint32_t timeout_ms = 90000;
constexpr float dark = 0.5f;          // Brightness during the dark half

// Feed one line of Marlin serial output. Starts or ends a blink as needed.
void observe(const std::string& line);

// Brightness multiplier for a heater right now: 1, or 'dark' during the dark half
float dim(const int heater);

// Stop all blinking
void clear();

// {"faults":[{"heater":"Hotend0 Heater","reason":"MAXTEMP triggered","seconds_left":..,"dark":..}]}
void serialize(agent::JsonWriter& writer);

} // namespace thermal_fault_blink

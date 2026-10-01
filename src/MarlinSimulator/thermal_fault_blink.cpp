#include "thermal_fault_blink.h"
#include "agent/json.h"

#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <mutex>

namespace thermal_fault_blink {

namespace {

// Wall-clock time: the blink is for a person watching, so it keeps its pace (and
// its timeout) whatever the simulation speed, and even while the sim is frozen.
int64_t now_ms() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

// When each heater's fault began (0 = no fault). Slot 0 is the bed, then the hotends.
std::array<std::atomic<int64_t>, max_hotends + 1> started {};
std::array<std::string, max_hotends + 1> reasons;
std::mutex reason_mutex;

int slot(const int heater) { return heater == bed ? 0 : heater + 1; }

bool starts_with(const std::string& s, const char* prefix) {
  return s.compare(0, std::char_traits<char>::length(prefix), prefix) == 0;
}

} // namespace

void observe(const std::string& line) {
  // After a kill the only way out is a reset, and every boot prints "start"
  if (line == "start") { clear(); return; }

  if (!starts_with(line, "Error:")) return;
  static constexpr char stopped[] = ", system stopped! Heater_ID: ";  // STR_STOPPED_HEATER
  const size_t at = line.find(stopped);
  if (at == std::string::npos) return;

  std::string id = line.substr(at + sizeof(stopped) - 1);
  if (starts_with(id, "redundant ")) id.erase(0, 10);  // STR_REDUNDANT: the real heater follows

  int heater;
  if (starts_with(id, "bed")) heater = bed;
  else if (id.size() > 1 && id[0] == 'E' && isdigit((unsigned char)id[1])) heater = atoi(id.c_str() + 1);
  else return;  // Chamber, cooler, probe, board: nothing in the viewport to blink
  if (heater >= max_hotends) return;

  // Marlin echoes the error on every serial port: the first one starts the clock
  auto& t = started[slot(heater)];
  if (t.load()) return;
  {
    std::scoped_lock lock(reason_mutex);
    reasons[slot(heater)] = line.substr(6, at - 6);  // e.g. "MAXTEMP triggered"
  }
  t.store(now_ms());
}

float dim(const int heater) {
  if (heater != bed && (heater < 0 || heater >= max_hotends)) return 1;
  auto& t = started[slot(heater)];
  const int64_t since = t.load();
  if (!since) return 1;
  const int64_t elapsed = now_ms() - since;
  if (elapsed >= int64_t(timeout_ms)) { t.store(0); return 1; }
  // Normal first, then dark, alternating every interval
  return (elapsed / interval_ms) % 2 ? dark : 1;
}

void clear() {
  for (auto& t : started) t.store(0);
}

void serialize(agent::JsonWriter& writer) {
  const int64_t now = now_ms();
  writer.begin_object();
  writer.key("faults");
  writer.begin_array();
  for (int h = bed; h < max_hotends; ++h) {
    const int64_t since = started[slot(h)].load();
    if (!since || now - since >= int64_t(timeout_ms)) continue;
    writer.begin_object();
    writer.member("heater", h == bed ? std::string("Bed Heater") : "Hotend" + std::to_string(h) + " Heater");
    {
      std::scoped_lock lock(reason_mutex);
      writer.member("reason", reasons[slot(h)]);
    }
    writer.member("seconds_left", double(int64_t(timeout_ms) - (now - since)) / 1000.0);
    writer.member("dark", dim(h) < 1);
    writer.end_object();
  }
  writer.end_array();
  writer.end_object();
}

} // namespace thermal_fault_blink

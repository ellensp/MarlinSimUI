#include "host_print.h"
#include "json.h"
#include "serial_log.h"

#include "../execution_control.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>

namespace agent {

HostPrint host_print;

namespace {

// Strip a ';' comment and surrounding whitespace. Returns "" for blank/comment-only lines.
std::string clean_line(const std::string& raw) {
  std::string s = raw.substr(0, raw.find(';'));
  const size_t a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) return "";
  const size_t b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

bool starts_with(const std::string& s, const char* prefix) {
  return s.compare(0, strlen(prefix), prefix) == 0;
}

} // namespace

std::string HostPrint::with_checksum(const uint32_t n, const std::string& gcode) {
  std::string line = "N" + std::to_string(n) + " " + gcode;
  uint8_t cs = 0;
  for (char c : line) cs ^= uint8_t(c);
  return line + "*" + std::to_string(cs);
}

const char* HostPrint::state_name(const State s) {
  switch (s) {
    case State::Idle:     return "idle";
    case State::Printing: return "printing";
    case State::Paused:   return "paused";
    case State::Done:     return "done";
    case State::Aborted:  return "aborted";
    case State::Halted:   return "halted";
  }
  return "?";
}

bool HostPrint::load(const std::string& path, std::vector<std::string>& loaded, std::string& error) {
  std::ifstream in(path);
  if (!in) { error = "can't read '" + path + "'"; return false; }
  loaded.clear();
  for (std::string raw; std::getline(in, raw);) {
    std::string cmd = clean_line(raw);
    if (!cmd.empty()) loaded.push_back(std::move(cmd));
  }
  if (loaded.empty()) { error = "'" + path + "' has no G-code"; return false; }
  return true;
}

bool HostPrint::start(const std::string& path, std::vector<std::string>&& loaded, const int _port, std::string& error) {
  if (active()) { error = "a serial print is already running (POST /print/control {\"action\":\"abort\"})"; return false; }
  if (!sender) { error = "no serial sender configured"; return false; }
  if (_port < 0 || _port >= SerialLog::stream_count) { error = "port must be 0.." + std::to_string(SerialLog::stream_count - 1); return false; }
  if (loaded.empty()) { error = "'" + path + "' has no G-code"; return false; }

  lines = std::move(loaded);
  file = path;
  port = _port;
  next = 0;
  oks = resends = stale_oks = 0;
  last_action.clear();
  started_at = Kernel::SimulationRuntime::seconds();
  finished_at = 0;
  log_cursor = serial_log.next_sequence();  // Only replies from here on count

  // The line counter reset goes out from pump(), once Marlin is idle
  opened = false;
  in_flight_n = 0;
  awaiting_ok = true;
  state = State::Printing;
  return true;
}

// Reset Marlin's line counter so N1 is the next line it expects. Numbered, as
// OctoPrint does, so ADVANCED_OK answers "ok N0", distinct from any earlier "ok".
void HostPrint::open() {
  log_cursor = serial_log.next_sequence();  // Ignore anything before the reset
  sender(port, with_checksum(0, "M110 N0") + "\n");
  opened = true;
}

void HostPrint::send_next() {
  if (next >= lines.size()) {
    state = State::Done;
    finished_at = Kernel::SimulationRuntime::seconds();
    return;
  }
  in_flight_n = uint32_t(next + 1);
  sender(port, with_checksum(in_flight_n, lines[next]) + "\n");
  ++next;
  awaiting_ok = true;
}

void HostPrint::handle_reply(const std::string& text) {
  if (starts_with(text, "ok")) {
    // ADVANCED_OK: "ok N<n> P<p> B<b>". A numbered "ok" for any other line is left
    // over from before the print (or from a command typed on this port) -- ignore it.
    const size_t npos = text.find(" N");
    if (npos != std::string::npos && awaiting_ok) {
      const long n = strtol(text.c_str() + npos + 2, nullptr, 10);
      if (n != long(in_flight_n)) { ++stale_oks; return; }
    }
    ++oks;
    awaiting_ok = false;
  }
  else if (starts_with(text, "Resend:")) {
    // "Resend: N" -- N is the line Marlin wants next; the "ok" that follows sends it
    const long n = strtol(text.c_str() + 7, nullptr, 10);
    if (n >= 1 && size_t(n) <= lines.size()) { next = size_t(n - 1); ++resends; }
  }
  else if (starts_with(text, "//action:")) {
    last_action = text.substr(9);
    const std::string action = last_action.substr(0, last_action.find(' '));
    if (action == "pause") pause();
    else if (action == "resume") resume();
    else if (action == "cancel") abort();
  }
  else if (text.find("Printer halted") != std::string::npos) {
    state = State::Halted;
    finished_at = Kernel::SimulationRuntime::seconds();
  }
}

void HostPrint::pump() {
  if (!active()) return;

  if (!opened) {
    if (state == State::Printing && (!ready || ready())) open();
    return;
  }

  // Cheap early-out: nothing new from Marlin and still waiting for an "ok"
  if (awaiting_ok && serial_log.next_sequence() <= log_cursor) return;

  // Read everything Marlin sent on our port since the last pump
  for (;;) {
    const auto replies = serial_log.query(log_cursor, 64, port);
    if (replies.empty()) break;
    for (const auto& line : replies) {
      log_cursor = line.sequence + 1;
      handle_reply(line.text);
      if (!active()) return;
    }
  }

  if (state == State::Printing && !awaiting_ok) send_next();
}

void HostPrint::serialize(JsonWriter& writer) const {
  writer.begin_object();
  writer.member("state", state_name(state));
  writer.member("file", file);
  writer.member("port", port);
  writer.member("lines_total", uint64_t(lines.size()));
  writer.member("lines_sent", uint64_t(next));
  writer.member("percent", lines.empty() ? 0.0 : 100.0 * double(next) / double(lines.size()));
  writer.member("awaiting_ok", awaiting_ok);
  writer.member("opened", opened);
  writer.member("oks", oks);
  writer.member("resends", resends);
  writer.member("stale_oks", stale_oks);
  writer.member("last_action", last_action);
  writer.member("started_sim_seconds", started_at);
  const double end = finished_at ? finished_at : (active() ? Kernel::SimulationRuntime::seconds() : started_at);
  writer.member("elapsed_sim_seconds", end - started_at);
  writer.end_object();
}

} // namespace agent

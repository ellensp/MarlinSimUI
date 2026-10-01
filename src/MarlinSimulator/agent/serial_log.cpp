#include "serial_log.h"

#include "../execution_control.h"

#include <algorithm>

namespace agent {

SerialLog serial_log;

void SerialLog::push(uint8_t stream, std::string&& text) {
  // Caller holds the mutex.
  if (observer) observer(stream, text);
  if (line_limit == 0) return;

  while (lines.size() >= line_limit) {
    lines.pop_front();
    ++dropped;
  }

  Line line;
  line.sequence = sequence_counter++;
  line.stream = stream;
  line.sim_seconds = Kernel::SimulationRuntime::seconds();
  line.text = std::move(text);
  lines.push_back(std::move(line));
}

void SerialLog::append(uint8_t stream, const uint8_t* data, size_t length) {
  if (data == nullptr || length == 0 || stream >= stream_count) return;

  std::scoped_lock lock(mutex);

  std::string& pending = partial[stream];

  for (size_t i = 0; i < length; ++i) {
    const char c = char(data[i]);

    if (c == '\n') {
      // Normalize CRLF; Marlin emits both depending on the call site.
      if (!pending.empty() && pending.back() == '\r') pending.pop_back();
      push(stream, std::move(pending));
      pending.clear();
    }
    else pending += c;
  }

  // Guard against a peer that never sends a newline: flush an over-long
  // partial rather than growing it without bound.
  constexpr size_t max_partial = 4096;
  if (pending.size() >= max_partial) {
    push(stream, std::move(pending));
    pending.clear();
  }
}

std::vector<SerialLog::Line> SerialLog::query(uint64_t from, size_t limit, int stream_filter) const {
  std::scoped_lock lock(mutex);

  std::vector<Line> out;
  if (limit == 0) return out;
  out.reserve(std::min(limit, lines.size()));

  for (auto const& line : lines) {
    if (line.sequence < from) continue;
    if (stream_filter >= 0 && line.stream != uint8_t(stream_filter)) continue;
    out.push_back(line);
    if (out.size() >= limit) break;
  }
  return out;
}

void SerialLog::flush_partial() {
  std::scoped_lock lock(mutex);
  for (uint8_t stream = 0; stream < stream_count; ++stream) {
    if (partial[stream].empty()) continue;
    push(stream, std::move(partial[stream]));
    partial[stream].clear();
  }
}

void SerialLog::clear() {
  std::scoped_lock lock(mutex);
  lines.clear();
  lines.shrink_to_fit();
  for (auto& pending : partial) pending.clear();
  dropped = 0;
  // sequence_counter deliberately NOT reset: an agent polling with ?since=
  // must never see sequence numbers go backwards and silently re-read lines.
}

void SerialLog::set_max_lines(size_t value) {
  std::scoped_lock lock(mutex);
  line_limit = value;
  while (lines.size() > line_limit) {
    lines.pop_front();
    ++dropped;
  }
  if (line_limit == 0) lines.shrink_to_fit();
}

size_t SerialLog::max_lines() const {
  std::scoped_lock lock(mutex);
  return line_limit;
}

size_t SerialLog::size() const {
  std::scoped_lock lock(mutex);
  return lines.size();
}

uint64_t SerialLog::next_sequence() const {
  std::scoped_lock lock(mutex);
  return sequence_counter;
}

uint64_t SerialLog::oldest_sequence() const {
  std::scoped_lock lock(mutex);
  // When empty, the next line written will carry sequence_counter, so that is
  // the correct "oldest available" answer for a polling reader.
  return lines.empty() ? sequence_counter : lines.front().sequence;
}

uint64_t SerialLog::dropped_lines() const {
  std::scoped_lock lock(mutex);
  return dropped;
}

} // namespace agent

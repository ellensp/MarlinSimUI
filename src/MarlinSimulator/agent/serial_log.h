#pragma once

#include <cstdint>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace agent {

/**
 * Line-oriented capture of everything Marlin transmits on its serial streams.
 *
 * Why this exists: the ImGui Serial Monitor shows recent output to a human, but
 * an agent needs to read back what the firmware said after the fact -- to
 * confirm an `ok`, catch an `Error:` or `//action:` it wasn't watching for, or
 * reconstruct what led to a kill. Scrollback in the UI widget is not reachable
 * programmatically.
 *
 * Retention is a bounded ring of whole lines (default 10,000), grown lazily --
 * nothing is allocated until output actually arrives. When full, the oldest
 * line is dropped and `dropped_lines` counts it, so a reader can tell the
 * difference between "nothing happened" and "you missed some".
 *
 * Every line carries a monotonic sequence number, so an agent polls with
 * ?since=<last_seen> and never re-reads or skips a line.
 *
 * THREADING: append() runs on the simulation thread; queries are served on the
 * server thread (Affinity::Direct). The mutex is what makes that safe, and it
 * is deliberate: the serial log stays readable even when the simulation is
 * frozen (realtime_scale 0) or wedged in a loop -- which is exactly when you
 * most want to see the last thing the firmware printed.
 */
class SerialLog {
public:
  static constexpr uint8_t stream_count = 4;
  static constexpr size_t default_max_lines = 10000;

  struct Line {
    uint64_t sequence = 0;     // monotonic, never reused
    uint8_t stream = 0;        // originating serial stream index
    double sim_seconds = 0.0;  // simulation clock when the line completed
    std::string text;          // line content, newline stripped
  };

  // Feed raw transmitted bytes. Splits on '\n', strips a trailing '\r'.
  // Incomplete trailing text is held until the rest of the line arrives.
  void append(uint8_t stream, const uint8_t* data, size_t length);

  // Lines with sequence >= from, oldest first, at most limit.
  // stream_filter < 0 means all streams.
  std::vector<Line> query(uint64_t from, size_t limit, int stream_filter) const;

  // Force any buffered partial line into the log (e.g. before shutdown), so a
  // final unterminated message like a bare prompt isn't lost.
  void flush_partial();

  void clear();
  void set_max_lines(size_t value);

  size_t max_lines() const;
  size_t size() const;
  uint64_t next_sequence() const;
  uint64_t oldest_sequence() const;
  uint64_t dropped_lines() const;

  // Called with every completed line, even when retention is 0, on the simulation
  // thread with the log's mutex held: it must be quick and must not call back into
  // the log. For in-sim watchers (e.g. the thermal fault blink). Set once at startup.
  using Observer = std::function<void(uint8_t stream, const std::string& text)>;
  void set_observer(Observer fn) { std::scoped_lock lock(mutex); observer = std::move(fn); }

private:
  Observer observer;

  void push(uint8_t stream, std::string&& text);

  mutable std::mutex mutex;
  std::deque<Line> lines;
  std::string partial[stream_count];
  uint64_t sequence_counter = 0;
  uint64_t dropped = 0;
  size_t line_limit = default_max_lines;
};

// Global instance, fed from Kernel::execute_loop.
extern SerialLog serial_log;

} // namespace agent

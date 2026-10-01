#pragma once

/**
 * A minimal G-code host for the agent interface: streams a file to Marlin over
 * one of the simulated serial ports the way Pronterface or OctoPrint would, so
 * serial printing can be tested with no external host.
 *
 * Protocol, as Marlin expects from a host:
 *   - Waits until Marlin's command queue is idle, then sends "N0 M110 N0*<cs>" to
 *     reset Marlin's line counter. Starting while Marlin is busy is unsafe: an
 *     injected command (e.g. STARTUP_COMMANDS) answers "ok" for whatever serial
 *     command heads the queue, so the M110 is acked before it runs, the next line
 *     goes out, and the late M110 then rewinds Marlin's line count -- which ends
 *     in a Resend and a line executed twice.
 *   - Sends each line as "N<n> <gcode>*<checksum>" (XOR of the bytes before '*').
 *   - Ping-pong: one line in flight, the next sent when Marlin acknowledges it.
 *     Marlin only answers "ok" once a command has been *processed*, and a move
 *     is processed when it enters the planner, so one line in flight keeps the
 *     planner full as long as lines arrive faster than moves finish.
 *   - With ADVANCED_OK the reply is "ok N<n> P<planner free> B<queue free>", and
 *     only the "ok" for the line in flight counts. Any other "ok" -- for a command
 *     Marlin had already queued on this port, or one typed while printing -- is
 *     ignored, so the sender can't get two lines ahead. Without ADVANCED_OK every
 *     "ok" counts (the classic host behavior), so don't type on the port mid-print.
 *   - "Resend: N" rewinds to line N (the plain "ok" that follows then sends it).
 *   - Host actions (HOST_ACTION_COMMANDS): "//action:pause" pauses sending,
 *     "//action:resume" resumes, "//action:cancel" aborts. "paused"/"resumed"
 *     are notices that Marlin paused itself, so sending continues and Marlin's
 *     queue simply holds.
 *   - "Error:Printer halted" (kill) stops the print.
 *
 * Marlin's replies are read from the agent serial log, filtered to the port, so
 * the sender sees exactly what a real host on that port would.
 *
 * THREADING: everything runs on the simulation thread -- start/control from
 * agent handlers, pump() from Kernel::execute_loop.
 */

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace agent {

class JsonWriter;

class HostPrint {
public:
  enum class State : uint8_t { Idle, Printing, Paused, Done, Aborted, Halted };

  // Writes bytes into Marlin's receive buffer for the given serial port
  using Sender = std::function<void(int port, const std::string& line)>;
  void set_sender(Sender s) { sender = s; }

  // True when Marlin has nothing queued or injected, so the M110 that opens a print
  // can't be answered by an "ok" meant for something else. See pump().
  using Ready = std::function<bool()>;
  void set_ready(Ready r) { ready = r; }

  // Read a G-code file into commands, dropping comments and blank lines.
  // Touches only the host filesystem, so it's safe on the agent server thread.
  static bool load(const std::string& path, std::vector<std::string>& lines, std::string& error);

  // Begin streaming loaded lines on 'port'. Fails if busy. 'path' is for reporting.
  bool start(const std::string& path, std::vector<std::string>&& lines, const int port, std::string& error);

  void pause()  { if (state == State::Printing) state = State::Paused; }
  void resume() { if (state == State::Paused) state = State::Printing; }
  void abort()  { if (active()) state = State::Aborted; }

  bool active() const { return state == State::Printing || state == State::Paused; }

  // Advance the protocol: read new replies, send the next line when due
  void pump();

  void serialize(JsonWriter& writer) const;

private:
  void handle_reply(const std::string& text);
  void open();
  void send_next();

  static std::string with_checksum(const uint32_t n, const std::string& gcode);
  static const char* state_name(const State s);

  Sender sender;
  Ready ready;
  State state = State::Idle;
  std::string file;
  int port = 0;
  std::vector<std::string> lines;  // Commands only: comments and blank lines dropped
  size_t next = 0;                 // Index of the next line to send (N = index + 1)
  bool awaiting_ok = false;
  bool opened = false;             // "N0 M110 N0" sent (waits for Marlin to be ready)
  uint32_t in_flight_n = 0;        // Line number awaiting its "ok" (0 = the M110)
  uint32_t stale_oks = 0;          // "ok N<n>" replies for other lines, ignored
  uint64_t log_cursor = 0;         // Serial log sequence to read from
  uint32_t oks = 0, resends = 0;
  double started_at = 0, finished_at = 0;  // Simulation seconds
  std::string last_action;                 // Last //action: seen
};

extern HostPrint host_print;

} // namespace agent

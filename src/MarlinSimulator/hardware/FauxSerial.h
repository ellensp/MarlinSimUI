#pragma once

//
// Faux serial devices: a peripheral on one of Marlin's serial streams that is
// simulated entirely in software. Marlin's driver code runs unmodified and talks
// to the "device" as if it were real hardware (MMU, serial display, smart
// sensor, ...). A device only has to interpret what Marlin sends and say what the
// real hardware would say back.
//
// A device claims a stream (0-3), sees every byte Marlin transmits on it, and sends
// replies after a delay in simulated time. The base class splits the input into lines;
// override on_bytes() instead for a binary protocol.
//
// The stream's bytes still go to its Serial Monitor, so the conversation can be watched.
//

#include <cstdint>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

class FauxSerialDevice {
public:
  FauxSerialDevice(const int stream);
  virtual ~FauxSerialDevice();

  int stream_index() const { return stream; }

  // Called by the execution loop with the bytes Marlin sent on a stream
  static void from_marlin(const int stream, const uint8_t* data, const size_t count);

  // Called by the execution loop to deliver replies that are due
  static void service();

protected:
  // Marlin sent these bytes. The default splits lines and calls on_line().
  virtual void on_bytes(const uint8_t* data, const size_t count);

  // Marlin sent a complete line (without the line ending)
  virtual void on_line(const std::string& line) { (void)line; }

  // Send bytes to Marlin after a delay in simulated milliseconds
  // `delivered` runs when the reply goes out, so a device can change state when its work is done.
  void reply(const std::string& text, const uint32_t delay_ms = 0, std::function<void()> delivered = nullptr);

private:
  struct Pending { uint64_t due_ticks; std::string text; std::function<void()> delivered; };

  void deliver_due();

  int stream;
  std::string line_buffer;
  std::deque<Pending> pending;
  std::mutex mutex;

  static std::mutex registry_mutex;
  static std::vector<FauxSerialDevice*> registry;
};

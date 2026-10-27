#include "FauxSerial.h"

#include "../execution_control.h"
#include "serial.h"

extern MSerialT serial_stream_0, serial_stream_1, serial_stream_2, serial_stream_3;

std::mutex FauxSerialDevice::registry_mutex;
std::vector<FauxSerialDevice*> FauxSerialDevice::registry;

static MSerialT* stream_by_index(const int index) {
  switch (index) {
    case 0: return &serial_stream_0;
    case 1: return &serial_stream_1;
    case 2: return &serial_stream_2;
    case 3: return &serial_stream_3;
  }
  return nullptr;
}

FauxSerialDevice::FauxSerialDevice(const int stream) : stream(stream) {
  std::scoped_lock lock(registry_mutex);
  registry.push_back(this);
}

FauxSerialDevice::~FauxSerialDevice() {
  std::scoped_lock lock(registry_mutex);
  for (auto it = registry.begin(); it != registry.end(); ++it) if (*it == this) { registry.erase(it); break; }
}

void FauxSerialDevice::on_bytes(const uint8_t* data, const size_t count) {
  for (size_t i = 0; i < count; ++i) {
    const char c = char(data[i]);
    if (c == '\n' || c == '\r') {
      if (!line_buffer.empty()) {
        const std::string line = line_buffer;
        line_buffer.clear();
        on_line(line);
      }
    }
    else if (line_buffer.size() < 256) line_buffer += c;
  }
}

void FauxSerialDevice::reply(const std::string& text, const uint32_t delay_ms, std::function<void()> delivered) {
  const uint64_t due = Kernel::TimeControl::getTicks() + Kernel::TimeControl::nanosToTicks(uint64_t(delay_ms) * Kernel::TimeControl::ONE_MILLION);
  std::scoped_lock lock(mutex);
  pending.push_back({ due, text, delivered });
}

void FauxSerialDevice::deliver_due() {
  MSerialT* port = stream_by_index(stream);
  if (!port) return;
  const uint64_t now = Kernel::TimeControl::getTicks();
  // Replies go out in order, so a late one holds back those behind it
  for (;;) {
    Pending next;
    {
      std::scoped_lock lock(mutex);
      if (pending.empty() || pending.front().due_ticks > now) return;
      next = pending.front();
      pending.pop_front();
    }
    if (next.delivered) next.delivered();
    port->receive_buffer.write((uint8_t*)next.text.data(), next.text.size());
  }
}

void FauxSerialDevice::from_marlin(const int stream, const uint8_t* data, const size_t count) {
  std::vector<FauxSerialDevice*> devices;
  { std::scoped_lock lock(registry_mutex); devices = registry; }
  for (auto device : devices) if (device->stream == stream) device->on_bytes(data, count);
}

void FauxSerialDevice::service() {
  std::vector<FauxSerialDevice*> devices;
  { std::scoped_lock lock(registry_mutex); devices = registry; }
  for (auto device : devices) device->deliver_due();
}

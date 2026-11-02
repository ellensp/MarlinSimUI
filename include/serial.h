/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include <src/inc/MarlinConfigPre.h>
#if ENABLED(EMERGENCY_PARSER)
  #include <src/feature/e_parser.h>
#endif
#include <src/core/serial_hook.h>

#include <algorithm>
#include <cstring>
#include <stdarg.h>
#include <stdio.h>

char* itoa(int num, char* str, int base);

#include <RingBuffer.h>

struct HalSerial {
  HalSerial() { host_connected = true; }

  void begin(int32_t) {}
  void end()          {}

  int peek() {
    uint8_t value;
    return receive_buffer.peek(&value) ? value : -1;
  }

  int16_t read() {
    uint8_t value = 0;
    uint32_t ret = receive_buffer.read(&value);
    return (ret ? value : -1);
  }

  size_t write(char c) {
    if (!host_connected) return 0;
    while (!transmit_buffer.free());
    return transmit_buffer.write(c);
  }

  // Write a buffer, as Arduino's Print does. (Used by the MMU3 driver.)
  size_t write(const uint8_t* buffer, size_t size) {
    size_t count = 0;
    while (size--) count += write(char(*buffer++));
    return count;
  }

  // Bytes arriving from the host. A UART's RX interrupt feeds each one to the emergency
  // parser (M108, M410, M112, M876...) so they act even while a command is running.
  // FauxSerial replies from simulated peripherals are NOT host input and bypass this.
  size_t rx_write(const uint8_t* src, size_t length) {
    length = std::min(length, receive_buffer.free());
    #if ENABLED(EMERGENCY_PARSER)
      for (size_t i = 0; i < length; ++i) emergency_parser.update(emergency_state, src[i]);
    #endif
    return receive_buffer.write(src, length);
  }

  // Move whatever the UI terminal has queued into the receive buffer as host input.
  template<typename Ring>
  void rx_write_from(Ring& source) {
    uint8_t chunk[256];
    while (size_t count = source.read(chunk, std::min(sizeof(chunk), receive_buffer.free()))) rx_write(chunk, count);
  }

  bool connected() { return host_connected; }
  operator bool() {return connected();}

  uint16_t available() {
    return (uint16_t)receive_buffer.available();
  }

  void flush() { receive_buffer.clear(); }

  uint8_t availableForWrite() {
    return transmit_buffer.free() > 255 ? 255 : (uint8_t)transmit_buffer.free();
  }

  void flushTX() {
    if (host_connected) {
      while (transmit_buffer.available()) {
        Kernel::yield();
      }
    }
  }

  static constexpr std::size_t receive_buffer_size = 32768;
  static constexpr std::size_t transmit_buffer_size = 32768;
  RingBuffer<uint8_t, receive_buffer_size> receive_buffer;
  RingBuffer<uint8_t, transmit_buffer_size> transmit_buffer;
  volatile bool host_connected;
  #if ENABLED(EMERGENCY_PARSER)
    EmergencyParser::State emergency_state = EmergencyParser::EP_RESET;
  #endif
};

typedef Serial1Class<HalSerial> MSerialT;

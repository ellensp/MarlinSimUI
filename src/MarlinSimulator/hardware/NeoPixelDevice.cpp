#include <imgui.h>

#include "pinmapping.h"
#include "NeoPixelDevice.h"
#include "../agent/json.h"

NeoPixelDevice::NeoPixelDevice(pin_type neopixel_pin, const uint8_t led_type, const uint16_t led_count) : VirtualPrinter::Component("NeoPixel"), neopixel_pin(neopixel_pin), led_type(led_type), led_count(led_count) {
  bits_per_word = led_type == NEO_GRBW ? 32 : 24;
  Gpio::attach(this->neopixel_pin, [this](GpioEvent& event){ this->interrupt(event); });
}

NeoPixelDevice::~NeoPixelDevice() {
}

void NeoPixelDevice::update() {
  if (Kernel::SimulationRuntime::nanos() - last_clock > 50000 && state != SignalState::INACTIVE) {
    if (bit_count == bits_per_word - 1) {
      if (state == SignalState::LOW_ZERO) {
          ++ bit_count;
          data_word <<= 1;
      } else if (state == SignalState::LOW_ONE) {
          ++ bit_count;
          data_word <<= 1;
          ++ data_word;
      }
      //printf("0x%08X\n", data_word);
      data_word <<= (32 - bit_count);
      update_led(data_word);
    } else {
      //printf("!TIMEOUT INACTIVE! (%d bits, 0x%08X)\n", bit_count, data_word);
    }
    state = SignalState::INACTIVE;
    bit_count = 0;
    data_word = 0;
    frame_index = 0;
  }
}

void NeoPixelDevice::ui_widget() {
  for (size_t i = 0; i < leds_display.size(); i++) {
    const ImVec4 &led = leds_display[i];
    ImGui::Text("%2zu |", i); ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Text, led);
    ImGui::Text("LED");
    ImGui::PopStyleColor();
    ImGui::SameLine(); ImGui::Text("| R %3d G %3d B %3d", int(led.x * 255 + 0.5f), int(led.y * 255 + 0.5f), int(led.z * 255 + 0.5f));
  }
}

// Last frame Marlin sent: "leds" is [[r, g, b], ...] (0-255), LED 0 first
void NeoPixelDevice::serialize(agent::JsonWriter& writer) const {
  writer.begin_object();
  writer.member("pin", int64_t(neopixel_pin));
  writer.member("led_count", int64_t(led_count));
  writer.key("leds");
  writer.begin_array();
  for (const auto& led : leds_display) {
    writer.begin_array();
    for (const float c : { led.x, led.y, led.z }) writer.value(int64_t(std::lround(c * 255)));
    writer.end_array();
  }
  writer.end_array();
  writer.end_object();
}

// models energy transfer but not time lag as it tranfers through the medium.
void NeoPixelDevice::interrupt(GpioEvent& ev) {
  if (ev.pin_id == neopixel_pin && (ev.event == ev.RISE || ev.event == ev.FALL)) {
    auto timestamp = Kernel::TimeControl::ticksToNanos(ev.timestamp);
    auto data = (uint32_t)(timestamp - last_clock);
    last_clock = timestamp;
    bool new_frame = false;

    switch (state) {
      case SignalState::INACTIVE:
        data_word = 0;
        bit_count = 0;
        if (ev.event == ev.RISE) { state = SignalState::HIGH; }
        break;
      case SignalState::HIGH:
        if (ev.event != ev.FALL) { state = SignalState::INACTIVE;}
        else if (data > 200 && data < 500) { state = SignalState::LOW_ZERO;}
        else if (data > 550 && data < 5500) { state = SignalState::LOW_ONE;}
        else { state = SignalState::INACTIVE;}
        break;
      case SignalState::LOW_ZERO:
      case SignalState::LOW_ONE:
        ++ bit_count;
        data_word <<= 1;
        if (state == SignalState::LOW_ONE) ++ data_word;
        if (ev.event != ev.RISE || data <= 450) { state = SignalState::INACTIVE; }
        else if (data < 5000) { state = SignalState::HIGH; }
        else { new_frame = true; }  // A long low is a latch: this rise starts the next frame
        break;
    }

    if (bit_count == bits_per_word) {
      data_word <<= (32 - bit_count);
      bit_count = 0;
      update_led(data_word);
      data_word = 0;
    }

    if (new_frame) {
      frame_index = 0;
      bit_count = 0;
      data_word = 0;
      state = SignalState::HIGH;
    }
  }
}

void NeoPixelDevice::update_led(uint32_t colour) {
  uint8_t idx = 3 - ((led_type & 0x30) >> 4);
  float red   =  (((0xFF << (idx * 8)) & colour) >> (idx * 8)) / 255.0;
  idx = 3 - ((led_type & 0x0C) >> 2);
  float green =  (((0xFF << (idx * 8)) & colour) >> (idx * 8)) / 255.0;
  idx = 3 - (led_type & 0x03);
  float blue  =  (((0xFF << (idx * 8)) & colour) >> (idx * 8)) / 255.0;
  // Each frame fills the LEDs from the first, so a short or long frame can't shift them
  const ImVec4 led(red, green, blue, 1.0f);
  if (frame_index < leds_display.size()) leds_display[frame_index] = led;
  else if (leds_display.size() < led_count) leds_display.push_back(led);
  frame_index++;
}

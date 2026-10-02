#pragma once

#include <SDL2/SDL.h>
#include "../user_interface.h"

#include <list>
#include <deque>
#include <mutex>
#include <atomic>
#include "Gpio.h"
#include "bus/spi.h"

#include "SPISlavePeripheral.h"
#include "XPT2046Device.h"

#ifndef TFT_WIDTH
  #define TFT_WIDTH 480
#endif
#ifndef TFT_HEIGHT
  #define TFT_HEIGHT 320
#endif

class ST7796Device: public SPISlavePeripheral {
public:
  enum KeyName {
    KILL_BUTTON, ENCODER_BUTTON, BACK_BUTTON, COUNT
  };

  struct Command {
    Command(uint8_t cmd, std::vector<uint8_t> data) : cmd(cmd), data(data){};
    uint8_t cmd = 0;
    std::vector<uint8_t> data;
  };

  ST7796Device(SpiBus& spi_bus, pin_type tft_cs, SpiBus& touch_spi_bus, pin_type touch_cs, pin_type dc, pin_type beeper, pin_type enc1, pin_type enc2, pin_type enc_but, pin_type back, pin_type kill);
  virtual ~ST7796Device();
  void process_command(Command cmd);
  void update();
  void interrupt(GpioEvent& ev);

  void ui_init();
  void ui_widget();

  bool capture(std::vector<uint8_t>& rgb, uint32_t& out_width, uint32_t& out_height) const override {
    // graphic_ram is RGB565; expand to 8 bits per channel with bit
    // replication so full-scale values map to 255 rather than 248/252.
    // The touch rects overlay is included, as it appears on screen.
    std::vector<uint16_t> ram;
    const uint16_t * const src = composed(ram);
    out_width = width; out_height = height;
    rgb.resize(size_t(width) * size_t(height) * 3);
    for (size_t i = 0; i < size_t(width) * size_t(height); ++i) {
      const uint16_t pixel = src[i];
      const uint8_t r5 = uint8_t((pixel >> 11) & 0x1F);
      const uint8_t g6 = uint8_t((pixel >> 5)  & 0x3F);
      const uint8_t b5 = uint8_t( pixel        & 0x1F);
      rgb[i * 3 + 0] = uint8_t((r5 << 3) | (r5 >> 2));
      rgb[i * 3 + 1] = uint8_t((g6 << 2) | (g6 >> 4));
      rgb[i * 3 + 2] = uint8_t((b5 << 3) | (b5 >> 2));
    }
    return true;
  }

  void onByteReceived(uint8_t _byte) override;
  void onEndTransaction() override;

  void serialize(agent::JsonWriter& writer) const override;
  bool apply(const agent::JsonValue& body, std::string& error) override;

  /**
   * Touch rects overlay: Marlin (TFT_COLOR_UI + TOUCH_SCREEN) sends its touch
   * control rectangles with the custom panel command 0xF5 as the last stage of
   * drawing a screen. When shown, they are composited over the panel image
   * (2px white outline, 10% white interior) without changing graphic_ram, so
   * they can be toggled at any time. Shared by all panels, like a view option.
   */
  struct TouchRect { uint16_t x, y, w, h; };
  static std::atomic<bool> show_touch_rects;
  size_t touch_rect_count() const { std::lock_guard<std::mutex> lock(touch_rects_mutex); return touch_rects.size(); }

  // graphic_ram, or a copy in ram with the touch rects overlay composited
  const uint16_t* composed(std::vector<uint16_t>& ram) const;

  static constexpr uint32_t width = TFT_WIDTH, height = TFT_HEIGHT;
  bool render_integer_scaling = false, render_popout = false;

  pin_type dc_pin, beeper_pin, enc1_pin, enc2_pin, enc_but_pin, back_pin, kill_pin;

  uint8_t command = 0;
  std::vector<uint8_t> data;
  uint8_t incoming_cmd[3] = {};
  std::deque<Command> cmd_in;

  static constexpr uint32_t graphic_ram_size = width * height;
  uint16_t graphic_ram[graphic_ram_size] = {}; // 64 x 256bit
  uint16_t graphic_ram_index = 0;
  uint16_t graphic_ram_index_x = 0, graphic_ram_index_y = 0;

  uint32_t address_counter = 0;
  int8_t address_increment = 1;

  uint16_t xMin = 0;
  uint16_t xMax = 0;
  uint16_t yMin = 0;
  uint16_t yMax = 0;

  bool key_pressed[KeyName::COUNT] = {};
  uint8_t encoder_position = 0.0f;
  static constexpr int8_t encoder_table[4] = {1, 3, 2, 0};

  bool dirty = true;
  std::chrono::steady_clock clock;
  std::chrono::steady_clock::time_point last_update;
  float scaler;
  GLuint texture_id;

  std::shared_ptr<XPT2046Device> touch;

  mutable std::mutex touch_rects_mutex;
  std::vector<TouchRect> touch_rects;
};

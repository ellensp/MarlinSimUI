#pragma once

#include <functional>
#include <SDL2/SDL.h>
#include <glm/glm.hpp>

#include "../user_interface.h"
#include "../agent/json.h"
#include "SPISlavePeripheral.h"

/**
 * ST7565-family 128x64 graphic LCD controller (ST7565, ST7567, UC1701, LM6059).
 *
 * Used by the MKS/FYSETC/BTT Mini 12864, MINIPANEL, Viki2, AZSMZ 12864, Cartesio UI and
 * Ender-2 displays. Marlin drives them over hardware SPI: the shared SPI bus, with CS and A0.
 *
 * Display RAM is 8 pages of 8 pixel rows, 132 columns. A0 low is a command, A0 high a data
 * byte, written to the current page and column with the LSB at the top. RAM column
 * column_offset is drawn as the left edge and page 0 as the top: each module's glass is wired
 * to suit the segment direction, common direction and start line Marlin sets for it (e.g. the
 * LM6059 uses start line 32), so those don't change the picture.
 */
class ST7565Device : public SPISlavePeripheral {
public:
  enum KeyName {
    KILL_BUTTON, ENCODER_BUTTON, BACK_BUTTON, COUNT
  };

  // column_offset: the RAM column drawn as the left edge (e.g. ST7565_XOFFSET)
  // rotate_180: the panel is mounted upside down (LCD_SCREEN_ROTATE 180), so Marlin draws it rotated
  ST7565Device(SpiBus& spi_bus, pin_type cs, pin_type a0, uint8_t column_offset, bool rotate_180, pin_type beeper, pin_type enc1, pin_type enc2, pin_type enc_but, pin_type back, pin_type kill);
  virtual ~ST7565Device();

  void update() override;
  void interrupt(GpioEvent& ev);
  void onByteReceived(uint8_t _byte) override;
  void ui_init() override;
  void ui_widget() override;

  // The panel's backlight is lit by RGB LEDs (e.g. the FYSETC Mini 12864 2.0 RGB pins or 2.1 NeoPixel).
  // The source gives its colour (0-1 per channel), or false while it hasn't been set.
  void set_backlight(std::function<bool(glm::vec3&)> source) { backlight = source; dirty = true; }

  bool capture(std::vector<uint8_t>& rgb, uint32_t& out_width, uint32_t& out_height) const override {
    out_width = width; out_height = height;
    rgb.resize(size_t(width) * size_t(height) * 3);
    for (size_t i = 0; i < size_t(width) * size_t(height); ++i) {
      rgb[i * 3 + 0] = texture_data[i].x;
      rgb[i * 3 + 1] = texture_data[i].y;
      rgb[i * 3 + 2] = texture_data[i].z;
    }
    return true;
  }

  void serialize(agent::JsonWriter& writer) const override;

  static constexpr uint32_t width = 128, height = 64;
  static constexpr uint8_t ram_pages = 9, ram_columns = 132;

private:
  void process_byte(const bool is_data, const uint8_t value);
  void render();

  pin_type a0_pin;
  uint8_t column_offset;
  bool rotate_180;
  pin_type beeper_pin, enc1_pin, enc2_pin, enc_but_pin, back_pin, kill_pin;

  // Display RAM and controller state
  uint8_t ram[ram_pages][ram_columns] = {};
  uint8_t page = 0, column = 0, start_line = 0;
  uint8_t pending_argument = 0;   // Second byte of a two-byte command still to come
  bool display_on = false, inverted = false, all_pixels_on = false;
  uint32_t command_bytes = 0, data_bytes = 0;

  bool key_pressed[KeyName::COUNT] = {};
  uint8_t encoder_position = 0;
  static constexpr int8_t encoder_table[4] = {1, 3, 2, 0};

  bool render_integer_scaling = false, render_popout = false;
  bool dirty = true;
  std::chrono::steady_clock clock;
  std::chrono::steady_clock::time_point last_update;
  GLuint texture_id;
  glm::vec<3, uint8_t> texture_data[width * height] = {};

  glm::ivec3 foreground_color = {0x81, 0xF2, 0xFF};
  glm::ivec3 background_color = {0x33, 0x01, 0xFC};

  // RGB backlight on a negative panel: lit pixels let the backlight through, the rest stay dark
  std::function<bool(glm::vec3&)> backlight;
  glm::vec3 backlight_color = {0, 0, 0};
  static constexpr float dark_glass = 0x10 / 255.0f, off_leakage = 0.06f;
};

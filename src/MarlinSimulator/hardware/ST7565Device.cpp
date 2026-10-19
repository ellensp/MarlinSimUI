#include <cmath>
#include <cstring>
#include "Gpio.h"

#include <gl.h>
#include "../renderer/renderer.h"
#include "../renderer/imgui_sharp_bilinear.h"
#include "../imgui_custom.h"

#include "ST7565Device.h"

ST7565Device::ST7565Device(SpiBus& spi_bus, pin_type cs, pin_type a0, uint8_t column_offset, bool rotate_180, pin_type beeper, pin_type enc1, pin_type enc2, pin_type enc_but, pin_type back, pin_type kill)
  : SPISlavePeripheral(spi_bus, cs), a0_pin(a0), column_offset(column_offset), rotate_180(rotate_180), beeper_pin(beeper), enc1_pin(enc1), enc2_pin(enc2), enc_but_pin(enc_but), back_pin(back), kill_pin(kill) {

  Gpio::attach(beeper_pin, [this](GpioEvent& event){ this->interrupt(event); });
  Gpio::attach(enc1_pin, [this](GpioEvent& event){ this->interrupt(event); });
  Gpio::attach(enc2_pin, [this](GpioEvent& event){ this->interrupt(event); });
  Gpio::attach(enc_but_pin, [this](GpioEvent& event){ this->interrupt(event); });
  Gpio::attach(back_pin, [this](GpioEvent& event){ this->interrupt(event); });
  Gpio::attach(kill_pin, [this](GpioEvent& event){ this->interrupt(event); });
}

ST7565Device::~ST7565Device() {}

// One byte from the SPI bus while CS is low
void ST7565Device::onByteReceived(uint8_t _byte) {
  SPISlavePeripheral::onByteReceived(_byte);
  process_byte(Gpio::get_pin_value(a0_pin), _byte);
}

void ST7565Device::serialize(agent::JsonWriter& writer) const {
  writer.begin_object();
  writer.member("display_on", display_on);
  writer.member("inverted", inverted);
  writer.member("all_pixels_on", all_pixels_on);
  writer.member("page", int64_t(page));
  writer.member("column", int64_t(column));
  writer.member("start_line", int64_t(start_line));
  writer.member("command_bytes", int64_t(command_bytes));
  writer.member("data_bytes", int64_t(data_bytes));
  if (backlight) {
    writer.key("backlight");
    writer.begin_array();
    for (int i = 0; i < 3; i++) writer.value(int64_t(std::lround(backlight_color[i] * 255)));
    writer.end_array();
  }
  writer.end_object();
}

void ST7565Device::process_byte(const bool is_data, const uint8_t value) {
  if (is_data) data_bytes++; else command_bytes++;
  if (is_data) {
    if (page < ram_pages && column < ram_columns) {
      ram[page][column] = value;
      dirty = true;
    }
    if (column < ram_columns) column++;
    return;
  }

  if (pending_argument) {             // Argument of a two-byte command (contrast, booster, indicator)
    pending_argument = 0;
    return;
  }

  switch (value) {
    case 0x00 ... 0x0F: column = (column & 0xF0) | (value & 0x0F); break;          // Column address, low nibble
    case 0x10 ... 0x1F: column = ((value & 0x0F) << 4) | (column & 0x0F); break;   // Column address, high nibble
    case 0x40 ... 0x7F: start_line = value & 0x3F; break;                           // Display start line (panel wiring)
    case 0xA4: case 0xA5: all_pixels_on = value & 1; dirty = true; break;           // All pixels on
    case 0xA6: case 0xA7: inverted = value & 1; dirty = true; break;                // Normal / inverted
    case 0xAE: case 0xAF: display_on = value & 1; dirty = true; break;              // Display off / on
    case 0xB0 ... 0xBF: page = value & 0x0F; break;                                 // Page address
    case 0xE2:                                                                      // Software reset (RAM is kept)
      page = column = start_line = 0;
      inverted = all_pixels_on = false;
      dirty = true;
      break;
    case 0x81: case 0xAC: case 0xAD: case 0xF8: case 0xFA:                          // Two-byte commands
      pending_argument = value;
      break;
    default: break;   // Power, bias, regulator ratio, segment/common direction, read-modify-write, NOP
  }
}

void ST7565Device::render() {
  glm::ivec3 on_color = foreground_color, off_color = background_color;
  if (backlight) {
    on_color = glm::ivec3(glm::round(backlight_color * 255.0f));
    off_color = glm::ivec3(glm::round((dark_glass + backlight_color * off_leakage) * 255.0f));
  }
  for (uint32_t y = 0; y < height; y++) {
    const uint8_t *row_page = ram[y / 8];
    const uint8_t bit = 1 << (y % 8);
    for (uint32_t x = 0; x < width; x++) {
      const uint32_t col = x + column_offset;
      bool pixel = all_pixels_on || (col < ram_columns && (row_page[col] & bit));
      if (inverted) pixel = !pixel;
      const uint32_t index = rotate_180 ? (height - 1 - y) * width + (width - 1 - x) : y * width + x;
      texture_data[index] = (display_on && pixel) ? on_color : off_color;
    }
  }
}

void ST7565Device::update() {
  auto now = clock.now();
  float delta = std::chrono::duration_cast<std::chrono::duration<float>>(now - last_update).count();

  if (backlight) {
    glm::vec3 color(0.0f);              // Dark until the LED has been set
    backlight(color);
    if (color != backlight_color) { backlight_color = color; dirty = true; }
  }

  if (dirty && delta > 1.0 / 30.0) {
    last_update = now;
    dirty = false;
    render();
    renderer::gl_assert_call(glBindTexture, GL_TEXTURE_2D, texture_id);
    renderer::gl_assert_call(glTexImage2D, GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, texture_data);
    renderer::gl_assert_call(glBindTexture, GL_TEXTURE_2D, 0);
  }
}

void ST7565Device::interrupt(GpioEvent& ev) {
  if (ev.pin_id == beeper_pin) {
    // Sound is not modelled
  } else if (ev.pin_id == kill_pin) {
    Gpio::set_pin_value(kill_pin,  !key_pressed[KeyName::KILL_BUTTON]);
  } else if (ev.pin_id == enc_but_pin) {
    Gpio::set_pin_value(enc_but_pin,  !key_pressed[KeyName::ENCODER_BUTTON]);
  } else if (ev.pin_id == back_pin) {
    Gpio::set_pin_value(back_pin,  !key_pressed[KeyName::BACK_BUTTON]);
  } else if (ev.pin_id == enc1_pin || ev.pin_id == enc2_pin) {
    const uint8_t encoder_state = encoder_position % 4;
    Gpio::set_pin_value(enc1_pin,  encoder_table[encoder_state] & 0x01);
    Gpio::set_pin_value(enc2_pin,  encoder_table[encoder_state] & 0x02);
  }
}

void ST7565Device::ui_init() {
  renderer::gl_assert_call(glGenTextures, 1, &texture_id);
  renderer::gl_assert_call(glBindTexture, GL_TEXTURE_2D, texture_id);
  renderer::gl_assert_call(glTexParameteri, GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  renderer::gl_assert_call(glTexParameteri, GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  renderer::gl_assert_call(glBindTexture, GL_TEXTURE_2D, 0);
  renderer::imgui_sharp_bilinear::init(texture_id, render_integer_scaling);
}

void ST7565Device::ui_widget() {
  static uint8_t up_held = 0, down_held = 0;
  bool popout_begin = false;

  auto size = ImGui::GetContentRegionAvail();
  size.y = ((size.x / (width / (float)height)) * !render_popout) + 60;

  if (ImGui::BeginChild("ST7565Device", size)) {
    ImGui::GetCurrentWindow()->ScrollMax.y = 1.0f; // disable window scroll
    if (ImGui::Checkbox("Integer Scaling", &render_integer_scaling))
      renderer::imgui_sharp_bilinear::set_texture_filter(texture_id, render_integer_scaling);
    ImGui::Checkbox("Popout", &render_popout);

    if (render_popout) {
      const imgui_custom::constraint_t constraint { width + imgui_custom::hfeat, height + imgui_custom::vfeat, (width) / (float)(height) };
      ImGui::SetNextWindowSize(ImVec2(constraint.minw + width, constraint.minh + height), ImGuiCond_Once);
      ImGui::SetNextWindowSizeConstraints(ImVec2(0, 0), ImVec2(FLT_MAX, FLT_MAX), imgui_custom::CustomConstraints::AspectRatio, (void*)&constraint);
      popout_begin = ImGui::Begin("ST7565DeviceRender", &render_popout);
      if (!popout_begin)
        ImGui::End();   // Collapsed: end the window but stay inside BeginChild (see ST7920Device)
      else
        size = ImGui::GetContentRegionAvail();
    }

    if (!render_popout || popout_begin) {
      size = imgui_custom::scale_proportionally(size, width, height, render_integer_scaling);

      const ImVec2 screen_pos = ImGui::GetCursorScreenPos();
      renderer::imgui_sharp_bilinear::image(texture_id, size, render_integer_scaling);

      // Claim the screen rect so a press-hold on it acts as the encoder button
      bool screen_held = false;
      if (size.x > 0.0f && size.y > 0.0f) {
        ImGui::SetCursorScreenPos(screen_pos);
        ImGui::InvisibleButton("##screen", size, ImGuiButtonFlags_MouseButtonLeft);
        screen_held = ImGui::IsItemActive();
      }

      if (ImGui::IsWindowFocused()) {
        key_pressed[KeyName::KILL_BUTTON]    = ImGui::IsKeyDown(ImGuiKey_K);
        key_pressed[KeyName::ENCODER_BUTTON] = ImGui::IsKeyDown(ImGuiKey_Space) || ImGui::IsKeyDown(ImGuiKey_Enter) || ImGui::IsKeyDown(ImGuiKey_RightArrow);
        key_pressed[KeyName::BACK_BUTTON]    = ImGui::IsKeyDown(ImGuiKey_LeftArrow);

        // Turn keypresses (and repeat) into encoder clicks
        if (up_held) { up_held--; encoder_position--; }
        else if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) up_held = 4;
        if (down_held) { down_held--; encoder_position++; }
        else if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) down_held = 4;

        if (ImGui::IsItemHovered())
          encoder_position += ImGui::GetIO().MouseWheel > 0 ? 1 : ImGui::GetIO().MouseWheel < 0 ? -1 : 0;

        key_pressed[KeyName::ENCODER_BUTTON] |= screen_held;
      }

      if (popout_begin) ImGui::End();
    }
  }
  ImGui::EndChild();
}

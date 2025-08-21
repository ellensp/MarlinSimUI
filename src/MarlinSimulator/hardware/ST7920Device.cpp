#include <mutex>
#include <fstream>
#include <cmath>
#include <random>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <iostream>
#include "Gpio.h"

#include <gl.h>
#include "../renderer/renderer.h"
#include "../imgui_custom.h"

#include "ST7920Device.h"

ST7920Device::ST7920Device(pin_type clk, pin_type mosi, pin_type cs, pin_type beeper, pin_type enc1, pin_type enc2, pin_type enc_but, pin_type back, pin_type kill)
  : clk_pin(clk), mosi_pin(mosi), cs_pin(cs), beeper_pin(beeper), enc1_pin(enc1), enc2_pin(enc2), enc_but_pin(enc_but), back_pin(back), kill_pin(kill) {

  Gpio::attach(clk_pin, [this](GpioEvent& event){ this->interrupt(event); });
  Gpio::attach(cs_pin, [this](GpioEvent& event){ this->interrupt(event); });
  Gpio::attach(beeper_pin, [this](GpioEvent& event){ this->interrupt(event); });
  Gpio::attach(enc1_pin, [this](GpioEvent& event){ this->interrupt(event); });
  Gpio::attach(enc2_pin, [this](GpioEvent& event){ this->interrupt(event); });
  Gpio::attach(enc_but_pin, [this](GpioEvent& event){ this->interrupt(event); });
  Gpio::attach(back_pin, [this](GpioEvent& event){ this->interrupt(event); });
  Gpio::attach(kill_pin, [this](GpioEvent& event){ this->interrupt(event); });
}

ST7920Device::~ST7920Device() {}

void ST7920Device::process_command(Command cmd) {
  if (cmd.rs) {
    graphic_ram[coordinate[1] + (coordinate[0] * (256 / 8))] = cmd.data;
    if (++coordinate[1] > 32) coordinate[1] = 0;
    dirty = true;
  }
  else if (extended_instruction_set) {
    if (cmd.data & (1 << 7)) {
      // cmd [7] SET GRAPHICS RAM COORDINATE
      coordinate[coordinate_index++] = cmd.data & 0x7F;
      if(coordinate_index == 2) {
        coordinate_index = 0;
        coordinate[1] *= 2;
        if (coordinate[1] >= 128 / 8) {
          coordinate[1] = 0;
          coordinate[0] += 32;
        }
      }
    } else if (cmd.data & (1 << 6)) {
      //printf("cmd: [6] SET IRAM OR SCROLL ADDRESS (0x%02X)\n", cmd.data);
    } else if (cmd.data & (1 << 5)) {
      extended_instruction_set = cmd.data & 0b100;
      //printf("cmd: [5] EXTENDED FUNCTION SET (0x%02X)\n", cmd.data);
    } else if (cmd.data & (1 << 4)) {
      //printf("cmd: [4] UNKNOWN? (0x%02X)\n", cmd.data);
    } else if (cmd.data & (1 << 3)) {
      //printf("cmd: [3] DISPLAY STATUS (0x%02X)\n", cmd.data);
    } else if (cmd.data & (1 << 2)) {
      //printf("cmd: [2] REVERSE (0x%02X)\n", cmd.data);
    } else if (cmd.data & (1 << 1)) {
      //printf("cmd: [1] VERTICAL SCROLL OR RAM ADDRESS SELECT (0x%02X)\n", cmd.data);
    } else if (cmd.data & (1 << 0)) {
      //printf("cmd: [0] STAND BY\n");
    }
  } else {
    if (cmd.data & (1 << 7)) {
      //printf("cmd: [7] SET DDRAM ADDRESS (0x%02X)\n", cmd.data);
    } else if (cmd.data & (1 << 6)) {
      //printf("cmd: [6] SET CGRAM ADDRESS (0x%02X)\n", cmd.data);
    } else if (cmd.data & (1 << 5)) {
      extended_instruction_set = cmd.data & 0b100;
      //printf("cmd: [5] FUNCTION SET (0x%02X)\n", cmd.data);
    } else if (cmd.data & (1 << 4)) {
      //printf("cmd: [4] CURSOR DISPLAY CONTROL (0x%02X)\n", cmd.data);
    } else if (cmd.data & (1 << 3)) {
      //printf("cmd: [3] DISPLAY CONTROL (0x%02X)\n", cmd.data);
    } else if (cmd.data & (1 << 2)) {
      address_increment = cmd.data & 0x1;
      //printf("cmd: [2] ENTRY MODE SET (0x%02X)\n", cmd.data);
    } else if (cmd.data & (1 << 1)) {
      //printf("cmd: [1] RETURN HOME (0x%02X)\n", cmd.data);
    } else if (cmd.data & (1 << 0)) {
      //printf("cmd: [0] DISPLAY CLEAR\n");
    }
  }
}

void ST7920Device::update() {
  auto now = clock.now();
  // Check capture triggers before texture update
  check_capture_triggers();

  float delta = std::chrono::duration_cast<std::chrono::duration<float>>(now - last_update).count();

  if (dirty && delta > 1.0 / 30.0) {
    last_update = now;
    dirty = false;
    for (std::size_t x = 0; x < 128; x += 8) {                 // ST7920 graphics ram has 256 bit horizontal resolution, the second 128bit is unused
      for (std::size_t y = 0; y < 64; y++) {                   // 64 bit vertical resolution
        std::size_t texture_index = (y * 128) + x;             // indexed by pixel coordinate
        std::size_t graphic_ram_index = (y * 32) + (x / 8);    // indexed by byte (8 horizontal pixels), 32 byte (256 pixel) stride per row
        for (std::size_t j = 0; j < 8; j++) {
          texture_data[texture_index + j] = TEST(graphic_ram[graphic_ram_index], 7 - j) ? foreground_color :  background_color;
        }
      }
    }
    renderer::gl_assert_call(glBindTexture, GL_TEXTURE_2D, texture_id);
    renderer::gl_assert_call(glTexImage2D, GL_TEXTURE_2D, 0, GL_RGBA8, 128, 64, 0, GL_RGB, GL_UNSIGNED_BYTE, texture_data);
    renderer::gl_assert_call(glBindTexture, GL_TEXTURE_2D, 0);
  }
}

void ST7920Device::interrupt(GpioEvent& ev) {
  if (ev.pin_id == clk_pin && ev.event == GpioEvent::FALL && Gpio::get_pin_value(cs_pin)){
    incoming_byte = (incoming_byte << 1) | Gpio::get_pin_value(mosi_pin);
    if (++incoming_bit_count == 8) {
      if (incoming_byte_count == 0 && (incoming_byte & 0xF8) != 0xF8) {
        incoming_byte_count++;
      }
      incoming_cmd[incoming_byte_count++] = incoming_byte;
      incoming_byte = incoming_bit_count = 0;
      if (incoming_byte_count == 3) {
        process_command({(incoming_cmd[0] & 0b100) != 0, (incoming_cmd[0] & 0b010) != 0, uint8_t(incoming_cmd[1] | incoming_cmd[2] >> 4)});
        incoming_byte_count = 0;
      }
    }
  } else if (ev.pin_id == cs_pin && ev.event == GpioEvent::RISE) {
    incoming_bit_count = incoming_byte_count = incoming_byte = 0;
  } else if (ev.pin_id == beeper_pin) {
    if (ev.event == GpioEvent::RISE) {
      // play sound
    } else if (ev.event == GpioEvent::FALL) {
      // stop sound
    }
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

void ST7920Device::ui_init() {
  renderer::gl_assert_call(glGenTextures, 1, &texture_id);
  renderer::gl_assert_call(glBindTexture, GL_TEXTURE_2D, texture_id);
  renderer::gl_assert_call(glTexParameteri, GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  renderer::gl_assert_call(glTexParameteri, GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  renderer::gl_assert_call(glTexParameteri, GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  renderer::gl_assert_call(glTexParameteri, GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  renderer::gl_assert_call(glBindTexture, GL_TEXTURE_2D, 0);
}

void ST7920Device::ui_widget() {
  static long int call_count = 0;
  static uint8_t up_held = 0, down_held = 0;
  call_count++;
  bool popout_begin = false;

  auto size = ImGui::GetContentRegionAvail();
  size.y = ((size.x / (width / (float)height)) * !render_popout) + 60;

  if (ImGui::BeginChild("ST7920Device", size)) {
    ImGui::GetCurrentWindow()->ScrollMax.y = 1.0f; // disable window scroll
    ImGui::Checkbox("Integer Scaling", &render_integer_scaling);
    ImGui::Checkbox("Popout", &render_popout);

    if (render_popout) {
      const imgui_custom::constraint_t constraint { width + imgui_custom::hfeat, height + imgui_custom::vfeat, (width) / (float)(height) };

      // Init the window size to contain the 2x scaled screen, margin, and window features
      ImGui::SetNextWindowSize(ImVec2(constraint.minw + width, constraint.minh + height), ImGuiCond_Once);
      ImGui::SetNextWindowSizeConstraints(ImVec2(0, 0), ImVec2(FLT_MAX, FLT_MAX), imgui_custom::CustomConstraints::AspectRatio, (void*)&constraint);

      popout_begin = ImGui::Begin("ST7920DeviceRender", &render_popout);
      if (!popout_begin) {
        ImGui::End();
        return;
      }
      size = ImGui::GetContentRegionAvail();
    }

    // Apply the smallest scale that fits the window. Maintain proportions.
    size = imgui_custom::scale_proportionally(size, width, height, render_integer_scaling);

    ImGui::Image((ImTextureID)(intptr_t)texture_id, size, ImVec2(0,0), ImVec2(1,1));
    if (ImGui::IsWindowFocused()) {
      key_pressed[KeyName::KILL_BUTTON]    = ImGui::IsKeyDown(ImGuiKey_K);
      key_pressed[KeyName::ENCODER_BUTTON] = ImGui::IsKeyDown(ImGuiKey_Space) || ImGui::IsKeyDown(ImGuiKey_Enter) || ImGui::IsKeyDown(ImGuiKey_RightArrow);
      key_pressed[KeyName::BACK_BUTTON]    = ImGui::IsKeyDown(ImGuiKey_LeftArrow);

      // Turn keypresses (and repeat) into encoder clicks
      if (up_held) { up_held--; encoder_position--; }
      else if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) up_held = 4;
      if (down_held) { down_held--; encoder_position++; }
      else if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) down_held = 4;

      if (ImGui::IsItemHovered()) {
        key_pressed[KeyName::ENCODER_BUTTON] |= ImGui::IsMouseClicked(0);
        encoder_position += ImGui::GetIO().MouseWheel > 0 ? 1 : ImGui::GetIO().MouseWheel < 0 ? -1 : 0;
      }
    }

    if (popout_begin) ImGui::End();
  }
  ImGui::EndChild();
}

void ST7920Device::save_lcd_capture(const std::string& prefix) {
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
  
  std::stringstream ss;
  ss << prefix << "_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") 
     << "_" << std::setfill('0') << std::setw(3) << ms.count() << ".pbm";
  
  const int width = 128;
  const int height = 64; 
  const int graphic_ram_size = 64 * 32;
  
  // Save the graphic RAM as PBM image
  std::ofstream file(ss.str());
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << ss.str() << std::endl;
    return;
  }
  
  // PBM header
  file << "P1\n";
  file << "# LCD Capture from MarlinSimulator ST7920Device\n";
  file << width << " " << height << "\n";
  
  // Write pixel data (ST7920 graphic RAM format)
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // ST7920 graphic RAM is organized as 64 rows x 32 bytes (256 bits) per row
      // But only first 128 bits (16 bytes) per row are used for 128x64 display
      int ram_y = y;
      int ram_x = x;
      int byte_index = ram_y * 32 + (ram_x / 8); // 32 bytes per row in ST7920
      int bit_index = 7 - (ram_x % 8);
      
      if (byte_index < graphic_ram_size) {
        int pixel = (graphic_ram[byte_index] >> bit_index) & 1;
        file << (pixel ? "1" : "0") << " ";
      } else {
        file << "0 ";
      }
    }
    file << "\n";
  }
  
  file.close();
  std::cout << "LCD buffer saved: " << ss.str() << std::endl;
}

void ST7920Device::check_capture_triggers() {
  static auto start_time = std::chrono::steady_clock::now();
  static auto last_timer_capture = std::chrono::steady_clock::now();
  static auto last_periodic_capture = std::chrono::steady_clock::now();
  static bool first_update_captured = false;
  static bool boot_capture_done = false;
  static bool status_capture_done = false;
  
  auto current_time = std::chrono::steady_clock::now();
  auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
  
  // Capture first LCD update immediately  
  if (!first_update_captured) {
    save_lcd_capture("u8g2_first_update");
    first_update_captured = true;
    std::cout << "Auto-captured FIRST LCD UPDATE at " << elapsed_seconds << " seconds" << std::endl;
  }
  
  // Timer-based captures every 1 second for first 30 seconds
  if (elapsed_seconds <= 30) {
    auto timer_elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_timer_capture).count();
    if (timer_elapsed >= 1) {
      std::string prefix = "u8g2_timer_" + std::to_string(elapsed_seconds) + "s";
      save_lcd_capture(prefix);
      last_timer_capture = current_time;
      std::cout << "Timer-based capture " << elapsed_seconds << "s at " << 
        std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time).count() << " seconds" << std::endl;
    }
  }
  
  // Capture bootscreen transition at specific time (typically during 10s bootscreen)
  if (!boot_capture_done && elapsed_seconds >= 23.0) {
    save_lcd_capture("u8g2_23s_bootscreen");
    boot_capture_done = true;
    std::cout << "Auto-captured U8G2 boot screen at " << elapsed_seconds << " seconds (during bootscreen period)" << std::endl;
  }
  
  // Capture status screen at 35 seconds (well after 10s bootscreen timeout)
  if (!status_capture_done && elapsed_seconds >= 35.0) {
    save_lcd_capture("u8g2_35s_status");
    status_capture_done = true;
    std::cout << "Auto-captured U8G2 status screen at " << elapsed_seconds << " seconds (after bootscreen timeout)" << std::endl;
  }
  
  // Periodic captures every 10 seconds after status capture
  if (status_capture_done) {
    auto periodic_elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_periodic_capture).count();
    if (periodic_elapsed >= 10) {
      save_lcd_capture("periodic");
      last_periodic_capture = current_time;
      std::cout << "Auto-captured periodic screen at " << elapsed_seconds << " seconds" << std::endl;
    }
  }
  
  // Additional captures for font bug analysis at key transition points
  static bool capture_20s_done = false, capture_21s_done = false, capture_22s_done = false, capture_25s_done = false, capture_30s_done = false, capture_33s_done = false;
  
  // Granular captures every second from 0-20 seconds to catch bootscreen appearance
  static bool captures_0_20_done[21] = {false}; // 0, 1, 2, ... 20 seconds
  
  for (int sec = 0; sec <= 20; sec++) {
    if (!captures_0_20_done[sec] && elapsed_seconds >= sec) {
      std::string prefix = "u8g2_" + std::to_string(sec) + "s_scan";
      save_lcd_capture(prefix);
      captures_0_20_done[sec] = true;
      std::cout << "Auto-captured " << sec << "s scan at " << 
        std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time).count() << " seconds" << std::endl;
    }
  }
  
  // Capture as soon as LCD becomes active (first dirty update around 20s)
  if (!capture_20s_done && elapsed_seconds >= 20.0) {
    save_lcd_capture("u8g2_20s_first");
    capture_20s_done = true;
    std::cout << "Auto-captured first LCD update at " << elapsed_seconds << " seconds" << std::endl;
  }
  
  if (!capture_21s_done && elapsed_seconds >= 21.0) {
    save_lcd_capture("u8g2_21s_early");
    capture_21s_done = true;
    std::cout << "Auto-captured early transition at " << elapsed_seconds << " seconds" << std::endl;
  }
  
  if (!capture_22s_done && elapsed_seconds >= 22.0) {
    save_lcd_capture("u8g2_22s_transition");
    capture_22s_done = true;
    std::cout << "Auto-captured transition point at " << elapsed_seconds << " seconds" << std::endl;
  }
  
  if (!capture_25s_done && elapsed_seconds >= 25.0) {
    save_lcd_capture("u8g2_25s_mid");
    capture_25s_done = true;
    std::cout << "Auto-captured mid-period at " << elapsed_seconds << " seconds" << std::endl;
  }
  
  if (!capture_30s_done && elapsed_seconds >= 30.0) {
    save_lcd_capture("u8g2_30s_late");
    capture_30s_done = true;
    std::cout << "Auto-captured late transition at " << elapsed_seconds << " seconds" << std::endl;
  }
  
  if (!capture_33s_done && elapsed_seconds >= 33.0) {
    save_lcd_capture("u8g2_33s_final");
    capture_33s_done = true;
    std::cout << "Auto-captured final check at " << elapsed_seconds << " seconds" << std::endl;
  }
}

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <imgui.h>

#include "FilamentChanger.h"
#include "../agent/json.h"
#include "../execution_control.h"

#include <src/inc/MarlinConfig.h>

FilamentChanger* FilamentChanger::instance = nullptr;

static const glm::vec3 no_filament_color { 0.3f, 0.3f, 0.3f };

FilamentChanger::FilamentChanger(const char* model, size_t slot_count)
  : VirtualPrinter::Component("FilamentChanger"), model(model), slot_count(std::min(slot_count, max_slots)) {
  static const glm::vec3 defaults[max_slots] = {
    {1.0f, 0.0f, 0.0f}, {0.0f, 0.6f, 0.0f}, {0.0f, 0.2f, 1.0f}, {1.0f, 0.9f, 0.0f},
    {1.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 1.0f}, {1.0f, 0.5f, 0.0f}, {1.0f, 1.0f, 1.0f}
  };
  for (size_t i = 0; i < max_slots; ++i) slot_color[i] = defaults[i];
  instance = this;
}

glm::vec3 FilamentChanger::current_color() const {
  return loaded >= 0 ? slot_color[loaded] : no_filament_color;
}

void FilamentChanger::apply_colors(kinematic_state& state) {
  if (!instance) return;
  std::scoped_lock lock(instance->mutex);
  const glm::vec3 color = instance->current_color();
  for (auto& effector : state.effector_position) effector.color = color;
}

// Hold-time to command, and the whole device, as seen through the button
void FilamentChanger::select(const int tool, const bool only_unload) {
  selected = tool;
  unload_only = only_unload;
  phase = (loaded >= 0) ? Phase::AwaitUnload : (only_unload ? Phase::Ready : Phase::AwaitLoad);
  if (phase == Phase::AwaitUnload) status = "T" + std::to_string(loaded) + " cut. Press to unload.";
  else if (phase == Phase::AwaitLoad) status = "T" + std::to_string(tool) + " selected. Press to load.";
  else status = "Nothing to unload.";
}

void FilamentChanger::log_event(const std::string& text) {
  history.push_back(text);
  while (history.size() > 12) history.pop_front();
}

void FilamentChanger::process_hold(const uint32_t hold_ms) {
  last_hold_ms = hold_ms;

  switch (phase) {
    case Phase::Ready: {
      const int n = pulse_ms > 0 ? int(hold_ms / pulse_ms) : 0;
      if (n >= 1 && n <= 4) {
        if (size_t(n) > slot_count) status = "No filament T" + std::to_string(n - 1);
        else select(n - 1);
      }
      else if (n == 5) { loaded = -1; select(0); }                                   // Home and load T0
      else if (n == 6) { if (loaded >= 0) select(loaded, true); else status = "Nothing to unload."; }
      else if (n == 7) { loaded = -1; status = "Homed."; }                          // Forget what was loaded
      else if (n == 8) select(loaded < 0 ? 0 : int((loaded + 1) % slot_count));
      else if (n == 9) select(int(rand() % slot_count));
      else status = "No command (" + std::to_string(n) + " pulses).";
    } break;

    case Phase::AwaitUnload:
      if (int(hold_ms) < min_feed_ms) status = "Unload too short. Press again.";
      else {
        loaded = -1;
        if (unload_only) { phase = Phase::Ready; status = "Unloaded."; }
        else { phase = Phase::AwaitLoad; status = "Unloaded. T" + std::to_string(selected) + " selected. Press to load."; }
      }
      break;

    case Phase::AwaitLoad:
      if (int(hold_ms) < min_feed_ms) status = "Load too short. Press again.";
      else {
        loaded = selected;
        phase = Phase::Ready;
        status = "T" + std::to_string(loaded) + " loaded.";
      }
      break;
  }
  log_event(std::to_string(hold_ms) + " ms: " + status);
}

void FilamentChanger::button_changed(const bool now_pressed, const uint64_t now_ns) {
  if (now_pressed == pressed) return;
  pressed = now_pressed;
  log_event(std::string(pressed ? "down @" : "up   @") + std::to_string(now_ns / 1000000ULL) + " ms axis " + std::to_string(axis_position));
  if (pressed) { press_start_ns = now_ns; return; }
  process_hold(uint32_t((now_ns - press_start_ns) / 1000000ULL));
}

void FilamentChanger::observe(const glm::vec3& marlin_position) {
  if (!instance) return;
  #if HAS_CHAMELEON
    // The button is pressed once the axis is past the middle of its travel toward the button position.
    constexpr float axis_min[] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS }, axis_max[] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
    constexpr int axis = int(CHAMELEON_BUTTON_AXIS);
    constexpr bool at_max_end = (CHAMELEON_BUTTON_POS) > (axis_min[axis] + axis_max[axis]) * 0.5f;
    constexpr float threshold = (CHAMELEON_BUTTON_POS) + (at_max_end ? -0.5f : 0.5f) * (CHAMELEON_PRESS_MM);
    const float p = marlin_position[axis];
    const bool now_pressed = at_max_end ? p >= threshold : p <= threshold;
    std::scoped_lock lock(instance->mutex);
    instance->axis_position = p;
    instance->button_changed(now_pressed, Kernel::TimeControl::nanos());
  #else
    (void)marlin_position;
  #endif
}

void FilamentChanger::ui_widget() {
  std::scoped_lock lock(mutex);
  ImGui::Text("%s (%d filaments)", model.c_str(), int(slot_count));
  ImGui::Text("Button: %s   Last hold: %u ms   Axis: %.2f", pressed ? "PRESSED" : "released", last_hold_ms, axis_position);
  ImGui::Text("Loaded: %s", loaded >= 0 ? ("T" + std::to_string(loaded)).c_str() : "none");
  ImGui::TextWrapped("%s", status.c_str());
  if (ImGui::TreeNode("Button history")) {
    for (auto it = history.rbegin(); it != history.rend(); ++it) ImGui::TextUnformatted(it->c_str());
    ImGui::TreePop();
  }
  ImGui::SeparatorText("Filament colors");
  ImGui::TextDisabled("Click a color to change it. White border: loaded. Yellow: selected.");
  const ImVec2 swatch(40, 40);
  for (size_t i = 0; i < slot_count; ++i) {
    ImGui::PushID(int(i));
    ImGui::BeginGroup();
    const ImVec4 col(slot_color[i].r, slot_color[i].g, slot_color[i].b, 1.0f);
    if (ImGui::ColorButton("##swatch", col, ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoBorder, swatch))
      ImGui::OpenPopup("##picker");
    // Mark the active color with a border (and the one chosen to load next with a thin one)
    const bool is_loaded = int(i) == loaded, is_selected = int(i) == selected && !is_loaded && phase == Phase::AwaitLoad;
    if (is_loaded || is_selected) {
      const ImVec2 lo = ImGui::GetItemRectMin(), hi = ImGui::GetItemRectMax();
      ImGui::GetWindowDrawList()->AddRect(ImVec2(lo.x - 3, lo.y - 3), ImVec2(hi.x + 3, hi.y + 3),
        is_loaded ? IM_COL32(255, 255, 255, 255) : IM_COL32(255, 220, 0, 255), 2.0f, 0, is_loaded ? 3.0f : 1.5f);
    }
    ImGui::Text(" T%d", int(i));
    ImGui::EndGroup();
    if (ImGui::BeginPopup("##picker")) {
      ImGui::Text("T%d color", int(i));
      ImGui::ColorPicker3("##color", &slot_color[i].x, ImGuiColorEditFlags_PickerHueWheel | ImGuiColorEditFlags_NoSidePreview);
      ImGui::EndPopup();
    }
    ImGui::PopID();
    if (i + 1 < slot_count) ImGui::SameLine(0, 12);
  }
  ImGui::SeparatorText("Timing");
  ImGui::InputInt("Pulse (ms)", &pulse_ms, 10, 100);
  ImGui::InputInt("Min feed press (ms)", &min_feed_ms, 100, 1000);
  if (ImGui::Button("Unload")) { loaded = -1; phase = Phase::Ready; status = "Unloaded (UI)."; }
  ImGui::SameLine();
  if (ImGui::Button("Reset")) { loaded = -1; selected = -1; phase = Phase::Ready; status = "Reset."; }
}

static std::string to_hex(const glm::vec3& c) {
  char buf[8];
  snprintf(buf, sizeof(buf), "#%02x%02x%02x", int(c.r * 255 + 0.5f), int(c.g * 255 + 0.5f), int(c.b * 255 + 0.5f));
  return buf;
}

void FilamentChanger::serialize(agent::JsonWriter& writer) const {
  std::scoped_lock lock(mutex);
  writer.begin_object();
  writer.member("model", model);
  writer.member("button_pressed", pressed);
  writer.member("loaded", int64_t(loaded));
  writer.member("selected", int64_t(selected));
  writer.member("last_hold_ms", int64_t(last_hold_ms));
  writer.member("axis_position", double(axis_position));
  writer.member("status", status);
  writer.key("history");
  writer.begin_array();
  for (auto const& h : history) writer.value(h);
  writer.end_array();
  writer.key("colors");
  writer.begin_array();
  for (size_t i = 0; i < slot_count; ++i) writer.value(to_hex(slot_color[i]));
  writer.end_array();
  writer.end_object();
}

// {"slot": 1, "color": "#ff8800"} sets the color of a filament
bool FilamentChanger::apply(const agent::JsonValue& body, std::string& error) {
  double slot;
  std::string hex;
  if (!body.get_double("slot", slot) || !body.get_string("color", hex)) {
    error = "'slot' (number) and 'color' (\"#rrggbb\") are required";
    return false;
  }
  unsigned rgb;
  if (slot < 0 || size_t(slot) >= slot_count || hex.size() != 7 || hex[0] != '#' || sscanf(hex.c_str() + 1, "%x", &rgb) != 1) {
    error = "slot out of range or color is not \"#rrggbb\"";
    return false;
  }
  std::scoped_lock lock(mutex);
  slot_color[size_t(slot)] = { ((rgb >> 16) & 0xFF) / 255.0f, ((rgb >> 8) & 0xFF) / 255.0f, (rgb & 0xFF) / 255.0f };
  return true;
}

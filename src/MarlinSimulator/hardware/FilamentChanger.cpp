#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <imgui.h>

#include "FilamentChanger.h"
#include "../agent/json.h"
#include "FilamentRunoutSensor.h"
#include "../execution_control.h"

#include <src/inc/MarlinConfig.h>

FilamentChanger* FilamentChanger::instance = nullptr;

//
// The Prusa MMU2 as Marlin's mmu2.cpp sees it: text lines in, "ok" lines out.
//   X0 reset -> "start"    S1/S2 -> version/build + "ok"   P0 -> FINDA (1 if filament) + "ok"
//   T<n> load filament n   U0/E<n> unload                  L, C0, R0, F, M1 -> "ok"
//
class Mmu2Device : public FauxSerialDevice {
public:
  Mmu2Device(FilamentChanger& changer, const int stream) : FauxSerialDevice(stream), changer(changer) {}

protected:
  void on_line(const std::string& line) override {
    if (line.empty()) return;
    const char c = line[0];
    const int arg = line.size() > 1 ? atoi(line.c_str() + 1) : 0;
    if (line == "X0") { changer.note("MMU reset"); reply("start\n", 200); }
    else if (line == "S1") reply("9ok\n");
    else if (line == "S2") reply(ENABLED(MMU2_MODE_12V) ? "132ok\n" : "126ok\n");
    else if (line == "P0") { std::scoped_lock lock(changer.mutex); reply(std::string(changer.loaded >= 0 ? "1" : "0") + "ok\n"); }
    else if (c == 'T' || c == 'L') {
      changer.note("MMU <= " + line);
      reply("ok\n", changer.command_ms, [this, arg]{ changer.mmu_set_loaded(arg); });
    }
    else if (c == 'U' || c == 'E') {
      changer.note("MMU <= " + line);
      reply("ok\n", changer.command_ms, [this]{ changer.mmu_set_loaded(-1); });
    }
    else if (c == 'C' || c == 'R') reply("ok\n", changer.command_ms);
    else if (c == 'F' || c == 'M') reply("ok\n");
    // 'A' (abort) has no reply
  }

private:
  FilamentChanger& changer;
};


#if HAS_PRUSA_MMU3

#include <map>
#include <vector>
#include <src/feature/mmu3/mmu3_protocol.h>

//
// The Prusa MMU3 as Marlin's mmu3 code sees it. Requests are decoded and responses encoded
// with Marlin's own Protocol class, so the framing and CRCs are exactly what the MMU firmware makes.
//
// A command (T, L, U, E, K, ...) is accepted at once, then reports progress codes to Q0 queries over
// time and finally "Finished". The MMU works together with the printer's filament sensor: while
// feeding to the sensor the filament arrives, the printer sees it and tells the MMU with "f1",
// and only then does the MMU go on to finish. The simulated filament runout sensor follows the filament.
//
class Mmu3Device : public FauxSerialDevice {
  using Code = modules::protocol::RequestMsgCodes;
  using Param = modules::protocol::ResponseMsgParamCodes;
  using RequestMsg = modules::protocol::RequestMsg;
  using Protocol = modules::protocol::Protocol;
  using DecodeStatus = modules::protocol::DecodeStatus;
  using ResponseCommandStatus = modules::protocol::ResponseCommandStatus;
  using Progress = uint8_t;

public:
  Mmu3Device(FilamentChanger& changer, const int stream) : FauxSerialDevice(stream), changer(changer) {}

protected:
  void on_bytes(const uint8_t* data, const size_t count) override {
    for (size_t i = 0; i < count; ++i) {
      if (protocol.DecodeRequest(data[i]) == DecodeStatus::MessageCompleted) {
        const RequestMsg rq = protocol.GetRequestMsg();
        if (rq.code != Code::unknown) handle(rq);
      }
    }
  }

  void update() override {
    if (!sensor_initialized) { set_sensor(false); sensor_initialized = true; }
    if (!busy) return;
    const uint64_t now = Kernel::TimeControl::getTicks();
    while (busy) {
      const Phase& phase = phases[phase_index];
      const uint64_t elapsed_ms = Kernel::TimeControl::ticksToNanos(now - phase_start) / Kernel::TimeControl::ONE_MILLION;
      if (phase.action == Action::WaitSensor) {
        if (elapsed_ms >= phase.ms) set_sensor(true);                       // The filament arrives at the sensor
        if (!(fsensor_reported && elapsed_ms >= phase.ms) && elapsed_ms < 15000) return;   // Wait for the printer to see it
      }
      else if (elapsed_ms < phase.ms) return;
      switch (phase.action) {
        case Action::ClearSensor: set_sensor(false); break;
        case Action::SetLoaded:   changer.mmu_set_loaded(phase.arg); break;
        case Action::ClearLoaded: changer.mmu_set_loaded(-1); break;
        default: break;
      }
      phase_start += Kernel::TimeControl::nanosToTicks(uint64_t(phase.ms) * Kernel::TimeControl::ONE_MILLION);
      if (phase.action == Action::WaitSensor) phase_start = now;
      if (++phase_index >= phases.size()) busy = false;
    }
  }

private:
  enum class Action : uint8_t { None, ClearSensor, WaitSensor, SetLoaded, ClearLoaded };
  struct Phase { Progress code; uint32_t ms; Action action; int arg; };

  void send(const uint8_t* buffer, const uint8_t length) { reply(std::string((const char*)buffer, length), 2); }

  void set_sensor(const bool present) {
    if (auto sensor = VirtualPrinter::get_component<FilamentRunoutSensor>("Filament Runout Sensor 1")) sensor->set_filament_present(present);
  }

  uint16_t register_value(const uint8_t address) {
    switch (address) {
      case 0x00: return 3;      // Firmware 3.0.2
      case 0x02: return 2;
      case 0x03: return 1000;   // Build number
      case 0x05: return busy ? phases[phase_index].code : 0;
      case 0x07: return changer.loaded >= 0 ? 1 : 0;   // Filament state
      case 0x08: return changer.loaded >= 0 ? 1 : 0;   // FINDA
      case 0x09: return fsensor_reported;
      case 0x1B: case 0x1C: return changer.loaded >= 0 ? uint16_t(changer.loaded) : 5;
    }
    auto it = registers.find(address);
    return it != registers.end() ? it->second : 0;
  }

  void start_command(const RequestMsg& rq) {
    std::scoped_lock lock(changer.mutex);
    phases.clear();
    const int tool = rq.value;
    const int loaded = changer.loaded;
    using P = Phase;
    switch (rq.code) {
      case Code::Tool:
        if (loaded >= 0 && loaded != tool) {
          phases.push_back(P{ 3, 1500, Action::ClearSensor, 0 });    // Unloading to FINDA (the printer helps)
          phases.push_back(P{ 4, 600, Action::ClearLoaded, 0 });     // Unloading to pulley
        }
        if (loaded != tool) {
          phases.push_back(P{ 18, 700, Action::None, 0 });           // Selecting filament slot
          phases.push_back(P{ 5, 1200, Action::None, 0 });           // Feeding to FINDA
          phases.push_back(P{ 28, 1000, Action::WaitSensor, 0 });    // Feeding to the printer's filament sensor
          phases.push_back(P{ 7, 800, Action::SetLoaded, tool });    // Feeding to nozzle
        }
        phases.push_back(P{ 9, 300, Action::None, 0 });              // Finishing moves
        break;
      case Code::Load:
        phases.push_back(P{ 18, 700, Action::None, 0 });
        phases.push_back(P{ 17, 1200, Action::None, 0 });
        break;
      case Code::Unload:
        phases.push_back(P{ 3, 1500, Action::ClearSensor, 0 });
        phases.push_back(P{ 4, 600, Action::ClearLoaded, 0 });
        phases.push_back(P{ 2, 300, Action::None, 0 });
        break;
      case Code::Eject:
        phases.push_back(P{ 24, 1500, Action::ClearSensor, 0 });
        phases.push_back(P{ 2, 300, Action::ClearLoaded, 0 });
        break;
      case Code::Cut:
        phases.push_back(P{ 18, 700, Action::None, 0 });
        phases.push_back(P{ 21, 1000, Action::None, 0 });
        break;
      case Code::Home:
        phases.push_back(P{ 26, 1000, Action::None, 0 });
        break;
      default: break;   // Mode, Reset, ...: finished at once
    }
    running = rq;
    phase_index = 0;
    phase_start = Kernel::TimeControl::getTicks();
    busy = !phases.empty();
    changer.log_event(std::string("MMU3 <= ") + char(rq.code) + std::to_string(rq.value));
  }

  void handle(const RequestMsg& rq) {
    uint8_t buffer[Protocol::MaxResponseSize()];
    uint8_t length = 0;
    switch (rq.code) {
      case Code::Version:
        length = Protocol::EncodeResponseRead(rq, true, rq.value == 0 ? 3 : rq.value == 1 ? 0 : rq.value == 2 ? 2 : 1000, buffer);
        break;
      case Code::Write:
        registers[rq.value] = rq.value2;
        length = Protocol::EncodeResponseCmdAR(rq, Param::Accepted, buffer);
        break;
      case Code::Read:
        length = Protocol::EncodeResponseRead(rq, true, register_value(rq.value), buffer);
        break;
      case Code::FilamentSensor:
        fsensor_reported = rq.value;
        length = Protocol::EncodeResponseCmdAR(rq, Param::Accepted, buffer);
        break;
      case Code::Finda:
        length = Protocol::EncodeResponseReadFINDA(rq, changer.loaded >= 0 ? 1 : 0, buffer);
        break;
      case Code::Query: {
        const ResponseCommandStatus status = busy ? ResponseCommandStatus(Param::Processing, phases[phase_index].code) : ResponseCommandStatus(Param::Finished, 0);
        length = Protocol::EncodeResponseQueryOperation(running, status, buffer);
      } break;
      case Code::Button:
        length = Protocol::EncodeResponseCmdAR(rq, Param::Accepted, buffer);
        break;
      case Code::Tool: case Code::Load: case Code::Unload: case Code::Eject: case Code::Cut:
      case Code::Mode: case Code::Reset: case Code::Home:
        start_command(rq);
        length = Protocol::EncodeResponseCmdAR(rq, Param::Accepted, buffer);
        break;
      default:
        length = Protocol::EncodeResponseCmdAR(rq, Param::Rejected, buffer);
        break;
    }
    if (length) send(buffer, length);
  }

  FilamentChanger& changer;
  Protocol protocol;
  RequestMsg running { Code::Reset, 0 };   // The latest command, which Q0 reports on
  std::vector<Phase> phases;
  size_t phase_index = 0;
  uint64_t phase_start = 0;
  bool busy = false;
  bool sensor_initialized = false;
  uint8_t fsensor_reported = 0;            // What the printer last told the MMU about its filament sensor
  std::map<uint8_t, uint16_t> registers;   // Values the printer wrote
};

void FilamentChanger::attach_mmu3(const int stream) {
  serial_stream = stream;
  serial_device = std::make_unique<Mmu3Device>(*this, stream);
}

#endif // HAS_PRUSA_MMU3

void FilamentChanger::note(const std::string& text) {
  std::scoped_lock lock(mutex);
  log_event(text);
}

void FilamentChanger::mmu_set_loaded(const int tool) {
  std::scoped_lock lock(mutex);
  loaded = (tool >= 0 && size_t(tool) < slot_count) ? tool : -1;
  status = loaded >= 0 ? "T" + std::to_string(loaded) + " loaded." : "Unloaded.";
  log_event("MMU => " + status);
}

void FilamentChanger::attach_mmu2(const int stream) {
  serial_stream = stream;
  serial_device = std::make_unique<Mmu2Device>(*this, stream);
}

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
  if (serial_device) ImGui::Text("Serial: stream %d", serial_stream);
  else ImGui::Text("Button: %s   Last hold: %u ms   Axis: %.2f", pressed ? "PRESSED" : "released", last_hold_ms, axis_position);
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
  if (serial_device) ImGui::InputInt("MMU command time (ms)", &command_ms, 100, 1000);
  else {
    ImGui::InputInt("Pulse (ms)", &pulse_ms, 10, 100);
    ImGui::InputInt("Min feed press (ms)", &min_feed_ms, 100, 1000);
  }
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
  writer.member("serial_stream", int64_t(serial_stream));
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

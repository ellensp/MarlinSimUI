#pragma once

#include <atomic>

#include <imgui.h>

#include "Gpio.h"
#include "../virtual_printer.h"
#include "../agent/json.h"

class EndStop : public VirtualPrinter::Component {
public:
  EndStop(pin_type endstop, bool invert_logic, std::function<bool()> triggered) : VirtualPrinter::Component("EndStop"), endstop(endstop), invert_logic(invert_logic), triggered(triggered) {
    Gpio::attach(endstop, [this](GpioEvent& ev){ this->interrupt(ev); });
  }
  ~EndStop() {}

  void ui_widget() {
    // Display the logical trigger state (not the electrical pin level)
    bool logical_triggered = triggered();
    bool display_value = manual_override.load() ? manual_trigger_state.load() : (logical_triggered && enabled.load());

    if (ImGui::Checkbox("Triggered", &display_value)) {
      manual_override = true;
      manual_trigger_state = display_value;
    }
    ImGui::SameLine();

    bool is_auto = !manual_override.load();
    if (ImGui::Checkbox("Auto", &is_auto)) {
      manual_override = !is_auto;
    }

    bool enabled_value = enabled.load();
    if (ImGui::Checkbox("Enabled", &enabled_value)) {
      enabled = enabled_value;
    }
  }

  // {"triggered": bool} forces the switch (manual override); {"auto": true} returns it to
  // the geometric state; {"enabled": false} disconnects it (never triggers)
  bool apply(const agent::JsonValue& body, std::string& error) override {
    bool applied = false, flag;
    if (body.get_bool("triggered", flag)) { manual_override = true; manual_trigger_state = flag; applied = true; }
    if (body.get_bool("auto", flag)) { manual_override = !flag; applied = true; }
    if (body.get_bool("enabled", flag)) { enabled = flag; applied = true; }
    if (!applied) error = "no recognised field (triggered, auto, enabled)";
    return applied;
  }

  void serialize(agent::JsonWriter& writer) const override {
    // Report both the logical trigger state and how it was produced, so an
    // agent can tell a genuine geometric trigger from a manual override.
    const bool logical = triggered && triggered();
    const bool override_active = manual_override.load();
    const bool effective = override_active ? manual_trigger_state.load()
                                           : (logical && enabled.load());

    writer.begin_object();
    writer.member("triggered", effective);
    writer.member("geometric_triggered", logical);
    writer.member("enabled", enabled.load());
    writer.member("manual_override", override_active);
    writer.member("manual_trigger_state", manual_trigger_state.load());
    writer.member("invert_logic", invert_logic);
    writer.member("pin", int64_t(endstop));
    writer.member("pin_level", invert_logic ? !effective : effective);
    writer.end_object();
  }

  void interrupt(GpioEvent& ev) {
    if (ev.pin_id == endstop && ev.event == ev.GET_VALUE) {
      bool logical_state;
      if (manual_override.load()) {
        logical_state = manual_trigger_state.load();
      } else {
        logical_state = triggered() && enabled.load();
      }
      // Convert logical trigger state to electrical pin level based on HIT_STATE
      bool pin_level = invert_logic ? !logical_state : logical_state;
      Gpio::set_pin_value(ev.pin_id, pin_level);
    }
  }

private:
  const pin_type endstop;
  std::atomic_bool enabled = true;
  std::atomic_bool manual_override = false;
  std::atomic_bool manual_trigger_state = false;
  bool invert_logic;
  std::function<bool()> triggered;
};
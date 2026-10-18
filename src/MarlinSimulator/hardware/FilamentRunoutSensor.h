#pragma once

#include <cmath>
#include <functional>

#include "Gpio.h"
#include "../user_interface.h"
#include "../virtual_printer.h"
#include "../agent/json.h"

/**
 * Filament runout sensor, as a switch or as a motion (encoder) sensor.
 *
 * Switch: the pin reads the trigger level while filament is not present.
 *
 * Motion: like the BTT Smart Filament Sensor, the pin toggles every
 * mm_per_toggle of filament moved through it, in either direction, and holds
 * whatever level it was at when the filament stops. With filament_present
 * false (run out, or jammed) the wheel stops, so the pin stops toggling.
 * Filament movement comes from the extruder's E position (set_filament_source).
 */
class FilamentRunoutSensor : public VirtualPrinter::Component {
public:
  FilamentRunoutSensor(pin_type runout_pin, bool runtout_trigger_value, bool motion_sensor=false) : VirtualPrinter::Component("FilamentRunoutSensor"), runout_pin(runout_pin), runtout_trigger_value(runtout_trigger_value), motion_sensor(motion_sensor) {
    Gpio::attach(runout_pin, [this](GpioEvent& event){ this->interrupt(event); });
  }

  // Extruded filament length (mm) for the extruder this sensor watches
  void set_filament_source(std::function<double()> src) { filament_mm = src; }

  void interrupt(GpioEvent &ev) {
    if (ev.pin_id == runout_pin && ev.event == GpioEvent::GET_VALUE)
      Gpio::set_pin_value(runout_pin, motion_sensor ? motion_level() : (filament_present ? !runtout_trigger_value : runtout_trigger_value));
  }

  void ui_widget() {
    bool value_check = filament_present.load();
    ImGui::Checkbox("Filament Present ", &value_check);
    filament_present.store(value_check);

    bool motion_check = motion_sensor.load();
    ImGui::Checkbox("Motion Sensor ", &motion_check);
    motion_sensor.store(motion_check);

    if (motion_check) {
      float mm = mm_per_toggle.load();
      ImGui::SliderFloat("mm per toggle", &mm, 0.5f, 20.0f, "%.2f");
      mm_per_toggle.store(mm);
    }
  }

  void serialize(agent::JsonWriter& writer) const override {
    const bool present = filament_present.load(), motion = motion_sensor.load();
    writer.begin_object();
    writer.member("filament_present", present);
    writer.member("motion_sensor", motion);
    writer.member("mm_per_toggle", double(mm_per_toggle.load()));
    writer.member("filament_mm", filament_mm ? filament_mm() : 0.0);
    writer.member("pin", int64_t(runout_pin));
    writer.member("pin_level", motion ? last_level : (present ? !runtout_trigger_value : runtout_trigger_value));
    writer.end_object();
  }

  // {"filament_present": false} runs the filament out (or jams it); true loads it again.
  // {"motion_sensor": bool} switches between a switch and a motion sensor.
  // {"mm_per_toggle": mm} sets the filament length between motion sensor pin toggles.
  bool apply(const agent::JsonValue& body, std::string& error) override {
    bool applied = false, flag;
    double mm;
    if (body.get_bool("filament_present", flag)) { filament_present = flag; applied = true; }
    if (body.get_bool("motion_sensor", flag)) { motion_sensor = flag; applied = true; }
    if (body.get_double("mm_per_toggle", mm)) {
      if (!(mm > 0)) { error = "'mm_per_toggle' must be greater than 0"; return false; }
      mm_per_toggle = float(mm);
      applied = true;
    }
    if (!applied) error = "expected 'filament_present', 'motion_sensor' (bool) or 'mm_per_toggle' (number)";
    return applied;
  }

private:
  // The pin level of a motion sensor. Called on each pin read, so the level
  // follows the filament position without polling.
  bool motion_level() {
    const double e = filament_mm ? filament_mm() : 0.0, step = mm_per_toggle.load();
    const bool present = filament_present.load();
    if (present) {
      if (!last_present || step != last_step) { base_mm = e; base_level = last_level; }  // Resume from the held level
      const long toggles = long(std::floor((e - base_mm) / step));
      last_level = base_level != bool(toggles & 1);
    }
    last_present = present;
    last_step = step;
    return last_level;
  }

  pin_type runout_pin;
  bool runtout_trigger_value;
  std::atomic_bool filament_present = true;
  std::atomic_bool motion_sensor;
  std::atomic<float> mm_per_toggle = 3.5f;
  std::function<double()> filament_mm;

  // Motion sensor state, touched only on the simulation thread (pin reads)
  double base_mm = 0;
  bool base_level = false, last_level = false, last_present = true;
  float last_step = 0;
};

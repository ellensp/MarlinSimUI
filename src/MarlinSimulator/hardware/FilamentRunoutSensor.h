#pragma once

#include "Gpio.h"
#include "../user_interface.h"
#include "../virtual_printer.h"
#include "../agent/json.h"

class FilamentRunoutSensor : public VirtualPrinter::Component {
public:
  FilamentRunoutSensor(pin_type runout_pin, bool runtout_trigger_value) : VirtualPrinter::Component("FilamentRunoutSensor"), runout_pin(runout_pin), runtout_trigger_value(runtout_trigger_value) {
    Gpio::attach(runout_pin, [this](GpioEvent& event){ this->interrupt(event); });
  }

  void interrupt(GpioEvent &ev) {
    if (ev.pin_id == runout_pin && ev.event == GpioEvent::GET_VALUE) {
      Gpio::set_pin_value(runout_pin, filament_present ? !runtout_trigger_value : runtout_trigger_value);
    }
  }

  void ui_widget() {
    bool value_check = filament_present.load();
    ImGui::Checkbox("Filament Present ", &value_check);
    filament_present.store(value_check);
  }

  void serialize(agent::JsonWriter& writer) const override {
    const bool present = filament_present.load();
    writer.begin_object();
    writer.member("filament_present", present);
    writer.member("pin", int64_t(runout_pin));
    writer.member("pin_level", present ? !runtout_trigger_value : runtout_trigger_value);
    writer.end_object();
  }

  // {"filament_present": false} runs the filament out; true loads it again.
  bool apply(const agent::JsonValue& body, std::string& error) override {
    bool present;
    if (!body.get_bool("filament_present", present)) {
      error = "'filament_present' (bool) is required";
      return false;
    }
    filament_present = present;
    return true;
  }

private:
  pin_type runout_pin;
  bool runtout_trigger_value;
  std::atomic_bool filament_present = true;
};

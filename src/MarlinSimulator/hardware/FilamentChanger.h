#pragma once

//
// Filament changers (MMU / Chameleon), reduced to what Marlin can see of them:
// which filament is loaded, and so what color the extrusion is. Each slot has a
// color the user picks in the UI.
//
// CHAMELEON: Marlin sees one button on an axis. The simulator watches that axis
// (CHAMELEON_BUTTON_AXIS / CHAMELEON_BUTTON_POS from the firmware configuration)
// and runs the Mk4 button protocol on the hold times, using simulated time:
//
//   hold N * 500ms (N = 1..4) : select T(N-1). If a filament is loaded, the next press unloads it.
//   next press (long enough)  : load the selected filament
//   N = 5 : home and load T0 (one load press)    N = 6 : unload current (one press)
//   N = 7 : home (forget the loaded filament)    N = 8 : next filament    N = 9 : random filament
//

#include <array>
#include <deque>
#include <mutex>
#include <string>

#include <glm/glm.hpp>

#include "../virtual_printer.h"
#include "KinematicSystem.h"

class FilamentChanger : public VirtualPrinter::Component {
public:
  static constexpr size_t max_slots = 8;

  FilamentChanger(const char* model, size_t slot_count);

  // Called from the kinematic update with the nozzle position in Marlin's frame
  static void observe(const glm::vec3& marlin_position);

  // Give the extruding hotend the color of the loaded filament
  static void apply_colors(kinematic_state& state);

  void ui_widget() override;
  void serialize(agent::JsonWriter& writer) const override;
  bool apply(const agent::JsonValue& body, std::string& error) override;

private:
  static FilamentChanger* instance;

  enum class Phase { Ready, AwaitUnload, AwaitLoad };

  void button_changed(const bool pressed, const uint64_t now_ns);
  void process_hold(const uint32_t hold_ms);
  void log_event(const std::string& text);
  void select(const int tool, const bool unload_only = false);
  glm::vec3 current_color() const;

  std::string model;
  size_t slot_count;
  mutable std::mutex mutex;
  std::array<glm::vec3, max_slots> slot_color {};

  Phase phase = Phase::Ready;
  int loaded = -1;          // The filament Marlin gets to extrude, or -1
  int selected = -1;        // The filament chosen by the last select press
  bool unload_only = false; // The pending unload ends with nothing loaded
  bool pressed = false;
  float axis_position = 0;  // The button axis, as last seen
  uint64_t press_start_ns = 0;
  uint32_t last_hold_ms = 0;
  int pulse_ms = 500;       // One pulse of the Chameleon's button counter
  int min_feed_ms = 3000;   // A load or unload press must hold at least this long to move the filament far enough
  std::string status = "Idle";
  std::deque<std::string> history;   // Recent button holds and what the Chameleon made of them
};

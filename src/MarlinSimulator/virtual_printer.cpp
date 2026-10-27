#include <algorithm>
#include <imgui.h>

#include "hardware/Button.h"
#include "hardware/StepperDriver.h"
#include "hardware/EndStop.h"
#include "hardware/Heater.h"
#include "hardware/print_bed.h"
#include "hardware/bed_probe.h"
#include "hardware/ST7796Device.h"
#include "hardware/HD44780Device.h"
#include "hardware/ST7920Device.h"
#include "hardware/ST7565Device.h"
#include "hardware/SDCard.h"
#include "hardware/W25QxxDevice.h"
#include "hardware/FilamentRunoutSensor.h"
#include "hardware/FilamentChanger.h"
#include "hardware/NeoPixelDevice.h"
#include "hardware/KinematicSystem.h"
#include "hardware/pwm_reader.h"
#include "hardware/BLTouch.h"
#include "hardware/Buzzer.h"

#include "virtual_printer.h"
#include "agent/json.h"

#include <src/inc/MarlinConfig.h>

#if HAS_TFT_XPT2046 || HAS_TOUCH_XPT2046
  #include "paths.h"
  #include MARLIN_HAL_PATH(tft/xpt2046.h)
#endif

#ifndef SD_DETECT_STATE
  #define SD_DETECT_STATE HIGH
#endif

std::function<void(kinematic_state&)> VirtualPrinter::on_kinematic_update;
std::map<std::string, std::shared_ptr<VirtualPrinter::Component>> VirtualPrinter::component_map;
std::vector<std::shared_ptr<VirtualPrinter::Component>> VirtualPrinter::components;
std::shared_ptr<VirtualPrinter::Component> VirtualPrinter::root;

void VirtualPrinter::Component::ui_widgets() {
  ui_widget();
  for(auto const& it : children) {
    // Displays and the SD Card start open. "Collapse Components List" closes everything.
    if (it->ui_collapse_requested) {
      ImGui::SetNextItemOpen(false, ImGuiCond_Always);
      it->ui_collapse_requested = false;
      it->ui_initial_open_applied = true;
    }
    else if (!it->ui_initial_open_applied) {
      if (it->ui_default_open()) ImGui::SetNextItemOpen(true, ImGuiCond_Always);
      it->ui_initial_open_applied = true;
    }

    if (ImGui::CollapsingHeader(it->name.c_str())) {
      ImGui::Indent(6.0f);
      ImGui::PushID(it->name.c_str());
      it->ui_widgets();
      ImGui::PopID();
      ImGui::Unindent(6.0f);
    }
  }
}

bool VirtualPrinter::Component::ui_default_open() const {
  return name.find(" Display") != std::string::npos || name == "SD Card";
}

void VirtualPrinter::Component::request_ui_collapse() {
  ui_collapse_requested = true;
  for (auto const& it : children) it->request_ui_collapse();
}

void VirtualPrinter::collapse_components_list() {
  if (root) root->request_ui_collapse();
}

void VirtualPrinter::Component::serialize(agent::JsonWriter& writer) const {
  // Default: an empty object, so unimplemented components still appear in
  // GET /state with a well-formed (if uninformative) value.
  writer.begin_object();
  writer.end_object();
}

void VirtualPrinter::serialize_all(agent::JsonWriter& writer) {
  writer.begin_object();
  for (auto const& component : components) {
    writer.key(component->name);
    writer.begin_object();
    writer.member("identifier", component->identifier);
    writer.key("state");
    component->serialize(writer);
    writer.end_object();
  }
  writer.end_object();
}

bool VirtualPrinter::serialize_one(const std::string& name, agent::JsonWriter& writer) {
  auto it = component_map.find(name);
  if (it == component_map.end() || !it->second) return false;

  writer.begin_object();
  writer.member("identifier", it->second->identifier);
  writer.key("state");
  it->second->serialize(writer);
  writer.end_object();
  return true;
}

std::vector<std::string> VirtualPrinter::component_names() {
  std::vector<std::string> names;
  names.reserve(components.size());
  for (auto const& component : components) names.push_back(component->name);
  return names;
}

std::vector<std::string> VirtualPrinter::display_names() {
  std::vector<std::string> names;
  std::vector<uint8_t> probe;
  uint32_t width = 0, height = 0;
  for (auto const& component : components)
    if (component->capture(probe, width, height)) names.push_back(component->name);
  return names;
}

bool VirtualPrinter::capture_display(const std::string& name,
                                     std::vector<uint8_t>& rgb,
                                     uint32_t& width, uint32_t& height,
                                     std::string& matched_name) {
  for (auto const& component : components) {
    if (!name.empty() && component->name != name) continue;
    if (!component->capture(rgb, width, height)) continue;
    matched_name = component->name;
    return true;
  }
  return false;
}

bool VirtualPrinter::inject_touch(float rx, float ry, uint32_t hold_ms, std::string& matched_name) {
  for (auto const& component : components) {
    if (!component->inject_touch(rx, ry, hold_ms)) continue;
    matched_name = component->name;
    return true;
  }
  return false;
}

std::map<uint64_t, uint64_t> servo_pin_lookup { {0, SERVO0_PIN}, {1, SERVO1_PIN}, {2, SERVO2_PIN}, {3, SERVO3_PIN}};


void VirtualPrinter::build() {
  root = add_component<Component>("root");

  #if ENABLED(DELTA)
    auto kinematics = root->add_component<DeltaKinematicSystem>("Delta Kinematic System", on_kinematic_update);
    root->add_component<EndStop>("Endstop(Tower A Max)", X_MAX_PIN, !X_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].stepper_position.x >= DELTA_HEIGHT; });
    root->add_component<EndStop>("Endstop(Tower B Max)", Y_MAX_PIN, !Y_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].stepper_position.y >= DELTA_HEIGHT; });
    root->add_component<EndStop>("Endstop(Tower C Max)", Z_MAX_PIN, !Z_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].stepper_position.z >= DELTA_HEIGHT; });
  #elif ENABLED(SCARA)
    // Marlin homes the joints (degrees) and then sets them to SCARA_HOME_THETA / SCARA_HOME_PSI,
    // so the X (shoulder) and Y (elbow) endstops sit at those stepper angles.
    auto kinematics = root->add_component<ScaraKinematicSystem>("SCARA Kinematic System", on_kinematic_update);
    #ifdef SCARA_HOME_THETA
      constexpr float home_a = SCARA_HOME_THETA, home_b = SCARA_HOME_PSI + SCARA_HOME_THETA * float(SCARA_CROSSTALK_FACTOR);
    #else
      constexpr float home_a = 0, home_b = 0;
    #endif
    auto joint = [kinematics](const int i) { return kinematics->hardware_offset[0][i] + kinematics->state.effector_position[0].stepper_position[i]; };
    #if X_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(Shoulder Min)", X_MIN_PIN, !X_MIN_ENDSTOP_HIT_STATE, [=](){ return joint(0) <= home_a; });
    #else
      root->add_component<EndStop>("Endstop(Shoulder Max)", X_MAX_PIN, !X_MAX_ENDSTOP_HIT_STATE, [=](){ return joint(0) >= home_a; });
    #endif
    #if Y_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(Elbow Min)", Y_MIN_PIN, !Y_MIN_ENDSTOP_HIT_STATE, [=](){ return joint(1) <= home_b; });
    #else
      root->add_component<EndStop>("Endstop(Elbow Max)", Y_MAX_PIN, !Y_MAX_ENDSTOP_HIT_STATE, [=](){ return joint(1) >= home_b; });
    #endif
    #if Z_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(Z Min)", Z_MIN_PIN, !Z_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.z <= Z_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(Z Max)", Z_MAX_PIN, !Z_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.z >= Z_MAX_POS; });
    #endif
  #elif ENABLED(COREXY)
    auto kinematics = root->add_component<CoreXYKinematicSystem>("CoreXY Kinematic System", on_kinematic_update);
    #if X_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(X Min)", X_MIN_PIN, !X_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.x <= X_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(X Max)", X_MAX_PIN, !X_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.x >= X_MAX_POS; });
    #endif
    #if Y_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(Y Min)", Y_MIN_PIN, !Y_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.y <= Y_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(Y Max)", Y_MAX_PIN, !Y_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.y >= Y_MAX_POS; });
    #endif
    #if Z_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(Z Min)", Z_MIN_PIN, !Z_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.z <= Z_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(Z Max)", Z_MAX_PIN, !Z_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.z >= Z_MAX_POS; });
    #endif
  #elif ENABLED(COREXZ)
    auto kinematics = root->add_component<CoreXZKinematicSystem>("CoreXZ Kinematic System", on_kinematic_update);
    #if X_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(X Min)", X_MIN_PIN, !X_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.x <= X_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(X Max)", X_MAX_PIN, !X_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.x >= X_MAX_POS; });
    #endif
    #if Y_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(Y Min)", Y_MIN_PIN, !Y_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.y <= Y_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(Y Max)", Y_MAX_PIN, !Y_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.y >= Y_MAX_POS; });
    #endif
    #if Z_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(Z Min)", Z_MIN_PIN, !Z_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.z <= Z_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(Z Max)", Z_MAX_PIN, !Z_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.z >= Z_MAX_POS; });
    #endif
  #elif ENABLED(COREYZ)
    auto kinematics = root->add_component<CoreYZKinematicSystem>("CoreYZ Kinematic System", on_kinematic_update);
    #if X_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(X Min)", X_MIN_PIN, !X_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.x <= X_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(X Max)", X_MAX_PIN, !X_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.x >= X_MAX_POS; });
    #endif
    #if Y_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(Y Min)", Y_MIN_PIN, !Y_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.y <= Y_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(Y Max)", Y_MAX_PIN, !Y_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.y >= Y_MAX_POS; });
    #endif
    #if Z_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(Z Min)", Z_MIN_PIN, !Z_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.z <= Z_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(Z Max)", Z_MAX_PIN, !Z_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.z >= Z_MAX_POS; });
    #endif
  #elif ENABLED(COREYX)
    auto kinematics = root->add_component<CoreYXKinematicSystem>("CoreYX Kinematic System", on_kinematic_update);
    #if X_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(X Min)", X_MIN_PIN, !X_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.x <= X_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(X Max)", X_MAX_PIN, !X_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.x >= X_MAX_POS; });
    #endif
    #if Y_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(Y Min)", Y_MIN_PIN, !Y_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.y <= Y_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(Y Max)", Y_MAX_PIN, !Y_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.y >= Y_MAX_POS; });
    #endif
    #if Z_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(Z Min)", Z_MIN_PIN, !Z_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.z <= Z_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(Z Max)", Z_MAX_PIN, !Z_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.z >= Z_MAX_POS; });
    #endif
  #elif ENABLED(COREZX)
    auto kinematics = root->add_component<CoreZXKinematicSystem>("CoreZX Kinematic System", on_kinematic_update);
    #if X_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(X Min)", X_MIN_PIN, !X_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.x <= X_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(X Max)", X_MAX_PIN, !X_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.x >= X_MAX_POS; });
    #endif
    #if Y_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(Y Min)", Y_MIN_PIN, !Y_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.y <= Y_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(Y Max)", Y_MAX_PIN, !Y_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.y >= Y_MAX_POS; });
    #endif
    #if Z_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(Z Min)", Z_MIN_PIN, !Z_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.z <= Z_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(Z Max)", Z_MAX_PIN, !Z_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.z >= Z_MAX_POS; });
    #endif
  #elif ENABLED(COREZY)
    auto kinematics = root->add_component<CoreZYKinematicSystem>("CoreZY Kinematic System", on_kinematic_update);
    #if X_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(X Min)", X_MIN_PIN, !X_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.x <= X_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(X Max)", X_MAX_PIN, !X_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.x >= X_MAX_POS; });
    #endif
    #if Y_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(Y Min)", Y_MIN_PIN, !Y_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.y <= Y_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(Y Max)", Y_MAX_PIN, !Y_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.y >= Y_MAX_POS; });
    #endif
    #if Z_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(Z Min)", Z_MIN_PIN, !Z_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.z <= Z_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(Z Max)", Z_MAX_PIN, !Z_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.z >= Z_MAX_POS; });
    #endif
  #else
    auto kinematics = root->add_component<CartesianKinematicSystem>("Cartesian Kinematic System", on_kinematic_update);
    #if X_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(X Min)", X_MIN_PIN, !X_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.x <= X_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(X Max)", X_MAX_PIN, !X_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.x >= X_MAX_POS; });
    #endif
    #if ENABLED(DUAL_X_CARRIAGE)
      // IDEX homes X2 to X max (X_HOME_DIR must be -1), on the X_MAX_PIN switch.
      root->add_component<EndStop>("Endstop(X2 Max)", X_MAX_PIN, !X_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[1].position.x >= dxc_x2_endstop(); });
    #endif
    #if Y_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(Y Min)", Y_MIN_PIN, !Y_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.y <= Y_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(Y Max)", Y_MAX_PIN, !Y_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.y >= Y_MAX_POS; });
    #endif
    #if Z_HOME_DIR < 0
      root->add_component<EndStop>("Endstop(Z Min)", Z_MIN_PIN, !Z_MIN_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.z <= Z_MIN_POS; });
    #else
      root->add_component<EndStop>("Endstop(Z Max)", Z_MAX_PIN, !Z_MAX_ENDSTOP_HIT_STATE, [kinematics](){ return kinematics->state.effector_position[0].position.z >= Z_MAX_POS; });
    #endif
  #endif

  auto print_bed = root->add_component<PrintBed>("Print Bed", glm::vec2{X_BED_SIZE, Y_BED_SIZE});

  #if HAS_BED_PROBE
    #if ENABLED(BLTOUCH)
      root->add_component<BLTouchProbe>("BLTouch", servo_pin_lookup[Z_PROBE_SERVO_NR],
    #else
      root->add_component<BedProbe>("Probe",
    #endif
    #ifdef Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN
      Z_MIN_PIN
    #else
      Z_MIN_PROBE_PIN
    #endif
    , glm::vec3 NOZZLE_TO_PROBE_OFFSET, kinematics->state.effector_position[0].position, *print_bed);
  #endif

  #if HOTENDS
    {
      // The part-cooling fan blows on hotend 0, and its filament carries heat away
      auto h0 = root->add_component<Heater>("Hotend0 Heater", HEATER_0_PIN, TEMP_0_PIN, heater_data{12, 3.6}, hotend_data{13, 20, 0.897}, adc_data{4700, 12}
        #ifdef FAN0_PIN
          , FAN0_PIN
        #endif
      );
      h0->set_filament_source([kinematics](){ return kinematics->state.effector_position.empty() ? 0.0 : double(kinematics->state.effector_position[0].position.w); });
    }
    #if HOTENDS > 1
      root->add_component<Heater>("Hotend1 Heater", HEATER_1_PIN, TEMP_1_PIN, heater_data{12, 3.6}, hotend_data{13, 20, 0.897}, adc_data{4700, 12});
    #endif
    #if HOTENDS > 2
      root->add_component<Heater>("Hotend2 Heater", HEATER_2_PIN, TEMP_2_PIN, heater_data{12, 3.6}, hotend_data{13, 20, 0.897}, adc_data{4700, 12});
    #endif
    #if HOTENDS > 3
      root->add_component<Heater>("Hotend3 Heater", HEATER_3_PIN, TEMP_3_PIN, heater_data{12, 3.6}, hotend_data{13, 20, 0.897}, adc_data{4700, 12});
    #endif
    #if HOTENDS > 4
      root->add_component<Heater>("Hotend4 Heater", HEATER_4_PIN, TEMP_4_PIN, heater_data{12, 3.6}, hotend_data{13, 20, 0.897}, adc_data{4700, 12});
    #endif
    #if HOTENDS > 5
      root->add_component<Heater>("Hotend5 Heater", HEATER_5_PIN, TEMP_5_PIN, heater_data{12, 3.6}, hotend_data{13, 20, 0.897}, adc_data{4700, 12});
    #endif
    #if HOTENDS > 6
      root->add_component<Heater>("Hotend6 Heater", HEATER_6_PIN, TEMP_6_PIN, heater_data{12, 3.6}, hotend_data{13, 20, 0.897}, adc_data{4700, 12});
    #endif
    #if HOTENDS > 7
      root->add_component<Heater>("Hotend7 Heater", HEATER_7_PIN, TEMP_7_PIN, heater_data{12, 3.6}, hotend_data{13, 20, 0.897}, adc_data{4700, 12});
    #endif
  #endif

  #if TEMP_SENSOR_BED
    root->add_component<Heater>("Bed Heater", HEATER_BED_PIN, TEMP_BED_PIN, heater_data{12, 1.2}, hotend_data{325, 824, 0.897}, adc_data{4700, 12});
  #endif

  #if TEMP_SENSOR_CHAMBER
    root->add_component<Heater>("Chamber Heater", HEATER_CHAMBER_PIN, TEMP_CHAMBER_PIN, heater_data{12, 1.2}, hotend_data{325, 824, 0.897}, adc_data{4700, 12});
  #endif

  #if HAS_CHAMELEON
    root->add_component<FilamentChanger>("Filament Changer", "3D Chameleon Mk4", size_t(EXTRUDERS));
  #elif HAS_PRUSA_MMU3 && defined(MMU_SERIAL_PORT)
    root->add_component<FilamentChanger>("Filament Changer", "Prusa MMU3", size_t(EXTRUDERS))->attach_mmu3(MMU_SERIAL_PORT);
  #elif HAS_PRUSA_MMU2 && defined(MMU_SERIAL_PORT)
    root->add_component<FilamentChanger>("Filament Changer", "Prusa MMU2", size_t(EXTRUDERS))->attach_mmu2(MMU_SERIAL_PORT);
  #endif

  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    // Sensor N watches extruder N-1. With FILAMENT_MOTION_SENSOR alone the runout pins carry
    // encoder pulses; with FILAMENT_SWITCH_AND_MOTION they are switches and the motion pins pulse.
    auto add_runout_sensor = [&](const char * const name, const pin_type pin, const bool state, const size_t e, const bool motion) {
      auto sensor = root->add_component<FilamentRunoutSensor>(name, pin, state, motion);
      sensor->set_filament_source([kinematics, e](){
        const auto &ep = kinematics->state.effector_position;
        return ep.empty() ? 0.0 : double(ep[e < ep.size() ? e : 0].position.w);
      });
    };
    constexpr bool runout_pins_pulse = ENABLED(FILAMENT_MOTION_SENSOR) && DISABLED(FILAMENT_SWITCH_AND_MOTION);
    add_runout_sensor("Filament Runout Sensor 1", FIL_RUNOUT1_PIN, FIL_RUNOUT1_STATE, 0, runout_pins_pulse);
    #if NUM_RUNOUT_SENSORS > 1
      add_runout_sensor("Filament Runout Sensor 2", FIL_RUNOUT2_PIN, FIL_RUNOUT2_STATE, 1, runout_pins_pulse);
    #endif
    #if NUM_RUNOUT_SENSORS > 2
      add_runout_sensor("Filament Runout Sensor 3", FIL_RUNOUT3_PIN, FIL_RUNOUT3_STATE, 2, runout_pins_pulse);
    #endif
    #if NUM_RUNOUT_SENSORS > 3
      add_runout_sensor("Filament Runout Sensor 4", FIL_RUNOUT4_PIN, FIL_RUNOUT4_STATE, 3, runout_pins_pulse);
    #endif
    #if NUM_RUNOUT_SENSORS > 4
      add_runout_sensor("Filament Runout Sensor 5", FIL_RUNOUT5_PIN, FIL_RUNOUT5_STATE, 4, runout_pins_pulse);
    #endif
    #if NUM_RUNOUT_SENSORS > 5
      add_runout_sensor("Filament Runout Sensor 6", FIL_RUNOUT6_PIN, FIL_RUNOUT6_STATE, 5, runout_pins_pulse);
    #endif
    #if NUM_RUNOUT_SENSORS > 6
      add_runout_sensor("Filament Runout Sensor 7", FIL_RUNOUT7_PIN, FIL_RUNOUT7_STATE, 6, runout_pins_pulse);
    #endif
    #if NUM_RUNOUT_SENSORS > 7
      add_runout_sensor("Filament Runout Sensor 8", FIL_RUNOUT8_PIN, FIL_RUNOUT8_STATE, 7, runout_pins_pulse);
    #endif
    #if ENABLED(FILAMENT_SWITCH_AND_MOTION)
      add_runout_sensor("Filament Motion Sensor 1", FIL_MOTION1_PIN, false, 0, true);
      #if NUM_MOTION_SENSORS > 1
        add_runout_sensor("Filament Motion Sensor 2", FIL_MOTION2_PIN, false, 1, true);
      #endif
      #if NUM_MOTION_SENSORS > 2
        add_runout_sensor("Filament Motion Sensor 3", FIL_MOTION3_PIN, false, 2, true);
      #endif
      #if NUM_MOTION_SENSORS > 3
        add_runout_sensor("Filament Motion Sensor 4", FIL_MOTION4_PIN, false, 3, true);
      #endif
      #if NUM_MOTION_SENSORS > 4
        add_runout_sensor("Filament Motion Sensor 5", FIL_MOTION5_PIN, false, 4, true);
      #endif
      #if NUM_MOTION_SENSORS > 5
        add_runout_sensor("Filament Motion Sensor 6", FIL_MOTION6_PIN, false, 5, true);
      #endif
      #if NUM_MOTION_SENSORS > 6
        add_runout_sensor("Filament Motion Sensor 7", FIL_MOTION7_PIN, false, 6, true);
      #endif
      #if NUM_MOTION_SENSORS > 7
        add_runout_sensor("Filament Motion Sensor 8", FIL_MOTION8_PIN, false, 7, true);
      #endif
    #endif
  #endif

  #ifdef FAN0_PIN
    root->add_component<PWMReader>("Fan0", FAN0_PIN);
  #endif

  #ifdef Z_PROBE_SERVO_NR
    #define SERVO_TEST(N) Z_PROBE_SERVO_NR != N
  #else
    #define SERVO_TEST(N) 1
  #endif
  #if NUM_SERVOS > 0 && SERVO_TEST(0)
    root->add_component<PWMReader>("Servo0", SERVO0_PIN);
  #endif
  #if NUM_SERVOS > 1 && SERVO_TEST(1)
    root->add_component<PWMReader>("Servo1", SERVO1_PIN);
  #endif
  #if NUM_SERVOS > 2 && SERVO_TEST(2)
    root->add_component<PWMReader>("Servo2", SERVO2_PIN);
  #endif
  #if NUM_SERVOS > 3 && SERVO_TEST(3)
    root->add_component<PWMReader>("Servo3", SERVO3_PIN);
  #endif

  #if ENABLED(SPI_FLASH)
    root->add_component<W25QxxDevice>("SPI Flash", spi_bus_by_pins<SPI_FLASH_SCK_PIN, SPI_FLASH_MOSI_PIN, SPI_FLASH_MISO_PIN>(), SPI_FLASH_CS_PIN, SPI_FLASH_SIZE);
  #endif

  #ifdef SDSUPPORT
    root->add_component<SDCard>("SD Card", spi_bus_by_pins<SD_SCK_PIN, SD_MOSI_PIN, SD_MISO_PIN>(), SD_SS_PIN, SD_DETECT_PIN, SD_DETECT_STATE);
  #endif

  #ifndef LCD_PINS_EN
    #define LCD_PINS_EN LCD_PINS_ENABLE
  #endif

  #if ENABLED(TFT_INTERFACE_SPI)
    root->add_component<ST7796Device>("ST7796Device Display", spi_bus_by_pins<TFT_SCK_PIN, TFT_MOSI_PIN, TFT_MISO_PIN>(), TFT_CS_PIN, spi_bus_by_pins<TOUCH_SCK_PIN, TOUCH_MOSI_PIN, TOUCH_MISO_PIN>(), TOUCH_CS_PIN, TFT_DC_PIN, BEEPER_PIN, BTN_EN1, BTN_EN2, BTN_ENC, BTN_BACK, KILL_PIN);
  #elif defined(HAS_MARLINUI_HD44780)
    root->add_component<HD44780Device>("HD44780Device Display", LCD_PINS_RS, LCD_PINS_EN, LCD_PINS_D4, LCD_PINS_D5, LCD_PINS_D6, LCD_PINS_D7, BEEPER_PIN, BTN_EN1, BTN_EN2, BTN_ENC, BTN_BACK, KILL_PIN);
  #elif HAS_MARLINUI_U8GLIB && (ANY(CARTESIO_UI, FYSETC_MINI_12864, MKS_MINI_12864, ENDER2_STOCKDISPLAY, MINIPANEL) || IS_U8GLIB_LM6059_AF || IS_U8GLIB_ST7565_64128N)
    // ST7565-family panels, on the shared SPI bus
    #ifdef ST7565_XOFFSET
      constexpr uint8_t st7565_column_offset = ST7565_XOFFSET;
    #else
      constexpr uint8_t st7565_column_offset = TERN0(IS_U8GLIB_LM6059_AF, 1);  // U8glib's LM6059 device starts at column 1
    #endif
    #if defined(LCD_SCREEN_ROTATE) && LCD_SCREEN_ROTATE == 180
      constexpr bool st7565_rotate_180 = true;    // Panel mounted upside down
    #else
      constexpr bool st7565_rotate_180 = false;
    #endif
    root->add_component<ST7565Device>("ST7565Device Display", spi_bus_by_pins<SD_SCK_PIN, SD_MOSI_PIN, SD_MISO_PIN>(), DOGLCD_CS, DOGLCD_A0, st7565_column_offset, st7565_rotate_180, BEEPER_PIN, BTN_EN1, BTN_EN2, BTN_ENC, BTN_BACK, KILL_PIN);
    #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0) && ENABLED(RGB_LED)
      // Red, green and blue backlight LEDs, each on its own pin. PWM_PIN is false here, so Marlin
      // drives them as plain outputs: each LED is on or off.
      get_component<ST7565Device>("ST7565Device Display")->set_backlight([](glm::vec3& color) {
        auto lit = [](const pin_type pin) { return Gpio::get_pin_value(pin) ? 1.0f : 0.0f; };
        color = { lit(RGB_LED_R_PIN), lit(RGB_LED_G_PIN), lit(RGB_LED_B_PIN) };
        return true;
      });
    #endif
  #elif defined(HAS_MARLINUI_U8GLIB)
    root->add_component<ST7920Device>("ST7920Device Display", LCD_PINS_D4, LCD_PINS_EN, LCD_PINS_RS, BEEPER_PIN, BTN_EN1, BTN_EN2, BTN_ENC, BTN_BACK, KILL_PIN);
  #endif

  #if HAS_KILL
    root->add_component<Button>("Kill Button", KILL_PIN, false);
  #endif

  #ifdef NEOPIXEL_LED
    auto neopixel = root->add_component<NeoPixelDevice>("NeoPixelDevice", NEOPIXEL_PIN, NEOPIXEL_TYPE, NEOPIXEL_PIXELS);
    #if ENABLED(FYSETC_MINI_12864_2_1)
      // The panel's first NeoPixel is the LCD backlight; the other two light the encoder knob
      get_component<ST7565Device>("ST7565Device Display")->set_backlight([neopixel](glm::vec3& color) {
        ImVec4 led;
        if (!neopixel->led_color(0, led)) return false;
        color = { led.x, led.y, led.z };
        return true;
      });
    #endif
  #endif

  #ifdef BEEPER_PIN
    #ifdef SPEAKER
      // Real speaker device needed.
    #else
      root->add_component<Buzzer>("Buzzer", BEEPER_PIN, false);
    #endif
  #endif

  for(auto const& component : components) component->ui_init();

  kinematics->kinematic_update();
}

void VirtualPrinter::ui_widgets() {
  if (root) root->ui_widgets();
}

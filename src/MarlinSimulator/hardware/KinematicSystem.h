#pragma once

#include <memory>
#include <atomic>
#include <glm/glm.hpp>

#include <imgui.h>

#include "Gpio.h"
#include "../virtual_printer.h"
#include "StepperDriver.h"

struct extruder_state {
  glm::vec3 stepper_position {};
  glm::vec4 position {};
  glm::vec3 color{};
};

struct kinematic_state {
  std::vector<extruder_state> effector_position {};
  glm::vec3 position {};
};

// Nozzle offset of hotend h in Marlin XYZ mm (motion.hotend_offset, so M218 / EEPROM apply)
glm::vec3 hotend_offset_mm(const size_t h);

#if ENABLED(DUAL_X_CARRIAGE)
  // IDEX: X position of the X2 carriage's (X max) endstop. Marlin homes X2 there and
  // calls it hotend_offset[1].x, which defaults to the larger of X2_HOME_POS and X2_MAX_POS.
  constexpr float dxc_x2_endstop() { return (X2_HOME_POS) > (X2_MAX_POS) ? (X2_HOME_POS) : (X2_MAX_POS); }
#endif

class KinematicSystem : public VirtualPrinter::Component {
public:
  KinematicSystem(std::function<void(kinematic_state&)> on_kinematic_update) :  VirtualPrinter::Component("Kinematic System"), on_kinematic_update(on_kinematic_update) {};

  virtual void kinematic_update() = 0;
  void collect_steppers();
  virtual void serialize(agent::JsonWriter& writer) const override;

  std::vector<glm::vec3> hardware_offset {};
  std::vector<std::shared_ptr<VirtualPrinter::Component>> steppers;
  kinematic_state state{};
  std::function<void(kinematic_state&)> on_kinematic_update;
};

class CartesianKinematicSystem : public KinematicSystem {
public:
  CartesianKinematicSystem(std::function<void(kinematic_state&)> on_kinematic_update);
  virtual void ui_widget() override;
  virtual void kinematic_update() override;
  std::vector<double> extruder {};
};

class CoreXYKinematicSystem : public KinematicSystem {
public:
  CoreXYKinematicSystem(std::function<void(kinematic_state&)> on_kinematic_update);
  virtual void ui_widget() override;
  virtual void kinematic_update() override;
  std::vector<double> extruder {};
};

class CoreXZKinematicSystem : public KinematicSystem {
public:
  CoreXZKinematicSystem(std::function<void(kinematic_state&)> on_kinematic_update);
  virtual void ui_widget() override;
  virtual void kinematic_update() override;
  std::vector<double> extruder {};
};

class CoreYZKinematicSystem : public KinematicSystem {
public:
  CoreYZKinematicSystem(std::function<void(kinematic_state&)> on_kinematic_update);
  virtual void ui_widget() override;
  virtual void kinematic_update() override;
  std::vector<double> extruder {};
};

class CoreYXKinematicSystem : public KinematicSystem {
public:
  CoreYXKinematicSystem(std::function<void(kinematic_state&)> on_kinematic_update);
  virtual void ui_widget() override;
  virtual void kinematic_update() override;
  std::vector<double> extruder {};
};

class CoreZXKinematicSystem : public KinematicSystem {
public:
  CoreZXKinematicSystem(std::function<void(kinematic_state&)> on_kinematic_update);
  virtual void ui_widget() override;
  virtual void kinematic_update() override;
  std::vector<double> extruder {};
};

class CoreZYKinematicSystem : public KinematicSystem {
public:
  CoreZYKinematicSystem(std::function<void(kinematic_state&)> on_kinematic_update);
  virtual void ui_widget() override;
  virtual void kinematic_update() override;
  std::vector<double> extruder {};
};

class DeltaKinematicSystem : public KinematicSystem {
public:
  DeltaKinematicSystem(std::function<void(kinematic_state&)> on_kinematic_update);
  virtual void ui_widget() override;
  virtual void kinematic_update() override;

  double delta_height = 200.0;
  double delta_radius = 124.0;
  double delta_diagonal_rod = 250.0;
  glm::vec2 delta_tower[3]{};
  glm::vec3 delta_tower_angle_trim{};
  glm::vec3 delta_radius_trim_tower{};
  glm::vec3 delta_diagonal_rod_trim{};
  glm::vec3 delta_diagonal_rod_2_tower{};

  glm::vec3 forward_kinematics(const double z1, const double z2, const double z3);
  void recalc_delta_settings();
};

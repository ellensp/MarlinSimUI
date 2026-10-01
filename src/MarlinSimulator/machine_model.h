#pragma once

//
// Procedural 3D printer models for the Viewport: Bedslinger, Cube (Z-bed) and Delta.
// Geometry is sized from the Marlin configuration and animated from the nozzle position.
//
// GL coordinates: X = Marlin X, Y = Marlin Z (up), Z = -Marlin Y.
//

#include <functional>
#include <memory>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

#include "renderer/renderer.h"

// Simple-geometry models size themselves from the Marlin config.
enum MachineType : uint8_t { MACHINE_BEDSLINGER, MACHINE_CUBE, MACHINE_DELTA, MACHINE_TYPE_COUNT };

// Placement of the moving parts for one frame
struct MachinePose {
  glm::vec3 nozzle {};        // Nozzle position in Marlin coordinates (mm), bed-relative
  glm::vec3 bed_offset {};    // World translation of the bed and everything printed on it (GL)
  glm::vec3 nozzle_world {};  // Nozzle position in world GL coordinates
  bool has_towers = false;    // Delta: 'towers' holds the simulated carriage heights
  glm::vec3 towers {};        // Delta: carriage heights (A, B, C) from the tower steppers
  glm::vec3 x2_world {};      // IDEX: origin of the X2 carriage (its X, the X1 nozzle's height and depth)
  float probe_raise = 0;      // Bed probe lift above its deployed position (BLTouch stowed)
};

// Delta geometry, from the Marlin config (DELTA_RADIUS, DELTA_DIAGONAL_ROD,
// DELTA_HEIGHT, PRINTABLE_RADIUS). This is the simulated machine's physical
// build, so runtime M665 changes (Marlin's belief) intentionally don't alter it.
struct DeltaGeometry {
  float radius, diagonal_rod, height, printable_radius;
};

// Set by --machine on the command line; MACHINE_TYPE_COUNT = use the Marlin config
extern MachineType machine_type_option;
MachineType machine_type_from_name(const char* name); // MACHINE_TYPE_COUNT if unknown

class MachineModel {
public:
  static const char* type_name(const MachineType type);
  static MachineType default_type();  // From the Marlin kinematics (DELTA, IS_CORE, ...)
  static bool is_available(const MachineType type); // DELTA builds: Delta only. Otherwise: all but Delta.
  static DeltaGeometry default_delta();

  void build(const MachineType type, std::shared_ptr<renderer::ShaderProgram> program);
  void destroy();

  MachinePose pose(const glm::vec3 nozzle, const float x2 = 0) const;  // Nozzle in Marlin coordinates; IDEX: X2 nozzle X
  void update(const MachinePose& pose);
  void queue_render() const;                        // Add all parts to the render list
  void set_visible(const bool visible);

  // Hotends drawn on the carriage(s) / effector, one per HOTEND at motion.hotend_offset.
  // IDEX: T0 rides the X1 carriage and every other tool rides the X2 carriage, as in Marlin.
  static constexpr int max_hotends = 8;
  static int hotend_count();
  static int carriage_count();                       // 2 with DUAL_X_CARRIAGE, else 1
  static int carriage_of(const int h);               // Carriage carrying hotend h
  static glm::vec3 live_hotend_offset(const int h);  // GL offset of hotend h from its carriage origin, now
  glm::vec3 hotend_offset(const int h) const { return h > 0 && h < max_hotends ? hotend_layout[h] : glm::vec3(); } // As built
  bool layout_changed() const;                       // M218 / M851 / EEPROM moved a hotend or the probe: rebuild

  // Bed probe drawn on the (X1) carriage at the M851 offset (HAS_BED_PROBE, not NOZZLE_AS_PROBE).
  // A 6mm cylinder with its tip at the trigger point; a stowed BLTouch rides probe_stow_raise higher.
  static bool has_probe();
  static glm::vec3 live_probe_offset();              // GL offset of the probe tip from the nozzle, now
  static constexpr float probe_radius = 3, probe_length = 36, probe_stow_raise = 7;

  // Z drives (lead screw + motor) of the Cube's bed, in Marlin XY: one per Z stepper (NUM_Z_STEPPERS).
  // With 2+ they are placed where the bed attaches, which Marlin describes with the G34 settings:
  // Z_STEPPER_ALIGN_STEPPER_XY (the screws themselves) if set, else the probe points of
  // Z_STEPPER_ALIGN_XY / Z_STEPPERS_ORIENTATION, pushed out past the nearest bed edge.
  // Independent Z steppers tilt the bed about these points.
  static std::vector<glm::vec2> z_drive_xy();

  // Per-frame look of hotend h's heater block and nozzle, applied by the lit shader:
  // tint.rgb blended in by tint.a, plus a 'glow' brightness for the active hotend.
  // dim: 1 normal, 0.5 half as bright (the dark phase of a thermal fault blink)
  void set_hotend_look(const int h, const glm::vec4 tint, const float glow, const float dim=1);

  // World-space bounds of the machine at rest, for camera framing
  glm::vec3 bounds_min {}, bounds_max {};

  MachineType type = MACHINE_BEDSLINGER;
  DeltaGeometry delta = default_delta();
  bool visible = true;

private:
  using Animate = std::function<void(renderer::Mesh&, const MachinePose&)>;
  using Geometry = std::function<void(renderer::Buffer<renderer::vertex_data_t>&)>;
  struct Part {
    renderer::mesh_id_t mesh;
    Animate animate;
  };

  renderer::mesh_id_t add_part(Geometry geometry, Animate animate = nullptr, const bool in_bounds = true);
  // Shared by every printer type: each carriage / effector part calls add_cold_ends and
  // sizes itself with hotend_extent; add_hot_parts adds the tintable heater blocks.
  // carriage = -1 takes the extent of every hotend on every carriage.
  void hotend_extent(glm::vec3& lo, glm::vec3& hi, const int carriage = -1) const;
  void add_cold_ends(renderer::Buffer<renderer::vertex_data_t>& b, const int carriage = 0) const;
  void add_hot_parts();  // Heater block + nozzle of every hotend, each its own tintable mesh
  // Bed probe (X1 carriage / effector): a bracket from the part's nearest point (carriage-local XZ)
  // at a height within [mount_lo, mount_hi], and the body, lifted by pose.probe_raise
  void add_probe(std::function<glm::vec2(glm::vec2)> nearest, const float mount_lo, const float mount_hi);
  void x_travel(float& lo, float& hi) const;  // X range the nozzles can reach (IDEX: X1 min to the X2 endstop)
  // Carriage-local X extent of carriage c: its hotends plus 'margin' each side, and the probe on X1
  void carriage_span(const int c, const float margin, float& lo, float& hi) const;
  // World X of the carriages' outermost sides at the ends of X travel, to size the X axis around them
  void carriage_reach(const float margin, float& left, float& right) const;
  void build_bedslinger();
  void build_cube();
  void build_delta();

  std::vector<Part> parts;
  std::shared_ptr<renderer::ShaderProgram> program;

  // Shader uniforms hold pointers, so these must outlive the meshes. Every part sets
  // u_tint, u_glow and u_dim: uniforms are per program, so an unset one keeps the last mesh's value.
  glm::vec4 no_tint {};
  float no_glow = 0, no_dim = 1;
  glm::vec4 hotend_tint[max_hotends] {};
  float hotend_glow[max_hotends] {};
  float hotend_dim[max_hotends] { 1, 1, 1, 1, 1, 1, 1, 1 };
  glm::vec3 hotend_layout[max_hotends] {};  // hotend_offset(h) when the parts were built
  glm::vec3 probe_layout {};                // live_probe_offset() when the parts were built
};

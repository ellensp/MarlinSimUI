#pragma once

//
// Delta reach: where the active tool's nozzle can physically go, from Marlin's live
// delta geometry (M665 / M666 / G33 / M218), and a translucent volume showing it.
//
// In effector coordinates (nozzle minus the active hotend offset):
//  - Floor: each arm's length limits |effector - tower| <= arm. The intersection of the
//    three disks is a rounded triangle, corners toward the towers. It's convex, so it's
//    described by its radius along each angle. The soft endstops clip it to a circle.
//  - Height: a carriage sits sqrt(arm^2 - |effector - tower|^2) above the nozzle and
//    can't pass its endstop, so the highest nozzle Z at each point is
//    min over towers of (carriage top - sqrt(arm^2 - d^2)). Each term is a bowl around
//    its tower, so the top is full height at the center and between the towers, and
//    dips toward each tower. Z has no other limit, so the volume is this heightfield.
//

#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "renderer/renderer.h"

struct DeltaReach {
  bool valid = false;             // DELTA build with the geometry gathered
  glm::vec2 offset {};            // Active hotend XY offset: nozzle = effector + offset
  glm::vec2 tower[3] {};          // Tower positions (Marlin XY)
  float arm2[3] {};               // Arm length squared, per tower
  float top[3] {};                // Carriage limit (endstop), Marlin tower coordinate
  float z_top = 0;                // Highest nozzle Z (at the center)
  bool soft_on = false;           // Software endstops enabled (M211)
  float soft_radius = 0;          // Soft endstop XY radius, around the active tool's offset
  float soft_z_max = 0;           // Soft endstop Z max

  void gather();

  // Effector distance from the center to the floor boundary along 'angle' (radians).
  // 'soft' also applies the software endstops (if on). 0 if unreachable.
  float radius(const float angle, const bool soft=false) const;

  // Highest nozzle Z with the effector at 'e'. 'soft' also applies the Z soft endstop.
  float height(const glm::vec2 e, const bool soft=false) const;

  void append_signature(std::vector<float>& s) const;
};

// Translucent volume of the active tool's printable space: the physical reach, also
// limited by the software endstops while they're on. DELTA builds only.
class ReachVolume {
public:
  void create(std::shared_ptr<renderer::ShaderProgram> lit_program);
  bool update();                  // Rebuild if Marlin's inputs changed. True if rebuilt.
  void set_visible(const bool vis);

  renderer::mesh_id_t mesh {};
  bool visible = false;

private:
  std::vector<float> signature;
  glm::vec4 no_tint {};           // Uniforms are per program: set neutral values for this mesh
  float no_glow = 0, no_dim = 1;
};

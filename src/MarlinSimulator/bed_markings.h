#pragma once

//
// Bed markings: flat symbols and lines drawn just above the bed surface, in light
// colors that read well on the dark bed tint.
//
//  - The 0,0 origin: white circle with a cross
//  - The Z_SAFE_HOMING point: yellow circle with an X
//  - The probeable area: white dashed outline (probe bounds for the live M851 offset)
//  - The active tool's reach: orange dashed outline of the live software endstops,
//    light orange when enabled, darker orange when M211 has them off. On a Delta also
//    a solid light orange rounded triangle: where the arms let the active tool reach.
//  - SCARA: solid light orange rings around the shoulder for the outer reach (L1 + L2) and the
//    dead zone (|L1 - L2|, or MIDDLE_DEAD_ZONE_R if larger), drawn where they cross the bed
//    The software endstops are a box, as on a Cartesian, and the probeable area is the bed less the
//    probing margin, with dashed rings for the probe's own reach, which is a different ring (it hangs
//    from the forearm at the M851 offset)
//  - The leveling mesh grid: light blue lines (the live mesh, or the grid G29 would probe)
//
// Positions come from live Marlin state and are rebuilt when they change.
//

#include <functional>
#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "renderer/renderer.h"

class BedMarkings {
public:
  using BedZ = std::function<float(const glm::vec2)>; // Bed surface height at a Marlin XY

  void create(std::shared_ptr<renderer::ShaderProgram> program);

  // Rebuild when Marlin's inputs changed, or when 'force' is set (the bed surface moved).
  // Returns true if the mesh was rebuilt.
  bool update(const BedZ& bed_z, const bool force);

  void set_visible(const bool vis);

  renderer::mesh_id_t mesh {};
  bool visible = true;

private:
  std::vector<float> signature;  // The inputs of the last build
  glm::vec4 no_tint {};          // The default shader's u_tint, which must be set per mesh
  float no_dim = 1;              // ...and u_dim
};

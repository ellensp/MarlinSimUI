//
// Bed markings: origin, safe homing point, probeable area, tool reach, leveling mesh grid.
// See bed_markings.h.
//

#include "bed_markings.h"
#include "delta_reach.h"
#include "scara_frame.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <glm/gtc/constants.hpp>

#include <src/inc/MarlinConfig.h>
#if HAS_BED_PROBE || HAS_LEVELING
  #include <src/module/probe.h>
#endif
#if HAS_LEVELING
  #include <src/feature/bedlevel/bedlevel.h>
#endif
#include <src/module/motion.h>

#undef abs
#undef min
#undef max

using renderer::vertex_data_t;
using VertexBuffer = renderer::Buffer<vertex_data_t>;

namespace {

// Light colors that read well on the dark blue-to-red bed
constexpr glm::vec4 col_origin { 1.00f, 1.00f, 1.00f, 1.0f },  // White
                    col_home   { 1.00f, 0.88f, 0.20f, 1.0f },  // Yellow
                    col_probe  { 0.95f, 0.95f, 0.95f, 1.0f },  // White (dashed)
                    col_reach  { 1.00f, 0.70f, 0.40f, 1.0f },  // Light orange (dashed)
                    col_reach_off { 0.75f, 0.38f, 0.10f, 1.0f },  // Darker orange: soft endstops off (M211 S0)
                    col_grid   { 0.55f, 0.80f, 1.00f, 1.0f };  // Light blue

constexpr float lift = 0.2f;       // Above the bed surface, clear of depth fighting
constexpr float symbol_r = 5.0f,   // Origin / safe-home circle radius
                symbol_w = 0.8f,   // Symbol stroke width
                probe_w = 0.8f,    // Probe area outline width
                reach_w = 0.8f,    // Tool reach outline width
                grid_w = 0.5f,     // Mesh grid line width
                dash = 6.0f, gap = 4.0f;

// Marlin native XY -> the simulated bed's XY. Cartesian beds start at 0,0; a Delta
// (BED_CENTER_AT_0_0) has its center at 0,0 and the simulator's bed at X_BED_SIZE/2.
#if ENABLED(SCARA)
  // The shoulder is fixed in the world, and SCARA_OFFSET puts Marlin's XY relative to it (scara_frame.h)
  const glm::vec2 to_bed { scara_frame::shift_x, scara_frame::shift_y };
#else
  const glm::vec2 to_bed { float(X_BED_SIZE) * 0.5f - float(X_CENTER), float(Y_BED_SIZE) * 0.5f - float(Y_CENTER) };
#endif

// Flat strokes on the bed, following its surface. Points are Marlin native XY.
struct Painter {
  VertexBuffer& buf;
  const BedMarkings::BedZ& bed_z;

  glm::vec3 at(const glm::vec2 p) const {
    const glm::vec2 s = p + to_bed;
    return { s.x, bed_z(s) + lift, -s.y };
  }

  // Face up (+Y) so back-face culling keeps it
  void tri(glm::vec3 a, glm::vec3 b, glm::vec3 c, const glm::vec4& col) {
    if (glm::cross(b - a, c - a).y < 0) std::swap(b, c);
    constexpr glm::vec3 up { 0, 1, 0 };
    buf.add_vertex({a, up, col});
    buf.add_vertex({b, up, col});
    buf.add_vertex({c, up, col});
  }

  void quad(const glm::vec2 p0, const glm::vec2 p1, const glm::vec2 p2, const glm::vec2 p3, const glm::vec4& col) {
    const glm::vec3 a = at(p0), b = at(p1), c = at(p2), d = at(p3);
    tri(a, b, c, col);
    tri(a, c, d, col);
  }

  // A closed polyline of strokes
  void loop(const std::vector<glm::vec2>& pts, const float w, const glm::vec4& col) {
    for (size_t i = 0; i < pts.size(); ++i) line(pts[i], pts[(i + 1) % pts.size()], w, col);
  }

  // A straight stroke, split into short pieces so it follows a tilted or leveled bed
  void line(const glm::vec2 a, const glm::vec2 b, const float w, const glm::vec4& col) {
    const glm::vec2 d = b - a;
    const float len = glm::length(d);
    if (len <= 0) return;
    const glm::vec2 n = glm::vec2(-d.y, d.x) / len * (w * 0.5f);
    const int pieces = std::max(1, int(std::ceil(len / 10.0f)));
    for (int i = 0; i < pieces; ++i) {
      const glm::vec2 p = a + d * (float(i) / pieces), q = a + d * (float(i + 1) / pieces);
      quad(p - n, q - n, q + n, p + n, col);
    }
  }

  void dashed(const glm::vec2 a, const glm::vec2 b, const float w, const glm::vec4& col) {
    const float len = glm::length(b - a);
    if (len <= 0) return;
    const glm::vec2 u = (b - a) / len;
    for (float s = 0; s < len; s += dash + gap)
      line(a + u * s, a + u * std::min(s + dash, len), w, col);
  }

  // 'on_bed' keeps only the pieces over the bed, for a ring that reaches past its edge
  void ring(const glm::vec2 c, const float r, const float w, const glm::vec4& col, const bool dashes=false, const bool on_bed=false) {
    const int sides = std::max(24, int(glm::two_pi<float>() * r / 2.0f)); // ~2mm pieces, so dashes don't alias
    const float ri = r - w * 0.5f, ro = r + w * 0.5f, step = glm::two_pi<float>() / sides;
    for (int i = 0; i < sides; ++i) {
      const float a0 = i * step, a1 = a0 + step;
      if (dashes && std::fmod(a0 * r, dash + gap) >= dash) continue; // Dashes by arc length
      const glm::vec2 d0 { std::cos(a0), std::sin(a0) }, d1 { std::cos(a1), std::sin(a1) };
      if (on_bed) {
        const glm::vec2 m = c + (d0 + d1) * (0.5f * r) + to_bed;
        if (m.x < 0 || m.y < 0 || m.x > float(X_BED_SIZE) || m.y > float(Y_BED_SIZE)) continue;
      }
      quad(c + d0 * ri, c + d0 * ro, c + d1 * ro, c + d1 * ri, col);
    }
  }

  // Circle with a cross (+) or an X through it
  void target(const glm::vec2 c, const glm::vec4& col, const bool diagonal) {
    ring(c, symbol_r, symbol_w, col);
    const float k = symbol_r * 1.4f * (diagonal ? float(M_SQRT1_2) : 1.0f);
    if (diagonal) {
      line(c + glm::vec2(-k, -k), c + glm::vec2(k, k), symbol_w, col);
      line(c + glm::vec2(-k, k), c + glm::vec2(k, -k), symbol_w, col);
    }
    else {
      line(c + glm::vec2(-k, 0), c + glm::vec2(k, 0), symbol_w, col);
      line(c + glm::vec2(0, -k), c + glm::vec2(0, k), symbol_w, col);
    }
  }
};

// Everything the markings are drawn from, read from Marlin
struct Inputs {
  bool has_probe_area = false, round_area = false;
  glm::vec2 probe_min {}, probe_max {};
  float probe_radius = 0;
  std::vector<float> grid_x, grid_y;
  bool has_home = false;
  glm::vec2 home {};
  bool has_reach = false, reach_on = false;
  glm::vec2 reach_min {}, reach_max {};
  float reach_radius = 0;
  glm::vec2 reach_center {};      // Kinematic: soft endstop circle center (the active tool's offset)
  std::vector<glm::vec2> physical; // Delta: where the active tool can physically reach, on the bed
  bool has_scara = false;          // SCARA: the arms reach an annulus around the shoulder
  glm::vec2 scara_center {};       // The shoulder (Marlin native XY)
  float scara_outer = 0, scara_inner = 0;
  float scara_probe_outer = 0, scara_probe_inner = 0;  // SCARA: the ring the probe (a tool of its own) reaches, less PROBING_MARGIN

  void gather() {
    // Where the active tool can go: the software endstops (native coordinates), which
    // follow the active tool's offset (T, M218) and IDEX modes. M211 turns them on/off.
    #if HAS_SOFTWARE_ENDSTOPS
      has_reach = true;
      reach_on = motion.soft_endstop._enabled;    // M211 S0/S1 (not the brief "loose" state)
      reach_min = { motion.soft_endstop.min.x, motion.soft_endstop.min.y };
      reach_max = { motion.soft_endstop.max.x, motion.soft_endstop.max.y };
      #if IS_KINEMATIC && DISABLED(SCARA)   // (A SCARA's software endstops are a box, like a Cartesian's)
        // The radius Marlin clamps XY moves to (see Motion::update_software_endstops).
        // Motion::apply_limits clamps (target - hotend offset), so it's centered on the offset.
        reach_radius = std::min({ std::abs(std::max(reach_min.x, reach_min.y)), reach_max.x, reach_max.y });
        #if HAS_HOTEND_OFFSET && ENABLED(DELTA)
          reach_center = { motion.active_hotend_offset().x, motion.active_hotend_offset().y };
        #else
          reach_center = { float(X_CENTER), float(Y_CENTER) };
        #endif
      #endif
    #endif

    // Delta: the arms and carriages limit the nozzle to a rounded triangle
    #if ENABLED(DELTA)
      DeltaReach dr;
      dr.gather();
      constexpr int steps = 180;
      for (int j = 0; j < steps; ++j) {
        const float a = glm::two_pi<float>() * j / steps;
        physical.push_back(dr.offset + glm::vec2(std::cos(a), std::sin(a)) * dr.radius(a));
      }
    #endif

    // SCARA: the tool reaches from |L1 - L2| to L1 + L2 from the shoulder, and Marlin also
    // refuses anything inside MIDDLE_DEAD_ZONE_R (can_reach in motion.cpp)
    #if ENABLED(SCARA)
      has_scara = true;
      scara_center = { float(SCARA_OFFSET_X), float(SCARA_OFFSET_Y) };
      scara_outer = float(SCARA_LINKAGE_1) + float(SCARA_LINKAGE_2);
      scara_inner = std::abs(float(SCARA_LINKAGE_1) - float(SCARA_LINKAGE_2));
      #if MIDDLE_DEAD_ZONE_R > 0
        scara_inner = std::max(scara_inner, float(MIDDLE_DEAD_ZONE_R));
      #endif
    #endif

    #if HAS_BED_PROBE
      has_probe_area = true;
      probe_min = { probe.min_x(), probe.min_y() };
      probe_max = { probe.max_x(), probe.max_y() };
      #if ENABLED(SCARA)
        // The bounds are the bed less the margin. Within them the probe, bolted to the forearm, reaches its own ring
        scara_reach_ring(scara_tool(probe.offset_xy), PROBING_MARGIN, scara_probe_inner, scara_probe_outer);
      #elif IS_KINEMATIC
        round_area = true;
        probe_radius = probe.probe_radius();
      #endif
    #endif

    // The live mesh if there is one, otherwise the grid G29 would probe
    #if ABL_USES_GRID || HAS_MESH
      bool live = true;
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        live = bedlevel.has_mesh();
      #elif ENABLED(AUTO_BED_LEVELING_LINEAR)
        live = false;
      #endif

      // Number of grid points. With VARIABLE_GRID_POINTS the mesh may use fewer than the maximum.
      uint8_t nx = GRID_MAX_POINTS_X, ny = GRID_MAX_POINTS_Y;
      #if ENABLED(VARIABLE_GRID_POINTS)
        if (live) { nx = bedlevel.nr_grid_points.x; ny = bedlevel.nr_grid_points.y; }
      #endif
      #ifdef GRID_MIN_SPACING
        // G29 uses fewer points where they would be closer than GRID_MIN_SPACING
        if (!live) {
          const xy_uint8_t r = reduced_grid_points({ probe.max_x() - probe.min_x(), probe.max_y() - probe.min_y() },
                                                   { nx, ny }, { GRID_MIN_SPACING, GRID_MIN_SPACING });
          nx = r.x; ny = r.y;
        }
      #endif

      for (uint8_t i = 0; i < nx; ++i) {
        #if HAS_MESH
          if (live) { grid_x.push_back(bedlevel.get_mesh_x(i)); continue; }
        #endif
        grid_x.push_back(probe.min_x() + (probe.max_x() - probe.min_x()) * i / (nx - 1));
      }
      for (uint8_t j = 0; j < ny; ++j) {
        #if HAS_MESH
          if (live) { grid_y.push_back(bedlevel.get_mesh_y(j)); continue; }
        #endif
        grid_y.push_back(probe.min_y() + (probe.max_y() - probe.min_y()) * j / (ny - 1));
      }
      UNUSED(live);
    #endif

    #if ENABLED(Z_SAFE_HOMING)
      has_home = true;
      home = { float(Z_SAFE_HOMING_X_POINT), float(Z_SAFE_HOMING_Y_POINT) };
    #endif
  }

  std::vector<float> signature() const {
    std::vector<float> s { float(has_probe_area), probe_min.x, probe_min.y, probe_max.x, probe_max.y, probe_radius,
                           float(has_home), home.x, home.y, float(has_reach), float(reach_on), reach_min.x, reach_min.y, reach_max.x, reach_max.y,
                           reach_center.x, reach_center.y, float(grid_x.size()), float(grid_y.size()), float(physical.size()),
                           float(has_scara), scara_center.x, scara_center.y, scara_outer, scara_inner, scara_probe_outer, scara_probe_inner };
    for (const auto& p : physical) s.insert(s.end(), { p.x, p.y });
    s.insert(s.end(), grid_x.begin(), grid_x.end());
    s.insert(s.end(), grid_y.begin(), grid_y.end());
    return s;
  }
};

} // namespace

void BedMarkings::create(std::shared_ptr<renderer::ShaderProgram> program) {
  mesh = renderer::create_mesh();
  auto m = renderer::get_mesh_by_id(mesh);
  m->set_shader_program(program);
  m->m_shader_instance->set_uniform("u_tint", &no_tint); // Uniforms are per program; don't inherit the bed tint
  m->m_shader_instance->set_uniform("u_dim", &no_dim);   // ...or the bed's fault blink
  m->buffer_vector<vertex_data_t>().push_back(VertexBuffer::create());
  m->m_visible = visible;
  signature.clear();
}

bool BedMarkings::update(const BedZ& bed_z, const bool force) {
  Inputs in;
  in.gather();
  std::vector<float> sig = in.signature();
  if (!force && sig == signature) return false;
  signature = std::move(sig);

  auto m = renderer::get_mesh_by_id(mesh);
  if (!m) return false;
  auto buf = m->buffer<vertex_data_t>();
  buf->data().clear();
  Painter p { *buf, bed_z };

  // Mesh grid first so the outline and symbols draw over it
  if (!in.grid_x.empty() && !in.grid_y.empty()) {
    const float x0 = in.grid_x.front(), x1 = in.grid_x.back(),
                y0 = in.grid_y.front(), y1 = in.grid_y.back();
    #if IS_KINEMATIC
      // G29 only probes inside the probe radius; clip the lines to it
      if (in.round_area) {
        const glm::vec2 c { float(X_CENTER), float(Y_CENTER) };
        const float r2 = in.probe_radius * in.probe_radius;
        for (const float x : in.grid_x) {
          const float h2 = r2 - (x - c.x) * (x - c.x);
          if (h2 <= 0) continue;
          const float h = std::sqrt(h2);
          p.line({x, std::max(y0, c.y - h)}, {x, std::min(y1, c.y + h)}, grid_w, col_grid);
        }
        for (const float y : in.grid_y) {
          const float h2 = r2 - (y - c.y) * (y - c.y);
          if (h2 <= 0) continue;
          const float h = std::sqrt(h2);
          p.line({std::max(x0, c.x - h), y}, {std::min(x1, c.x + h), y}, grid_w, col_grid);
        }
      }
      else
    #endif
    {
      for (const float x : in.grid_x) p.line({x, y0}, {x, y1}, grid_w, col_grid);
      for (const float y : in.grid_y) p.line({x0, y}, {x1, y}, grid_w, col_grid);
    }
  }

  // Delta: the physical reach of the active tool (solid), with the soft endstops inside it (dashed)
  if (!in.physical.empty()) p.loop(in.physical, reach_w, col_reach);

  // SCARA: the outer reach and the dead zone around the shoulder (solid), where over the bed
  if (in.has_scara) {
    p.ring(in.scara_center, in.scara_outer, reach_w, col_reach, false, true);
    if (in.scara_inner > 1) p.ring(in.scara_center, in.scara_inner, reach_w, col_reach, false, true);
  }

  // Tool reach: dashed outline of the software endstops, darker when M211 has them off
  if (in.has_reach) {
    const glm::vec4& col = in.reach_on ? col_reach : col_reach_off;
    #if IS_KINEMATIC && DISABLED(SCARA)
      p.ring(in.reach_center, in.reach_radius, reach_w, col, true);
    #else
      const glm::vec2 a = in.reach_min, c = in.reach_max, b { c.x, a.y }, d { a.x, c.y };
      p.dashed(a, b, reach_w, col);
      p.dashed(b, c, reach_w, col);
      p.dashed(c, d, reach_w, col);
      p.dashed(d, a, reach_w, col);
    #endif
  }

  if (in.has_probe_area) {
    if (in.round_area)
      p.ring({float(X_CENTER), float(Y_CENTER)}, in.probe_radius, probe_w, col_probe, true);
    else {
      const glm::vec2 a = in.probe_min, c = in.probe_max, b { c.x, a.y }, d { a.x, c.y };
      p.dashed(a, b, probe_w, col_probe);
      p.dashed(b, c, probe_w, col_probe);
      p.dashed(c, d, probe_w, col_probe);
      p.dashed(d, a, probe_w, col_probe);
    }
    // SCARA: the probe reaches a ring too, so what G29 can probe is the bounds where they're inside it
    if (in.scara_probe_outer > 0) {
      p.ring(in.scara_center, in.scara_probe_outer, probe_w, col_probe, true, true);
      if (in.scara_probe_inner > 1) p.ring(in.scara_center, in.scara_probe_inner, probe_w, col_probe, true, true);
    }
  }

  if (in.has_home) p.target(in.home, col_home, true);  // Safe homing point: circle with an X
  p.target({0, 0}, col_origin, false);                 // Origin: circle with a cross

  return true;
}

void BedMarkings::set_visible(const bool vis) {
  visible = vis;
  if (auto m = renderer::get_mesh_by_id(mesh)) m->m_visible = vis;
}

#include "machine_model.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <utility>

#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <src/inc/MarlinConfig.h>
#if HAS_BED_PROBE
  #include <src/module/probe.h> // probe.offset (M851)
#endif

#include "hardware/KinematicSystem.h" // hotend_offset_mm

#undef abs
#undef min
#undef max

using renderer::vertex_data_t;
using VertexBuffer = renderer::Buffer<vertex_data_t>;

//
// Orientation: the Viewport is Y-up. World X = Marlin X, world Y = Marlin Z (up),
// world -Z = Marlin +Y (toward the back), so the printer's front faces +Z.
// Mirrored left/right parts take a sign ("in", toward the bed) that must apply to
// every offset of the part. Rotations turn about the mesh origin, so translate to
// the pivot first. Check both sides of each symmetric pair in close-up; a flipped
// part is easy to miss when it hides inside a bigger one.
//

//
// Colors
//
namespace color {
  constexpr glm::vec4 frame   { 0.55f, 0.57f, 0.60f, 1.0f }; // Aluminum extrusion
  constexpr glm::vec4 dark    { 0.20f, 0.21f, 0.23f, 1.0f }; // Motors, carriage plates
  constexpr glm::vec4 bed     { 0.10f, 0.10f, 0.11f, 1.0f }; // Heated bed
  constexpr glm::vec4 rod     { 0.78f, 0.80f, 0.82f, 1.0f }; // Smooth rods
  constexpr glm::vec4 printed { 0.95f, 0.45f, 0.10f, 1.0f }; // Printed parts (Prusa orange)
  constexpr glm::vec4 heater  { 0.70f, 0.72f, 0.75f, 1.0f }; // Heater block
  constexpr glm::vec4 brass   { 0.85f, 0.65f, 0.25f, 1.0f }; // Nozzle
  constexpr glm::vec4 screw   { 0.45f, 0.46f, 0.50f, 1.0f }; // Lead screws
  constexpr glm::vec4 joint   { 0.62f, 0.63f, 0.66f, 1.0f }; // Ball joints
  constexpr glm::vec4 probe   { 0.20f, 0.55f, 0.90f, 1.0f }; // Bed probe body (stands out from the head)
}

//
// Geometry helpers. Triangles are wound counter-clockwise when seen from the
// side the normal points to, so back-face culling works.
//
static void add_triangle(VertexBuffer& buf, glm::vec3 a, glm::vec3 na, glm::vec3 b, glm::vec3 nb, glm::vec3 c, glm::vec3 nc, const glm::vec3 face, const glm::vec4 col) {
  if (glm::dot(glm::cross(b - a, c - a), face) < 0) { std::swap(b, c); std::swap(nb, nc); }
  buf.add_vertex({a, na, col});
  buf.add_vertex({b, nb, col});
  buf.add_vertex({c, nc, col});
}

static void add_quad(VertexBuffer& buf, const glm::vec3 p0, const glm::vec3 p1, const glm::vec3 p2, const glm::vec3 p3, const glm::vec3 n, const glm::vec4 col) {
  add_triangle(buf, p0, n, p1, n, p2, n, n, col);
  add_triangle(buf, p0, n, p2, n, p3, n, n, col);
}

// Axis-aligned box from two opposite corners
static void add_box(VertexBuffer& buf, const glm::vec3 a, const glm::vec3 b, const glm::vec4 col) {
  const glm::vec3 lo = glm::min(a, b), hi = glm::max(a, b);
  add_quad(buf, {hi.x, lo.y, lo.z}, {hi.x, hi.y, lo.z}, {hi.x, hi.y, hi.z}, {hi.x, lo.y, hi.z}, { 1, 0, 0}, col);
  add_quad(buf, {lo.x, lo.y, lo.z}, {lo.x, hi.y, lo.z}, {lo.x, hi.y, hi.z}, {lo.x, lo.y, hi.z}, {-1, 0, 0}, col);
  add_quad(buf, {lo.x, hi.y, lo.z}, {hi.x, hi.y, lo.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}, { 0, 1, 0}, col);
  add_quad(buf, {lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, lo.y, hi.z}, {lo.x, lo.y, hi.z}, { 0,-1, 0}, col);
  add_quad(buf, {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}, { 0, 0, 1}, col);
  add_quad(buf, {lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, hi.y, lo.z}, {lo.x, hi.y, lo.z}, { 0, 0,-1}, col);
}

// Box centered on a point and turned about the vertical axis so its local +Z
// faces "facing" (horizontal). Size is {width across, height, depth along facing}.
static void add_block_facing(VertexBuffer& buf, const glm::vec3 center, const glm::vec3 facing, const glm::vec3 size, const glm::vec4 col) {
  const glm::vec3 f = glm::normalize(glm::vec3(facing.x, 0, facing.z)), u { 0, 1, 0 };
  const glm::vec3 r = glm::cross(u, f);   // Right-handed: r x u = f
  const glm::vec3 hx = r * (size.x * 0.5f), hy = u * (size.y * 0.5f), hz = f * (size.z * 0.5f);
  auto p = [&](const int sx, const int sy, const int sz) { return center + hx * float(sx) + hy * float(sy) + hz * float(sz); };
  // Each face listed counter-clockwise seen from outside
  add_quad(buf, p( 1,-1,-1), p( 1, 1,-1), p( 1, 1, 1), p( 1,-1, 1),  r, col);
  add_quad(buf, p(-1,-1, 1), p(-1, 1, 1), p(-1, 1,-1), p(-1,-1,-1), -r, col);
  add_quad(buf, p(-1, 1,-1), p(-1, 1, 1), p( 1, 1, 1), p( 1, 1,-1),  u, col);
  add_quad(buf, p(-1,-1,-1), p( 1,-1,-1), p( 1,-1, 1), p(-1,-1, 1), -u, col);
  add_quad(buf, p(-1,-1, 1), p( 1,-1, 1), p( 1, 1, 1), p(-1, 1, 1),  f, col);
  add_quad(buf, p( 1,-1,-1), p(-1,-1,-1), p(-1, 1,-1), p( 1, 1,-1), -f, col);
}

// Box centered on a point, given its full size
static void add_block(VertexBuffer& buf, const glm::vec3 center, const glm::vec3 size, const glm::vec4 col) {
  add_box(buf, center - size * 0.5f, center + size * 0.5f, col);
}

// Capped cylinder between two points
static void add_cylinder(VertexBuffer& buf, const glm::vec3 a, const glm::vec3 b, const float radius, const glm::vec4 col, const int segments = 16) {
  const glm::vec3 axis = glm::normalize(b - a);
  const glm::vec3 ref = std::abs(axis.y) < 0.9f ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
  const glm::vec3 u = glm::normalize(glm::cross(axis, ref)), v = glm::cross(axis, u);
  for (int i = 0; i < segments; ++i) {
    const float t0 = glm::two_pi<float>() * i / segments, t1 = glm::two_pi<float>() * (i + 1) / segments;
    const glm::vec3 n0 = u * std::cos(t0) + v * std::sin(t0), n1 = u * std::cos(t1) + v * std::sin(t1);
    const glm::vec3 a0 = a + n0 * radius, a1 = a + n1 * radius, b0 = b + n0 * radius, b1 = b + n1 * radius;
    const glm::vec3 face = glm::normalize(n0 + n1);
    add_triangle(buf, a0, n0, a1, n1, b1, n1, face, col);
    add_triangle(buf, a0, n0, b1, n1, b0, n0, face, col);
    add_triangle(buf, a, -axis, a0, -axis, a1, -axis, -axis, col);
    add_triangle(buf, b,  axis, b0,  axis, b1,  axis,  axis, col);
  }
}

// UV sphere around a center
static void add_sphere(VertexBuffer& buf, const glm::vec3 c, const float radius, const glm::vec4 col, const int slices = 12, const int stacks = 8) {
  auto n = [&](const int i, const int j) {
    const float th = glm::pi<float>() * j / stacks, ph = glm::two_pi<float>() * i / slices;
    return glm::vec3(std::sin(th) * std::cos(ph), std::cos(th), std::sin(th) * std::sin(ph));
  };
  for (int j = 0; j < stacks; ++j)
    for (int i = 0; i < slices; ++i) {
      const glm::vec3 a = n(i, j), b = n(i + 1, j), d = n(i, j + 1), e = n(i + 1, j + 1);
      if (j > 0)          add_triangle(buf, c + a * radius, a, c + b * radius, b, c + e * radius, e, glm::normalize(a + b + e), col);
      if (j < stacks - 1) add_triangle(buf, c + a * radius, a, c + e * radius, e, c + d * radius, d, glm::normalize(a + e + d), col);
    }
}

// Hotend parts. The hot end (nozzle and heater block, tip at the origin) is its own mesh
// so it can be tinted by temperature; the cold end (heat break and heatsink, over a tip
// at 'at') is part of the carriage.
static void add_hotend_hot(VertexBuffer& buf) {
  add_cylinder(buf, {0, 0, 0}, {0, 3, 0}, 1.5f, color::brass, 12);
  add_cylinder(buf, {0, 3, 0}, {0, 6, 0}, 4.0f, color::brass, 6);
  add_box(buf, {-8, 6, -6}, {12, 17, 6}, color::heater);
}
static void add_hotend_cold(VertexBuffer& buf, const glm::vec3 at) {
  add_cylinder(buf, at + glm::vec3(0, 17, 0), at + glm::vec3(0, 22, 0), 1.5f, color::rod, 8);
  for (int i = 0; i < 6; ++i) {
    const float y = 22.0f + i * 4.0f;
    add_cylinder(buf, at + glm::vec3(0, y, 0), at + glm::vec3(0, y + 2.0f, 0), 11.0f, color::frame, 20);
  }
  add_cylinder(buf, at + glm::vec3(0, 22, 0), at + glm::vec3(0, 46, 0), 4.0f, color::frame, 12);
}

static void move_to(renderer::Mesh& m, const glm::vec3 pos) { m.m_position = pos; }

//
// Hotends on the carriage(s), one per HOTEND at motion.hotend_offset (M218 / EEPROM), read via
// hotend_offset_mm() in KinematicSystem.cpp, which also puts each extruder's nozzle there.
//
// IDEX (DUAL_X_CARRIAGE): Marlin puts T0 on the X1 carriage and every other tool on the X2
// carriage (X_APPLY_STEP drives X2 whenever the moving extruder isn't 0). hotend_offset[1].x is
// the X2 home position rather than an offset, and tool changes zero the X difference, so the
// X2 tools share one X and differ only by their Y/Z offsets, which are relative to T0.
//
int MachineModel::hotend_count() {
  #if HOTENDS == 0
    return 1;
  #else
    return std::min(HOTENDS, max_hotends);
  #endif
}

int MachineModel::carriage_count() { return ENABLED(DUAL_X_CARRIAGE) ? 2 : 1; }

int MachineModel::carriage_of(const int h) { return ENABLED(DUAL_X_CARRIAGE) && h > 0 ? 1 : 0; }

// Live offset of hotend h from its carriage origin, Marlin XYZ -> GL (x, z, -y). A +Z offset
// raises the nozzle. The X1 origin is the T0 nozzle; the X2 origin is the X2 nozzle X at T0's Y/Z.
glm::vec3 MachineModel::live_hotend_offset(const int h) {
  #if HOTENDS > 1
    if (h > 0 && h < hotend_count()) {
      #if ENABLED(DUAL_X_CARRIAGE)
        const glm::vec3 o = hotend_offset_mm(h);
        return { 0, o.z, -o.y };
      #else
        const glm::vec3 d = hotend_offset_mm(h) - hotend_offset_mm(0);
        return { d.x, d.z, -d.y };
      #endif
    }
  #else
    UNUSED(h);
  #endif
  return {};
}

//
// Bed probe, at Marlin's probe.offset (NOZZLE_TO_PROBE_OFFSET, M851, EEPROM) from the nozzle.
// This is the offset Marlin plans with, so the probe lines up with the mesh points during G29.
//
bool MachineModel::has_probe() { return ENABLED(HAS_BED_PROBE) && DISABLED(NOZZLE_AS_PROBE); }

glm::vec3 MachineModel::live_probe_offset() {
  #if HAS_BED_PROBE && DISABLED(NOZZLE_AS_PROBE)
    return { float(probe.offset.x), float(probe.offset.z), -float(probe.offset.y) };
  #else
    return {};
  #endif
}

// The parts are built for the offsets at build time. When M218, M851, M501 or the LCD
// changes an offset the model must be rebuilt (Visualisation::update checks each frame).
bool MachineModel::layout_changed() const {
  for (int h = 1; h < hotend_count(); ++h)
    if (live_hotend_offset(h) != hotend_layout[h]) return true;
  return has_probe() && live_probe_offset() != probe_layout;
}

// Carriage-local extent of the hotends' tips on one carriage (-1 = all), to size the carriage around them
void MachineModel::hotend_extent(glm::vec3& lo, glm::vec3& hi, const int carriage) const {
  bool first = true;
  lo = hi = {};
  for (int h = 0; h < hotend_count(); ++h) {
    if (carriage >= 0 && carriage_of(h) != carriage) continue;
    const glm::vec3 o = hotend_offset(h);
    lo = first ? o : glm::min(lo, o);
    hi = first ? o : glm::max(hi, o);
    first = false;
  }
}

// X range the nozzles travel. IDEX parks X1 at X_MIN_POS and X2 at its endstop, which is
// usually well past the bed, so the X axis is built to reach both.
void MachineModel::x_travel(float& lo, float& hi) const {
  lo = 0; hi = X_BED_SIZE;
  #if ENABLED(DUAL_X_CARRIAGE)
    lo = std::min(lo, float(X1_MIN_POS));
    hi = std::max(hi, dxc_x2_endstop());
  #endif
}

//
// Machine dimensions from the Marlin configuration
//
static constexpr float bed_x = X_BED_SIZE, bed_y = Y_BED_SIZE, bed_z = Z_MAX_POS - Z_MIN_POS;
// Top of the modeled bed plates, just under the bed surface mesh (at Y = 0, where the
// nozzle tip is at Z0) so the nozzle visibly meets the bed without depth fighting.
static constexpr float bed_top = -0.3f;

DeltaGeometry MachineModel::default_delta() {
  #if ENABLED(DELTA)
    #ifdef PRINTABLE_RADIUS
      constexpr float printable = PRINTABLE_RADIUS;
    #elif defined(DELTA_PRINTABLE_RADIUS)
      constexpr float printable = DELTA_PRINTABLE_RADIUS;
    #else
      constexpr float printable = X_BED_SIZE * 0.5f;
    #endif
    return { DELTA_RADIUS, DELTA_DIAGONAL_ROD, DELTA_HEIGHT, printable };
  #else
    return { 0, 0, 0, 0 }; // The Delta model is only available with DELTA
  #endif
}

MachineType machine_type_option = MACHINE_TYPE_COUNT;

MachineType machine_type_from_name(const char* name) {
  const std::string n = name;
  if (n == "bedslinger")                 return MACHINE_BEDSLINGER;
  if (n == "cube" || n == "corexy")      return MACHINE_CUBE;
  if (n == "delta")                      return MACHINE_DELTA;
  return MACHINE_TYPE_COUNT;
}

const char* MachineModel::type_name(const MachineType type) {
  switch (type) {
    case MACHINE_BEDSLINGER: return "Bedslinger";
    case MACHINE_CUBE:       return "Cube (Z-Bed)";
    case MACHINE_DELTA:      return "Delta";
    default:                 return "?";
  }
}

// The model must match the simulated kinematics: a DELTA build shows only the Delta,
// and Cartesian / Core builds can switch between the Cartesian-style layouts.
// TODO: BELTPRINTER model.
bool MachineModel::is_available(const MachineType type) {
  #if ENABLED(DELTA)
    return type == MACHINE_DELTA;
  #else
    return type < MACHINE_TYPE_COUNT && type != MACHINE_DELTA;
  #endif
}

MachineType MachineModel::default_type() {
  #if ENABLED(DELTA)
    return MACHINE_DELTA;
  #elif IS_CORE || ANY(MARKFORGED_XY, MARKFORGED_YX)
    return MACHINE_CUBE;
  #else
    return MACHINE_BEDSLINGER;
  #endif
}

renderer::mesh_id_t MachineModel::add_part(Geometry geometry, Animate animate, const bool in_bounds) {
  const auto id = renderer::create_mesh();
  auto mesh = renderer::get_mesh_by_id(id);
  auto buffer = VertexBuffer::create();
  geometry(*buffer);
  mesh->buffer_vector<vertex_data_t>().push_back(buffer);
  mesh->set_shader_program(program);
  mesh->m_shader_instance->set_uniform("u_model", &mesh->m_transform);
  mesh->m_shader_instance->set_uniform("u_tint", &no_tint);
  mesh->m_shader_instance->set_uniform("u_glow", &no_glow);
  mesh->m_visible = visible;
  if (in_bounds)
    for (auto& v : buffer->cdata()) {
      bounds_min = glm::min(bounds_min, v.position);
      bounds_max = glm::max(bounds_max, v.position);
    }
  parts.push_back({id, animate});
  return id;
}

// Every hotend's cold end (heat break + heatsink) on one carriage, baked into that carriage / effector part
void MachineModel::add_cold_ends(VertexBuffer& b, const int carriage) const {
  for (int h = 0; h < hotend_count(); ++h)
    if (carriage_of(h) == carriage) add_hotend_cold(b, hotend_offset(h));
}

// One heater block + nozzle per hotend, each following its carriage's nozzle at its offset.
// Each has its own tint and glow uniforms, updated per frame by set_hotend_look().
void MachineModel::add_hot_parts() {
  for (int h = 0; h < hotend_count(); ++h) {
    const glm::vec3 offset = hotend_offset(h);
    const bool on_x2 = carriage_of(h) == 1;
    const auto id = add_part([](VertexBuffer& b) { add_hotend_hot(b); },
      [offset, on_x2](renderer::Mesh& m, const MachinePose& p) { move_to(m, (on_x2 ? p.x2_world : p.nozzle_world) + offset); }, false);
    auto mesh = renderer::get_mesh_by_id(id);
    mesh->m_shader_instance->set_uniform("u_tint", &hotend_tint[h]);
    mesh->m_shader_instance->set_uniform("u_glow", &hotend_glow[h]);
  }
}

//
// Bed probe on the X1 carriage / effector: a 6mm cylinder whose tip is the trigger point at
// probe.offset from the T0 nozzle, so during G29 it sits right over each mesh point. A bracket
// joins its top to the carriage, using 'nearest' to find the closest carriage point (carriage-
// local XZ) and holding the bracket within the carriage's height range [mount_lo, mount_hi].
// A stowed BLTouch pulls its body probe_stow_raise up into the bracket.
//
void MachineModel::add_probe(std::function<glm::vec2(glm::vec2)> nearest, const float mount_lo, const float mount_hi) {
  if (!has_probe()) return;
  const glm::vec3 tip = probe_layout;
  const float top = tip.y + probe_length;                        // Body top when deployed; stowed, it rises through the bracket
  const float by = std::clamp(top, mount_lo, std::max(mount_lo, mount_hi - 4));
  add_part([=](VertexBuffer& b) {
    const glm::vec2 p { tip.x, tip.z }, c = nearest(p);
    const glm::vec2 lo = glm::min(p, c) - 5.0f, hi = glm::max(p, c) + 5.0f;
    add_box(b, {lo.x, by, lo.y}, {hi.x, by + 4, hi.y}, color::printed);
  }, [](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.nozzle_world); }, false);
  add_part([=](VertexBuffer& b) {
    add_cylinder(b, tip, tip + glm::vec3(0, probe_length, 0), probe_radius, color::probe, 16);
  }, [](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.nozzle_world + glm::vec3(0, p.probe_raise, 0)); }, false);
}

void MachineModel::set_hotend_look(const int h, const glm::vec4 tint, const float glow) {
  if (h < 0 || h >= max_hotends) return;
  hotend_tint[h] = tint;
  hotend_glow[h] = glow;
}

void MachineModel::build(const MachineType new_type, std::shared_ptr<renderer::ShaderProgram> shader) {
  destroy();
  type = new_type;
  program = shader;
  for (int h = 0; h < max_hotends; ++h) hotend_layout[h] = live_hotend_offset(h);
  probe_layout = live_probe_offset();
  bounds_min = glm::vec3(0, 0, -bed_y);
  bounds_max = glm::vec3(bed_x, bed_z, 0);
  switch (type) {
    case MACHINE_CUBE:  build_cube();  break;
    case MACHINE_DELTA: build_delta(); break;
    default:            build_bedslinger(); break;
  }
}

void MachineModel::destroy() {
  for (auto& part : parts) renderer::destroy_mesh(part.mesh);
  parts.clear();
}

void MachineModel::queue_render() const {
  for (auto& part : parts) renderer::render_mesh(part.mesh);
}

void MachineModel::set_visible(const bool vis) {
  visible = vis;
  for (auto& part : parts)
    if (auto mesh = renderer::get_mesh_by_id(part.mesh)) mesh->m_visible = vis;
}

// Where the bed and nozzle(s) are for a given T0 nozzle position (and, for IDEX, the X2 nozzle X)
MachinePose MachineModel::pose(const glm::vec3 nozzle, const float x2) const {
  MachinePose p;
  p.nozzle = nozzle;
  switch (type) {
    case MACHINE_BEDSLINGER: {
      // The gantry stays in the Y center; the bed slides to put the nozzle over its Y.
      // Hotends behind hotend 0 (Marlin +Y offset) push the carriage plate back, so the
      // whole group moves forward by that depth: the rearmost hotend sits where a single
      // hotend would, and the carriage plate stays centered on the X rods.
      glm::vec3 lo, hi;
      hotend_extent(lo, hi);
      p.bed_offset = { 0, 0, nozzle.y - bed_y * 0.5f - lo.z };
    } break;
    case MACHINE_CUBE:
      // The nozzle stays at the top; the bed drops as Z increases
      p.bed_offset = { 0, bed_z - nozzle.z, 0 };
      break;
    default: break; // Delta: the bed is fixed
  }
  p.nozzle_world = glm::vec3(nozzle.x, nozzle.z, -nozzle.y) + p.bed_offset;
  // The X2 carriage rides the same rods, so only its X differs from the X1 carriage origin
  p.x2_world = { x2 + p.bed_offset.x, p.nozzle_world.y, p.nozzle_world.z };
  return p;
}

void MachineModel::update(const MachinePose& p) {
  for (auto& part : parts) {
    if (!part.animate) continue;
    auto mesh = renderer::get_mesh_by_id(part.mesh);
    if (!mesh) continue;
    const glm::vec3 old_pos = mesh->m_position, old_scale = mesh->m_scale;
    const glm::quat old_rot = mesh->m_rotation;
    part.animate(*mesh, p);
    if (mesh->m_position != old_pos || mesh->m_scale != old_scale || mesh->m_rotation != old_rot)
      mesh->m_transform_dirty = true;
  }
}

//
// Bedslinger (i3 style), simple geometry sized from the config.
// Bed slides on Y; the X gantry rides two Z screws, one on each Z motor, with an
// 8mm smooth rod beside each screw on the motor mount. The X ends carry the
// Z-rod bearings and the screw nuts, so each nut trap sits over its motor's center.
//
void MachineModel::build_bedslinger() {
  constexpr float screw_gap = 17;       // Z screw to Z rod (X ends put the nut 17mm outboard of the bearing)
  constexpr float rod_clear = 45;       // Minimum Z rod distance outside the X travel (clears the X carriage)
  constexpr float motor = 42;           // NEMA17 body
  constexpr float screw_r = 4, zrod_r = 5, yrod_r = 4; // 8mm Z screws, 10mm Z rods, 8mm Y rods
  constexpr float w = 20;               // Frame member width
  constexpr float xend_lo = 15, xend_hi = 90;         // X-end bottom and top above the nozzle tip
  constexpr float xrod_lo = 30, xrod_hi = 75;         // X rod heights above the nozzle tip
  constexpr float yrod_y = -35;                       // Y rod height (below the bed carriage)
  const float yc = bed_y * 0.5f;                      // Nozzle plane is at world Z = -yc
  const float xrod_z = -yc - 22;                      // X rods, through the middle of the carriage
  const float rz = xrod_z - 20;                       // Z rods and screws, behind the X rods
  const float fz = rz - 32;                           // Frame plate, just behind the Z motors
  const float front = yc + 40, back = -bed_y - yc - 40; // Y legs span the bed travel

  // Frame: two side frames, each a Y base leg with the Z upright standing on it,
  // joined by front/back base cross members and the top bar. Everything is placed
  // from the bed size, X travel (IDEX parks X2 past the bed) and Z height, so the printer follows the config.
  float tx0, tx1;
  x_travel(tx0, tx1);
  constexpr float side = rod_clear + screw_gap + motor * 0.5f + 25;  // Frame outer edge to the X travel
  const float xl = tx0 - side, xr = tx1 + side;       // Frame outer edges
  const float leg[2] = { xl, xr - w };                // Left edge of each side frame
  // Z motors and screws are centered on the uprights; the Z rods are just inboard
  const float screw[2] = { leg[0] + w * 0.5f, leg[1] + w * 0.5f };
  const float rod[2] = { screw[0] + screw_gap, screw[1] - screw_gap };
  const float base_top = -18 - motor - 4;             // Top of the base, just under the Z motors
  const float bar_lo = bed_z + xend_hi + 70;          // Underside of the top bar
  const float bar_mid = bar_lo + w * 0.5f;            // Z tops are centered on the top bar

  add_part([&](VertexBuffer& b) {
    for (int s = 0; s < 2; ++s) {
      add_box(b, {leg[s], base_top - w, back}, {leg[s] + w, base_top, front}, color::frame);   // Y base leg
      add_box(b, {leg[s], base_top, fz - w * 0.5f}, {leg[s] + w, bar_lo, fz + w * 0.5f}, color::frame); // Z upright
    }
    add_box(b, {xl + w, base_top - w, front - w}, {xr - w, base_top, front}, color::frame);    // Front cross member
    add_box(b, {xl + w, base_top - w, back}, {xr - w, base_top, back + w}, color::frame);      // Back cross member
    add_box(b, {xl, bar_lo, fz - w * 0.5f}, {xr, bar_lo + w, fz + w * 0.5f}, color::frame);    // Top bar
    // Y smooth rods, held by printed Y corners standing on the front/back cross members
    for (const float x : { bed_x * 0.3f, bed_x * 0.7f }) {
      add_cylinder(b, {x, yrod_y, back + w * 0.5f}, {x, yrod_y, front - w * 0.5f}, yrod_r, color::rod);
      add_box(b, {x - 10, base_top, front - w}, {x + 10, yrod_y + 8, front}, color::printed);
      add_box(b, {x - 10, base_top, back}, {x + 10, yrod_y + 8, back + w}, color::printed);
    }
    // Y motor, on the back cross member, flush with the back of the frame
    add_block(b, {bed_x * 0.5f, base_top + motor * 0.5f, back + motor * 0.5f}, {motor, motor, motor}, color::dark);
    for (int s = 0; s < 2; ++s) {
      const float in = s ? -1.0f : 1.0f;  // Toward the bed
      // Z motor, centered on the upright, under the screw
      add_block(b, {screw[s], -18 - motor * 0.5f, rz}, {motor, motor, motor}, color::dark);
      // Z motor mount: caps the motor, holds the Z rod, and fastens to the upright
      add_box(b, {screw[s] - in * (motor * 0.5f + 2), -18, fz}, {rod[s] + in * (zrod_r + 8), -10, rz + motor * 0.5f + 2}, color::printed);
      // Z screw from the motor shaft; Z rod from the motor mount; both run up to the middle of the top bar
      add_cylinder(b, {screw[s], -10, rz}, {screw[s], bar_mid, rz}, screw_r, color::screw);
      add_cylinder(b, {rod[s], -10, rz}, {rod[s], bar_mid, rz}, zrod_r, color::rod);
      // Z top: centered on the top bar, reaching 1cm past the Z rod
      add_box(b, {screw[s] - in * 12, bar_mid - 8, fz}, {rod[s] + in * (zrod_r + 10), bar_mid + 8, rz + zrod_r + 10}, color::printed);
    }
  });

  // Bed carriage, Y bearing blocks on the Y rods, and heated bed, which slide on Y
  add_part([&](VertexBuffer& b) {
    add_box(b, {-15, -16, -bed_y - 15}, {bed_x + 15, -10, 15}, color::dark);
    add_box(b, {-10, -9, -bed_y - 10}, {bed_x + 10, bed_top, 10}, color::bed);
    for (const float x : { bed_x * 0.3f, bed_x * 0.7f })
      for (const float z : { -bed_y * 0.2f, -bed_y * 0.8f })
        add_box(b, {x - 11, yrod_y - yrod_r - 4, z - 17}, {x + 11, -16, z + 17}, color::printed);
  }, [](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.bed_offset); });

  // X axis: X ends (Z bearing + nut trap), X rods and X motor, which ride Z
  add_part([&](VertexBuffer& b) {
    for (int s = 0; s < 2; ++s) {
      const float in = s ? -1.0f : 1.0f;
      // Bearing body around the Z rod, reaching forward to clamp the X rods
      add_box(b, {rod[s] - 12, xend_lo, rz - 12}, {rod[s] + 12, xend_hi, xrod_z + 10}, color::printed);
      // Nut trap over the screw (and the Z motor center), reaching outboard from the
      // bearing body. The nut sits in the trap with its top 2mm proud.
      constexpr float trap_h = 18, nut_h = 6, nut_proud = 2;
      add_box(b, {screw[s] - in * 11, xend_lo, rz - 11}, {rod[s] - in * 12, xend_lo + trap_h, rz + 11}, color::printed);
      add_cylinder(b, {screw[s], xend_lo + trap_h + nut_proud - nut_h, rz}, {screw[s], xend_lo + trap_h + nut_proud, rz}, 7.5f, color::rod, 6);
    }
    add_cylinder(b, {rod[0], xrod_lo, xrod_z}, {rod[1], xrod_lo, xrod_z}, 4, color::rod);
    add_cylinder(b, {rod[0], xrod_hi, xrod_z}, {rod[1], xrod_hi, xrod_z}, 4, color::rod);
    // X motor on the left X end, outboard of the Z screw, 2cm behind the X rod plane.
    // IDEX drives the X2 carriage with its own motor on the right X end.
    for (int s = 0; s < carriage_count(); ++s) {
      const float in = s ? -1.0f : 1.0f;
      add_block(b, {screw[s] - in * 14, (xrod_lo + xrod_hi) * 0.5f, xrod_z - 10 + motor * 0.5f}, {motor, motor, motor}, color::dark);
    }
  }, [](renderer::Mesh& m, const MachinePose& p) { move_to(m, {0, p.nozzle_world.y, 0}); });

  // X carriage(s) and the hotends' cold ends, which follow their nozzles.
  // Each carriage plate spans its own hotends and sits just behind the rearmost hotend
  // of either carriage, so both plates ride the same X rods.
  glm::vec3 all_lo, all_hi;
  hotend_extent(all_lo, all_hi);
  const float plate_front = all_lo.z - 10;
  for (int c = 0; c < carriage_count(); ++c) {
    glm::vec3 lo, hi;
    hotend_extent(lo, hi, c);
    add_part([&](VertexBuffer& b) {
      add_box(b, {lo.x - 25, lo.y + 18, all_lo.z - 34}, {hi.x + 25, hi.y + 88, plate_front}, color::printed);
      add_cold_ends(b, c);
      // Hotends in front of the rearmost one hang from a nozzle holder: one block reaching
      // forward from the plate at the top of the heatsinks, spanning every hotend in X
      if (hi.z > all_lo.z) add_box(b, {lo.x - 12, lo.y + 38, plate_front}, {hi.x + 12, hi.y + 48, hi.z + 12}, color::printed);
    }, c ? Animate([](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.x2_world); })
         : Animate([](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.nozzle_world); }), false);
    // The probe hangs from the X1 carriage plate, bracketed to its nearest point
    if (c == 0) {
      const glm::vec2 plo { lo.x - 25, all_lo.z - 34 }, phi { hi.x + 25, plate_front };
      add_probe([=](const glm::vec2 q) { return glm::clamp(q, plo, phi); }, lo.y + 18, hi.y + 88);
    }
  }
  add_hot_parts();
}

//
// Cube (CoreXY style): head moves in XY at the top, bed moves down on Z
//
void MachineModel::build_cube() {
  constexpr float w = 20;                             // Frame member width
  constexpr float motor = 42;                         // NEMA17 body
  constexpr float rod_r = 4;                          // 8mm rods
  constexpr float zrod_r = 6, zscrew_r = 5;           // 12mm Z rods, 10mm Z lead screw
  float tx0, tx1;
  x_travel(tx0, tx1);                                 // IDEX parks X2 past the bed, so the gantry reaches it
  const float x0 = tx0 - 60, x1 = tx1 + 60, y0 = -60, y1 = bed_z + 110, z0 = -bed_y - 60, z1 = 60;
  const float top = bed_z + 55;                       // Y rail height
  const float xrod_y = top + 15;                      // X rods, side by side, above the Y rails
  constexpr float xrod_dz = 14;                       // X rods in front of / behind the carriage center
  const float yrail[2] = { tx0 - 30, tx1 + 30 };
  const float in_face[2] = { x0 + w, x1 - w };        // Inner faces of the left / right frame sides
  const float zr = z0 + w + 10;                       // Z rods and screw, just in front of the back frame
  const float zrod_top = y1 - 4;                      // Z rods end inside their top mounts

  add_part([&](VertexBuffer& b) {
    // Uprights
    for (const float x : {x0, x1 - w}) for (const float z : {z0, z1 - w})
      add_box(b, {x, y0, z}, {x + w, y1, z + w}, color::frame);
    // Top and bottom rings
    for (const float y : {y0, y1 - w}) {
      add_box(b, {x0 + w, y, z0}, {x1 - w, y + w, z0 + w}, color::frame);
      add_box(b, {x0 + w, y, z1 - w}, {x1 - w, y + w, z1}, color::frame);
      add_box(b, {x0, y, z0 + w}, {x0 + w, y + w, z1 - w}, color::frame);
      add_box(b, {x1 - w, y, z0 + w}, {x1, y + w, z1 - w}, color::frame);
    }
    for (int s = 0; s < 2; ++s) {
      const float in = s ? -1.0f : 1.0f;  // Toward the bed
      // Y rail, running 5mm past the inner faces of the uprights at each end
      add_cylinder(b, {yrail[s], top, z0 + w - 5}, {yrail[s], top, z1 - w + 5}, rod_r, color::rod);
      // Y rail mounts, from the inner face of each upright to the rail end, in line with the uprights
      for (const float ze : {z0 + w * 0.5f, z1 - w * 0.5f})
        add_box(b, {in_face[s], top - 10, ze - 10}, {yrail[s] + in * 10, top + 10, ze + 10}, color::printed);
      // XY motors in the back corners, bottoms level with the underside of the top ring
      add_block(b, {in_face[s] + in * motor * 0.5f, y1 - w + motor * 0.5f, z0 + w + motor * 0.5f}, {motor, motor, motor}, color::dark);
    }
    // Z rods, with top and bottom mounts fastened to the inside of the back crossbeams, in line with them
    for (const float x : { bed_x * 0.25f, bed_x * 0.75f }) {
      add_cylinder(b, {x, y0 + 4, zr}, {x, zrod_top, zr}, zrod_r, color::rod);
      for (const float y : { y0, y1 - w })
        add_box(b, {x - 12, y, z0 + w}, {x + 12, y + w, zr + zrod_r + 6}, color::printed);
    }
    // Z motor, with the lead screw rising from its shaft to just under the top crossbeam
    add_block(b, {bed_x * 0.5f, y0 + w + motor * 0.5f, zr}, {motor, motor, motor}, color::dark);
    add_cylinder(b, {bed_x * 0.5f, y0 + w + motor, zr}, {bed_x * 0.5f, y1 - w - 2, zr}, zscrew_r, color::screw);
  });

  // Bed, which rides Z
  add_part([&](VertexBuffer& b) {
    add_box(b, {-10, -9, -bed_y - 10}, {bed_x + 10, bed_top, 10}, color::bed);
    // Bed arms under the bed, 10mm wide and 5mm tall, from each Z rod to the middle of the bed
    for (const float x : { bed_x * 0.25f, bed_x * 0.75f })
      add_box(b, {x - 5, -14, zr - zrod_r - 2}, {x + 5, -9, -bed_y * 0.5f}, color::printed);
    // Collars on the Z rods and the lead screw (nut at the center), with a tab to the bed
    for (const float x : { bed_x * 0.25f, bed_x * 0.5f, bed_x * 0.75f })
      add_cylinder(b, {x, -24, zr}, {x, -4, zr}, 10, color::printed);
    add_box(b, {bed_x * 0.5f - 5, -14, zr + 9}, {bed_x * 0.5f + 5, -9, -bed_y - 10}, color::printed);
  }, [](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.bed_offset); });

  // X gantry, which follows the nozzle on Y: two X rods side by side (front and back)
  // and the Y blocks riding the Y rails, deeper than they are tall
  add_part([&](VertexBuffer& b) {
    for (const float dz : { -xrod_dz, xrod_dz })
      add_cylinder(b, {yrail[0], xrod_y, dz}, {yrail[1], xrod_y, dz}, rod_r, color::rod);
    for (int s = 0; s < 2; ++s) {
      const float in = s ? -1.0f : 1.0f;
      add_box(b, {yrail[s] - in * 15, top - 12, -30}, {yrail[s] + in * 15, xrod_y + 12, 30}, color::printed);
    }
  }, [](renderer::Mesh& m, const MachinePose& p) { move_to(m, {0, 0, p.nozzle_world.z}); });

  // Toolhead(s), which follow their nozzles (at the top of the bed travel), each enclosing its hotends' heatsinks.
  // With IDEX both toolheads ride the same X rods, so each spans the depth of every hotend.
  glm::vec3 all_lo, all_hi;
  hotend_extent(all_lo, all_hi);
  for (int c = 0; c < carriage_count(); ++c) {
    glm::vec3 lo, hi;
    hotend_extent(lo, hi, c);
    const glm::vec3 blo { lo.x - 28, lo.y + 30, all_lo.z - 28 }, bhi { hi.x + 28, hi.y + 85, all_hi.z + 28 };
    add_part([&](VertexBuffer& b) {
      add_box(b, blo, bhi, color::printed);
      add_cold_ends(b, c);
    }, c ? Animate([](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.x2_world); })
         : Animate([](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.nozzle_world); }), false);
    // The probe mounts to the side of the X1 toolhead nearest to it
    if (c == 0) {
      const glm::vec2 plo { blo.x, blo.z }, phi { bhi.x, bhi.z };
      add_probe([=](const glm::vec2 q) { return glm::clamp(q, plo, phi); }, blo.y, bhi.y);
    }
  }
  add_hot_parts();
}

//
// Delta: three towers with carriages, six arms, and an effector
//
void MachineModel::build_delta() {
  constexpr float joint_y = 14;      // Arm joints above the nozzle tip
  constexpr float arm_spacing = 25;  // Half the gap between paired arms
  constexpr float arm_r = 3;         // Arm (connecting rod) radius
  constexpr float ball_r = 5.5f;     // 11mm ball joints at both ends of every arm
  constexpr float bar_r = 4;         // 8mm bars joining each arm pair (effector and carriages)
  constexpr float bar_half = arm_spacing + arm_r - 1.5f; // Pair gap + arm diameter, less 3mm for the balls
  const glm::vec2 center { bed_x * 0.5f, bed_y * 0.5f };   // Marlin XY
  const float delta_radius = delta.radius, delta_rod = delta.diagonal_rod;
  constexpr float tower_w = 30, tower_d = 20;               // Tower extrusion: wide face toward the center
  constexpr float carriage_d = 11;                          // Carriage thickness, front (joints) to back (tower face)
  const float tower_r = delta_radius + 60;                  // Tower columns
  const float carriage_r = tower_r - tower_d * 0.5f - carriage_d; // Carriage joints, on the carriage's front face
  // The arms span DELTA_RADIUS horizontally at the center, so the effector joints are that far inside the carriage joints
  const float effector_r = carriage_r - delta_radius;       // Effector joint radius
  const float home_h = delta.height + std::sqrt(std::max(0.0f, delta_rod * delta_rod - delta_radius * delta_radius));
  const float top = home_h + joint_y + 70 - 14;              // Top beam centerline (its top 70mm above the home joints)

  // Tower angles as in Marlin (A front-left, B front-right, C back)
  constexpr float angles[3] = { 210, 330, 90 };
  constexpr float beam_r = 14;                              // Top and base beam radius
  glm::vec3 dir[3];   // Outward direction (GL)
  glm::vec2 tower[3]; // Tower position at DELTA_RADIUS (Marlin XY)
  for (int i = 0; i < 3; ++i) {
    const float a = glm::radians(angles[i]);
    dir[i] = { std::cos(a), 0, -std::sin(a) };
    tower[i] = center + glm::vec2(std::cos(a), std::sin(a)) * delta_radius;
  }
  const glm::vec3 c3 { center.x, 0, -center.y };

  // Carriage height: the simulated tower stepper position when available
  // (Delta steppers are linear, so they give the carriage heights directly),
  // otherwise inverse kinematics from the nozzle.
  auto carriage_height = [=](const int i, const MachinePose& p) {
    if (p.has_towers) return p.towers[i];
    const glm::vec2 d = tower[i] - glm::vec2(p.nozzle);
    return p.nozzle.z + std::sqrt(std::max(0.0f, delta_rod * delta_rod - glm::dot(d, d)));
  };

  // Frame: towers, base and top triangles, round bed
  add_part([&](VertexBuffer& b) {
    constexpr float base_y = -45;                           // Base beam centerline
    glm::vec3 column[3];
    for (int i = 0; i < 3; ++i) {
      column[i] = c3 + dir[i] * tower_r;
      // Tower from the bottom of the base beams to the top of the top beams,
      // turned so its wide face points at the center
      const float lo = base_y - beam_r, hi = top + beam_r;
      add_block_facing(b, {column[i].x, (lo + hi) * 0.5f, column[i].z}, -dir[i], {tower_w, hi - lo, tower_d}, color::frame);
      // Tower motor, standing on top of the tower, centered on it and facing the center
      constexpr float motor = 42;                           // NEMA17 body
      add_block_facing(b, {column[i].x, hi + motor * 0.5f, column[i].z}, -dir[i], {motor, motor, motor}, color::dark);
    }
    for (int i = 0; i < 3; ++i) {
      const glm::vec3 a = column[i], z = column[(i + 1) % 3];
      add_cylinder(b, {a.x, base_y, a.z}, {z.x, base_y, z.z}, beam_r, color::frame, 8);
      add_cylinder(b, {a.x, top, a.z}, {z.x, top, z.z}, beam_r, color::frame, 8);
    }
    add_cylinder(b, c3 + glm::vec3(0, -9, 0), c3 + glm::vec3(0, bed_top, 0), delta.printable_radius + 10, color::bed, 48);
  });

  // Carriages ride the towers; each faces the center
  for (int i = 0; i < 3; ++i) {
    add_part([&](VertexBuffer& b) {
      add_box(b, {-35, -20, 0}, {35, 25, carriage_d}, color::printed);
      add_cylinder(b, {-bar_half, 0, 0}, {bar_half, 0, 0}, bar_r, color::rod, 12);
      // Ball joints for the arm pair (local X is along the tower's tangent)
      for (const float side : {-1.0f, 1.0f}) add_sphere(b, {side * arm_spacing, 0, 0}, ball_r, color::joint);
    }, [=](renderer::Mesh& m, const MachinePose& p) {
      const float h = carriage_height(i, p);
      m.m_position = c3 + dir[i] * carriage_r + glm::vec3(0, h + joint_y, 0);
      m.m_rotation = glm::angleAxis(std::atan2(dir[i].x, dir[i].z), glm::vec3(0, 1, 0));
    }, false);
  }

  // Arms: unit cylinders stretched from the effector joint to the carriage joint
  for (int i = 0; i < 3; ++i) for (const float side : {-1.0f, 1.0f}) {
    add_part([&](VertexBuffer& b) {
      add_cylinder(b, {0, 0, 0}, {0, 1, 0}, arm_r, color::dark, 8);
    }, [=](renderer::Mesh& m, const MachinePose& p) {
      const glm::vec3 tangent { -dir[i].z, 0, dir[i].x };
      const float h = carriage_height(i, p);
      const glm::vec3 lower = p.nozzle_world + glm::vec3(0, joint_y, 0) + dir[i] * effector_r + tangent * (arm_spacing * side);
      const glm::vec3 upper = c3 + dir[i] * carriage_r + glm::vec3(0, h + joint_y, 0) + tangent * (arm_spacing * side);
      const glm::vec3 span = upper - lower;
      const float len = glm::length(span);
      m.m_position = lower;
      m.m_scale = { 1, len, 1 };
      if (len > 0) m.m_rotation = glm::quat(glm::vec3(0, 1, 0), span / len);
    }, false);
  }

  // Effector and hotend
  add_part([&](VertexBuffer& b) {
    add_cylinder(b, {0, joint_y - 5, 0}, {0, joint_y + 5, 0}, effector_r + 8, color::printed, 6);
    // At each arm pair: a bar spanning the pair (pair gap + arm diameter) with ball joints at its ends.
    // Same placement as the arms' lower ends: joint_y up, effector_r out along dir, +/-arm_spacing along the tangent.
    for (int i = 0; i < 3; ++i) {
      const glm::vec3 tangent { -dir[i].z, 0, dir[i].x };
      const glm::vec3 mid = glm::vec3(0, joint_y, 0) + dir[i] * effector_r;
      add_cylinder(b, mid - tangent * bar_half, mid + tangent * bar_half, bar_r, color::printed, 12);
      for (const float side : {-1.0f, 1.0f}) add_sphere(b, mid + tangent * (arm_spacing * side), ball_r, color::joint);
    }
    add_cold_ends(b);
  }, [](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.nozzle_world); }, false);
  // The probe mounts to the effector plate's rim
  const float rim = effector_r + 8;
  add_probe([=](const glm::vec2 q) { const float l = glm::length(q); return l > rim ? q * (rim / l) : q; }, joint_y - 5, joint_y + 5);
  add_hot_parts();
}

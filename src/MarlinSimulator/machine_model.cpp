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
#include "scara_frame.h"

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

// Flat band (a belt run) from a to b, 'width' wide across the world X axis. Drawn from both sides: with
// back-face culling a single quad would vanish when seen from behind.
static void add_ribbon(VertexBuffer& buf, const glm::vec3 a, const glm::vec3 b, const float width, const glm::vec4 col) {
  const glm::vec3 x { width * 0.5f, 0, 0 };
  const glm::vec3 n = glm::normalize(glm::cross(glm::vec3(1, 0, 0), b - a));
  add_quad(buf, a - x, a + x, b + x, b - x,  n, col);
  add_quad(buf, a - x, a + x, b + x, b - x, -n, col);
}

// Hotend parts. The hot end (nozzle and heater block, tip at the origin) is its own mesh
// so it can be tinted by temperature; the cold end (heat break and heatsink, over a tip
// at 'at') is part of the carriage.
static void add_hotend_hot(VertexBuffer& buf) {
  add_cylinder(buf, {0, 0, 0}, {0, 3, 0}, 1.5f, color::brass, 12);
  add_cylinder(buf, {0, 3, 0}, {0, 6, 0}, 4.0f, color::brass, 6);
  add_box(buf, {-8, 6, -6}, {12, 17, 6}, color::heater);
}
constexpr float hotend_cold_top = 46;   // Height of the top of a hotend's cold end above its nozzle tip
static void add_hotend_cold(VertexBuffer& buf, const glm::vec3 at) {
  add_cylinder(buf, at + glm::vec3(0, 17, 0), at + glm::vec3(0, 22, 0), 1.5f, color::rod, 8);
  for (int i = 0; i < 6; ++i) {
    const float y = 22.0f + i * 4.0f;
    add_cylinder(buf, at + glm::vec3(0, y, 0), at + glm::vec3(0, y + 2.0f, 0), 11.0f, color::frame, 20);
  }
  add_cylinder(buf, at + glm::vec3(0, 22, 0), at + glm::vec3(0, hotend_cold_top, 0), 4.0f, color::frame, 12);
}

static void move_to(renderer::Mesh& m, const glm::vec3 pos) { m.m_position = pos; }

// SCARA: a part fixed to the forearm turns about the vertical through its origin. It is as modeled
// (in Marlin's XY frame) when the forearm points along +Y, and turns with the forearm's direction from there.
// A belt printer's toolhead also turns with the frame's tilt (head_turn, identity for other types).
static glm::quat arm_turn(const MachinePose& p) { return p.head_turn * glm::angleAxis(glm::radians(p.scara_angles.y - 90.0f), glm::vec3(0, 1, 0)); }
static void turn_with_arm(renderer::Mesh& m, const MachinePose& p, const bool turns) {
  m.m_rotation = turns ? arm_turn(p) : glm::quat(1, 0, 0, 0);
}

//
// Parts shared by the printer models
//
constexpr float nema17 = 42;  // NEMA17 motor body (a cube, for simplicity)

static void add_motor(VertexBuffer& b, const glm::vec3 center) { add_block(b, center, glm::vec3(nema17), color::dark); }
static void add_motor_facing(VertexBuffer& b, const glm::vec3 center, const glm::vec3 facing) {
  add_block_facing(b, center, facing, glm::vec3(nema17), color::dark);
}

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

// Carriage-local X extent of carriage c: 'margin' past its outermost hotends, and on X1 the probe bracket
void MachineModel::carriage_span(const int c, const float margin, float& lo, float& hi) const {
  glm::vec3 elo, ehi;
  hotend_extent(elo, ehi, c);
  lo = elo.x - margin;
  hi = ehi.x + margin;
  if (c == 0 && has_probe()) {
    lo = std::min(lo, probe_layout.x - 5);  // The probe bracket reaches 5mm past the probe center
    hi = std::max(hi, probe_layout.x + 5);
  }
}

// World X of the carriages' outer sides at the ends of X travel: X1 at its minimum on the left,
// and X2 at its endstop (IDEX) or X1 at X_MAX_POS on the right
void MachineModel::carriage_reach(const float margin, float& left, float& right) const {
  float lo, hi;
  carriage_span(0, margin, lo, hi);
  #if ENABLED(DUAL_X_CARRIAGE)
    left = float(X1_MIN_POS) + lo;
    carriage_span(1, margin, lo, hi);
    right = dxc_x2_endstop() + hi;
  #else
    left = float(X_MIN_POS) + lo;
    right = float(X_MAX_POS) + hi;
  #endif
}

//
// Machine dimensions from the Marlin configuration
//
static constexpr float bed_x = X_BED_SIZE, bed_y = Y_BED_SIZE, bed_z = Z_MAX_POS - Z_MIN_POS;
// Top of the modeled bed plates, just under the bed surface mesh (at Y = 0, where the
// nozzle tip is at Z0) so the nozzle visibly meets the bed without depth fighting.
static constexpr float bed_top = -0.3f;

//
// Z drives of the Cube's bed, one per Z stepper, in Marlin XY.
//
// The screw positions aren't in the config directly, but the G34 (Z_STEPPER_AUTO_ALIGN) settings
// describe them: Z_STEPPER_ALIGN_STEPPER_XY gives the screws themselves, and otherwise each probe
// point of Z_STEPPER_ALIGN_XY (or the default points from Z_STEPPERS_ORIENTATION, as computed in
// z_stepper_align.cpp) sits near its stepper, so its screw is placed just past the nearest bed edge.
// With one Z stepper the screw is centered behind the bed.
//
std::vector<glm::vec2> MachineModel::z_drive_xy() {
  constexpr float out = 30;       // Screw center past the bed edge (bed plate lip 10mm + nut collar 10mm + 10mm)
  std::vector<glm::vec2> xy;
  #if NUM_Z_STEPPERS < 2
    xy.push_back({ bed_x * 0.5f, bed_y + out });
  #else
    #if HAS_Z_STEPPER_ALIGN_STEPPER_XY
      constexpr xy_pos_t pts[] = Z_STEPPER_ALIGN_STEPPER_XY;
      for (const auto& p : pts) xy.push_back({ float(p.x), float(p.y) });
    #elif defined(Z_STEPPER_ALIGN_XY)
      constexpr xy_pos_t pts[] = Z_STEPPER_ALIGN_XY;
      for (const auto& p : pts) xy.push_back({ float(p.x), float(p.y) });
    #else
      // Marlin's default points, with the probe's reach approximated by the probing margins
      #ifdef Z_STEPPERS_ORIENTATION
        constexpr int o = Z_STEPPERS_ORIENTATION;
      #else
        constexpr int o = 0;
      #endif
      const float l = PROBING_MARGIN_LEFT, r = bed_x - (PROBING_MARGIN_RIGHT),
                  f = PROBING_MARGIN_FRONT, k = bed_y - (PROBING_MARGIN_BACK),
                  cx = bed_x * 0.5f, cy = bed_y * 0.5f;
      const glm::vec2 sw { l, f }, nw { l, k }, ne { r, k }, se { r, f };
      UNUSED(sw); UNUSED(nw); UNUSED(ne); UNUSED(se); UNUSED(cx); UNUSED(cy);
      #if NUM_Z_STEPPERS == 2
        if (o) xy = { { cx, f }, { cx, k } };   // Front and back
        else   xy = { { l, cy }, { r, cy } };   // Left and right
      #elif NUM_Z_STEPPERS == 3
        switch (o) {                            // Two corners and the middle of the opposite side
          default: xy = { sw, se, { cx, k } }; break;
          case 1:  xy = { nw, sw, { r, cy } }; break;
          case 2:  xy = { ne, nw, { cx, f } }; break;
          case 3:  xy = { se, ne, { l, cy } }; break;
        }
      #else
        const glm::vec2 ring[4] = { sw, nw, ne, se };   // Clockwise from the first corner
        for (int i = 0; i < 4; ++i) xy.push_back(ring[(o + i) % 4]);
      #endif
    #endif
    // Screws under (or too close to) the bed move out past the nearest bed edge, preferring the sides
    constexpr float clear = 20;
    for (auto& p : xy) {
      if (p.x <= -clear || p.x >= bed_x + clear || p.y <= -clear || p.y >= bed_y + clear) continue;
      const float dl = p.x, dr = bed_x - p.x, df = p.y, db = bed_y - p.y;
      if (std::min(dl, dr) <= std::min(df, db)) p.x = dl <= dr ? -out : bed_x + out;
      else                                      p.y = df <= db ? -out : bed_y + out;
    }
  #endif
  return xy;
}

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

ScaraGeometry MachineModel::default_scara() {
  #if ENABLED(SCARA)
    // The shoulder is hardware, fixed behind the bed's center; SCARA_OFFSET only says where it is in Marlin's XY (scara_frame.h)
    return { float(SCARA_LINKAGE_1), float(SCARA_LINKAGE_2), { scara_frame::shoulder_x, scara_frame::shoulder_y } };
  #else
    return { 0, 0, {} }; // The SCARA model is only available with SCARA
  #endif
}

MachineType machine_type_option = MACHINE_TYPE_COUNT;
float belt_angle_option = 0;

MachineType machine_type_from_name(const char* name) {
  const std::string n = name;
  if (n == "bedslinger")                 return MACHINE_BEDSLINGER;
  if (n == "cube" || n == "corexy")      return MACHINE_CUBE;
  if (n == "delta")                      return MACHINE_DELTA;
  if (n == "scara")                      return MACHINE_SCARA;
  if (n == "belt" || n == "beltprinter") return MACHINE_BELT;
  return MACHINE_TYPE_COUNT;
}

const char* MachineModel::type_name(const MachineType type) {
  switch (type) {
    case MACHINE_BEDSLINGER: return "Bedslinger";
    case MACHINE_CUBE:       return "Cube (Z-Bed)";
    case MACHINE_DELTA:      return "Delta";
    case MACHINE_SCARA:      return "SCARA";
    case MACHINE_BELT:       return "Belt Printer";
    default:                 return "?";
  }
}

// The model must match the simulated kinematics: a DELTA build shows only the Delta,
// a SCARA build only the SCARA, and Cartesian / Core builds can switch between the
// Cartesian-style layouts. A BELTPRINTER build shows only the belt printer.
bool MachineModel::is_available(const MachineType type) {
  #if ENABLED(DELTA)
    return type == MACHINE_DELTA;
  #elif ENABLED(SCARA)
    return type == MACHINE_SCARA;
  #elif ENABLED(BELTPRINTER)
    return type == MACHINE_BELT;
  #else
    return type < MACHINE_TYPE_COUNT && type != MACHINE_DELTA && type != MACHINE_SCARA && type != MACHINE_BELT;
  #endif
}

MachineType MachineModel::default_type() {
  #if ENABLED(DELTA)
    return MACHINE_DELTA;
  #elif ENABLED(SCARA)
    return MACHINE_SCARA;
  #elif ENABLED(BELTPRINTER)
    return MACHINE_BELT;
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
  mesh->m_shader_instance->set_uniform("u_dim", &no_dim);
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
void MachineModel::add_hot_parts(const bool turns) {
  for (int h = 0; h < hotend_count(); ++h) {
    const glm::vec3 offset = hotend_offset(h);
    const bool on_x2 = carriage_of(h) == 1;
    const auto id = add_part([](VertexBuffer& b) { add_hotend_hot(b); },
      [offset, on_x2, turns](renderer::Mesh& m, const MachinePose& p) {
        // On a SCARA forearm the hotend offsets turn about the T0 nozzle with the arm
        move_to(m, (on_x2 ? p.x2_world : p.nozzle_world) + (turns ? arm_turn(p) * offset : offset));
        turn_with_arm(m, p, turns);
      }, false);
    auto mesh = renderer::get_mesh_by_id(id);
    mesh->m_shader_instance->set_uniform("u_tint", &hotend_tint[h]);
    mesh->m_shader_instance->set_uniform("u_glow", &hotend_glow[h]);
    mesh->m_shader_instance->set_uniform("u_dim", &hotend_dim[h]);
  }
}

//
// Bed probe on the X1 carriage / effector: a 6mm cylinder whose tip is the trigger point at
// probe.offset from the T0 nozzle, so during G29 it sits right over each mesh point. A bracket
// joins its top to the carriage, using 'nearest' to find the closest carriage point (carriage-
// local XZ) and holding the bracket within the carriage's height range [mount_lo, mount_hi].
// A stowed BLTouch pulls its body probe_stow_raise up into the bracket.
//
void MachineModel::add_probe(std::function<glm::vec2(glm::vec2)> nearest, const float mount_lo, const float mount_hi, const bool reach_down, const bool turns) {
  if (!has_probe()) return;
  const glm::vec3 tip = probe_layout;
  const float top = tip.y + probe_length;                        // Body top when deployed; stowed, it rises through the bracket
  const float by = std::clamp(top, mount_lo, std::max(mount_lo, mount_hi - 4));
  add_part([=](VertexBuffer& b) {
    const glm::vec2 p { tip.x, tip.z }, c = nearest(p);
    const glm::vec2 lo = glm::min(p, c) - 5.0f, hi = glm::max(p, c) + 5.0f;
    // Reaching down, the block's bottom is exactly the top of the deployed probe and it runs up to the mount
    const float y0 = reach_down ? std::min(top, by) : by, y1 = reach_down ? std::max(top + 4, by + 4) : by + 4;
    add_box(b, {lo.x, y0, lo.y}, {hi.x, y1, hi.y}, color::printed);
  }, [=](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.nozzle_world); turn_with_arm(m, p, turns); }, false);
  add_part([=](VertexBuffer& b) {
    add_cylinder(b, tip, tip + glm::vec3(0, probe_length, 0), probe_radius, color::probe, 16);
  }, [=](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.nozzle_world + glm::vec3(0, p.probe_raise, 0)); turn_with_arm(m, p, turns); }, false);
}

void MachineModel::set_hotend_look(const int h, const glm::vec4 tint, const float glow, const float dim) {
  if (h < 0 || h >= max_hotends) return;
  hotend_tint[h] = tint;
  hotend_glow[h] = glow;
  hotend_dim[h] = dim;
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
    case MACHINE_SCARA: build_scara(); break;
    case MACHINE_BELT:  build_belt(); break;
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
    case MACHINE_SCARA:
      // The nozzle stays at the top; the bed drops as Z increases
      p.bed_offset = { 0, bed_z - nozzle.z, 0 };
      break;
    case MACHINE_BELT: {
      // The nozzle rides the tilted gantry (X across, Y up the slope) and never moves with Z: the belt does.
      // The belt, and the print on it, travel with Z.
      p.bed_offset = { 0, 0, nozzle.z };
      p.nozzle_world = belt_nozzle_world(nozzle);
      p.x2_world = belt_nozzle_world({ x2, nozzle.y, nozzle.z });
      p.head_turn = glm::quat_cast(belt_frame_rotation());
      p.scara_angles = { 0, 90 };   // No arm turn: arm_turn(p) is then just head_turn
      return p;
    }
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
// Assembly, left (s = 0) and right (s = 1) mirrored with in(s) pointing toward the bed:
//   Frame (fixed):  two side frames (Y base leg + Z upright) joined by the front and back cross
//                   members and the top bar; Y rods on printed corners; the Y motor at the back;
//                   and a Z drive (motor, mount, screw, rod, Z top) on each upright.
//   Bed (Y):        carriage plate, heated bed and Y bearing blocks.
//   X axis (Z):     an X end on each Z rod, the X rods between them, and the X motor(s).
//   Carriages (XZ): one plate per X carriage holding its hotends, and the probe on X1.
//
namespace bedslinger {
  constexpr float screw_gap = 17;       // Z screw to Z rod (X ends put the nut 17mm outboard of the bearing)
  constexpr float rod_clear = 45;       // Minimum Z rod distance outside the X travel (clears the X carriage)
  constexpr float screw_r = 4, zrod_r = 5, yrod_r = 4, xrod_r = 4; // 8mm Z screws, 10mm Z rods, 8mm Y and X rods
  constexpr float w = 20;               // Frame member width
  constexpr float xend_lo = 15, xend_hi = 90;   // X-end bottom and top above the nozzle tip
  constexpr float xrod_lo = 30, xrod_hi = 75;   // X rod heights above the nozzle tip
  constexpr float yrod_y = -35;                 // Y rod height (below the bed carriage)
  constexpr float motor_top = -18;              // Z motor tops, just under the bed carriage
  constexpr float mount_top = -10;              // Z motor mount tops; the screws and rods start here

  static float in(const int s) { return s ? -1.0f : 1.0f; }  // Toward the bed from side s

  // Frame layout from the bed size, X travel (IDEX parks X2 past the bed) and Z height
  struct Layout {
    float xrod_z, rz, fz;         // X rods, Z rods / screws, and frame plate depths (GL Z)
    float front, back;            // Y legs span the bed travel
    float xl, xr;                 // Frame outer edges
    float leg[2], screw[2], rod[2];
    float base_top, bar_lo, bar_mid;
    Layout(const float tx0, const float tx1) {
      const float yc = bed_y * 0.5f;              // Nozzle plane is at world Z = -yc
      xrod_z = -yc - 22;                          // X rods, through the middle of the carriage
      rz = xrod_z - 20;                           // Z rods and screws, behind the X rods
      fz = rz - 32;                               // Frame plate, just behind the Z motors
      front = yc + 40; back = -bed_y - yc - 40;
      constexpr float side = rod_clear + screw_gap + nema17 * 0.5f + 25;  // Frame outer edge to the X travel
      xl = tx0 - side; xr = tx1 + side;
      leg[0] = xl; leg[1] = xr - w;               // Left edge of each side frame
      for (int s = 0; s < 2; ++s) {
        screw[s] = leg[s] + w * 0.5f;             // Z motors and screws are centered on the uprights,
        rod[s] = screw[s] + in(s) * screw_gap;    // and the Z rods are just inboard
      }
      base_top = motor_top - nema17 - 4;          // Top of the base, just under the Z motors
      bar_lo = bed_z + xend_hi + 70;              // Underside of the top bar
      bar_mid = bar_lo + w * 0.5f;                // Z tops are centered on the top bar
    }
  };

  // Side frame: a Y base leg with the Z upright standing on it
  static void add_side_frame(VertexBuffer& b, const Layout& lay, const int s) {
    add_box(b, {lay.leg[s], lay.base_top - w, lay.back}, {lay.leg[s] + w, lay.base_top, lay.front}, color::frame);
    add_box(b, {lay.leg[s], lay.base_top, lay.fz - w * 0.5f}, {lay.leg[s] + w, lay.bar_lo, lay.fz + w * 0.5f}, color::frame);
  }

  // Front and back cross members joining the base legs, and the top bar joining the uprights
  static void add_cross_members(VertexBuffer& b, const Layout& lay) {
    add_box(b, {lay.xl + w, lay.base_top - w, lay.front - w}, {lay.xr - w, lay.base_top, lay.front}, color::frame);
    add_box(b, {lay.xl + w, lay.base_top - w, lay.back}, {lay.xr - w, lay.base_top, lay.back + w}, color::frame);
    add_box(b, {lay.xl, lay.bar_lo, lay.fz - w * 0.5f}, {lay.xr, lay.bar_lo + w, lay.fz + w * 0.5f}, color::frame);
  }

  // Y smooth rod at x, held by printed Y corners standing on the front and back cross members
  static void add_y_rod(VertexBuffer& b, const Layout& lay, const float x) {
    add_cylinder(b, {x, yrod_y, lay.back + w * 0.5f}, {x, yrod_y, lay.front - w * 0.5f}, yrod_r, color::rod);
    add_box(b, {x - 10, lay.base_top, lay.front - w}, {x + 10, yrod_y + 8, lay.front}, color::printed);
    add_box(b, {x - 10, lay.base_top, lay.back}, {x + 10, yrod_y + 8, lay.back + w}, color::printed);
  }

  // Z drive on side s: the motor centered on the upright under its screw, a printed mount that
  // caps the motor, holds the Z rod and fastens to the upright, the screw and rod running up to
  // the middle of the top bar, and the Z top there, reaching 1cm past the Z rod.
  static void add_z_drive(VertexBuffer& b, const Layout& lay, const int s) {
    add_motor(b, {lay.screw[s], motor_top - nema17 * 0.5f, lay.rz});
    add_box(b, {lay.screw[s] - in(s) * (nema17 * 0.5f + 2), motor_top, lay.fz}, {lay.rod[s] + in(s) * (zrod_r + 8), mount_top, lay.rz + nema17 * 0.5f + 2}, color::printed);
    add_cylinder(b, {lay.screw[s], mount_top, lay.rz}, {lay.screw[s], lay.bar_mid, lay.rz}, screw_r, color::screw);
    add_cylinder(b, {lay.rod[s], mount_top, lay.rz}, {lay.rod[s], lay.bar_mid, lay.rz}, zrod_r, color::rod);
    add_box(b, {lay.screw[s] - in(s) * 12, lay.bar_mid - 8, lay.fz}, {lay.rod[s] + in(s) * (zrod_r + 10), lay.bar_mid + 8, lay.rz + zrod_r + 10}, color::printed);
  }

  // X end on side s (relative to the nozzle height): a bearing body around the Z rod reaching
  // forward to clamp the X rods, and a nut trap over the screw (and the Z motor center) reaching
  // outboard from the bearing body. The nut sits in the trap with its top 2mm proud.
  static void add_x_end(VertexBuffer& b, const Layout& lay, const int s) {
    constexpr float trap_h = 18, nut_h = 6, nut_proud = 2;
    add_box(b, {lay.rod[s] - 12, xend_lo, lay.rz - 12}, {lay.rod[s] + 12, xend_hi, lay.xrod_z + 10}, color::printed);
    add_box(b, {lay.screw[s] - in(s) * 11, xend_lo, lay.rz - 11}, {lay.rod[s] - in(s) * 12, xend_lo + trap_h, lay.rz + 11}, color::printed);
    add_cylinder(b, {lay.screw[s], xend_lo + trap_h + nut_proud - nut_h, lay.rz}, {lay.screw[s], xend_lo + trap_h + nut_proud, lay.rz}, 7.5f, color::rod, 6);
  }

  // X motor on the X end of side s, outboard of the Z screw, 2cm behind the X rod plane
  static void add_x_motor(VertexBuffer& b, const Layout& lay, const int s) {
    add_motor(b, {lay.screw[s] - in(s) * 14, (xrod_lo + xrod_hi) * 0.5f, lay.xrod_z - 10 + nema17 * 0.5f});
  }
}

void MachineModel::build_bedslinger() {
  using namespace bedslinger;
  float tx0, tx1;
  x_travel(tx0, tx1);
  const Layout lay(tx0, tx1);

  // Frame. Small bedslingers have a single Y motor; both Z drives are drawn even with one
  // Z stepper, as the two motors are often wired to the same driver.
  add_part([&](VertexBuffer& b) {
    for (int s = 0; s < 2; ++s) add_side_frame(b, lay, s);
    add_cross_members(b, lay);
    for (const float x : { bed_x * 0.3f, bed_x * 0.7f }) add_y_rod(b, lay, x);
    add_motor(b, {bed_x * 0.5f, lay.base_top + nema17 * 0.5f, lay.back + nema17 * 0.5f}); // Y motor, flush with the back
    for (int s = 0; s < 2; ++s) add_z_drive(b, lay, s);
  });

  // Bed carriage, Y bearing blocks on the Y rods, and heated bed, which slide on Y
  add_part([&](VertexBuffer& b) {
    add_box(b, {-15, -16, -bed_y - 15}, {bed_x + 15, -10, 15}, color::dark);
    add_box(b, {-10, -9, -bed_y - 10}, {bed_x + 10, bed_top, 10}, color::bed);
    for (const float x : { bed_x * 0.3f, bed_x * 0.7f })
      for (const float z : { -bed_y * 0.2f, -bed_y * 0.8f })
        add_box(b, {x - 11, yrod_y - yrod_r - 4, z - 17}, {x + 11, -16, z + 17}, color::printed);
  }, [](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.bed_offset); });

  // X axis, which rides Z: X ends, X rods, and the X motor on the left X end.
  // IDEX drives the X2 carriage with its own motor on the right X end.
  add_part([&](VertexBuffer& b) {
    for (int s = 0; s < 2; ++s) add_x_end(b, lay, s);
    for (const float y : { xrod_lo, xrod_hi })
      add_cylinder(b, {lay.rod[0], y, lay.xrod_z}, {lay.rod[1], y, lay.xrod_z}, xrod_r, color::rod);
    for (int s = 0; s < carriage_count(); ++s) add_x_motor(b, lay, s);
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
      // Nozzle holder: one block reaching forward from the plate face at the top of the
      // heatsinks, spanning every hotend in X, so no nozzle is left floating. With one
      // hotend (or several level in Z) lo and hi coincide, so the holder is centered on
      // the carriage and still reaches 12mm past the nozzle tip.
      add_box(b, {lo.x - 12, lo.y + 38, plate_front}, {hi.x + 12, hi.y + 48, hi.z + 12}, color::printed);
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
// Cube (Z-bed): the head moves in XY at the top on an X gantry riding two Y rails, and the bed
// moves down on Z.
//
// Assembly (GL: X right, Y up, Z toward the front):
//   Frame (fixed):  a box of extrusions (uprights + top and bottom rings), a Y rail on each side
//                   with mounts at the uprights, and the XY motors in the back top corners.
//                   Each Z drive (motor on the bottom ring, lead screw up to a bearing bracket
//                   on a beam across that side of the frame) sits where the bed attaches.
//   Bed (Z):        the bed plate, with a collar on each Z screw (and guide rod) and an arm
//                   from the collar into the plate. Z drives come from z_drive_xy(), so a
//                   multi-Z bed is held at its G34 alignment points and could tilt about them.
//   Gantry (Y):     X rods between two gantry ends (Y blocks) on the Y rails. The ends' inner
//                   faces meet the toolheads' outer sides at the ends of X travel. Non-Core
//                   (Cartesian) machines drive Y from a motor on the far gantry end, and on both
//                   ends when there's a Y2 stepper; a single Y driver may drive both.
//   Toolheads (XY): one per X carriage (two with IDEX), and the probe on X1.
//
namespace cube {
  constexpr float w = 20;                           // Frame member width
  constexpr float rod_r = 4;                        // 8mm Y rails and X rods
  constexpr float zrod_r = 6, zscrew_r = 5;         // 12mm Z guide rods, 10mm Z lead screws
  constexpr float collar_r = 10, collar_lo = -24, collar_hi = -4; // Bed collars (Z nuts / bearings)
  constexpr float arm_lo = -14, arm_hi = -9;        // Bed arms, from the collars into the bed plate
  constexpr float plate_lip = 10;                   // Bed plate margin around the bed
  constexpr float end_half = 15;                    // Gantry end half-width (X)
  constexpr float end_depth = 30;                   // Gantry end half-depth (Z)
  constexpr float xrod_dz = 14;                     // X rods in front of / behind the carriage center
  constexpr float head_margin = 28;                 // Toolhead sides past the outermost hotends
  constexpr float rail_top_gap = 55;                // Y rails above the bed at its top
  constexpr float xrod_rise = 15;                   // X rods above the Y rails

  static float in(const int s) { return s ? -1.0f : 1.0f; }  // Toward the center from left (0) / right (1)

  enum Side : uint8_t { SIDE_LEFT, SIDE_RIGHT, SIDE_BACK, SIDE_FRONT };

  // Outer bounds of the frame box. The inner faces of its members bound everything inside.
  struct Frame {
    float x0, x1, y0, y1, z0, z1;
    float inner(const Side s) const {
      switch (s) {
        case SIDE_LEFT:  return x0 + w;
        case SIDE_RIGHT: return x1 - w;
        case SIDE_BACK:  return z0 + w;
        default:    return z1 - w;
      }
    }
    // The side whose inner face is nearest to a point (GL XZ)
    Side nearest_side(const glm::vec2 p) const {
      const float d[4] = { p.x - inner(SIDE_LEFT), inner(SIDE_RIGHT) - p.x, p.y - inner(SIDE_BACK), inner(SIDE_FRONT) - p.y };
      int best = 0;
      for (int i = 1; i < 4; ++i) if (d[i] < d[best]) best = i;
      return Side(best);
    }
  };

  // Uprights at the corners, and the top and bottom rings
  static void add_frame_box(VertexBuffer& b, const Frame& fr) {
    for (const float x : { fr.x0, fr.x1 - w }) for (const float z : { fr.z0, fr.z1 - w })
      add_box(b, {x, fr.y0, z}, {x + w, fr.y1, z + w}, color::frame);
    for (const float y : { fr.y0, fr.y1 - w }) {
      add_box(b, {fr.x0 + w, y, fr.z0}, {fr.x1 - w, y + w, fr.z0 + w}, color::frame);
      add_box(b, {fr.x0 + w, y, fr.z1 - w}, {fr.x1 - w, y + w, fr.z1}, color::frame);
      add_box(b, {fr.x0, y, fr.z0 + w}, {fr.x0 + w, y + w, fr.z1 - w}, color::frame);
      add_box(b, {fr.x1 - w, y, fr.z0 + w}, {fr.x1, y + w, fr.z1 - w}, color::frame);
    }
  }

  // A beam across one side of the frame, between its uprights, with its bottom at y
  static void add_side_beam(VertexBuffer& b, const Frame& fr, const Side s, const float y) {
    switch (s) {
      case SIDE_LEFT:  add_box(b, {fr.x0, y, fr.z0 + w}, {fr.x0 + w, y + w, fr.z1 - w}, color::frame); break;
      case SIDE_RIGHT: add_box(b, {fr.x1 - w, y, fr.z0 + w}, {fr.x1, y + w, fr.z1 - w}, color::frame); break;
      case SIDE_BACK:  add_box(b, {fr.x0 + w, y, fr.z0}, {fr.x1 - w, y + w, fr.z0 + w}, color::frame); break;
      case SIDE_FRONT: add_box(b, {fr.x0 + w, y, fr.z1 - w}, {fr.x1 - w, y + w, fr.z1}, color::frame); break;
    }
  }

  // Printed bracket from the inner face of the nearest frame side to a point p (GL XZ),
  // 'half' wide around p, spanning heights [lo, hi]
  static void add_bracket(VertexBuffer& b, const Frame& fr, const glm::vec2 p, const float lo, const float hi, const float half) {
    const Side s = fr.nearest_side(p);
    const float f = fr.inner(s);
    switch (s) {
      case SIDE_LEFT:  add_box(b, {f, lo, p.y - half}, {p.x + half, hi, p.y + half}, color::printed); break;
      case SIDE_RIGHT: add_box(b, {p.x - half, lo, p.y - half}, {f, hi, p.y + half}, color::printed); break;
      case SIDE_BACK:  add_box(b, {p.x - half, lo, f}, {p.x + half, hi, p.y + half}, color::printed); break;
      case SIDE_FRONT: add_box(b, {p.x - half, lo, p.y - half}, {p.x + half, hi, f}, color::printed); break;
    }
  }

  // Z drive at p (GL XZ): the motor standing on a plate from the bottom ring, and the lead
  // screw rising from its shaft into a bearing bracket on the side beam at beam_y
  static void add_z_drive(VertexBuffer& b, const Frame& fr, const glm::vec2 p, const float beam_y) {
    const float base = fr.y0 + w;                    // Top of the bottom ring
    add_bracket(b, fr, p, base - 6, base, nema17 * 0.5f + 2);
    add_motor(b, {p.x, base + nema17 * 0.5f, p.y});
    add_cylinder(b, {p.x, base + nema17, p.y}, {p.x, beam_y + w - 2, p.y}, zscrew_r, color::screw);
    add_bracket(b, fr, p, beam_y, beam_y + w, 12);
  }

  // Z guide rod at p (GL XZ), running between mounts on the bottom and top rings
  static void add_guide_rod(VertexBuffer& b, const Frame& fr, const glm::vec2 p) {
    add_cylinder(b, {p.x, fr.y0 + 4, p.y}, {p.x, fr.y1 - 4, p.y}, zrod_r, color::rod);
    for (const float y : { fr.y0, fr.y1 - w }) add_bracket(b, fr, p, y, y + w, 12);
  }

  // Y rail on side s at x, running 5mm past the inner faces of the frame's front and back, with
  // mounts from the side's inner face to the rail ends, in line with the uprights
  static void add_y_rail(VertexBuffer& b, const Frame& fr, const int s, const float x, const float y) {
    add_cylinder(b, {x, y, fr.inner(SIDE_BACK) - 5}, {x, y, fr.inner(SIDE_FRONT) + 5}, rod_r, color::rod);
    const float face = fr.inner(s ? SIDE_RIGHT : SIDE_LEFT);
    for (const float ze : { fr.z0 + w * 0.5f, fr.z1 - w * 0.5f })
      add_box(b, {face, y - 10, ze - 10}, {x + in(s) * 10, y + 10, ze + 10}, color::printed);
  }

  // Bed collar on a Z screw or guide rod at p (GL XZ, bed-relative), and an arm from the
  // collar to 20mm inside the nearest point of the bed plate
  static void add_bed_collar(VertexBuffer& b, const glm::vec2 p) {
    add_cylinder(b, {p.x, collar_lo, p.y}, {p.x, collar_hi, p.y}, collar_r, color::printed);
    const glm::vec2 edge = glm::clamp(p, glm::vec2(-plate_lip, -bed_y - plate_lip), glm::vec2(bed_x + plate_lip, plate_lip));
    const glm::vec2 d = edge - p;
    const float len = glm::length(d);
    if (len < 1) return;                            // Under the plate: the collar holds it directly
    const glm::vec2 dir = d / len, mid = p + dir * ((len + 20) * 0.5f);
    add_block_facing(b, {mid.x, (arm_lo + arm_hi) * 0.5f, mid.y}, {dir.x, 0, dir.y}, {10, arm_hi - arm_lo, len + 20}, color::printed);
  }

  // Gantry end (Y block) on side s riding the Y rail at x: deeper than it is tall, holding the X rods
  static void add_gantry_end(VertexBuffer& b, const int s, const float x, const float rail_y, const float xrod_y) {
    add_box(b, {x - in(s) * end_half, rail_y - 12, -end_depth}, {x + in(s) * end_half, xrod_y + 12, end_depth}, color::printed);
  }

  // Y motor on a gantry end at x, standing on a riser so its bottom clears the toolheads at 'bottom'
  static void add_gantry_motor(VertexBuffer& b, const float x, const float riser_lo, const float bottom) {
    if (bottom > riser_lo) add_box(b, {x - end_half, riser_lo, -nema17 * 0.5f}, {x + end_half, bottom, nema17 * 0.5f}, color::printed);
    add_motor(b, {x, bottom + nema17 * 0.5f, 0});
  }
}

void MachineModel::build_cube() {
  using namespace cube;
  constexpr bool is_core = ANY(IS_CORE, MARKFORGED_XY, MARKFORGED_YX);
  const float rail_y = bed_z + rail_top_gap;          // Y rail height
  const float xrod_y = rail_y + xrod_rise;            // X rods, side by side, above the Y rails

  // Toolheads, carriage-local: each spans its hotends plus head_margin, and the depth of every hotend
  glm::vec3 all_lo, all_hi;
  hotend_extent(all_lo, all_hi);
  const float head_top = bed_z + all_hi.y + 85;       // World height of the toolhead tops

  // Gantry ends, whose inner faces meet the toolheads' outer sides at the ends of X travel
  float reach_l, reach_r;
  carriage_reach(head_margin, reach_l, reach_r);
  const float yrail[2] = { reach_l - end_half, reach_r + end_half };

  // Cartesian Y motor(s) on the gantry ends, raised to clear the toolheads
  const float gm_bottom = std::max(xrod_y + 12, head_top + 2);

  // Z drives (GL XZ), and with a single Z stepper two guide rods beside its screw
  std::vector<glm::vec2> zd;
  for (const auto& p : z_drive_xy()) zd.push_back({ p.x, -p.y });
  std::vector<glm::vec2> guides;
  if (zd.size() == 1) guides = { { bed_x * 0.25f, zd[0].y }, { bed_x * 0.75f, zd[0].y } };

  // Frame: inner faces just outside everything inside it (Y rails, bed plate, Z drives, gantry travel)
  float in_l = std::min(yrail[0] - 10, -plate_lip - 2), in_r = std::max(yrail[1] + 10, bed_x + plate_lip + 2);
  float in_b = std::min({ -float(Y_MAX_POS) - end_depth - 10, -float(Y_MAX_POS) + all_lo.z - head_margin - 10, -bed_y - plate_lip - 2 });
  float in_f = std::max({ -float(Y_MIN_POS) + end_depth + 10, -float(Y_MIN_POS) + all_hi.z + head_margin + 10, plate_lip + 2 });
  for (const auto& p : zd) {
    constexpr float r = nema17 * 0.5f + 2;
    in_l = std::min(in_l, p.x - r); in_r = std::max(in_r, p.x + r);
    in_b = std::min(in_b, p.y - r); in_f = std::max(in_f, p.y + r);
  }
  Frame fr;
  fr.x0 = in_l - w; fr.x1 = in_r + w;
  fr.z0 = in_b - w; fr.z1 = in_f + w;
  fr.y0 = collar_lo - 2 - nema17 - w;                  // Z motors fit under the bed collars at the bottom
  fr.y1 = is_core ? bed_z + 110 : std::max(bed_z + 110, gm_bottom + nema17 + 2 + w);

  const float beam_y = bed_z + 5;                     // Side beams holding the Z screw tops, just over the top of the bed travel

  add_part([&](VertexBuffer& b) {
    add_frame_box(b, fr);
    bool beam[4] {};
    for (const auto& p : zd) beam[fr.nearest_side(p)] = true;
    for (int s = 0; s < 4; ++s) if (beam[s]) add_side_beam(b, fr, Side(s), beam_y);
    for (const auto& p : zd) add_z_drive(b, fr, p, beam_y);
    for (const auto& p : guides) add_guide_rod(b, fr, p);
    for (int s = 0; s < 2; ++s) {
      add_y_rail(b, fr, s, yrail[s], rail_y);
      // XY motors in the back corners, bottoms level with the underside of the top ring
      if (is_core) add_motor(b, {fr.inner(s ? SIDE_RIGHT : SIDE_LEFT) + in(s) * nema17 * 0.5f, fr.y1 - w + nema17 * 0.5f, fr.inner(SIDE_BACK) + nema17 * 0.5f});
    }
    // Cartesian X motor in the left back corner, and an X2 motor (IDEX or dual X steppers) in the right
    if (!is_core) for (int s = 0; s < (ENABLED(HAS_X2_STEPPER) ? 2 : 1); ++s)
      add_motor(b, {fr.inner(s ? SIDE_RIGHT : SIDE_LEFT) + in(s) * nema17 * 0.5f, fr.y1 - w + nema17 * 0.5f, fr.inner(SIDE_BACK) + nema17 * 0.5f});
  });

  // Bed, which rides Z: the plate, and a collar and arm at every Z screw and guide rod
  add_part([&](VertexBuffer& b) {
    add_box(b, {-plate_lip, -9, -bed_y - plate_lip}, {bed_x + plate_lip, bed_top, plate_lip}, color::bed);
    for (const auto& p : zd) add_bed_collar(b, p);
    for (const auto& p : guides) add_bed_collar(b, p);
  }, [](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.bed_offset); });

  // X gantry, which follows the nozzle on Y: two X rods side by side (front and back) between
  // the gantry ends, and on Cartesian machines the Y motor on the far end (both ends with Y2)
  add_part([&](VertexBuffer& b) {
    for (const float dz : { -xrod_dz, xrod_dz })
      add_cylinder(b, {yrail[0], xrod_y, dz}, {yrail[1], xrod_y, dz}, rod_r, color::rod);
    for (int s = 0; s < 2; ++s) add_gantry_end(b, s, yrail[s], rail_y, xrod_y);
    if (!is_core) {
      add_gantry_motor(b, yrail[1], xrod_y + 12, gm_bottom);
      if (ENABLED(HAS_Y2_STEPPER)) add_gantry_motor(b, yrail[0], xrod_y + 12, gm_bottom);
    }
  }, [](renderer::Mesh& m, const MachinePose& p) { move_to(m, {0, 0, p.nozzle_world.z}); });

  // Toolhead(s), which follow their nozzles (at the top of the bed travel), each enclosing its hotends' heatsinks.
  // With IDEX both toolheads ride the same X rods, so each spans the depth of every hotend.
  for (int c = 0; c < carriage_count(); ++c) {
    glm::vec3 lo, hi;
    hotend_extent(lo, hi, c);
    const glm::vec3 blo { lo.x - head_margin, lo.y + 30, all_lo.z - head_margin }, bhi { hi.x + head_margin, hi.y + 85, all_hi.z + head_margin };
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
// Assembly: each tower (extrusion + motor on top) stands at tower_r, joined by triangles of
// base and top beams around a round bed. A carriage rides each tower, and a pair of arms runs
// from its joint bar down to the matching joint bar on the effector, which carries the hotend.
//
namespace delta_parts {
  constexpr float arm_spacing = 25;  // Half the gap between paired arms
  constexpr float arm_r = 3;         // Arm (connecting rod) radius
  constexpr float ball_r = 5.5f;     // 11mm ball joints at both ends of every arm
  constexpr float bar_r = 4;         // 8mm bars joining each arm pair (effector and carriages)
  constexpr float bar_half = arm_spacing + arm_r - 1.5f; // Pair gap + arm diameter, less 3mm for the balls
  constexpr float tower_w = 30, tower_d = 20;            // Tower extrusion: wide face toward the center

  // Joint bar for one arm pair: a bar centered on 'mid' along 'tangent', with a ball joint for each arm
  static void add_joint_bar(VertexBuffer& b, const glm::vec3 mid, const glm::vec3 tangent, const glm::vec4 col) {
    add_cylinder(b, mid - tangent * bar_half, mid + tangent * bar_half, bar_r, col, 12);
    for (const float side : { -1.0f, 1.0f }) add_sphere(b, mid + tangent * (arm_spacing * side), ball_r, color::joint);
  }

  // Tower at 'column' from height lo to hi, turned so its wide face points at the center (along -out),
  // with its motor standing on top
  static void add_tower(VertexBuffer& b, const glm::vec3 column, const glm::vec3 out, const float lo, const float hi) {
    add_block_facing(b, {column.x, (lo + hi) * 0.5f, column.z}, -out, {tower_w, hi - lo, tower_d}, color::frame);
    add_motor_facing(b, {column.x, hi + nema17 * 0.5f, column.z}, -out);
  }
}

void MachineModel::build_delta() {
  using namespace delta_parts;
  constexpr float joint_y = 14;      // Arm joints above the nozzle tip
  const glm::vec2 center { bed_x * 0.5f, bed_y * 0.5f };   // Marlin XY
  const float delta_radius = delta.radius, delta_rod = delta.diagonal_rod;
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

  // Frame: towers from the bottom of the base beams to the top of the top beams, the base and top triangles, and the round bed
  add_part([&](VertexBuffer& b) {
    constexpr float base_y = -45;                           // Base beam centerline
    glm::vec3 column[3];
    for (int i = 0; i < 3; ++i) {
      column[i] = c3 + dir[i] * tower_r;
      add_tower(b, column[i], dir[i], base_y - beam_r, top + beam_r);
    }
    for (int i = 0; i < 3; ++i) {
      const glm::vec3 a = column[i], z = column[(i + 1) % 3];
      add_cylinder(b, {a.x, base_y, a.z}, {z.x, base_y, z.z}, beam_r, color::frame, 8);
      add_cylinder(b, {a.x, top, a.z}, {z.x, top, z.z}, beam_r, color::frame, 8);
    }
    add_cylinder(b, c3 + glm::vec3(0, -9, 0), c3 + glm::vec3(0, bed_top, 0), delta.printable_radius + 10, color::bed, 48);
  });

  // Carriages ride the towers; each faces the center (local X is along the tower's tangent)
  for (int i = 0; i < 3; ++i) {
    add_part([&](VertexBuffer& b) {
      add_box(b, {-35, -20, 0}, {35, 25, carriage_d}, color::printed);
      add_joint_bar(b, {0, 0, 0}, {1, 0, 0}, color::rod);
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

  // Effector and hotend: a plate with a joint bar at each arm pair, placed like the arms' lower
  // ends (joint_y up, effector_r out along dir, +/-arm_spacing along the tangent)
  add_part([&](VertexBuffer& b) {
    add_cylinder(b, {0, joint_y - 5, 0}, {0, joint_y + 5, 0}, effector_r + 8, color::printed, 6);
    for (int i = 0; i < 3; ++i)
      add_joint_bar(b, glm::vec3(0, joint_y, 0) + dir[i] * effector_r, { -dir[i].z, 0, dir[i].x }, color::printed);
    add_cold_ends(b);
  }, [](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.nozzle_world); }, false);
  // The probe mounts to the effector plate's rim
  const float rim = effector_r + 8;
  add_probe([=](const glm::vec2 q) { const float l = glm::length(q); return l > rim ? q * (rim / l) : q; }, joint_y - 5, joint_y + 5);
  add_hot_parts();
}

//
// SCARA: one arm over a Z bed, on a frame of just a back and a bottom.
//
// The simplest SCARA: the bed rides Z like the Cube's and the arm stays at one height.
// Assembly, from the bottom up:
//   Frame (fixed):  a bottom slab and a back slab. The Z drive(s) stand on the bottom and are
//                   fastened to the back (screw tops and guide rods), as the Cube's are.
//                   Motor A, the shoulder, hangs on a printed plate from the back.
//   Bed (Z):        the bed plate with collars on the Z screw(s) and guide rods.
//   Upper arm:      turns on motor A's shaft (shoulder, on top of the motor). It carries the
//                   elbow motor B upside-down on top of its far end, shaft pointing down.
//   Forearm:        under the upper arm, turned by B's shaft through the upper arm. Its far
//                   end cylinder is centered on the tool point and the hotend hangs under it.
//                   Extra hotends and the probe are bolted to the forearm: their offsets are in
//                   the forearm's frame (as built when it points along +Y) and turn with it.
//
// Arm lengths are joint to joint (shoulder to elbow, elbow to tool): the bar spans exactly the
// length, and each end cylinder is centered on its joint, so it stands past the bar's end by its
// radius, "(|=====|)", which is all the extra length the arms need.
//
namespace scara_parts {
  constexpr float w = 20;                       // Frame slab thickness
  constexpr float bar_h = 10, bar_half = 7;     // Arm bar thickness, and half its width
  constexpr float hub_r = 11, hub_extra = 2;    // End cylinders: radius, and how far they stand past the bar's top and bottom
  constexpr float arm_gap = 2;                  // Gap between the forearm's end cylinders and the upper arm's
  constexpr float shaft_r = 3;                  // Motor B's shaft, through the upper arm into the forearm
  constexpr float plate_h = 6, plate_half = nema17 * 0.5f + 2; // Motor A's mounting plate

  // An arm along local +X from its first joint at the origin: a bar between two vertical end
  // cylinders, the second at 'end_x'. The bar spans [y0, y0 + bar_h]; the end cylinders stand proud.
  static void add_arm(VertexBuffer& b, const float y0, const float end_x) {
    add_box(b, {0, y0, -bar_half}, {end_x, y0 + bar_h, bar_half}, color::printed);
    for (const float x : { 0.0f, end_x })
      add_cylinder(b, {x, y0 - hub_extra, 0}, {x, y0 + bar_h + hub_extra, 0}, hub_r, color::printed, 24);
  }

  // A fixed part's bracket from the back slab's inner face (at z = back) to a post at p (GL XZ)
  static void add_back_bracket(VertexBuffer& b, const float back, const glm::vec2 p, const float lo, const float hi, const float half) {
    add_box(b, {p.x - half, lo, back}, {p.x + half, hi, p.y + half}, color::printed);
  }
}

void MachineModel::build_scara() {
  using namespace scara_parts;
  const float arm1 = scara.arm1, arm2 = scara.arm2;
  const glm::vec3 shoulder_w { scara.shoulder.x, bed_z, -scara.shoulder.y };  // Fixed; the arms ride at the nozzle's (top) height

  // Heights above the nozzle tip: the forearm's underside meets the hotends' tops, the upper arm is
  // over it with a gap between their end cylinders, and each motor stands against its arm
  glm::vec3 all_lo, all_hi;
  hotend_extent(all_lo, all_hi);
  const float fy0 = all_hi.y + hotend_cold_top, fy1 = fy0 + bar_h;       // Forearm
  const float ay0 = fy1 + hub_extra * 2 + arm_gap, ay1 = ay0 + bar_h;    // Upper arm
  const float motor_a_lo = bed_z + ay0 - nema17;                         // World height of motor A's underside

  // Z drives (GL XZ), and with a single Z stepper two guide rods beside its screw
  std::vector<glm::vec2> zd;
  for (const auto& p : z_drive_xy()) zd.push_back({ p.x, -p.y });
  std::vector<glm::vec2> guides;
  if (zd.size() == 1) guides = { { bed_x * 0.25f, zd[0].y }, { bed_x * 0.75f, zd[0].y } };

  // Frame extents: everything stands in front of the back slab and over the bottom slab
  const glm::vec2 sh { scara.shoulder.x, -scara.shoulder.y };            // Shoulder (GL XZ)
  constexpr float r = nema17 * 0.5f + 2;
  float xl = std::min(-cube::plate_lip - 2, sh.x - plate_half), xr = std::max(bed_x + cube::plate_lip + 2, sh.x + plate_half);
  float zb = std::min(-bed_y - cube::plate_lip - 4, sh.y - nema17 * 0.5f - 4);   // Inner face of the back slab
  float zf = cube::plate_lip + 2;
  for (const auto& p : zd) {
    xl = std::min(xl, p.x - r); xr = std::max(xr, p.x + r);
    zb = std::min(zb, p.y - r); zf = std::max(zf, p.y + r);
  }
  xl -= 10; xr += 10;
  const float y0 = cube::collar_lo - 2 - nema17 - w;                     // Bottom of the frame: Z motors fit under the bed collars
  const float base = y0 + w;                                             // Top of the bottom slab
  const float beam_y = bed_z + 5;                                        // Screw and rod tops, just over the top of the bed travel
  const float top = std::max(motor_a_lo, beam_y + w) + 10;               // Top of the back slab: a little over motor A's plate and the Z tops,
                                                                         // so the arms swing past it

  add_part([&](VertexBuffer& b) {
    add_box(b, {xl, y0, zb - w}, {xr, top, zb}, color::frame);           // Back
    add_box(b, {xl, y0, zb}, {xr, base, zf}, color::frame);              // Bottom
    // Motor A, the shoulder: on a plate reaching from the back slab, shaft up
    add_box(b, {sh.x - plate_half, motor_a_lo - plate_h, zb}, {sh.x + plate_half, motor_a_lo, sh.y + plate_half}, color::printed);
    add_motor(b, {sh.x, motor_a_lo + nema17 * 0.5f, sh.y});
    // Z drives: motor on the bottom slab, screw up to a bracket on the back slab
    for (const auto& p : zd) {
      add_motor(b, {p.x, base + nema17 * 0.5f, p.y});
      add_cylinder(b, {p.x, base + nema17, p.y}, {p.x, beam_y + w - 2, p.y}, cube::zscrew_r, color::screw);
      add_back_bracket(b, zb, p, beam_y, beam_y + w, 12);
    }
    for (const auto& p : guides) {
      add_cylinder(b, {p.x, base, p.y}, {p.x, beam_y + w - 2, p.y}, cube::zrod_r, color::rod);
      add_back_bracket(b, zb, p, base, base + w, 12);
      add_back_bracket(b, zb, p, beam_y, beam_y + w, 12);
    }
  });

  // Bed, which rides Z: the plate, and a collar and arm at every Z screw and guide rod
  add_part([&](VertexBuffer& b) {
    add_box(b, {-cube::plate_lip, -9, -bed_y - cube::plate_lip}, {bed_x + cube::plate_lip, bed_top, cube::plate_lip}, color::bed);
    for (const auto& p : zd) cube::add_bed_collar(b, p);
    for (const auto& p : guides) cube::add_bed_collar(b, p);
  }, [](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.bed_offset); });

  // Upper arm (origin at the shoulder, turning about the vertical with the angle of the arm in
  // Marlin XY), and motor B upside-down on its elbow end with the shaft going down through it
  add_part([=](VertexBuffer& b) {
    add_arm(b, ay0, arm1);
    add_motor(b, {arm1, ay1 + hub_extra + nema17 * 0.5f, 0});
  }, [=](renderer::Mesh& m, const MachinePose& p) {
    m.m_position = shoulder_w;
    m.m_rotation = glm::angleAxis(glm::radians(p.scara_angles.x), glm::vec3(0, 1, 0));
  }, false);

  // Forearm (origin at the elbow): from B's shaft to the tool, which is arm2 along it
  add_part([=](VertexBuffer& b) {
    add_arm(b, fy0, arm2);
    add_cylinder(b, {0, fy1, 0}, {0, ay1 + hub_extra, 0}, shaft_r, color::rod, 12);
  }, [=](renderer::Mesh& m, const MachinePose& p) {
    const float t1 = glm::radians(p.scara_angles.x);
    m.m_position = shoulder_w + glm::vec3(std::cos(t1), 0, -std::sin(t1)) * arm1;
    m.m_rotation = glm::angleAxis(glm::radians(p.scara_angles.y), glm::vec3(0, 1, 0));
  }, false);

  // Hotends' cold ends, hanging from the forearm's end cylinder, and the mounts that hold the others: a cylinder on top
  // of each (like the arms' end cylinders) joined to the center of the forearm's end cylinder, which is the tool point,
  // by a bar the thickness of the arms. They turn with the forearm, as the offsets are in its frame.
  add_part([&](VertexBuffer& b) {
    add_cold_ends(b, 0);
    for (int h = 1; h < hotend_count(); ++h) {
      const glm::vec3 o = hotend_offset(h);                          // (x, height, z) from the T0 nozzle: the tool point is straight over it
      const float top = o.y + hotend_cold_top, hub_lo = fy0 - hub_extra, hub_hi = fy1 + hub_extra;
      if (top < hub_lo) add_cylinder(b, {o.x, top, o.z}, {o.x, hub_lo, o.z}, 4.0f, color::frame, 12);   // Stem up from the cold end
      add_cylinder(b, {o.x, std::min(top, hub_lo), o.z}, {o.x, hub_hi, o.z}, hub_r, color::printed, 24);
      const glm::vec2 d { o.x, o.z };
      if (glm::length(d) > 0.5f)                                     // The bar from the end cylinder's center to this one's
        add_block_facing(b, {o.x * 0.5f, (fy0 + fy1) * 0.5f, o.z * 0.5f}, {d.x, 0, d.y}, {2 * bar_half, bar_h, glm::length(d)}, color::printed);
    }
  },
    [](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.nozzle_world); turn_with_arm(m, p, true); }, false);
  // The probe's bracket reaches the end cylinder, which looks the same at any arm angle
  add_probe([=](const glm::vec2 q) { const float l = glm::length(q); return l > hub_r ? q * (hub_r / l) : q; }, fy0 - hub_extra, fy1 + hub_extra, true, true);
  add_hot_parts(true);
}

//
// Belt printer (BELTPRINTER)
//
// Marlin Y moves the nozzle up a gantry tilted by belt_tilt_deg() from the belt, and Marlin Z feeds the belt
// (endless: CR-30 sets Z_MAX_POS 20000000, so bed_z is never used here). Y = 0 is the tip on the belt;
// the tip is Y*sin(tilt) above it and Y*cos(tilt) along it, toward the exit. That is the inverse of the
// slicer's machine-frame transform (OrcaSlicer: Y = h/sin, Z = y + h*cot), so G-code sliced for a belt
// printer draws upright and un-sheared with no conversion here.
//
// The frame is the Cube's, modeled in its own axes (X right, up, Marlin Y toward -z) and turned into place
// by belt_frame_rotation(): no bottom and no back, leaning back by the tilt. Its open bottom and open back
// both face the exit. Marlin +X points toward GL -x, which keeps the model-to-machine map a rotation (the
// slicer's "Rev X"); a print is mirrored if X goes the other way.
//
// The extrusion trail is stored in belt space (nozzle world position minus the belt feed) and drawn at
// pose.bed_offset = (0, 0, Z), so it rides the belt like the Cube's print rides its bed (see
// Visualisation::set_head_position).
//
float MachineModel::belt_tilt_deg() { return belt_angle_option > 0 ? belt_angle_option : belt_default_tilt; }

glm::mat3 MachineModel::belt_frame_rotation() {
  const float a = glm::radians(belt_tilt_deg()), s = std::sin(a), c = std::cos(a);
  // Columns: where frame X, up and (Marlin -Y) go in the world
  return glm::mat3(glm::vec3(-1, 0, 0), glm::vec3(0, c, -s), glm::vec3(0, -s, -c));
}

glm::vec3 MachineModel::belt_contact() { return { bed_x, 0, 0 }; }

glm::vec3 MachineModel::belt_nozzle_world(const glm::vec3 m) {
  return belt_contact() + belt_frame_rotation() * glm::vec3(m.x, 0, -m.y);
}

namespace belt_parts {
  constexpr float roller_r = 20, belt_t = 2;        // Roller radius and belt thickness
  constexpr float roller_out = 12;                  // Rollers stick out past the belt on each side
  constexpr float base_w = 20;                      // Bottom frame rails: 20mm extrusion, like the struts
  constexpr float foot_margin = 15;                 // The belt and base reach this far past the strut feet
  constexpr float stripe_pitch = 40, stripe_w = 6;  // Cross stripes that make the belt feed visible
  constexpr float pulley_r = 9, roller_pulley_r = 12, pulley_w = 8; // Z drive loop: motor pulley, roller pulley, and their length (8mm)
  constexpr float loop_w = 6;                       // The belt loop is 6mm wide (a GT2 belt)
  constexpr float loop_shift = 1;                   // The loop sits this far toward the belt (inward) so it reads as centered on the 8mm gears
  constexpr float xext = 20;                        // The X axis is one 20x20 extrusion, as on the CR-30
  constexpr float gantry_end_half = 12, gantry_end_depth = 22; // Gantry end blocks: half width (X) and half depth (Z), smaller than the Cube's
  constexpr glm::vec4 body   { 0.10f, 0.10f, 0.11f, 1.0f };
  constexpr glm::vec4 stripe { 0.34f, 0.35f, 0.38f, 1.0f };
}

//
// The frame is a right triangle seen from the side, modeled on the CR-30 (a left and a right one, joined at the
// peak and at the base). Corners: R (rear foot), P (peak), F (front foot):
//
//                      P            peak: crossbar between the two sides; the AB motors sit just under it
//                    /   \ 
//     front strut  /       \  rear strut     the struts meet at a right angle, so the base is the hypotenuse:
//                /           \                sqrt(2) x a strut at 45 degrees, plus a little at each end
//              F ______________ R
//        exit (front)  <-- +Z        rear     the belt runs along the base and travels toward F as Z increases
//
// The nozzle meets the belt at R, about where the belt flattens out at the rear roller (Marlin Y = 0). Marlin Y
// climbs the rear strut from R toward P. (The base is horizontal in the world and the rear strut rises toward P.)
//
// - Rear struts: along the gantry's Y direction, at the height of the Y rails, from the peak end down to the
//   base. The rails hang on their inner faces.
// - Front struts: perpendicular to the rear struts, from the peak down to the base on the exit side. The AB
//   (CoreXY) motors sit on their inner faces, just under the peak crossbar, i.e. inside the peak of the triangle.
//   A Cartesian build keeps the same mounts for its X motor(s).
// - Base: bottom rails under the strut feet. The belt starts and ends at the feet; its rollers are carried by
//   the base through short bearing blocks.
// - Z (belt) motor: on the inner face of the rear-right strut, near its foot (right = GL +x, the right side seen
//   from the exit end, which is the default view). A short belt loop joins its pulley to the end of the rear roller.
//
// Everything is modeled in the frame's own axes (X right, up, Marlin -Y back) and turned into place with
// belt_frame_rotation(): local y is "up" from the gantry plane, local z = -Marlin Y, local x = Marlin X.
//
void MachineModel::build_belt() {
  using namespace cube;
  using namespace belt_parts;
  constexpr bool is_core = ANY(IS_CORE, MARKFORGED_XY, MARKFORGED_YX);
  const glm::mat3 R = belt_frame_rotation();
  const glm::vec3 C = belt_contact();
  const float tilt = glm::radians(belt_tilt_deg()), s = std::sin(tilt), c = std::cos(tilt);

  bounds_min = bounds_max = C;   // build() starts from the Cartesian bed; Z_MAX_POS is endless here

  // Parts are modeled in the frame's axes and turned into place. Fixed parts also move to the contact
  // point; moving ones are placed by their animation.
  auto bake = [R, C](VertexBuffer& b, const bool fixed) {
    for (auto& v : b.data()) {
      v.position = R * v.position + (fixed ? C : glm::vec3(0));
      v.normal = R * v.normal;
    }
  };
  auto frame_part = [&](std::function<void(VertexBuffer&)> geometry, const bool fixed, Animate animate = nullptr, const bool in_bounds = true) {
    return add_part([&, geometry, fixed](VertexBuffer& b) { geometry(b); bake(b, fixed); }, animate, in_bounds);
  };

  //
  // Belt constants (world). The base height is where the struts end, so it comes first.
  //
  constexpr float R_o = roller_r + belt_t;      // Outer radius where the belt wraps a roller
  const float bx0 = -10, bx1 = bed_x + 10;      // Belt width: a little wider than the X travel
  const float y_c = bed_top - R_o;              // Roller axes: the top run is tangent to the belt surface (Z0)
  const float y_base = y_c + base_w * 0.5f;     // Top of the bottom rails, where the struts end

  // The Z motor sits on the rear-right strut. Its inboard face must clear the roller stub and the belt-loop
  // pulley, so the right strut is placed to leave exactly that room, and the frame is wider on that side.
  const float x_loop = bx1 + roller_out + 6;                  // World x of the belt loop (pulleys), just past the roller end
  const float x_face = x_loop + pulley_w * 0.5f + 1;          // World x of the motor's inboard face

  //
  // Frame size, in the frame's own axes
  //
  glm::vec3 all_lo, all_hi;
  hotend_extent(all_lo, all_hi);
  const float rail_y = rail_top_gap, xrod_y = rail_y + xrod_rise;   // Y rails, and X rods above them
  const float strut_y = rail_y;                                      // Struts are centered on the Y rails' height
  const float head_top = all_hi.y + 85;                              // Toolhead tops
  float reach_l, reach_r;
  carriage_reach(head_margin, reach_l, reach_r);
  const float yrail[2] = { reach_l - gantry_end_half, reach_r + gantry_end_half }; // Gantry ends
  const float gm_bottom = std::max(xrod_y + 12, head_top + 2);       // Cartesian Y motor, clear of the toolheads

  // Struts are just outside the gantry. The right strut (local x0, world +x) is pushed out to make room for the Z motor.
  const float x0 = std::min(yrail[0] - 10 - w, C.x - w - nema17 - x_face);   // Local x of the right strut's outer face
  const float x1 = yrail[1] + 10 + w;                                         // And the left strut's

  // Gantry travel in local z (= -Marlin Y): the frame's inner limits behind and in front of the gantry
  const float in_b = std::min(-float(Y_MAX_POS) - gantry_end_depth - 10, -float(Y_MAX_POS) + all_lo.z - head_margin - 10);
  const float in_f = std::max(-float(Y_MIN_POS) + gantry_end_depth + 10, -float(Y_MIN_POS) + all_hi.z + head_margin + 10);

  // Triangle corners (local z of the rear strut's two ends)
  const float z0 = in_b - w - nema17;                                  // Peak end: room behind the gantry for the AB motors
  const float z_foot = std::max((c * strut_y - y_base) / s, in_f + 12); // Foot end: where the strut centerline meets the base, but past the rails
  const float y_peak = c * strut_y - s * (z0 + w * 0.5f);              // World height of the peak
  const float t_front = (y_peak - y_base) / c;                         // Front strut length, local -y from the peak down to the base

  // World z of the two feet, from the frame transform: world z = -sin*y - cos*z
  const float z_rear = -s * strut_y - c * z_foot;                      // Rear foot R
  const float z_front = -s * strut_y - c * (z0 + w * 0.5f) + s * t_front; // Front foot F

  // The belt starts and ends just past the feet; roller centers are one roller radius inside its ends
  const float rear_c = z_rear - foot_margin + R_o, front_c = z_front + foot_margin - R_o;

  //
  // The fixed frame: struts, peak crossbar, Y rails with their mounts, AB motors, and the Z motor with its pulley
  //
  const float zm = z_foot - nema17 * 0.5f - 4;                         // Local z of the Z motor, just above the rear foot
  const float x_loop_l = C.x - x_loop;                                 // The belt loop's plane in local x
  frame_part([&](VertexBuffer& b) {
    for (const float x : { x0, x1 - w }) {
      add_box(b, {x, strut_y - w * 0.5f, z0}, {x + w, strut_y + w * 0.5f, z_foot}, color::frame);   // Rear strut: peak to rear foot
      add_box(b, {x, strut_y - t_front, z0}, {x + w, strut_y + w * 0.5f, z0 + w}, color::frame);    // Front strut: peak to front foot
    }
    add_box(b, {x0 + w, strut_y - w * 0.5f, z0}, {x1 - w, strut_y + w * 0.5f, z0 + w}, color::frame); // Peak crossbar

    for (int i = 0; i < 2; ++i) {
      // Y rail on the strut's inner side, held at both ends by printed mounts that fasten to the strut
      const float x = yrail[i], inner = i ? x1 - w : x0 + w;   // The strut's inner face: mounts start there (not at the outer face, which would z-fight)
      add_cylinder(b, {x, strut_y, z0 + 5}, {x, strut_y, in_f + 5}, rod_r, color::rod);
      add_box(b, {inner, strut_y - 10, z0}, {x + in(i) * 10, strut_y + 10, z0 + 20}, color::printed);
      add_box(b, {inner, strut_y - 10, in_f - 8}, {x + in(i) * 10, strut_y + 10, in_f + 12}, color::printed);
      // AB (CoreXY) motors, or the Cartesian X motor(s): on the front struts' inner faces, tucked under the crossbar
      // inside the peak, and behind the gantry's travel so the carriage never meets them
      if (is_core || i == 0 || ENABLED(HAS_X2_STEPPER))
        add_motor(b, {i ? x1 - w - nema17 * 0.5f : x0 + w + nema17 * 0.5f, strut_y - w * 0.5f - nema17 * 0.5f, z0 + nema17 * 0.5f});
    }

    // Z (belt) motor on the right rear strut's inner face, with its shaft and pulley reaching the belt loop
    add_motor(b, {x0 + w + nema17 * 0.5f, strut_y, zm});
    add_cylinder(b, {x0 + w + nema17, strut_y, zm}, {x_loop_l + pulley_w * 0.5f, strut_y, zm}, 2.5f, color::rod, 10);
    add_cylinder(b, {x_loop_l - pulley_w * 0.5f, strut_y, zm}, {x_loop_l + pulley_w * 0.5f, strut_y, zm}, pulley_r, color::joint, 16);   // Gray gear
  }, true);

  //
  // X gantry, which rides the Y rails with the nozzle: one 20x20 extrusion across (as on the CR-30), the two end
  // blocks that ride the rails, and on Cartesian machines the Y motor on the far end (both ends with Y2). The toolhead
  // rides the extrusion, so it is drawn around it.
  //
  frame_part([&](VertexBuffer& b) {
    add_box(b, {yrail[0], xrod_y - xext * 0.5f, -xext * 0.5f}, {yrail[1], xrod_y + xext * 0.5f, xext * 0.5f}, color::frame);
    for (int i = 0; i < 2; ++i)   // End blocks: from just under the rail up to the top of the extrusion
      add_box(b, {yrail[i] - gantry_end_half, rail_y - 10, -gantry_end_depth}, {yrail[i] + gantry_end_half, xrod_y + xext * 0.5f, gantry_end_depth}, color::printed);
    if (!is_core) {
      // Y motor on a riser from the end block, its bottom clear of the toolheads
      for (const int i : { 1, 0 }) {
        if (i == 0 && !ENABLED(HAS_Y2_STEPPER)) continue;
        if (gm_bottom > xrod_y + 12) add_box(b, {yrail[i] - gantry_end_half, xrod_y + 12, -nema17 * 0.5f}, {yrail[i] + gantry_end_half, gm_bottom, nema17 * 0.5f}, color::printed);
        add_motor(b, {yrail[i], gm_bottom + nema17 * 0.5f, 0});
      }
    }
  }, false, [R, C](renderer::Mesh& m, const MachinePose& p) { move_to(m, C + R * glm::vec3(0, 0, -p.nozzle.y)); }, false);

  // Toolhead(s), which follow their nozzles, each enclosing its hotends' heatsinks
  for (int cc = 0; cc < carriage_count(); ++cc) {
    glm::vec3 lo, hi;
    hotend_extent(lo, hi, cc);
    const glm::vec3 blo { lo.x - head_margin, lo.y + 30, all_lo.z - head_margin }, bhi { hi.x + head_margin, hi.y + 85, all_hi.z + head_margin };
    frame_part([&, cc](VertexBuffer& b) {
      add_box(b, blo, bhi, color::printed);
      add_cold_ends(b, cc);
    }, false, cc ? Animate([](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.x2_world); })
                 : Animate([](renderer::Mesh& m, const MachinePose& p) { move_to(m, p.nozzle_world); }), false);
  }
  add_hot_parts(true);   // Heater blocks turn with the frame (pose.head_turn)

  //
  // The belt and its base, in world coordinates: the bottom rails under the strut feet, the two rollers (carried by
  // the base through bearing blocks), the top and return runs, and the short belt loop from the Z motor to the
  // rear roller. The belt spans the feet plus a small margin.
  //
  const float zb0 = z_rear - foot_margin, zb1 = z_front + foot_margin;       // Base and belt ends (world z)
  const glm::vec3 motor_pulley = C + R * glm::vec3(x_loop_l, strut_y, zm);   // Z motor pulley center, world
  add_part([&](VertexBuffer& b) {
    // Bottom rails, directly under the struts
    for (const float x : { x0, x1 - w })
      add_box(b, {C.x - x - w, y_base - base_w, zb0}, {C.x - x, y_base, zb1}, color::frame);
    add_box(b, {bx0, bed_top - belt_t, rear_c}, {bx1, bed_top, front_c}, body);             // Top run, tangent to Z0
    add_box(b, {bx0, y_c - R_o, rear_c}, {bx1, y_c - R_o + belt_t, front_c}, body);         // Return run
    for (const float zc : { rear_c, front_c }) {
      add_cylinder(b, {bx0, y_c, zc}, {bx1, y_c, zc}, R_o, body, 24);                       // Belt around the roller
      // Roller: past the belt on both sides. The rear roller's right end reaches the belt loop pulley.
      const float xr = (zc == rear_c) ? x_loop + pulley_w * 0.5f : bx1 + roller_out;
      add_cylinder(b, {bx0 - roller_out, y_c, zc}, {xr, y_c, zc}, roller_r - 1, color::rod, 20);
      // Bearing blocks: from each roller end to the base rail. The rear roller's right end is carried by its pulley.
      const float left_inner = C.x - (x1 - w), right_inner = C.x - (x0 + w);
      add_box(b, {left_inner, y_c - 8, zc - 10}, {bx0 - roller_out, y_c + 8, zc + 10}, color::printed);
      if (zc != rear_c) add_box(b, {bx1 + roller_out, y_c - 8, zc - 10}, {right_inner, y_c + 8, zc + 10}, color::printed);
    }
    // Belt loop: a gray 8mm gear on the roller end, and the two 6mm-wide runs between it and the Z motor gear
    add_cylinder(b, {x_loop - pulley_w * 0.5f, y_c, rear_c}, {x_loop + pulley_w * 0.5f, y_c, rear_c}, roller_pulley_r, color::joint, 16);
    const glm::vec2 d { motor_pulley.z - rear_c, motor_pulley.y - y_c };                    // (z, y) from roller to motor
    const float len = glm::length(d);
    if (len > 1) {
      const glm::vec2 n { -d.y / len, d.x / len };                                          // Perpendicular, in the (z, y) plane
      for (const float sgn : { -1.0f, 1.0f }) {
        const float xl = x_loop - loop_shift;   // Inward is toward the belt: world -x on this (+x) side
        const glm::vec3 p0 { xl, y_c + sgn * n.y * roller_pulley_r, rear_c + sgn * n.x * roller_pulley_r },
                        p1 { xl, motor_pulley.y + sgn * n.y * pulley_r, motor_pulley.z + sgn * n.x * pulley_r };
        add_ribbon(b, p0, p1, loop_w, body);
      }
    }
  });

  //
  // Stripes scroll with the belt feed: the top run toward the exit with +Z, the return run the other way. They are
  // placed so a full pitch of travel never carries one past an end of the belt.
  //
  auto stripe_z = [](const float z) { return z - std::floor(z / stripe_pitch) * stripe_pitch; }; // Feed within one pitch
  add_part([&](VertexBuffer& b) {
    for (float z = rear_c; z + stripe_pitch + stripe_w <= front_c; z += stripe_pitch)
      add_box(b, {bx0, bed_top - 0.5f, z}, {bx1, bed_top + 0.15f, z + stripe_w}, stripe);
  }, [stripe_z](renderer::Mesh& m, const MachinePose& p) { move_to(m, {0, 0, stripe_z(p.nozzle.z)}); });
  add_part([&](VertexBuffer& b) {
    for (float z = rear_c + stripe_pitch; z + stripe_w <= front_c; z += stripe_pitch)
      add_box(b, {bx0, y_c - R_o - 0.15f, z}, {bx1, y_c - R_o + 0.5f, z + stripe_w}, stripe);
  }, [stripe_z](renderer::Mesh& m, const MachinePose& p) { move_to(m, {0, 0, -stripe_z(p.nozzle.z)}); });
}

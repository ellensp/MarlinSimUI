//
// Delta reach and printable volume. See delta_reach.h.
//

#include "delta_reach.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <glm/gtc/constants.hpp>

#include <src/inc/MarlinConfig.h>
#if ENABLED(DELTA)
  #include <src/module/delta.h>
  #include <src/module/motion.h>
  #if HAS_BED_PROBE
    #include <src/module/probe.h>
  #endif
#endif

#undef abs
#undef min
#undef max

using renderer::vertex_data_t;
using VertexBuffer = renderer::Buffer<vertex_data_t>;

void DeltaReach::gather() {
  #if ENABLED(DELTA)
    valid = true;
    #if HAS_HOTEND_OFFSET
      offset = { motion.active_hotend_offset().x, motion.active_hotend_offset().y };
    #endif
    // Homed nozzle height, as in Motion::set_axis_is_at_home
    const float home_z = TERN(HAS_BED_PROBE, delta_height - probe.offset.z, delta_height);
    float max_adj = -std::numeric_limits<float>::infinity();
    for (int i = 0; i < 3; ++i) {
      tower[i] = { delta_tower[i].x, delta_tower[i].y };
      arm2[i] = delta_diagonal_rod_2_tower[i];
      // Homing backs each carriage off its endstop by delta_endstop_adj (<= 0), then calls
      // that the home position, so the endstop is -adj above the homed carriage.
      const float adj = delta_endstop_adj[i];
      top[i] = home_z - adj + std::sqrt(std::max(0.0f, arm2[i] - glm::dot(tower[i], tower[i])));
      max_adj = std::max(max_adj, adj);
    }
    z_top = home_z - max_adj;

    #if HAS_SOFTWARE_ENDSTOPS
      // As in Motion::update_software_endstops: the smallest radius within the XY limits.
      // Motion::apply_limits keeps (target - hotend offset) inside it.
      const auto& se = motion.soft_endstop;
      soft_on = se._enabled;
      soft_radius = std::min({ std::abs(std::max(se.min.x, se.min.y)), se.max.x, se.max.y });
      soft_z_max = se.max.z;
    #endif
  #endif
}

float DeltaReach::radius(const float angle, const bool soft) const {
  if (!valid) return 0;
  const glm::vec2 u { std::cos(angle), std::sin(angle) };
  float r = std::numeric_limits<float>::infinity();
  for (int i = 0; i < 3; ++i) {
    // Along the ray p = u * r: |p - T|^2 = r^2 - 2br + |T|^2 <= arm^2
    const float b = glm::dot(u, tower[i]), t2 = glm::dot(tower[i], tower[i]);
    if (t2 >= arm2[i]) return 0;
    r = std::min(r, b + std::sqrt(b * b - (t2 - arm2[i])));
  }
  if (soft && soft_on) r = std::min(r, soft_radius);
  return std::max(r, 0.0f);
}

float DeltaReach::height(const glm::vec2 e, const bool soft) const {
  if (!valid) return 0;
  float z = std::numeric_limits<float>::infinity();
  for (int i = 0; i < 3; ++i) {
    const glm::vec2 d = e - tower[i];
    z = std::min(z, top[i] - std::sqrt(std::max(0.0f, arm2[i] - glm::dot(d, d))));
  }
  if (soft && soft_on) z = std::min(z, soft_z_max);
  return z;
}

void DeltaReach::append_signature(std::vector<float>& s) const {
  s.insert(s.end(), { float(valid), offset.x, offset.y, z_top, float(soft_on), soft_radius, soft_z_max });
  for (int i = 0; i < 3; ++i) s.insert(s.end(), { tower[i].x, tower[i].y, arm2[i], top[i] });
}

//
// Printable volume
//

namespace {

constexpr glm::vec4 col_volume { 1.00f, 0.70f, 0.40f, 0.16f }; // Light orange, translucent
constexpr int sides = 180, rings = 60;

// Marlin native XYZ -> world (GL Y-up), on the simulated bed (Delta's center is Marlin 0,0)
glm::vec3 to_world(const glm::vec2 p, const float z) {
  const glm::vec2 s = p + glm::vec2(float(X_BED_SIZE) * 0.5f - float(X_CENTER), float(Y_BED_SIZE) * 0.5f - float(Y_CENTER));
  return { s.x, z, -s.y };
}

} // namespace

void ReachVolume::create(std::shared_ptr<renderer::ShaderProgram> lit_program) {
  mesh = renderer::create_mesh();
  auto m = renderer::get_mesh_by_id(mesh);
  m->set_shader_program(lit_program);
  m->m_shader_instance->set_uniform("u_model", &m->m_transform);
  m->m_shader_instance->set_uniform("u_tint", &no_tint);
  m->m_shader_instance->set_uniform("u_glow", &no_glow);
  m->m_shader_instance->set_uniform("u_dim", &no_dim);
  m->buffer_vector<vertex_data_t>().push_back(VertexBuffer::create());
  m->m_translucent = true;
  m->m_visible = visible;
  signature.clear();
}

void ReachVolume::set_visible(const bool vis) {
  visible = TERN0(DELTA, vis);
  if (auto m = renderer::get_mesh_by_id(mesh)) m->m_visible = visible;
  if (visible) signature.clear(); // Rebuild on the next update
}

bool ReachVolume::update() {
  if (!visible) return false;
  DeltaReach reach;
  reach.gather();
  std::vector<float> sig;
  reach.append_signature(sig);
  if (sig == signature) return false;
  signature = std::move(sig);

  auto m = renderer::get_mesh_by_id(mesh);
  if (!m) return false;
  auto buf = m->buffer<vertex_data_t>();
  buf->data().clear();
  if (!reach.valid) return true;

  // Polar grid over the floor, in effector coordinates. Ring k is k/rings of the way to
  // the boundary, so the outer ring follows the rounded triangle / soft endstop circle.
  std::vector<float> edge(sides);
  for (int j = 0; j < sides; ++j) edge[j] = reach.radius(glm::two_pi<float>() * j / sides, true);
  auto at = [&](const int k, const int j) {
    const float a = glm::two_pi<float>() * j / sides;
    return glm::vec2(std::cos(a), std::sin(a)) * (edge[j] * k / rings);
  };

  // Top surface normal from the height gradient (central differences)
  auto top_normal = [&](const glm::vec2 e) {
    constexpr float h = 0.5f;
    const float dx = (reach.height(e + glm::vec2(h, 0), true) - reach.height(e - glm::vec2(h, 0), true)) / (2 * h),
                dy = (reach.height(e + glm::vec2(0, h), true) - reach.height(e - glm::vec2(0, h), true)) / (2 * h);
    return glm::normalize(glm::vec3(-dx, 1, dy)); // Marlin +Y is world -Z
  };

  auto add = [&](const glm::vec3 p, const glm::vec3 n) { buf->add_vertex({p, n, col_volume}); };
  auto world = [&](const glm::vec2 e, const float z) { return to_world(reach.offset + e, z); };

  // Top: counter-clockwise seen from above (Marlin XY is right-handed, world maps it to X,-Z)
  for (int k = 0; k < rings; ++k)
    for (int j = 0; j < sides; ++j) {
      const int jn = (j + 1) % sides;
      const glm::vec2 e[4] = { at(k, j), at(k + 1, j), at(k + 1, jn), at(k, jn) };
      glm::vec3 p[4], n[4];
      for (int q = 0; q < 4; ++q) { p[q] = world(e[q], reach.height(e[q], true)); n[q] = top_normal(e[q]); }
      // Face up (the top is a heightfield), whichever way the grid winds
      auto tri = [&](const int a, int b, int c) {
        if (glm::cross(p[b] - p[a], p[c] - p[a]).y < 0) std::swap(b, c);
        add(p[a], n[a]); add(p[b], n[b]); add(p[c], n[c]);
      };
      tri(0, 1, 2);
      if (k > 0) tri(0, 2, 3);  // The center ring is a single point
    }

  // Wall: from the floor up to the top along the boundary, facing out
  for (int j = 0; j < sides; ++j) {
    const int jn = (j + 1) % sides;
    const glm::vec2 e0 = at(rings, j), e1 = at(rings, jn);
    const glm::vec3 b0 = world(e0, 0), b1 = world(e1, 0),
                    t0 = world(e0, reach.height(e0, true)), t1 = world(e1, reach.height(e1, true));
    glm::vec3 n = glm::cross(b1 - b0, t0 - b0);
    const glm::vec3 out = (b0 + b1) * 0.5f - world({0, 0}, 0);
    if (glm::dot(n, glm::vec3(out.x, 0, out.z)) < 0) n = -n;
    n = glm::normalize(glm::vec3(n.x, 0, n.z));
    // Order so the face points along n
    if (glm::dot(glm::cross(b1 - b0, t1 - b0), n) > 0) { add(b0, n); add(b1, n); add(t1, n); add(b0, n); add(t1, n); add(t0, n); }
    else                                               { add(b0, n); add(t1, n); add(b1, n); add(b0, n); add(t0, n); add(t1, n); }
  }

  return true;
}

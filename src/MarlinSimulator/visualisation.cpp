#include "visualisation.h"

#include <glm/vec3.hpp>          // https://glm.g-truc.net/
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/epsilon.hpp>

#include <vector>
#include <array>
#include <imgui_internal.h>      // https://github.com/ocornut/imgui
#include <implot.h>

#include "resources/resources.h"
#include "view_control.h"

#include <src/inc/MarlinConfig.h>
#include <src/module/motion.h>      // motion.extruder: the active tool
#include <src/module/temperature.h> // Heater targets, which set the full-pink point

#include "hardware/Heater.h"
#include "thermal_fault_blink.h"
#if ENABLED(BLTOUCH)
  #include "hardware/print_bed.h"
  #include "hardware/bed_probe.h"
  #include "hardware/pwm_reader.h"
  #include "hardware/BLTouch.h"
#endif

// Prevent glm::abs confusion
#undef abs

Visualisation::Visualisation(VirtualPrinter& virtual_printer) : virtual_printer(virtual_printer) {
  //
  // Bind printer kinematic updates → head position
  //
  virtual_printer.on_kinematic_update = [this](kinematic_state& state){
    for (size_t i = 0; i < state.effector_position.size(); ++i) {
      this->set_head_position(i, state.effector_position[i]);
    }
  };

  //
  // Initialise extrusion containers
  //
  for (int i = 0; i < EXTRUDERS; ++i) {
    extrusion.push_back({});
  }

  SERIAL_ECHOLNPGM("\nCamera Controls:\nW A S D : Pan             F G : Follow Z / XY\nE Q     : Zoom In / Out   F1  : Path (Full)\nI       : Invert Pan      F2  : Path (Line)\nR       : Reset View      F4  : Path Clear\n");
}

Visualisation::~Visualisation() {
  destroy();
}

//
// Camera zoom sensitivity
//
constexpr float turntable_wheel_zoom = 0.05f;  // Turntable: fraction of the view distance per wheel notch
constexpr float turntable_key_zoom   = 1.0f;   // Turntable: fraction of the view distance per second holding +/-
constexpr float fly_wheel_zoom       = 20.0f;  // Fly: mm moved per wheel notch
constexpr float fly_key_zoom         = 100.0f; // Fly: mm per second holding E/Q

//
// Camera defaults – tweak to add new modes
//
static PerspectiveCamera initCamera = {
  { 37.0f, 121.0f, 129.0f }, // Position
  { -192.0f, -25.0, 0.0f },  // Rotation
  { 0.0f, 1.0f, 0.0f },      // Up = Y-Axis
  float(100) / float(100),   // Aspect Ratio
  glm::radians(45.0f), 2.0f, 5000.0f // FOV, Near, Far (near > 1 keeps depth precision for whole-printer views)
};

void Visualisation::create() {
  //
  // Load shaders (extrusion + default)
  //
  extrusion_program = renderer::ShaderProgram::create(
    "data/shaders/extrusion.vs",
    "data/shaders/extrusion.fs",
    "data/shaders/extrusion.gs"
  );
  default_program = renderer::ShaderProgram::create(
    "data/shaders/default.vs",
    "data/shaders/default.fs"
  );
  lit_program = renderer::ShaderProgram::create(
    "data/shaders/lit.vs",
    "data/shaders/lit.fs"
  );

  //
  // Framebuffer – MSAA first, fallback to texture
  //
  framebuffer = new opengl_util::MsaaFrameBuffer();
  if (!((opengl_util::MsaaFrameBuffer*)framebuffer)->create(100, 100, 4)) {
    logger::warning("Failed to initialise MSAA Framebuffer falling back to TextureFramebuffer\n");
    delete framebuffer;
    framebuffer = new opengl_util::TextureFrameBuffer();
    if (!((opengl_util::TextureFrameBuffer*)framebuffer)->create(100,100)) {
      logger::error("Unable to initialise a Framebuffer\n");
    }
  }

  camera = initCamera;
  camera.generate();

  //
  // Printer model, chosen from the Marlin kinematics
  //
  machine.build(machine_type_option < MACHINE_TYPE_COUNT ? machine_type_option : MachineModel::default_type(), lit_program);
  machine.set_visible(show_machine);

  turntable = turntable_home();
  if (camera_mode == CAMERA_TURNTABLE) turntable_apply();

  //
  // Build the extruder “head” meshes
  //
  if (EXTRUDERS > 0) {
    auto mesh = renderer::create_mesh();
    auto buffer = renderer::Buffer<renderer::vertex_data_t>::create();

    buffer->data() = {
        renderer::vertex_data_t EFFECTOR_VERTEX(0.0, 0.0, 0.0, EFFECTOR_COLOR_1),
        EFFECTOR_VERTEX(-0.5, 0.5, 0.5, EFFECTOR_COLOR_2),
        EFFECTOR_VERTEX(-0.5, 0.5, -0.5, EFFECTOR_COLOR_3),
        EFFECTOR_VERTEX(0.0, 0.0, 0.0, EFFECTOR_COLOR_1),
        EFFECTOR_VERTEX(-0.5, 0.5, -0.5, EFFECTOR_COLOR_3),
        EFFECTOR_VERTEX(0.5, 0.5, -0.5, EFFECTOR_COLOR_2),
        EFFECTOR_VERTEX(0.0, 0.0, 0.0, EFFECTOR_COLOR_1),
        EFFECTOR_VERTEX(0.5, 0.5, -0.5, EFFECTOR_COLOR_2),
        EFFECTOR_VERTEX(0.5, 0.5, 0.5, EFFECTOR_COLOR_3),
        EFFECTOR_VERTEX(0.0, 0.0, 0.0, EFFECTOR_COLOR_1),
        EFFECTOR_VERTEX(0.5, 0.5, 0.5, EFFECTOR_COLOR_3),
        EFFECTOR_VERTEX(-0.5, 0.5, 0.5, EFFECTOR_COLOR_2),
        EFFECTOR_VERTEX(0.5, 0.5, -0.5, EFFECTOR_COLOR_2),
        EFFECTOR_VERTEX(-0.5, 0.5, -0.5, EFFECTOR_COLOR_3),
        EFFECTOR_VERTEX(-0.5, 0.5, 0.5, EFFECTOR_COLOR_2),
        EFFECTOR_VERTEX(0.5, 0.5, -0.5, EFFECTOR_COLOR_2),
        EFFECTOR_VERTEX(-0.5, 0.5, 0.5, EFFECTOR_COLOR_2),
        EFFECTOR_VERTEX(0.5, 0.5, 0.5, EFFECTOR_COLOR_3),
    };
    renderer::get_mesh_by_id(mesh)->buffer_vector<renderer::vertex_data_t>().push_back(buffer);

    m_extruder_mesh.push_back(mesh);
    for (int i = 1; i < EXTRUDERS; ++i) {
      m_extruder_mesh.push_back(mesh);
    }
  }

  for (auto& m : m_extruder_mesh) {
    auto mesh_object = renderer::get_mesh_by_id(m);
    mesh_object->set_shader_program(default_program);
    mesh_object->m_shader_instance->set_uniform("u_tint", &no_tint); // Uniforms are per program; don't inherit the bed tint
    mesh_object->m_shader_instance->set_uniform("u_dim", &no_dim);   // ...or the bed's fault blink
    mesh_object->m_scale = effector_scale;
  }

  //
  // Heaters for the temperature tint
  //
  #if HOTENDS
    for (int h = 0; h < MachineModel::hotend_count(); ++h)
      hotend_heaters.push_back(virtual_printer.get_component<Heater>("Hotend" + std::to_string(h) + " Heater"));
  #endif
  #if TEMP_SENSOR_BED
    bed_heater = virtual_printer.get_component<Heater>("Bed Heater");
  #endif

  //
  // Set up the bed plane
  //
  m_bed_mesh = renderer::create_mesh();
  auto mesh_object = renderer::get_mesh_by_id(m_bed_mesh);
  mesh_object->set_shader_program(default_program);
  mesh_object->m_shader_instance->set_uniform("u_tint", &bed_tint);
  mesh_object->m_shader_instance->set_uniform("u_dim", &bed_dim);
  auto bed_mesh_buffer = renderer::Buffer<renderer::vertex_data_t>::create();
  mesh_object->buffer_vector<renderer::vertex_data_t>().push_back(bed_mesh_buffer);
  bed_mesh_buffer->data().reserve((BED_NUM_VERTEXES_PER_AXIS * BED_NUM_VERTEXES_PER_AXIS * 6));

  #if ENABLED(DELTA)

    // A Delta bed is round: a disc of rings and sectors, centered on the build plate
    {
      constexpr int rings = 50, sectors = 120;
      const GLfloat cx = build_plate_dimension.x * 0.5f, cz = -build_plate_dimension.y * 0.5f,
                    radius = std::min(build_plate_dimension.x, build_plate_dimension.y) * 0.5f;
      auto point = [&](const int ring, const int sector) {
        const GLfloat r = radius * ring / rings, a = glm::two_pi<GLfloat>() * sector / sectors;
        return glm::vec2(cx + r * std::cos(a), cz + r * std::sin(a));
      };
      // Emit one triangle wound counter-clockwise seen from above (+Y), like the square bed
      auto triangle = [&](glm::vec2 p0, glm::vec2 p1, glm::vec2 p2) {
        const glm::vec2 e1 = p1 - p0, e2 = p2 - p0;
        if (e1.y * e2.x - e1.x * e2.y < 0) std::swap(p1, p2); // Y of cross((e1.x,0,e1.y), (e2.x,0,e2.y))
        bed_mesh_buffer->add_vertex(BED_VERTEX(p0.x, p0.y));
        bed_mesh_buffer->add_vertex(BED_VERTEX(p1.x, p1.y));
        bed_mesh_buffer->add_vertex(BED_VERTEX(p2.x, p2.y));
      };
      for (int ring = 0; ring < rings; ++ring)
        for (int sector = 0; sector < sectors; ++sector) {
          const glm::vec2 a = point(ring, sector), b = point(ring, sector + 1),
                          c = point(ring + 1, sector), d = point(ring + 1, sector + 1);
          if (ring) triangle(a, b, d);   // The innermost ring is a fan
          triangle(a, d, c);
        }
    }

  #else

  // Calculate the number of divisions (line segments) along each axis.
  const GLfloat x_div = GLfloat(build_plate_dimension.x) / (BED_NUM_VERTEXES_PER_AXIS - 1);
  const GLfloat y_div = GLfloat(-build_plate_dimension.y) / (BED_NUM_VERTEXES_PER_AXIS - 1);

  // Generate a subdivided plane mesh for the bed
  for (int row = 0; row < BED_NUM_VERTEXES_PER_AXIS - 1; ++row) {
    for (int col = 0; col < BED_NUM_VERTEXES_PER_AXIS - 1; ++col) {
      // For each division, calculate the coordinates of the four corners.
      GLfloat x1 = col * x_div;
      GLfloat x2 = (col + 1) * x_div;
      GLfloat y1 = row * y_div;
      GLfloat y2 = (row + 1) * y_div;

      // Create two triangles from the four corners.

      // |--/
      // | /
      // |/
      bed_mesh_buffer->add_vertex(BED_VERTEX(x2, y2));
      bed_mesh_buffer->add_vertex(BED_VERTEX(x1, y2));
      bed_mesh_buffer->add_vertex(BED_VERTEX(x1, y1));
      //    /|
      //   / |
      //  /--|
      bed_mesh_buffer->add_vertex(BED_VERTEX(x2, y2));
      bed_mesh_buffer->add_vertex(BED_VERTEX(x1, y1));
      bed_mesh_buffer->add_vertex(BED_VERTEX(x2, y1));
    }
  }

  #endif // !DELTA

  //
  // Optional: Load external 3‑D geometry
  //
  /*
   * Example placeholder – replace with your loader:
   *
   * auto external_mesh = renderer::create_mesh();
   * renderer::load_obj("path/to/model.obj", external_mesh);
   * renderer::get_mesh_by_id(external_mesh)->set_shader_program(default_program);
   *
   * // Store it in a list for later rendering
   * external_geometries.push_back(external_mesh);
   */

  //
  // Initialise extruder positions from the printer state
  //
  auto kin = virtual_printer.get_component<KinematicSystem>("Cartesian Kinematic System");
  if (kin == nullptr) kin = virtual_printer.get_component<KinematicSystem>("Delta Kinematic System");
  if (kin == nullptr) kin = virtual_printer.get_component<KinematicSystem>("CoreXY Kinematic System");
  if (kin == nullptr) kin = virtual_printer.get_component<KinematicSystem>("CoreXZ Kinematic System");
  if (kin == nullptr) kin = virtual_printer.get_component<KinematicSystem>("CoreYZ Kinematic System");
  if (kin == nullptr) kin = virtual_printer.get_component<KinematicSystem>("CoreYX Kinematic System");
  if (kin == nullptr) kin = virtual_printer.get_component<KinematicSystem>("CoreZX Kinematic System");
  if (kin == nullptr) kin = virtual_printer.get_component<KinematicSystem>("CoreZY Kinematic System");
  if (kin != nullptr && kin->state.effector_position.size() == extrusion.size()) {
    size_t i = 0;
    for (auto state : kin->state.effector_position) {
      glm::vec4 pos = {state.position.x, state.position.z, state.position.y * -1.0, state.position.w};
      extrusion[i].last_position = pos;
      extrusion[i].position = pos;
      ++i;
    }
  }

  //
  // Bed markings, on the bed surface
  //
  bed_markings.create(default_program);
  reach_volume.create(lit_program);

  renderer::render_mesh(m_bed_mesh);
  renderer::render_mesh(bed_markings.mesh);
  renderer::render_mesh(reach_volume.mesh);
  for (auto mesh : m_extruder_mesh) {
    renderer::render_mesh(mesh);
  }
  machine.queue_render();
  //
  // Render any externally loaded geometry
  //
  /*
   * for (auto mesh_id : external_geometries)
   *     renderer::render_mesh(mesh_id);
   */
  renderer::render_list_is_ready();
  m_initialised = true;
}

void Visualisation::process_event(SDL_Event& e) { }
void Visualisation::gpio_event_handler(GpioEvent& event) { }
void Visualisation::on_position_update() { }

using millisec = std::chrono::duration<float, std::milli>;
void Visualisation::update() {
  std::scoped_lock extrusion_lock(extrusion_mutex);
  // auto now = clock.now();
  // float delta = std::chrono::duration_cast<std::chrono::duration<float>>(now - last_update).count();
  // last_update = now;
  auto effector_pos = extrusion[0].position;

  //
  // Printer model: place the bed and moving parts for the current nozzle position.
  // M218, M501 (EEPROM) etc. can move a hotend, so rebuild the parts to match.
  //
  if (machine.layout_changed()) {
    machine.build(machine.type, lit_program);
    machine.set_visible(show_machine);
    render_list_dirty = true;
  }
  const glm::vec3 nozzle { effector_pos.x, -effector_pos.z, effector_pos.y }; // Back to Marlin XYZ
  // IDEX: the X2 carriage's nozzle X, from the first tool it carries
  const float x2 = MachineModel::carriage_count() > 1 && extrusion.size() > 1 ? extrusion[1].position.x : 0.0f;
  MachinePose pose = machine.pose(nozzle, x2);
  if (!show_machine) {
    pose.bed_offset = {};
    pose.nozzle_world = glm::vec3(effector_pos);
  }
  #if ENABLED(BLTOUCH)
    // A stowed BLTouch pulls its pin up, so the probe rides higher until the servo deploys it
    if (auto bltouch = virtual_printer.get_component<BLTouchProbe>("BLTouch"); bltouch && !bltouch->deployed())
      pose.probe_raise = MachineModel::probe_stow_raise;
  #endif
  if (machine.type == MACHINE_DELTA) {
    // Delta tower steppers are linear, so they give the carriage heights directly
    if (auto delta = virtual_printer.get_component<DeltaKinematicSystem>("Delta Kinematic System")) {
      pose.has_towers = true;
      pose.towers = delta->hardware_offset[0] + delta->state.effector_position[0].stepper_position;
    }
  }
  if (show_machine) machine.update(pose);

  //
  // Agent camera requests (POST /view), applied on the UI thread
  //
  apply_view_request();

  //
  // Camera: Turntable aims at a target; Fly uses follow modes
  //
  if (camera_mode == CAMERA_TURNTABLE) {
    if (follow_nozzle) turntable.target = pose.nozzle_world;
    turntable_apply();
  }
  else {
    switch (follow_mode) {
      case FOLLOW_Z:  camera.position = glm::vec3(effector_pos.x, camera.position.y, effector_pos.z); break;
      case FOLLOW_XY: camera.position = glm::vec3(camera.position.x, effector_pos.y + follow_offset.y, camera.position.z); break;
      default: break;
    }
    camera.update_view();
  }

  //
  // Update bed mesh based on PrintBed component
  //
  auto print_bed = virtual_printer.get_component<PrintBed>("Print Bed");

  auto bed_mesh = renderer::get_mesh_by_id(m_bed_mesh);
  if (print_bed->dirty) {
    GLfloat max_abs_z = 0.0f;
    // todo: move vertex generation
    auto bed_mesh_buffer = bed_mesh->buffer<renderer::vertex_data_t>();

    for (auto& v : bed_mesh_buffer->data()) {
      GLfloat z = print_bed->calculate_z({v.position.x, -v.position.z});
      v.position.y = z;
      max_abs_z = std::max(max_abs_z, abs(z));
    }

    for (auto&v : bed_mesh_buffer->data()) {
      GLfloat r = 0.0, g = 0.0, b = 0.0;

      if (print_bed->gradient_enabled) {
        GLfloat z = v.position.y;
        GLfloat *dest = z < 0.0 ? &r : &b;
        GLfloat gradient_range = std::max(1.0f, max_abs_z);

        *dest = std::min(1.0f, abs(z) / gradient_range);
        g = 0.5f - *dest;
      } else {
        // default color
        r = g = b = 0.5f;
      }

      v.color.r = r;
      v.color.g = g;
      v.color.b = b;
    }
    print_bed->dirty = false;
    bed_markings_dirty = true;
  }
  update_heat_tint(print_bed->gradient_enabled);
  if (bed_mesh->m_position != pose.bed_offset) {
    bed_mesh->m_position = pose.bed_offset;
    bed_mesh->m_transform_dirty = true;
  }

  // Bed markings follow the bed surface and Marlin's probe, mesh and homing settings
  if (bed_markings.visible) {
    bed_markings.update([&](const glm::vec2 p) { return print_bed->calculate_z(p); }, bed_markings_dirty);
    bed_markings_dirty = false;
    if (auto m = renderer::get_mesh_by_id(bed_markings.mesh); m && m->m_position != pose.bed_offset) {
      m->m_position = pose.bed_offset;
      m->m_transform_dirty = true;
    }
  }

  // Delta printable volume follows the active tool and Marlin's delta settings
  reach_volume.update();

  // update the position of the extruder mesh for visualisation
  size_t mesh_id = 0;
  bool draw_list_update = false;
  for (auto& ext : extrusion ) {
    auto pos = glm::vec3(ext.position.x, ext.position.y, ext.position.z) + pose.bed_offset;
    auto mesh_object = renderer::get_mesh_by_id(m_extruder_mesh[mesh_id]);
    if (mesh_object->m_position != pos) {
      mesh_object->m_position = pos;
      mesh_object->m_transform_dirty = true;
    }
    // The simple pointer marks the nozzle only when the printer model is hidden
    mesh_object->m_visible = !show_machine && (camera_mode == CAMERA_TURNTABLE || follow_mode != FOLLOW_Z);

    if (ext.should_clear) {
      draw_list_update = true;
      ext.should_clear = false;
      auto obj = renderer::get_mesh_by_id(ext.mesh);
      auto old_mesh_index = ext.mesh;
      ext.mesh = renderer::create_mesh();
      auto new_mesh_object = renderer::get_mesh_by_id(ext.mesh);

      if (obj != nullptr) {
        new_mesh_object->set_shader_program(obj->m_shader_instance);
        renderer::destroy_mesh(old_mesh_index);
      } else {
        new_mesh_object->set_shader_program(extrusion_program);
        new_mesh_object->m_shader_instance->set_uniform("u_layer_thickness", &extrude_thickness);
        new_mesh_object->m_shader_instance->set_uniform("u_layer_width", &extrude_width);
        new_mesh_object->m_shader_instance->set_uniform("u_view_position", &camera.position);
      }

    }
    auto extrusion_mesh_object = renderer::get_mesh_by_id(ext.mesh);
    if (extrusion_mesh_object != nullptr) {
      extrusion_mesh_object->m_visible = ext.is_visible;
      // The print rides on the bed
      if (extrusion_mesh_object->m_position != pose.bed_offset) {
        extrusion_mesh_object->m_position = pose.bed_offset;
        extrusion_mesh_object->m_transform_dirty = true;
      }
    }

    mesh_id ++;
  }

  if (render_list_dirty) {
    draw_list_update = true;
    render_list_dirty = false;
  }
  if (draw_list_update) {
    renderer::render_mesh(m_bed_mesh);
    renderer::render_mesh(bed_markings.mesh);
    renderer::render_mesh(reach_volume.mesh);
    for (auto mesh : m_extruder_mesh) {
      renderer::render_mesh(mesh);
    }
    machine.queue_render();
    for (auto& ext : extrusion) {
      renderer::render_mesh(ext.mesh);
    }
    renderer::render_list_is_ready();
  }

  renderer::render(camera.proj * camera.view);
}

//
// Temperature tint: light blue at ambient blending to pink at the heater's target
// (or 200°C hotend / 60°C bed while the heater is off, so cooling still shows).
// The active hotend glows when there's more than one. The leveling gradient, when
// enabled, owns the bed colours, so the bed tint is dropped.
//
// Tint ramps: cold (ambient) -> hot (target). Hotends are light blue -> pink; the bed is
// darker, deep blue -> deep red, so it doesn't overpower the model. Later: color themes.
struct HeatRamp { glm::vec3 cold, hot; float strength; };
static constexpr HeatRamp hotend_ramp { { 0.55f, 0.78f, 1.00f }, { 1.00f, 0.42f, 0.72f }, 0.80f },
                          bed_ramp    { { 0.12f, 0.24f, 0.55f }, { 0.55f, 0.08f, 0.12f }, 0.85f };

static glm::vec4 heat_tint(const double temp, const double target, const double fallback, const HeatRamp& ramp) {
  constexpr double ambient = 25;
  const double full = target > ambient ? target : fallback;
  const float t = float(glm::clamp((temp - ambient) / (full - ambient), 0.0, 1.0));
  return glm::vec4(glm::mix(ramp.cold, ramp.hot, t), ramp.strength);
}

void Visualisation::update_heat_tint(const bool gradient_enabled) {
  const int count = MachineModel::hotend_count();
  const int active = std::min(int(motion.extruder), count - 1);
  for (int h = 0; h < count; ++h) {
    glm::vec4 tint {};
    if (h < int(hotend_heaters.size()) && hotend_heaters[h]) {
      #if HOTENDS
        const double target = thermalManager.temp_hotend[h].target;
      #else
        const double target = 0;
      #endif
      tint = heat_tint(hotend_heaters[h]->hotend_temperature, target, 200, hotend_ramp);
    }
    machine.set_hotend_look(h, tint, count > 1 && h == active ? 0.6f : 0.0f, thermal_fault_blink::dim(h));
  }

  bed_dim = thermal_fault_blink::dim(thermal_fault_blink::bed);
  bed_tint = {};
  #if HAS_HEATED_BED
    if (bed_heater && !gradient_enabled)
      bed_tint = heat_tint(bed_heater->hotend_temperature, thermalManager.temp_bed.target, 60, bed_ramp);
  #else
    UNUSED(gradient_enabled);
  #endif
}

void Visualisation::destroy() {
  if (framebuffer != nullptr) {
    framebuffer->release();
    delete framebuffer;
  }
  machine.destroy();
  extrusion_program.reset();
  default_program.reset();
  lit_program.reset();
  renderer::destroy();
}

//
// Set the head position – called by the printer kinematic update
//
void Visualisation::set_head_position(size_t hotend_index, extruder_state& state) {
  if (!m_initialised || hotend_index >= extrusion.size()) return;
  glm::vec4 sim_pos = state.position;
  glm::vec4 position = {sim_pos.x, sim_pos.z, sim_pos.y * -1.0, sim_pos.w}; // correct for opengl coordinate system
  std::scoped_lock extrusion_lock(extrusion_mutex);
  auto& extruder = extrusion[hotend_index];
  glm::vec3 extrude_color = state.color;

  auto active_mesh = renderer::get_mesh_by_id(extruder.mesh);
  if (!active_mesh) return;
  auto active_buffer = active_mesh->buffer<renderer::vertex_data_t>();

  if (position != extruder.position) {

    // smooths out extrusion over a minimum length to fill in gaps todo: implement an simulation to do this better
    // also use Z (Y in opengl) change to reduce minimum extrude length
    if (glm::length(glm::vec3(position) - glm::vec3(extruder.last_extrusion_check)) > m_config.extrusion_check_min_line_length || glm::abs(position.y - extruder.last_extrusion_check.y) > m_config.extrusion_check_max_vertical_deviation) {
      extruder.extruding = position.w - extruder.last_extrusion_check.w > 0.0f;
      extruder.last_extrusion_check = position;
    }

    if (active_buffer != nullptr && active_buffer->size() > 1 && active_buffer->size() < renderer::MAX_BUFFER_SIZE) {

      if (glm::length(glm::vec3(position) - glm::vec3(extruder.last_position)) > m_config.extrusion_segment_minimum_length) { // smooth out the path so the model renders with less geometry, rendering each individual step hurts the fps
        if ((points_are_collinear(position, active_buffer->cdata().end()[-3].position, active_buffer->cdata().end()[-2].position, m_config.extrusion_segment_collinearity_max_deviation) && extruder.extruding == extruder.last_extruding) || ( extruder.extruding == false && extruder.last_extruding == false)) {
          // collinear and extrusion state has not changed so we can just change the current point.
          active_buffer->data().end()[-2].position = position;
          active_buffer->data().end()[-1].position = position;
        } else { // new point is not collinear with current path add new point
          active_buffer->data().end()[-1] = {position, {0.0, 1.0, 0.0}, {extrude_color, extruder.extruding}};
          active_buffer->add_vertex(active_buffer->cdata().back());
        }
        extruder.last_position = position;
        extruder.last_extruding = extruder.extruding;
      }

    } else { // need to change geometry buffer
      if (active_buffer == nullptr) {
        auto buffer = renderer::Buffer<renderer::vertex_data_t>::create();
        buffer->data().reserve(renderer::MAX_BUFFER_SIZE);
        buffer->m_geometry_type = renderer::GeometryPrimitive::LINE_STRIP_ADJACENCY;
        buffer->data().push_back({position, {0.0, 1.0, 0.0}, {extrude_color, 0.0}});
        active_mesh->buffer_vector<renderer::vertex_data_t>().push_back(buffer);
        active_buffer = buffer;
        extruder.last_extrusion_check = position;
      } else {
        renderer::vertex_data_t last_vertex = active_buffer->cdata().back();
        auto buffer = renderer::Buffer<renderer::vertex_data_t>::create();
        buffer->data().reserve(renderer::MAX_BUFFER_SIZE);
        buffer->m_geometry_type = renderer::GeometryPrimitive::LINE_STRIP_ADJACENCY;
        buffer->add_vertex(last_vertex);
        buffer->add_vertex({position, {0.0, 1.0, 0.0}, {extrude_color, extruder.extruding}});
        buffer->add_vertex(buffer->cdata().back());

        active_buffer = buffer;
        active_mesh->buffer_vector<renderer::vertex_data_t>().push_back(buffer);
      }
      // extra dummy verticies for line strip adjacency
      active_buffer->add_vertex(active_buffer->cdata().back());
      active_buffer->add_vertex(active_buffer->cdata().back());
      extruder.last_position = position;
    }
    extruder.position = position;
  }
}

bool Visualisation::points_are_collinear(const glm::vec3 a, const glm::vec3 b, const glm::vec3 c, double const threshold) const {
  return glm::abs(glm::dot(b - a, c - a) - (glm::length(b - a) * glm::length(c - a))) < threshold;
}

//
// UI – viewport menu (camera settings, extrusion controls)
//
void Visualisation::ui_viewport_menu_callback(UiWindow*) {
  std::scoped_lock extrusion_lock(extrusion_mutex);
  bool open_settings = false;
  if (ImGui::BeginMenuBar()) {
    if (ImGui::BeginMenu("Camera")) {
      if (ImGui::MenuItem("Reset", "R")) {
        if (camera_mode == CAMERA_TURNTABLE)
          turntable = turntable_home();
        else {
          follow_mode = FOLLOW_NONE;
          camera = initCamera;
          camera.generate();
        }
      }
      if (ImGui::BeginMenu("Mode")) {
        if (ImGui::MenuItem("Turntable", nullptr, camera_mode == CAMERA_TURNTABLE)) set_camera_mode(CAMERA_TURNTABLE);
        if (ImGui::MenuItem("Fly", nullptr, camera_mode == CAMERA_FLY)) set_camera_mode(CAMERA_FLY);
        ImGui::EndMenu();
      }

      if (camera_mode == CAMERA_TURNTABLE) {
        // Turntable view presets (yaw, pitch)
        if (ImGui::BeginMenu("View")) {
          if (ImGui::MenuItem("Front", "1"))  turntable_preset(  0.0f,  0.0f);
          if (ImGui::MenuItem("Right", "3"))  turntable_preset( 90.0f,  0.0f);
          if (ImGui::MenuItem("Back"))        turntable_preset(180.0f,  0.0f);
          if (ImGui::MenuItem("Left"))        turntable_preset(-90.0f,  0.0f);
          if (ImGui::MenuItem("Top", "7"))    turntable_preset(  0.0f, 89.0f);
          if (ImGui::MenuItem("3/4", "0"))    turntable_preset( 30.0f, 25.0f);
          ImGui::EndMenu();
        }
        ImGui::MenuItem("Follow Nozzle", "F", &follow_nozzle);
        if (!follow_nozzle && ImGui::MenuItem("Center on Bed")) turntable.target = turntable_home().target;
        ImGui::MenuItem("Auto-Rotate", nullptr, &auto_rotate);
      }
      else if (ImGui::BeginMenu("Focus View")) {
        // Fly camera presets
        if (ImGui::MenuItem("Centre X (Right)")) {
          camera.position = {build_plate_dimension.x, 10.0f, -(build_plate_dimension.y / 2.0f)};
          camera.rotation = {-90.0f, 0.0, 0.0f};
        }
        if (ImGui::MenuItem("Centre Y (Front)")) {
          camera.position = {build_plate_dimension.x / 2.0f, 10.0f, 0.0f};
          camera.rotation = {-180.0f, 0.0, 0.0f};
        }
        if (ImGui::MenuItem("Centre Z (Top)")) {
          camera.position = {build_plate_dimension.x / 2.0, 200.0f, -(build_plate_dimension.y / 2.0f)};
          camera.rotation = {90.0f, -90.0, 0.0f};
        }
        ImGui::EndMenu();
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Printer")) {
      if (ImGui::MenuItem("Show Printer", "P", show_machine)) toggle_printer();
      if (ImGui::MenuItem("Bed Markings", "M", bed_markings.visible)) toggle_markings();
      #if ENABLED(DELTA)
        if (ImGui::MenuItem("Printable Volume", "V", reach_volume.visible)) toggle_volume();
      #endif
      ImGui::Separator();
      for (uint8_t t = 0; t < MACHINE_TYPE_COUNT; ++t) {
        const MachineType type = MachineType(t);
        if (!MachineModel::is_available(type)) continue;
        const bool is_default = type == MachineModel::default_type();
        char label[40];
        snprintf(label, sizeof(label), "%s%s", MachineModel::type_name(type), is_default ? " (config)" : "");
        if (ImGui::MenuItem(label, nullptr, machine.type == type) && machine.type != type)
          set_machine_type(type);
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Extrusion")) {
      if (ImGui::MenuItem("Settings")) {
        open_settings = true;
      }
      if (ImGui::MenuItem("Hide All")) {
        for (auto& ext : extrusion) {
          ext.is_visible = false;
        }
      }
      if (ImGui::MenuItem("Show All")) {
        for (auto& ext : extrusion) {
          ext.is_visible = true;
        }
      }
      if (ImGui::MenuItem("Clear Bed")) {
        for (auto& ext : extrusion) {
          ext.should_clear = true;
        }
      }
      if (ImGui::BeginMenu("Visible")) {
        size_t count = 0;
        for (auto& extruder : extrusion) {
          bool vis          = extruder.is_visible;
          std::string label = "Extrusion ";
          label += std::to_string(count);
          if (ImGui::Checkbox(label.c_str(), &vis)) {
            extruder.is_visible = vis;
          }
          ++count;
        }
        ImGui::EndMenu();
      }
      ImGui::EndMenu();
    }
    ImGui::EndMenuBar();
  }

  if (open_settings) {
    ImGui::OpenPopup("Extrusion Settings");
  }
}

//
// Apply a pending POST /view request and publish the camera for GET /view
//
void Visualisation::apply_view_request() {
  view_control::Request r;
  bool have = false;
  {
    std::scoped_lock lock(view_control::mutex);
    if (view_control::pending) { r = view_control::request; view_control::pending = false; have = true; }
  }
  if (have) {
    if (!r.machine.empty()) {
      const MachineType type = machine_type_from_name(r.machine.c_str());
      if (MachineModel::is_available(type) && type != machine.type) {
        machine.build(type, lit_program);
        machine.set_visible(show_machine);
        render_list_dirty = true;
      }
    }
    camera_mode = CAMERA_TURNTABLE;
    if (!r.preset.empty()) {
      if (r.preset == "home") turntable = turntable_home();
      else if (r.preset == "front") turntable_preset(0, 15);
      else if (r.preset == "right") turntable_preset(90, 15);
      else if (r.preset == "back")  turntable_preset(180, 15);
      else if (r.preset == "left")  turntable_preset(-90, 15);
      else if (r.preset == "top")   turntable_preset(0, 89);
      else if (r.preset == "iso")   turntable_preset(30, 25);
    }
    if (r.has_yaw) turntable.yaw = r.yaw;
    if (r.has_pitch) turntable.pitch = r.pitch;
    if (r.has_distance) turntable.distance = r.distance;
    if (r.has_target) turntable.target = { r.target[0], r.target[2], -r.target[1] }; // Marlin -> GL
    if (r.has_follow) follow_nozzle = r.follow;
    if (r.has_markings) { bed_markings.set_visible(r.markings); bed_markings_dirty = true; }
    if (r.has_volume) reach_volume.set_visible(r.volume);
    auto_rotate = false;
  }
  std::scoped_lock lock(view_control::mutex);
  auto& s = view_control::state;
  s.turntable = camera_mode == CAMERA_TURNTABLE;
  s.yaw = turntable.yaw; s.pitch = turntable.pitch; s.distance = turntable.distance;
  s.target[0] = turntable.target.x; s.target[1] = -turntable.target.z; s.target[2] = turntable.target.y;
  s.follow = follow_nozzle;
  s.machine = MachineModel::type_name(machine.type);
  s.markings = bed_markings.visible;
  s.volume = reach_volume.visible;
}

//
// Switch the printer model shown in the viewport.
// The caller must hold extrusion_mutex (the viewport menu callback does). It's
// a plain std::mutex, so locking it again here deadlocked the UI thread.
//
void Visualisation::set_machine_type(const MachineType type) {
  machine.build(type, lit_program);
  machine.set_visible(show_machine);
  render_list_dirty = true;
  if (camera_mode == CAMERA_TURNTABLE && !follow_nozzle) {
    const TurntableView home = turntable_home();
    turntable.target = home.target;
    turntable.distance = home.distance;
  }
}

//
// Turntable camera
//

// Home view: 3/4 from the front-right, framing the whole build volume
TurntableView Visualisation::turntable_home() const {
  // Frame the printer model, or the build volume when the model is hidden.
  // GL coordinates: X right, Y up, Z toward the viewer (Marlin -Y)
  glm::vec3 lo { 0.0f, 0.0f, -float(build_plate_dimension.y) },
            hi { float(build_plate_dimension.x), float(Z_MAX_POS - Z_MIN_POS), 0.0f };
  if (show_machine) { lo = machine.bounds_min; hi = machine.bounds_max; }
  const glm::vec3 size = hi - lo;
  TurntableView v;
  v.target = (lo + hi) * 0.5f;
  // Distance at which the bounding sphere fits the vertical field of view
  const float radius = glm::length(size) * 0.5f;
  v.distance = radius / sin(camera.fov * 0.5f) * 0.9f;
  return v;
}

void Visualisation::turntable_preset(const float yaw, const float pitch) {
  turntable.yaw = yaw;
  turntable.pitch = pitch;
  auto_rotate = false;
}

// Place the camera on a sphere around the target
void Visualisation::turntable_apply() {
  turntable.pitch = glm::clamp(turntable.pitch, -10.0f, 89.0f);
  turntable.distance = glm::clamp(turntable.distance, 20.0f, 4500.0f);
  turntable.yaw = fmod(turntable.yaw, 360.0f);
  const float yaw = glm::radians(turntable.yaw), pitch = glm::radians(turntable.pitch);
  const glm::vec3 offset { sin(yaw) * cos(pitch), sin(pitch), cos(yaw) * cos(pitch) };
  camera.look_at(turntable.target + offset * turntable.distance, turntable.target);
}

void Visualisation::set_camera_mode(const CameraMode mode) {
  if (mode == camera_mode) return;
  if (mode == CAMERA_FLY) {
    // Fly continues from the current turntable view (look_at synced the angles)
    follow_mode = FOLLOW_NONE;
    camera.update_view();
  }
  else
    turntable_apply();
  camera_mode = mode;
}

// Printer menu toggles, also on the P / M / V keys
void Visualisation::toggle_printer() {
  show_machine = !show_machine;
  machine.set_visible(show_machine);
}
void Visualisation::toggle_markings() {
  bed_markings.set_visible(!bed_markings.visible);
  bed_markings_dirty = true;
}
void Visualisation::toggle_volume() {
  TERN_(DELTA, reach_volume.set_visible(!reach_volume.visible));
}

// Left-drag rotates the printer, right/middle-drag pans, wheel zooms.
// The pointer is never captured.
void Visualisation::turntable_input(Viewport& viewport, const float delta) {
  ImGuiIO& io = ImGui::GetIO();

  if (viewport.active) {
    const ImVec2 d = io.MouseDelta;
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && !io.KeyShift) {
      turntable.yaw   -= d.x * 0.3f;  // Drag right: printer turns right
      turntable.pitch += d.y * 0.3f;  // Drag down: tip the top toward the viewer
      auto_rotate = false;
    }
    else if (ImGui::IsMouseDown(ImGuiMouseButton_Right) || ImGui::IsMouseDown(ImGuiMouseButton_Middle) || io.KeyShift) {
      // Pan in the view plane, scaled so the point under the cursor tracks it
      const float units_per_pixel = 2.0f * turntable.distance * tan(camera.fov * 0.5f) / std::max(1.0f, viewport.viewport_size.y);
      turntable.target += (-d.x * camera.right + d.y * camera.up) * units_per_pixel;
      follow_nozzle = false;
    }
  }

  if (viewport.hovered && io.MouseWheel != 0)
    turntable.distance *= pow(1.0f - turntable_wheel_zoom, io.MouseWheel);

  if (viewport.double_clicked) turntable = turntable_home();

  if (viewport.focused) {
    if (ImGui::IsKeyPressed(ImGuiKey_R)) turntable = turntable_home();
    if (ImGui::IsKeyPressed(ImGuiKey_F)) follow_nozzle ^= true;
    if (ImGui::IsKeyPressed(ImGuiKey_1) || ImGui::IsKeyPressed(ImGuiKey_Keypad1)) turntable_preset( 0.0f,  0.0f);
    if (ImGui::IsKeyPressed(ImGuiKey_3) || ImGui::IsKeyPressed(ImGuiKey_Keypad3)) turntable_preset(90.0f,  0.0f);
    if (ImGui::IsKeyPressed(ImGuiKey_7) || ImGui::IsKeyPressed(ImGuiKey_Keypad7)) turntable_preset( 0.0f, 89.0f);
    if (ImGui::IsKeyPressed(ImGuiKey_0) || ImGui::IsKeyPressed(ImGuiKey_Keypad0)) turntable_preset(30.0f, 25.0f);
    // Arrow keys turn and tilt; +/- zoom
    const float turn = 90.0f * delta;
    if (ImGui::IsKeyDown(ImGuiKey_LeftArrow))  turntable.yaw   += turn;
    if (ImGui::IsKeyDown(ImGuiKey_RightArrow)) turntable.yaw   -= turn;
    if (ImGui::IsKeyDown(ImGuiKey_UpArrow))    turntable.pitch += turn;
    if (ImGui::IsKeyDown(ImGuiKey_DownArrow))  turntable.pitch -= turn;
    if (ImGui::IsKeyDown(ImGuiKey_Equal) || ImGui::IsKeyDown(ImGuiKey_KeypadAdd))      turntable.distance *= 1.0f - turntable_key_zoom * delta;
    if (ImGui::IsKeyDown(ImGuiKey_Minus) || ImGui::IsKeyDown(ImGuiKey_KeypadSubtract)) turntable.distance *= 1.0f + turntable_key_zoom * delta;
  }

  if (auto_rotate && !viewport.active) turntable.yaw += 12.0f * delta;
}

// Free-flying camera (the original controls)
void Visualisation::fly_input(Viewport& viewport, const float delta) {
  static bool invert_pan = false;
  auto& ex = extrusion[0];

  if (viewport.focused) {
    // R = Camera Reset
    if (ImGui::IsKeyDown(ImGuiKey_R)) {
      follow_mode = FOLLOW_NONE;
      camera = initCamera;
      camera.generate();
    }
    // W A S D = Camera Pan
    if (ImGui::IsKeyDown(ImGuiKey_W)) {
      const glm::vec3 dist = camera.world_up * camera.speed * delta;
      camera.position += invert_pan ? -dist : dist;
    }
    if (ImGui::IsKeyDown(ImGuiKey_S)) {
      const glm::vec3 dist = camera.world_up * camera.speed * delta;
      camera.position -= invert_pan ? -dist : dist;
    }
    if (ImGui::IsKeyDown(ImGuiKey_A)) {
      const glm::vec3 dist = glm::normalize(glm::cross(camera.direction, camera.up)) * camera.speed * delta;
      camera.position -= invert_pan ? -dist : dist;
    }
    if (ImGui::IsKeyDown(ImGuiKey_D)) {
      const glm::vec3 dist = glm::normalize(glm::cross(camera.direction, camera.up)) * camera.speed * delta;
      camera.position += invert_pan ? -dist : dist;
    }
    // I = Invert WASD
    if (ImGui::IsKeyPressed(ImGuiKey_I)) {
      invert_pan ^= true;
    }
    // E / Q = Camera Zoom / Unzoom
    if (ImGui::IsKeyDown(ImGuiKey_E)) {
      camera.position += fly_key_zoom * camera.direction * delta;
    }
    if (ImGui::IsKeyDown(ImGuiKey_Q)) {
      camera.position -= fly_key_zoom * camera.direction * delta;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F)) {
      follow_mode = follow_mode == FOLLOW_Z ? FOLLOW_NONE : FOLLOW_Z;
      if (follow_mode != FOLLOW_NONE) {
        camera.position = glm::vec3(ex.position.x, ex.position.y + 10.0, ex.position.z);
        camera.rotation.y = -89.99999;
      }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_G)) {
      follow_mode = follow_mode == FOLLOW_XY ? FOLLOW_NONE : FOLLOW_XY;
      if (follow_mode != FOLLOW_NONE)
        follow_offset = camera.position - glm::vec3(ex.position);
    }
    if (ImGui::GetIO().MouseWheel != 0 && viewport.hovered) {
      camera.position += fly_wheel_zoom * camera.direction * ImGui::GetIO().MouseWheel;
    }
  }

  bool last_mouse_captured = mouse_captured;
  if (ImGui::IsMouseDown(0) && viewport.active) {
    mouse_captured = true;
  } else if (!ImGui::IsMouseDown(0)) {
    mouse_captured = false;
  }

  // IMGUI: Relative Mouse mode within a window does not seem to be supported through the imgui mouse api
  if (mouse_captured && !last_mouse_captured) {        // Mouse button was just pressed
    ImVec2 mouse_pos = ImGui::GetMousePos();
    mouse_lock_pos = { mouse_pos.x, mouse_pos.y };
    SDL_SetWindowGrab(SDL_GL_GetCurrentWindow(), SDL_TRUE);
    SDL_SetRelativeMouseMode(SDL_TRUE);
    SDL_GetRelativeMouseState(nullptr, nullptr);
  } else if (!mouse_captured && last_mouse_captured) { // Mouse button was just released
    SDL_SetRelativeMouseMode(SDL_FALSE);
    SDL_SetWindowGrab(SDL_GL_GetCurrentWindow(), SDL_FALSE);
    SDL_WarpMouseInWindow(SDL_GL_GetCurrentWindow(), mouse_lock_pos.x, mouse_lock_pos.y);
    SDL_GetRelativeMouseState(nullptr, nullptr);
  } else if (mouse_captured) {                         // Mouse button is being held
    int rel_x, rel_y;
    SDL_GetRelativeMouseState(&rel_x, &rel_y);
    camera.rotation.x -= rel_x * 0.2;
    camera.rotation.y -= rel_y * 0.2;
    if (camera.rotation.y > 89.0f) camera.rotation.y = 89.0f;
    else if (camera.rotation.y < -89.0f) camera.rotation.y = -89.0f;
  }
}

//
// UI – viewport rendering + camera controls
//
void Visualisation::ui_viewport_callback(UiWindow* window) {
  std::scoped_lock extrusion_lock(extrusion_mutex);
  auto now = clock.now();
  float delta = std::chrono::duration_cast<std::chrono::duration<float>>(now- last_update).count();
  last_update = now;

  Viewport& viewport = *((Viewport*)window);

  if (viewport.dirty) {
    viewport.viewport_size.x = viewport.viewport_size.x > 0 ? viewport.viewport_size.x : 0;
    viewport.viewport_size.y = viewport.viewport_size.y > 0 ? viewport.viewport_size.y : 0;
    framebuffer->update(viewport.viewport_size.x, viewport.viewport_size.y);
    viewport.texture_id = framebuffer->texture_id();
    camera.update_aspect_ratio(viewport.viewport_size.x / viewport.viewport_size.y);
  }

  if (camera_mode == CAMERA_TURNTABLE)
    turntable_input(viewport, delta);
  else
    fly_input(viewport, delta);

  // Show / hide keys, in either camera mode
  if (viewport.focused) {
    if (ImGui::IsKeyPressed(ImGuiKey_P, false)) toggle_printer();
    if (ImGui::IsKeyPressed(ImGuiKey_M, false)) toggle_markings();
    if (ImGui::IsKeyPressed(ImGuiKey_V, false)) toggle_volume();
  }

  //
  // Render the “Extrusion Settings” popup
  //
  if (ImGui::BeginPopup("Extrusion Settings")) {
    ImGui::PushItemWidth(150);
    ImGui::Text("Extrude Width    ");
    ImGui::PopItemWidth();
    ImGui::PushItemWidth(50);
    ImGui::SameLine();
    ImGui::InputFloat("##Extrude_Width", &extrude_width);
    ImGui::PopItemWidth();

    ImGui::PushItemWidth(150);
    ImGui::Text("Extrude Thickness");
    ImGui::PopItemWidth();
    ImGui::PushItemWidth(50);
    ImGui::SameLine();
    ImGui::InputFloat("##Extrude_Thickness", &extrude_thickness);
    ImGui::PopItemWidth();

    ImGui::PushItemWidth(150);
    ImGui::Text("Extrusion Check Min");
    ImGui::PopItemWidth();
    ImGui::PushItemWidth(100);
    ImGui::InputDouble("##Extrusion_Check_Min", &m_config.extrusion_check_min_line_length);
    ImGui::PopItemWidth();

    ImGui::PushItemWidth(150);
    ImGui::Text("Extrusion Check Vertical Max");
    ImGui::PopItemWidth();
    ImGui::PushItemWidth(100);
    ImGui::InputDouble("##Extrusion_Check_Vertical_Max", &m_config.extrusion_check_max_vertical_deviation);
    ImGui::PopItemWidth();

    ImGui::PushItemWidth(150);
    ImGui::Text("Extrusion Segment Min Length");
    ImGui::PopItemWidth();
    ImGui::PushItemWidth(100);
    ImGui::InputDouble("##Extrusion_Segment_Min_Length", &m_config.extrusion_segment_minimum_length);
    ImGui::PopItemWidth();

    ImGui::PushItemWidth(150);
    ImGui::Text("Extrusion Collinearity Max Deviation");
    ImGui::PopItemWidth();
    ImGui::PushItemWidth(100);
    ImGui::InputDouble("##Extrusion_Collinearity_Max_Deviation", &m_config.extrusion_segment_collinearity_max_deviation);
    ImGui::PopItemWidth();
    ImGui::EndPopup();
  }
};

//
// UI – info panel (FPS, shader reload)
//
void Visualisation::ui_info_callback(UiWindow* w) {
  ImGui::Text("Application average %.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
  ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);

  if (ImGui::Button("Reload Shaders")) {
    if (!extrusion_program->reload()) {
      logger::warning("Shader Reload Failed!\n");
    }
  }
}

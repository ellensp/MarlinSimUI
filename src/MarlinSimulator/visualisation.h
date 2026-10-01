#pragma once

#include <vector>
#include <array>

#include "hardware/print_bed.h"
#include "hardware/bed_probe.h"
#include "hardware/KinematicSystem.h"

#include <gl.h>
#include <SDL2/SDL.h>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/epsilon.hpp>

#include "window.h"
#include "user_interface.h"

#include "renderer/renderer.h"
#include "machine_model.h"
#include "bed_markings.h"
#include "delta_reach.h"

constexpr glm::ivec2 build_plate_dimension{X_BED_SIZE, Y_BED_SIZE};
constexpr glm::ivec2 build_plate_offset{X_MIN_POS, Y_MIN_POS};

using millisec = std::chrono::duration<float, std::milli>;

class PerspectiveCamera {
public:
  PerspectiveCamera() = default;
  PerspectiveCamera(glm::vec3 position,
                    glm::vec3 rotation,
                    glm::vec3 world_up,
                    float aspect_ratio,
                    float fov,
                    float clip_near,
                    float clip_far
                    ):position{position},
                    rotation{rotation},
                    world_up{world_up},
                    aspect_ratio{aspect_ratio},
                    fov{fov},
                    clip_near{clip_near},
                    clip_far{clip_far},
                    speed{50.0f},
                    front{0.0f, 0.0f, -1.0f} {
    generate();
  }

  void generate() {
    proj = glm::perspective(fov, aspect_ratio, clip_near, clip_far);
    update_view();
  }

  void update_view() {
    direction = glm::vec3(  cos(glm::radians(rotation.y)) * sin(glm::radians(rotation.x)),
                            sin(glm::radians(rotation.y)),
                            cos(glm::radians(rotation.y)) * cos(glm::radians(rotation.x)));

    right = glm::vec3(sin(glm::radians(rotation.x) - (glm::pi<float>() / 2.0)), 0, cos(glm::radians(rotation.x) - (glm::pi<float>() / 2.0)));
    up = glm::cross(right, direction);
    view = glm::lookAt(position, position + direction, up);
  }

  // Aim the camera from 'eye' at 'target', keeping the fly-mode angles in sync
  // so switching to Fly mode continues from the same view.
  void look_at(const glm::vec3 eye, const glm::vec3 target) {
    position  = eye;
    direction = glm::normalize(target - eye);
    rotation.x = glm::degrees(atan2(direction.x, direction.z));
    rotation.y = glm::degrees(asin(glm::clamp(direction.y, -1.0f, 1.0f)));
    right = glm::normalize(glm::cross(direction, world_up));
    up    = glm::cross(right, direction);
    view  = glm::lookAt(position, target, up);
  }

  void update_aspect_ratio(float ar) {
    aspect_ratio = ar;
    proj = glm::perspective(fov, aspect_ratio, clip_near, clip_far);
  }

  void update_direction() {
  }

  float* proj_ptr() { return glm::value_ptr(proj); }
  float* view_ptr() { return glm::value_ptr(view); }

  // view
  glm::vec3 position;
  glm::vec3 rotation;
  glm::vec3 focal_point;
  glm::vec3 world_up;
  glm::vec3 right;
  glm::vec3 direction;
  glm::vec3 up;

  // projection
  float aspect_ratio;
  float fov;
  float clip_near;
  float clip_far;
  glm::mat4 view;
  glm::mat4 proj;
  float speed;
  glm::vec3 front;
};

enum FollowMode : uint8_t { FOLLOW_NONE, FOLLOW_Z, FOLLOW_XY };

// Turntable: fixed camera aimed at the printer; dragging rotates the printer.
// Fly: free WASD camera with captured-mouse look.
enum CameraMode : uint8_t { CAMERA_TURNTABLE, CAMERA_FLY };

// Turntable view state. Angles in degrees; yaw 0 = looking from the front.
struct TurntableView {
  glm::vec3 target {};    // Point the camera orbits and looks at (GL coordinates)
  float yaw      = 30.0f; // Positive = camera moves to the printer's right
  float pitch    = 25.0f; // Elevation above the bed plane
  float distance = 500.0f;
};

struct Extrusion {
  glm::vec4 last_position = {};
  glm::vec4 last_extrusion_check = {};
  bool extruding = false;
  bool last_extruding  = false;
  float extrude_width = 0.4;
  float extrude_thickness = 0.3;
  glm::vec4 position = {};
  bool is_visible = true;
  bool should_clear = true;

  std::shared_ptr<renderer::Buffer<renderer::vertex_data_t>> active_mesh_buffer {};
  renderer::mesh_id_t mesh {};
};
struct visualisation_config {
  double extrusion_check_min_line_length {0.5};
  double extrusion_check_max_vertical_deviation {0.01};
  double extrusion_segment_minimum_length {0.5};
  double extrusion_segment_collinearity_max_deviation {0.001};
};

class Visualisation {
public:
  Visualisation(VirtualPrinter& virtual_printer);
  ~Visualisation();
  void create();
  void process_event(SDL_Event& e);

  VirtualPrinter& virtual_printer;

  void gpio_event_handler(GpioEvent& event);
  void update();
  void destroy();
  void on_position_update();

  void ui_viewport_callback(UiWindow*);
  void ui_viewport_menu_callback(UiWindow*);
  void ui_info_callback(UiWindow*);

  std::vector<Extrusion> extrusion {};
  std::mutex extrusion_mutex {};

  void set_head_position(size_t hotend_index, extruder_state& position);
  bool points_are_collinear(const glm::vec3 a, const glm::vec3 b, const glm::vec3 c, double const threshold) const;

  CameraMode camera_mode = CAMERA_TURNTABLE;
  TurntableView turntable {};
  bool follow_nozzle = false; // Turntable: keep the nozzle at the view center
  bool auto_rotate = false;   // Turntable: slow spin while idle
  TurntableView turntable_home() const;
  void turntable_preset(const float yaw, const float pitch);
  void turntable_apply();
  void set_camera_mode(const CameraMode mode);
  void turntable_input(struct Viewport& viewport, const float delta);
  void fly_input(struct Viewport& viewport, const float delta);
  void toggle_printer();
  void toggle_markings();
  void toggle_volume();

  FollowMode follow_mode = FOLLOW_NONE;
  bool render_full_path = true;
  bool render_path_line = false;
  glm::vec3 follow_offset = {0.0f, 0.0f, 0.0f};
  std::chrono::steady_clock clock;
  std::chrono::steady_clock::time_point last_update;

  glm::vec3 effector_scale = {3.0f ,10.0f, 3.0f};

  PerspectiveCamera camera;
  opengl_util::FrameBuffer* framebuffer = nullptr;
  std::vector<renderer::mesh_id_t> m_extruder_mesh;
  renderer::mesh_id_t m_bed_mesh;

  std::shared_ptr<renderer::ShaderProgram> extrusion_program;
  std::shared_ptr<renderer::ShaderProgram> default_program;
  std::shared_ptr<renderer::ShaderProgram> lit_program;

  MachineModel machine;
  bool show_machine = true;

  // Origin, safe homing point, probeable area and mesh grid, drawn on the bed
  BedMarkings bed_markings;
  bool bed_markings_dirty = true; // The bed surface changed (leveling sliders)

  // Delta: translucent printable volume of the active tool
  ReachVolume reach_volume;
  bool render_list_dirty = false;
  void set_machine_type(const MachineType type);
  void apply_view_request();

  // Temperature tint (light blue cold -> pink hot) for the bed surface and each hotend's
  // heater block and nozzle, plus a glow on the active hotend. See update_heat_tint().
  std::vector<std::shared_ptr<class Heater>> hotend_heaters;
  std::shared_ptr<class Heater> bed_heater;
  glm::vec4 bed_tint {}, no_tint {};
  float bed_dim = 1, no_dim = 1;  // u_dim: 0.5 during the dark phase of a thermal fault blink
  void update_heat_tint(const bool gradient_enabled);

  bool mouse_captured = false;
  glm::vec<2, int> mouse_lock_pos;

  #define BED_NORMAL 0.0, 1.0, 0.0
  #define BED_COLOR 0.5, 0.5, 0.5, 1.0

  #define EFFECTOR_NORMAL 0.0, 0.0, 0.0
  #define EFFECTOR_COLOR_1 1.0, 0.0, 0.0, 1.0
  #define EFFECTOR_COLOR_2 0.0, 1.0, 0.0, 1.0
  #define EFFECTOR_COLOR_3 0.0, 0.0, 1.0, 1.0

  #define BED_VERTEX(X, Y) {{X, 0.0, Y}, {BED_NORMAL}, {BED_COLOR}}
  #define EFFECTOR_VERTEX(X, Z, Y, COLOR) {{X, Z, Y}, {EFFECTOR_NORMAL}, {COLOR}}
  #define BED_NUM_VERTEXES_PER_AXIS 100

  float extrude_width = 0.4;
  float extrude_thickness = 0.3;

  bool m_initialised = false;
  visualisation_config m_config {};
};

struct Viewport : public UiWindow {
  bool hovered = false;
  bool focused = false;
  bool active  = false;       // Mouse is held on the viewport (any button)
  bool double_clicked = false;
  ImVec2 viewport_size;
  GLuint texture_id = 0;
  bool dirty        = false;

  template<class... Args> Viewport(std::string name, Args... args) : UiWindow(name, args...) { }

  void show() {

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2 {0, 0});
    ImGui::Begin((char*)name.c_str(), nullptr, ImGuiWindowFlags_MenuBar);
    ImGui::PopStyleVar();
    if (menu_callback) menu_callback(this);

    auto size = ImGui::GetContentRegionAvail();
    if (viewport_size.x != size.x || viewport_size.y != size.y) {
      viewport_size = size;
      dirty         = true;
    }
    const ImVec2 image_pos = ImGui::GetCursorScreenPos();
    ImGui::Image((ImTextureID)(intptr_t)texture_id, viewport_size, ImVec2(0, 1), ImVec2(1, 0));
    hovered = ImGui::IsItemHovered();
    active = double_clicked = false;
    if (viewport_size.x > 0 && viewport_size.y > 0) {
      // Overlay an invisible button so drags on the image belong to the viewport
      // (camera control) instead of moving an undocked window.
      ImGui::SetCursorScreenPos(image_pos);
      ImGui::InvisibleButton("##viewport_input", viewport_size, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
      hovered = ImGui::IsItemHovered();
      active = ImGui::IsItemActive();
      double_clicked = hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);
    }
    focused = ImGui::IsWindowFocused();

    if (show_callback) show_callback(this);

    ImGui::End();
  }
};

#pragma once

#include <cstddef>
#include <cstdint>

#include <imgui.h>

#include "renderer.h"

namespace renderer {
namespace imgui_sharp_bilinear {

namespace detail {

inline GLuint& program() {
  static GLuint value = 0;
  return value;
}

inline GLint& projection_location() {
  static GLint value = -1;
  return value;
}

inline GLint& texture_location() {
  static GLint value = -1;
  return value;
}

inline const char* vertex_shader() {
  return R"glsl(
#version 330 core
layout (location = 0) in vec2 Position;
layout (location = 1) in vec2 UV;
layout (location = 2) in vec4 Color;

uniform mat4 ProjMtx;

out vec2 Frag_UV;
out vec4 Frag_Color;

void main() {
  Frag_UV = UV;
  Frag_Color = Color;
  gl_Position = ProjMtx * vec4(Position.xy, 0.0, 1.0);
}
)glsl";
}

inline const char* fragment_shader() {
  return R"glsl(
#version 330 core
uniform sampler2D Texture;

in vec2 Frag_UV;
in vec4 Frag_Color;

layout (location = 0) out vec4 Out_Color;

void main() {
  vec2 texture_size = vec2(textureSize(Texture, 0));
  vec2 texel_position = Frag_UV * texture_size - vec2(0.5);
  vec2 texel_base = floor(texel_position);
  vec2 texel_fraction = fract(texel_position);

  // Keep pixel interiors sharp and limit interpolation to about one
  // output pixel around each source-pixel boundary.
  vec2 transition_width = clamp(fwidth(texel_position), vec2(0.0001), vec2(1.0));
  texel_fraction = clamp(
    (texel_fraction - vec2(0.5)) / transition_width + vec2(0.5),
    vec2(0.0),
    vec2(1.0)
  );

  vec2 sample_uv = (texel_base + texel_fraction + vec2(0.5)) / texture_size;
  Out_Color = Frag_Color * texture(Texture, sample_uv);
}
)glsl";
}

inline void render_callback(const ImDrawList*, const ImDrawCmd*) {
  if (!program()) return;

  GLint imgui_program = 0;
  renderer::gl_assert_call(glGetIntegerv, GL_CURRENT_PROGRAM, &imgui_program);
  if (!imgui_program) return;

  const GLint imgui_projection_location =
    renderer::gl_assert_call(glGetUniformLocation, static_cast<GLuint>(imgui_program), "ProjMtx");
  if (imgui_projection_location < 0) return;

  GLfloat projection[16] = {};
  GLfloat* projection_ptr = projection;
  renderer::gl_assert_call(
    glGetUniformfv,
    static_cast<GLuint>(imgui_program),
    imgui_projection_location,
    projection_ptr
  );

  renderer::gl_assert_call(glUseProgram, program());

  // ImGui's own vertex shader has no layout qualifiers, so the linker picks its attribute
  // slots and they may not be 0/1/2 (they aren't on macOS). Point this program's fixed
  // slots at ImGui's vertex buffer, which is still bound. ImDrawCallback_ResetRenderState,
  // queued right after the image, restores ImGui's own attribute setup.
  renderer::gl_assert_call(glEnableVertexAttribArray, 0);
  renderer::gl_assert_call(glEnableVertexAttribArray, 1);
  renderer::gl_assert_call(glEnableVertexAttribArray, 2);
  renderer::gl_assert_call(glVertexAttribPointer, 0, 2, GL_FLOAT, GL_FALSE, GLsizei(sizeof(ImDrawVert)), (const GLvoid*)offsetof(ImDrawVert, pos));
  renderer::gl_assert_call(glVertexAttribPointer, 1, 2, GL_FLOAT, GL_FALSE, GLsizei(sizeof(ImDrawVert)), (const GLvoid*)offsetof(ImDrawVert, uv));
  renderer::gl_assert_call(glVertexAttribPointer, 2, 4, GL_UNSIGNED_BYTE, GL_TRUE, GLsizei(sizeof(ImDrawVert)), (const GLvoid*)offsetof(ImDrawVert, col));

  renderer::gl_assert_call(glUniform1i, texture_location(), 0);
  renderer::gl_assert_call(
    glUniformMatrix4fv,
    projection_location(),
    1,
    GL_FALSE,
    projection_ptr
  );
}

} // namespace detail

inline void set_texture_filter(const GLuint texture_id, const bool integer_scaling) {
  renderer::gl_assert_call(glBindTexture, GL_TEXTURE_2D, texture_id);
  const GLint texture_filter = integer_scaling ? GL_NEAREST : GL_LINEAR;
  renderer::gl_assert_call(glTexParameteri, GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, texture_filter);
  renderer::gl_assert_call(glTexParameteri, GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, texture_filter);
  renderer::gl_assert_call(glBindTexture, GL_TEXTURE_2D, 0);
}

inline void init(const GLuint texture_id, const bool integer_scaling) {
  set_texture_filter(texture_id, integer_scaling);
  if (detail::program()) return;

  GLint previous_program = 0;
  renderer::gl_assert_call(glGetIntegerv, GL_CURRENT_PROGRAM, &previous_program);

  detail::program() = renderer::ShaderProgram::load_program(
    detail::vertex_shader(),
    detail::fragment_shader()
  );

  if (detail::program()) {
    detail::projection_location() =
      renderer::gl_assert_call(glGetUniformLocation, detail::program(), "ProjMtx");
    detail::texture_location() =
      renderer::gl_assert_call(glGetUniformLocation, detail::program(), "Texture");
  }

  renderer::gl_assert_call(glUseProgram, static_cast<GLuint>(previous_program));
}

inline void image(const GLuint texture_id, const ImVec2 size, const bool integer_scaling) {
  if (integer_scaling || !detail::program()) {
    ImGui::Image((ImTextureID)(intptr_t)texture_id, size, ImVec2(0,0), ImVec2(1,1));
    return;
  }

  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  draw_list->AddCallback(detail::render_callback, nullptr);
  ImGui::Image((ImTextureID)(intptr_t)texture_id, size, ImVec2(0,0), ImVec2(1,1));
  draw_list->AddCallback(ImDrawCallback_ResetRenderState, nullptr);
}

} // namespace imgui_sharp_bilinear
} // namespace renderer

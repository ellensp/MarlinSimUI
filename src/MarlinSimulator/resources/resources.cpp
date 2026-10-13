#include "resources.h"
#include "resource_data.h"

#include <fstream>

namespace resource {
  std::map<std::filesystem::path, std::shared_ptr<Resource>> ResourceManager::s_loaded_resource = {};
  std::map<std::filesystem::path, std::shared_ptr<Resource>> ResourceManager::s_embedded_resource = {
    {{"null"}, std::make_shared<Resource>(nullptr)},
    {{"data/shaders/extrusion.gs"}, std::make_shared<Resource>(data_shader_extrusion_gs)},
    {{"data/shaders/extrusion.vs"}, std::make_shared<Resource>(data_shader_extrusion_vs)},
    {{"data/shaders/extrusion.fs"}, std::make_shared<Resource>(data_shader_extrusion_fs)},
    {{"data/shaders/default.vs"}, std::make_shared<Resource>(data_shader_default_vs)},
    {{"data/shaders/default.fs"}, std::make_shared<Resource>(data_shader_default_fs)},
    {{"data/shaders/lit.vs"}, std::make_shared<Resource>(data_shader_lit_vs)},
    {{"data/shaders/lit.fs"}, std::make_shared<Resource>(data_shader_lit_fs)},
    {{"imgui.ini"}, std::make_shared<Resource>(imgui_ini)},
  } ;

  FileResource::FileResource(std::filesystem::path path) : m_path{path} {
    load();
  }

  void FileResource::load() {
    std::error_code ec;
    m_modified = std::filesystem::last_write_time(m_path, ec);
    std::ifstream file(m_path, std::ios::binary | std::ios::ate);
    std::streamsize size = file ? std::streamsize(file.tellg()) : -1;
    if (size >= 0 && file.seekg(0, std::ios::beg)) {
      // resize, not reserve: the vector must own the bytes read into it, plus a null terminator for get_as_cstr
      m_data.resize(size + 1);
      if (file.read(m_data.data(), size)) {
        m_data[size] = 0;
        m_buffer = std::string_view(m_data.data(), size);
        return;
      }
    }
    m_data.clear();
    m_buffer = {};
  }

  bool FileResource::reload() {
    if (m_modified < std::filesystem::last_write_time(m_path)) {
      load();
      return true;
    }
    return false;
  }

  std::shared_ptr<Resource> ResourceManager::get(std::filesystem::path path) {
    if (!path.empty() && std::filesystem::exists(std::filesystem::current_path() / path)) {
      if (s_loaded_resource.count(path)) {
        s_loaded_resource[path]->reload();
        return s_loaded_resource[path];
      }
      auto file_buffer = std::make_shared<FileResource>(path);
      s_loaded_resource[path] = file_buffer;
      return file_buffer;
    }
    return s_embedded_resource.count(path) > 0 ? s_embedded_resource[path] : nullptr;
  }

  const char* ResourceManager::get_as_cstr(std::filesystem::path path) {
    auto value = get(path);
    return value != nullptr ? value->m_buffer.data() : nullptr;
  }

  std::string_view ResourceManager::get_as_sv(std::filesystem::path path) {
    auto value = get(path);
    return value != nullptr ? value->m_buffer : std::string_view {};
  }

}

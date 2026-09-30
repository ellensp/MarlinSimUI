#pragma once

//
// Thread-safe mailbox between the agent server and the Viewport camera.
// The agent posts a request; Visualisation::update() applies it on the UI thread
// and publishes the resulting view.
//

#include <mutex>
#include <string>
#include <vector>
#include <memory>
#include <future>
#include <cstdint>

namespace view_control {

struct Request {
  bool has_yaw = false, has_pitch = false, has_distance = false, has_target = false, has_follow = false, has_markings = false, has_volume = false;
  float yaw = 0, pitch = 0, distance = 0;
  float target[3] {};  // Marlin coordinates (mm)
  bool follow = false;
  bool markings = true; // Bed markings shown
  bool volume = false;  // Delta printable volume shown
  std::string preset;  // home, front, right, back, left, top, iso
  std::string machine; // bedslinger, cube, delta, scara
};

struct State {
  float yaw = 0, pitch = 0, distance = 0;
  float target[3] {};  // Marlin coordinates (mm)
  bool follow = false;
  bool turntable = true;
  bool markings = true;
  bool volume = false;
  std::string machine;
};

inline std::mutex mutex;
inline bool pending = false;
inline Request request;
inline State state;

inline void post(const Request& r) {
  std::scoped_lock lock(mutex);
  request = r;
  pending = true;
}

inline State current() {
  std::scoped_lock lock(mutex);
  return state;
}

//
// Viewport capture. The agent thread asks for a frame and waits on the future.
// Application::render() takes the request after the Viewport is drawn, reads the
// framebuffer on the UI thread (the only thread with the GL context), and fulfills it.
// A newer request replaces an older one, whose future then reports a broken promise.
//
struct Capture {
  std::vector<uint8_t> rgb;         // Top row first, 3 bytes per pixel
  uint32_t width = 0, height = 0;   // 0 if the framebuffer couldn't be read
};

inline std::shared_ptr<std::promise<Capture>> capture_request;

inline std::future<Capture> request_capture() {
  std::scoped_lock lock(mutex);
  capture_request = std::make_shared<std::promise<Capture>>();
  return capture_request->get_future();
}

inline std::shared_ptr<std::promise<Capture>> take_capture_request() {
  std::scoped_lock lock(mutex);
  auto r = capture_request;
  capture_request.reset();
  return r;
}

} // namespace view_control

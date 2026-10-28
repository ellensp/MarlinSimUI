#pragma once

/**
 * Webcam — serves the 3D Viewport as an HTTP camera, for hosts such as OctoPrint and Mainsail
 * that can show a printer webcam from a URL.
 *
 *   GET /snapshot  (or /?action=snapshot)   one JPEG
 *   GET /stream    (or /?action=stream)     MJPEG, multipart/x-mixed-replace
 *   GET /                                   a small page showing the stream
 *
 * Off by default. Turn it on with the Viewport's "Webcam" menu, or start the simulator with
 * --webcam-port <n> (or MARLIN_SIM_WEBCAM_PORT). Nothing is read back from the GPU or encoded
 * unless a client asked for a frame within the last two seconds.
 *
 * Unlike the agent interface this listens on every interface, because a host in a Docker
 * container reaches the simulator through the machine's network address. It only serves
 * images; it cannot send G-code or change any state.
 *
 * THREADING: the GL context lives on the UI thread, so only webcam::wants_frame() and
 * webcam::submit_frame() (called from Application::render()) touch the framebuffer. Sockets
 * and JPEG encoding run on their own threads.
 */

#include <cstdint>
#include <vector>

namespace webcam {

constexpr uint16_t default_port = 8101;   // for the default serial port 8099; see set_default_port()

// The port the Viewport menu uses when none was given. main() sets it to the serial port + 2, so
// two simulators with different --serial-port values get different webcam ports too.
void set_default_port(uint16_t port);

// Start listening. port 0 keeps the last port (or default_port). Returns false if the
// socket can't be opened. Safe to call when already enabled.
bool enable(uint16_t port = 0);
void disable();
void shutdown();            // disable and join all threads (at exit)

bool enabled();
uint16_t port();
uint32_t client_count();    // streams currently connected

// UI thread, once per rendered frame after the Viewport is drawn.
bool wants_frame();
void submit_frame(std::vector<uint8_t>&& rgb, uint32_t width, uint32_t height);

} // namespace webcam

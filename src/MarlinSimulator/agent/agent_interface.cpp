#include "agent_interface.h"
#include "json.h"
#include "serial_log.h"
#include "png_writer.h"

#include "../execution_control.h"
#include "../virtual_printer.h"
#include "../view_control.h"
#include "../machine_model.h"

#include "host_print.h"
#include "../thermal_fault_blink.h"
#include "../hardware/fat_image.h"
#include "../hardware/SDCard.h"
#include "../hardware/ST7796Device.h"

#include "serial.h"
#include <src/gcode/queue.h>
#include <src/MarlinCore.h>   // marlin.isRunning(): the serial host waits for a ready machine
#include <src/module/planner.h>
#if HAS_MEDIA
  #include <src/sd/cardreader.h>
#endif

extern MSerialT serial_stream_0, serial_stream_1, serial_stream_2, serial_stream_3;

#include <cstring>
#include <strings.h>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <future>


#if ANY(TFT_INTERFACE_SPI, HAS_MARLINUI_HD44780, HAS_MARLINUI_U8GLIB)
  #define HAS_SIM_DISPLAY 1
#endif

namespace agent {

AgentServer server;

namespace {

const char* timer_state(const KernelTimer& timer) {
  if (timer.running) return "running";
  return timer.active ? "enabled" : "disabled";
}

void write_kernel_state(JsonWriter& writer) {
  writer.begin_object();

  writer.member("ticks", Kernel::TimeControl::getTicks());
  writer.member("realtime_ticks", Kernel::TimeControl::getRealtimeTicks());
  writer.member("tick_frequency", uint64_t(Kernel::TimeControl::frequency));
  writer.member("realtime_scale", Kernel::TimeControl::realtime_scale.load());
  writer.member("sim_nanos", Kernel::SimulationRuntime::nanos());
  writer.member("sim_seconds", Kernel::SimulationRuntime::seconds());
  writer.member("isr_timing_error_ns", Kernel::isr_timing_error.load());
  writer.member("timers_active", Kernel::timers_active);
  writer.member("quit_requested", Kernel::quit_requested);

  writer.key("timers");
  writer.begin_array();
  for (auto& timer : Kernel::Timers::timers) {
    writer.begin_object();
    writer.member("name", timer.name);
    writer.member("state", timer_state(timer));
    writer.member("priority", timer.priority);
    writer.member("compare", timer.compare);
    writer.member("timer_frequency", timer.timer_frequency);
    writer.end_object();
  }
  writer.end_array();

  writer.end_object();
}

// GET /kernel — kernel clock, timers and scheduler health.
void handle_get_kernel(const Request&, Response& response) {
  JsonWriter writer;
  write_kernel_state(writer);
  response.json(writer.str());
}

// POST /kernel/control — time and execution control.
//
// Registered with Affinity::Direct: it touches only atomics, and MUST remain
// serviceable while the simulation thread is blocked — otherwise a client that
// sets realtime_scale to 0 can never resume it.
//
//   {"realtime_scale": 100.0}  fast-forward (0 freezes, 1.0 is wall clock)
//   {"break": true}            trigger Kernel::execution_break()
//   {"quit": true}             request clean shutdown
void handle_post_kernel_control(const Request& request, Response& response) {
  JsonValue body;
  if (!body.parse(request.body)) {
    response.error(400, "body must be a flat JSON object");
    return;
  }

  bool changed = false;

  double scale = 0.0;
  if (body.get_double("realtime_scale", scale)) {
    // 0 is legal (freeze). Negative would run time backwards.
    if (scale < 0.0 || scale > 1000.0) {
      response.error(400, "realtime_scale must be between 0 and 1000");
      return;
    }
    Kernel::TimeControl::realtime_scale = float(scale);
    changed = true;
  }

  bool flag = false;
  if (body.get_bool("break", flag) && flag) {
    Kernel::execution_break();
    changed = true;
  }

  if (body.get_bool("quit", flag) && flag) {
    Kernel::quit_requested = true;
    changed = true;
  }

  if (!changed) {
    response.error(400, "no recognised field (realtime_scale, break, quit)");
    return;
  }

  // Report only atomics — reading the timer array here would race the
  // simulation thread. Use GET /kernel for the full picture.
  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("realtime_scale", Kernel::TimeControl::realtime_scale.load());
  writer.member("ticks", Kernel::TimeControl::getTicks());
  writer.end_object();
  response.json(writer.str());
}

// GET /state — every registered component's state in one snapshot.
void handle_get_state(const Request&, Response& response) {
  JsonWriter writer;
  writer.begin_object();

  writer.key("kernel");
  write_kernel_state(writer);

  writer.key("components");
  VirtualPrinter::serialize_all(writer);

  // Heaters blinking for a thermal fault (until reset, or the blink times out)
  writer.key("thermal_faults");
  thermal_fault_blink::serialize(writer);

  writer.end_object();
  response.json(writer.str());
}

// GET /state/<component-name> — one component, by registry name.
// Names contain spaces and parentheses (e.g. "Endstop(X Min)"), so the path
// segment is percent-decoded before lookup.
void handle_get_state_component(const Request& request, Response& response) {
  constexpr const char* prefix = "/state/";
  const size_t prefix_length = strlen(prefix);

  if (request.path.size() <= prefix_length) {
    response.error(400, "missing component name");
    return;
  }

  std::string name = AgentServer::percent_decode(request.path.substr(prefix_length));

  JsonWriter writer;
  if (!VirtualPrinter::serialize_one(name, writer)) {
    response.error(404, "no component named '" + name + "' (see GET /components)");
    return;
  }
  response.json(writer.str());
}

// GET /components — the list of names usable with GET /state/<name>.
void handle_get_components(const Request&, Response& response) {
  JsonWriter writer;
  writer.begin_object();
  writer.key("components");
  writer.begin_array();
  for (auto const& name : VirtualPrinter::component_names()) writer.value(name);
  writer.end_array();
  writer.end_object();
  response.json(writer.str());
}

// GET /serial?since=<seq>&limit=<n>&stream=<0-3>
//
// Registered Affinity::Direct: the log has its own mutex, and it must stay
// readable when the simulation is frozen or wedged -- that is exactly when you
// want the last thing the firmware printed.
void handle_get_serial(const Request& request, Response& response) {
  uint64_t since = 0;
  size_t limit = 200;
  int stream = -1;

  std::string value;
  if (request.query_param("since", value))  since = strtoull(value.c_str(), nullptr, 10);
  if (request.query_param("limit", value))  limit = size_t(strtoull(value.c_str(), nullptr, 10));
  if (request.query_param("stream", value)) stream = atoi(value.c_str());

  if (limit > 5000) limit = 5000;  // bound the response size

  auto lines = serial_log.query(since, limit, stream);

  JsonWriter writer;
  writer.begin_object();
  writer.key("lines");
  writer.begin_array();
  for (auto const& line : lines) {
    writer.begin_object();
    writer.member("seq", double(line.sequence));
    writer.member("stream", double(line.stream));
    writer.member("sim_seconds", line.sim_seconds);
    writer.member("text", line.text);
    writer.end_object();
  }
  writer.end_array();

  // Resume point for the next poll. This MUST be one past the last line
  // actually RETURNED, not the global counter -- with limit-capped or
  // stream-filtered results those differ, and using the counter would silently
  // skip every line the page didn't include.
  uint64_t next = since;
  if (!lines.empty()) next = lines.back().sequence + 1;

  writer.member("next_seq", double(next));
  // Where the log itself is up to; next_seq < head_seq means more is pending.
  writer.member("head_seq", double(serial_log.next_sequence()));
  writer.member("returned", double(lines.size()));
  writer.member("retained", double(serial_log.size()));
  // Oldest sequence still held. If your ?since= is below this, the lines in
  // between were evicted and you have a gap.
  writer.member("oldest_seq", double(serial_log.oldest_sequence()));
  writer.member("max_lines", double(serial_log.max_lines()));
  // Non-zero means output was evicted before being read: the agent has a gap.
  writer.member("dropped", double(serial_log.dropped_lines()));
  writer.end_object();
  response.json(writer.str());
}

// POST /serial — log maintenance.
//   {"clear": true}        drop retained lines (sequence numbers keep rising)
//   {"max_lines": 50000}   resize retention
void handle_post_serial(const Request& request, Response& response) {
  JsonValue body;
  if (!body.parse(request.body)) {
    response.error(400, "body is not valid JSON");
    return;
  }

  bool flag = false;
  if (body.get_bool("clear", flag) && flag) serial_log.clear();

  double limit = 0;
  if (body.get_double("max_lines", limit)) {
    if (limit < 0 || limit > 1000000) {
      response.error(400, "max_lines must be 0..1000000");
      return;
    }
    serial_log.set_max_lines(size_t(limit));
  }

  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("retained", double(serial_log.size()));
  writer.member("max_lines", double(serial_log.max_lines()));
  writer.member("next_seq", double(serial_log.next_sequence()));
  writer.end_object();
  response.json(writer.str());
}

// GET /view  — current Viewport camera
// POST /view — set the Viewport camera (Turntable mode). All keys optional:
//   {"preset":"home|front|right|back|left|top|iso", "yaw":deg, "pitch":deg,
//    "distance":mm, "x":mm, "y":mm, "z":mm (target, Marlin coordinates; all three),
//    "follow":bool, "machine":"bedslinger|cube|delta", "markings":bool, "volume":bool}
// Applied on the next UI frame; poll GET /view to confirm.
void handle_get_view(const Request&, Response& response) {
  const view_control::State v = view_control::current();
  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("mode", v.turntable ? "turntable" : "fly");
  writer.member("machine", v.machine);
  writer.member("yaw", double(v.yaw));
  writer.member("pitch", double(v.pitch));
  writer.member("distance", double(v.distance));
  writer.member("x", double(v.target[0]));
  writer.member("y", double(v.target[1]));
  writer.member("z", double(v.target[2]));
  writer.member("follow", v.follow);
  writer.member("markings", v.markings);
  writer.member("volume", v.volume);
  writer.end_object();
  response.json(writer.str());
}

void handle_post_view(const Request& request, Response& response) {
  JsonValue body;
  if (!body.parse(request.body)) {
    response.error(400, "body is not valid JSON");
    return;
  }
  view_control::Request r;
  double d = 0;
  if (body.get_double("yaw", d))      { r.has_yaw = true; r.yaw = float(d); }
  if (body.get_double("pitch", d))    { r.has_pitch = true; r.pitch = float(d); }
  if (body.get_double("distance", d)) {
    if (d <= 0) { response.error(400, "distance must be > 0"); return; }
    r.has_distance = true; r.distance = float(d);
  }
  if (body.get_bool("follow", r.follow)) r.has_follow = true;
  if (body.get_bool("markings", r.markings)) r.has_markings = true;
  if (body.get_bool("volume", r.volume)) r.has_volume = true;
  body.get_string("preset", r.preset);
  body.get_string("machine", r.machine);
  if (!r.machine.empty()) {
    const MachineType type = machine_type_from_name(r.machine.c_str());
    if (type == MACHINE_TYPE_COUNT) { response.error(400, "machine must be bedslinger, cube, or delta"); return; }
    if (!MachineModel::is_available(type)) { response.error(400, "machine not available for this build's kinematics"); return; }
  }
  const int axes = body.has("x") + body.has("y") + body.has("z");
  if (axes) {
    double x = 0, y = 0, z = 0;
    if (axes != 3 || !body.get_double("x", x) || !body.get_double("y", y) || !body.get_double("z", z)) {
      response.error(400, "target needs numeric x, y and z");
      return;
    }
    r.target[0] = float(x); r.target[1] = float(y); r.target[2] = float(z);
    r.has_target = true;
  }
  view_control::post(r);
  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("hint", "applied on the next UI frame; GET /view to confirm");
  writer.end_object();
  response.json(writer.str());
}

#if HAS_SIM_DISPLAY

// POST /screenshot — write a display capture to a file.
//   {"path": "/tmp/lcd.png"}                  first display found
//   {"path": "...", "display": "<name>"}      a specific display
//
// Writing to a caller-supplied path is the point: an agent asks for its own
// tmp dir, then reads the PNG back with its normal file tools.
// POST /capture/lcd is an alias.
void handle_post_screenshot(const Request& request, Response& response) {
  JsonValue body;
  if (!body.parse(request.body)) {
    response.error(400, "body is not valid JSON");
    return;
  }

  std::string path;
  if (!body.get_string("path", path) || path.empty()) {
    response.error(400, "'path' is required, e.g. {\"path\": \"/tmp/lcd.png\"}");
    return;
  }

  std::string display;
  body.get_string("display", display);

  std::vector<uint8_t> rgb;
  uint32_t width = 0, height = 0;
  std::string matched;

  if (!VirtualPrinter::capture_display(display, rgb, width, height, matched)) {
    response.error(404, display.empty()
      ? "no display component found (see GET /displays)"
      : "no display named '" + display + "' (see GET /displays)");
    return;
  }

  std::string error;
  if (!write_png_rgb(path, width, height, rgb.data(), error)) {
    response.error(500, error);
    return;
  }

  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("path", path);
  writer.member("display", matched);
  writer.member("width", double(width));
  writer.member("height", double(height));
  writer.member("bytes", double(size_t(width) * size_t(height) * 3));
  writer.end_object();
  response.json(writer.str());
}

// GET /displays — registry names of components that can be captured.
void handle_get_displays(const Request&, Response& response) {
  JsonWriter writer;
  writer.begin_object();
  writer.key("displays");
  writer.begin_array();
  for (auto const& name : VirtualPrinter::display_names()) writer.value(name);
  writer.end_array();
  writer.end_object();
  response.json(writer.str());
}

#endif // HAS_SIM_DISPLAY

//
// POST /capture/<panel> — capture a panel that renders visuals, by short name.
// The name is case-insensitive (see canonicalize_request). Supported:
//   lcd              alias for POST /screenshot (same body)
//   viewport, vp     the 3D Viewport: {"path": "/tmp/vp.png"}
// Other panels are added case by case, only for panels that render visuals.
//

// Lower-case the /capture/<name> segment and resolve aliases, so each panel
// is one exact route with its own thread affinity.
void canonicalize_request(Request& request) {
  constexpr const char* prefix = "/capture/";
  const size_t n = strlen(prefix);
  if (request.path.size() <= n || strncasecmp(request.path.c_str(), prefix, n) != 0) return;
  std::string name = AgentServer::percent_decode(request.path.substr(n));
  for (auto& c : name) c = char(tolower((unsigned char)c));
  if (name == "vp") name = "viewport";
  request.path = prefix + name;
}

// Read the Viewport framebuffer. The UI thread owns the GL context, so this
// runs on the server thread and waits for the next rendered frame. It never
// touches the simulation, so it also works while the simulation is frozen.
void handle_post_capture_viewport(const Request& request, Response& response) {
  JsonValue body;
  if (!body.parse(request.body)) {
    response.error(400, "body is not valid JSON");
    return;
  }
  std::string path;
  if (!body.get_string("path", path) || path.empty()) {
    response.error(400, "'path' is required, e.g. {\"path\": \"/tmp/viewport.png\"}");
    return;
  }

  auto future = view_control::request_capture();
  if (future.wait_for(std::chrono::milliseconds(3000)) != std::future_status::ready) {
    response.error(503, "the UI did not render a frame in time");
    return;
  }
  view_control::Capture capture;
  try { capture = future.get(); }
  catch (const std::future_error&) {
    response.error(409, "superseded by a newer viewport capture request");
    return;
  }
  if (!capture.width || !capture.height) {
    response.error(503, "the Viewport has no image yet (is its panel open?)");
    return;
  }

  std::string error;
  if (!write_png_rgb(path, capture.width, capture.height, capture.rgb.data(), error)) {
    response.error(500, error);
    return;
  }

  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("path", path);
  writer.member("display", "Viewport");
  writer.member("width", double(capture.width));
  writer.member("height", double(capture.height));
  writer.member("bytes", double(capture.rgb.size()));
  writer.end_object();
  response.json(writer.str());
}

// Any other /capture/<name>: not a capturable panel (yet)
void handle_post_capture_other(const Request& request, Response& response) {
  const std::string name = request.path.substr(strlen("/capture/"));
  response.error(404, "no capture for '" + name + "'. Supported: "
    #if HAS_SIM_DISPLAY
      "lcd, "
    #endif
    "viewport (vp)");
}

// POST /gcode — submit G-code the way a host would.
//   {"command": "G28"}
//
// Writes into the serial RX buffer rather than GCodeQueue::inject(), which has
// a single 64-char slot that silently overwrites a pending command.
//
// This NEVER blocks. A handler cannot wait for motion to finish: it is serviced
// from inside the marlin_loop ISR, and execute_loop only fires timers with
// priority < the active one (execution_control.cpp), so Kernel::yield() cannot
// re-enter marlin_loop. The G-code queue would never be pumped and the wait
// would deadlock the simulation thread permanently -- taking every route with
// it, since the accept loop is single-threaded.
//
// To wait, poll GET /idle until {"idle": true}.
void handle_post_gcode(const Request& request, Response& response) {
  JsonValue body;
  if (!body.parse(request.body)) {
    response.error(400, "body is not valid JSON");
    return;
  }

  std::string command;
  if (!body.get_string("command", command) || command.empty()) {
    response.error(400, "'command' is required, e.g. {\"command\": \"G28\"}");
    return;
  }
  if (command.find('\n') != std::string::npos) {
    response.error(400, "'command' must be a single line");
    return;
  }

  // Sequence before submitting, so the caller can read exactly the output this
  // command produced via GET /serial?since=<serial_from>.
  const uint64_t serial_from = serial_log.next_sequence();

  command += '\n';
  serial_stream_3.receive_buffer.write((uint8_t*)command.data(), command.size());

  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("command", command.substr(0, command.size() - 1));
  writer.member("serial_from", double(serial_from));
  writer.member("hint", std::string("poll GET /idle until idle:true, then GET /serial?since=serial_from"));
  writer.end_object();
  response.json(writer.str());
}

// GET /idle — is the machine done with everything submitted so far?
//
// Cheap and non-blocking, so an agent can poll it between moves. rx_pending
// matters: immediately after POST /gcode the bytes are still in the serial
// receive buffer and both the queue and planner are empty, so a check that
// omitted it would report idle before the command had even been parsed.
void handle_get_idle(const Request&, Response& response) {
  const bool rx_pending   = serial_stream_3.receive_buffer.available() != 0;
  const bool queued       = queue.has_commands_queued();
  const bool blocks       = planner.has_blocks_queued();
  const bool planner_busy = planner.busy();

  JsonWriter writer;
  writer.begin_object();
  writer.member("idle", !(rx_pending || queued || blocks || planner_busy));
  writer.member("rx_pending", rx_pending);
  writer.member("commands_queued", queued);
  writer.member("blocks_queued", blocks);
  writer.member("planner_busy", planner_busy);
  writer.member("moves_queued", double(planner.movesplanned()));
  writer.end_object();
  response.json(writer.str());
}

#if ENABLED(TOUCH_SCREEN)

// POST /touch — press the touchscreen, as a user would with the mouse.
//   {"x": 0.5, "y": 0.5}                 tap at panel ratios 0..1
//   {"x": 0.5, "y": 0.5, "hold_ms": 1500} press-and-hold (simulated ms)
//
// Never blocks: the press is released by the device once hold_ms has elapsed
// AND Marlin has polled it enough times to act on it. Take a screenshot
// afterwards to see the result.
void handle_post_touch(const Request& request, Response& response) {
  JsonValue body;
  if (!body.parse(request.body)) {
    response.error(400, "body is not valid JSON");
    return;
  }

  double x = -1.0, y = -1.0, hold = 0.0;
  if (!body.get_double("x", x) || !body.get_double("y", y)) {
    response.error(400, "'x' and 'y' (ratios 0..1) are required");
    return;
  }
  if (x < 0.0 || x > 1.0 || y < 0.0 || y > 1.0) {
    response.error(400, "'x' and 'y' must be between 0 and 1");
    return;
  }
  body.get_double("hold_ms", hold);
  if (hold < 0.0 || hold > 60000.0) {
    response.error(400, "'hold_ms' must be between 0 and 60000");
    return;
  }

  std::string matched;
  if (!VirtualPrinter::inject_touch(float(x), float(y), uint32_t(hold), matched)) {
    response.error(404, "no touch device found");
    return;
  }

  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("device", matched);
  writer.member("x", x);
  writer.member("y", y);
  writer.member("hold_ms", hold);
  writer.end_object();
  response.json(writer.str());
}

#if ENABLED(TFT_INTERFACE_SPI)

// POST /touch/rects — show or hide Marlin's touch control rectangles on the LCD
//   {"show": true}   show       {"show": false}   hide       {}   toggle
//
// Marlin (simulator build) sends the rectangles as the last stage of drawing each
// screen. They are drawn over the panel image with a 2px white outline and a 10%
// white interior, in the window and in screenshots. Also a "Touch Rects" checkbox
// on the display, and {"touch_rects": bool} on POST /component/<display>.
void handle_post_touch_rects(const Request& request, Response& response) {
  JsonValue body;
  if (!request.body.empty() && !body.parse(request.body)) {
    response.error(400, "body is not valid JSON");
    return;
  }
  bool show = !ST7796Device::show_touch_rects;
  body.get_bool("show", show);
  ST7796Device::show_touch_rects = show;

  auto display = std::static_pointer_cast<ST7796Device>(VirtualPrinter::find_component("ST7796Device Display"));
  if (display) display->dirty = true;

  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("show", show);
  writer.member("count", double(display ? display->touch_rect_count() : 0));
  writer.end_object();
  response.json(writer.str());
}

#endif // TFT_INTERFACE_SPI

#endif // TOUCH_SCREEN

// GET /ping — liveness probe that proves the simulation thread is servicing
// requests (it can only be answered from execute_loop).
void handle_get_ping(const Request&, Response& response) {
  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("sim_seconds", Kernel::SimulationRuntime::seconds());
  writer.end_object();
  response.json(writer.str());
}


//
// Fault injection and sensor simulation by component name
//

// POST /component/<name> -- set a component's simulated state, by name, with the
// same keys GET /state/<name> reports. No pin or ADC numbers needed:
//   Filament Runout Sensor 1   {"filament_present": false}
//   Hotend0 Heater             {"sensor_mode": "ForceMax"} | {"sensor_temperature": 300}
//                              {"element_ok": false} | {"temperature": 60}
//   SD Card                    {"present": false} | {"image": "/abs/card.img"}
//   Endstop(X Min)             {"triggered": true} | {"auto": true} | {"enabled": false}
// Answers with the component's state after the change.
void handle_post_component(const Request& request, Response& response) {
  constexpr const char* prefix = "/component/";
  const size_t prefix_length = strlen(prefix);
  if (request.path.size() <= prefix_length) {
    response.error(400, "missing component name");
    return;
  }
  const std::string name = AgentServer::percent_decode(request.path.substr(prefix_length));
  auto component = VirtualPrinter::find_component(name);
  if (!component) {
    response.error(404, "no component named '" + name + "' (see GET /components)");
    return;
  }

  JsonValue body;
  if (!body.parse(request.body)) {
    response.error(400, "body must be a flat JSON object");
    return;
  }

  std::string error;
  if (!component->apply(body, error)) {
    response.error(400, error);
    return;
  }

  JsonWriter writer;
  VirtualPrinter::serialize_one(name, writer);
  response.json(writer.str());
}

//
// Printing: from SD (/print) and from a simulated serial host (/print with "via":"serial")
//

// The SD card component, if the config has one
std::shared_ptr<SDCard> sd_card() {
  auto sd = VirtualPrinter::find_component("SD Card");
  return sd ? std::static_pointer_cast<SDCard>(sd) : nullptr;
}

// Fresh scratch image path for each build. The SD slot may still be reading the
// previous agent image, so never overwrite one in place. See insert_prepared_card().
std::string new_scratch_image_path() {
  namespace fs = std::filesystem;
  static std::atomic_uint32_t counter { 0 };
  std::error_code ec;
  fs::path dir = fs::temp_directory_path(ec);
  if (ec) dir = ".";
  return (dir / ("marlinsim_agent_sd_" + std::to_string(counter++) + ".img")).u8string();
}

// A card image built by a preparer on the server thread, for the handler to insert
struct PreparedCard {
  std::vector<fat_image::Entry> entries;
  std::string image;
  ~PreparedCard() { if (!image.empty() && !inserted) { std::error_code ec; std::filesystem::remove(std::filesystem::u8path(image), ec); } }
  bool inserted = false;
};

// SERVER THREAD: pack a host file or folder into a new FAT image. Touches only the
// host filesystem, so the simulated machine keeps running while a big file is copied.
bool build_card(const std::string& path, PreparedCard& card_out, std::string& error) {
  namespace fs = std::filesystem;
  std::error_code ec;
  const auto p = fs::u8path(path);
  if (fs::is_directory(p, ec)) {
    card_out.entries = fat_image::entries_in_folder(path);
    if (card_out.entries.empty()) { error = "no files in '" + path + "'"; return false; }
  }
  else if (fs::is_regular_file(p, ec))
    card_out.entries = { { path, p.filename().u8string(), "" } };
  else {
    error = "'" + path + "' is not a file or folder";
    return false;
  }
  card_out.image = new_scratch_image_path();
  if (!fat_image::build(card_out.image, card_out.entries, error)) return false;
  return true;
}

// SIMULATION THREAD: put a prepared image in the SD slot. Marlin still has to mount it (M21).
bool insert_prepared_card(PreparedCard& prepared, std::string& error) {
  static std::string last_image;  // The previous agent image, deleted once replaced
  auto sd = sd_card();
  if (!sd) { error = "no SD card in this configuration (enable SDSUPPORT)"; return false; }
  if (!sd->use_image(prepared.image)) { error = "the built image failed the FAT audit"; return false; }
  prepared.inserted = true;
  if (!last_image.empty() && last_image != prepared.image) {
    std::error_code ec;
    std::filesystem::remove(std::filesystem::u8path(last_image), ec);
  }
  last_image = prepared.image;
  return true;
}

// SERVER THREAD preparer for POST /sd/mount and POST /print (via sd, with a path):
// build the image now. Errors in the body are left for the handler to report.
bool prepare_card(Request& request, Response& response) {
  JsonValue body;
  if (!body.parse(request.body)) return true;
  std::string via = "sd", path, file;
  body.get_string("via", via);
  body.get_string("path", path);
  body.get_string("file", file);
  if (via != "sd" || path.empty() || !file.empty()) return true;
  #if HAS_MEDIA
    auto prepared = std::make_shared<PreparedCard>();
    std::string error;
    if (!build_card(path, *prepared, error)) { response.error(400, error); return false; }
    request.prepared = prepared;
  #else
    UNUSED(response);
  #endif
  return true;
}

// SERVER THREAD preparer for POST /print via serial: read and clean the file now
bool prepare_host_file(Request& request, Response& response) {
  JsonValue body;
  if (!body.parse(request.body)) return true;
  std::string via, path;
  body.get_string("via", via);
  body.get_string("path", path);
  if (via != "serial" || path.empty()) return true;
  auto lines = std::make_shared<std::vector<std::string>>();
  std::string error;
  if (!HostPrint::load(path, *lines, error)) { response.error(400, error); return false; }
  request.prepared = lines;
  return true;
}

// Queue a line of G-code as if typed on the agent's serial port
void enqueue_gcode(const std::string& line) {
  const std::string command = line + "\n";
  serial_stream_3.receive_buffer.write((uint8_t*)command.data(), command.size());
}

// POST /sd/mount -- put host files on the simulated SD card, no image prep needed.
//   {"path": "/abs/file.gcode"}   one file
//   {"path": "/abs/folder"}       every file in the folder (flat)
//   {"path": ..., "mount": false} build + insert only; don't send M21
// Builds a FAT16 image in the temp dir (on the server thread, so the machine
// keeps running), swaps it into the SD slot, and sends
// M21 so Marlin mounts it. Answers with the 8.3 and long name of each file.
void handle_post_sd_mount(const Request& request, Response& response) {
  JsonValue body;
  if (!body.parse(request.body)) { response.error(400, "body is not valid JSON"); return; }
  std::string path;
  if (!body.get_string("path", path) || path.empty()) {
    response.error(400, "'path' (a G-code file or a folder) is required");
    return;
  }
  bool mount = true;
  body.get_bool("mount", mount);

  #if HAS_MEDIA
    if (card.isFileOpen()) { response.error(409, "an SD print is in progress; stop it first (M524)"); return; }
  #endif

  auto prepared = std::static_pointer_cast<PreparedCard>(request.prepared);
  std::string error;
  if (!prepared) { response.error(400, "no SD card in this configuration (enable SDSUPPORT)"); return; }
  if (!insert_prepared_card(*prepared, error)) { response.error(400, error); return; }

  const uint64_t serial_from = serial_log.next_sequence();
  if (mount) enqueue_gcode("M21");

  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("image", prepared->image);
  writer.key("files");
  writer.begin_array();
  for (auto const& e : prepared->entries) {
    writer.begin_object();
    writer.member("name", e.long_name);
    writer.member("dos_name", e.short_name);
    writer.end_object();
  }
  writer.end_array();
  writer.member("serial_from", double(serial_from));
  writer.end_object();
  response.json(writer.str());
}

// POST /print -- start a print, from the SD card or from a simulated serial host.
//
// SD (the default):
//   {"path": "/abs/file.gcode"}   put the file on a fresh card image, mount, M23 + M24
//   {"file": "BENCHY.GCO"}        print a file already on the mounted card (8.3 name)
// Serial host:
//   {"path": "/abs/file.gcode", "via": "serial"[, "port": 0]}
//       Stream it like a host would: M110 N0, then N<n> lines with checksums, one
//       per "ok", honoring Resend and //action:pause|resume|cancel. Port defaults
//       to 0 (SERIAL_PORT). Progress: GET /print.
void handle_post_print(const Request& request, Response& response) {
  JsonValue body;
  if (!body.parse(request.body)) { response.error(400, "body is not valid JSON"); return; }

  std::string via = "sd", path, file;
  body.get_string("via", via);
  body.get_string("path", path);
  body.get_string("file", file);
  const uint64_t serial_from = serial_log.next_sequence();

  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("via", via);

  if (via == "serial") {
    if (path.empty()) { response.error(400, "'path' (a G-code file on this computer) is required"); return; }
    double port = 0;
    body.get_double("port", port);
    auto lines = std::static_pointer_cast<std::vector<std::string>>(request.prepared);
    if (!lines) { response.error(400, "'" + path + "' could not be read"); return; }
    std::string error;
    if (!host_print.start(path, std::move(*lines), int(port), error)) { response.error(409, error); return; }
    writer.member("file", path);
    writer.member("port", int(port));
  }
  else if (via == "sd") {
    #if HAS_MEDIA
      if (card.isFileOpen()) { response.error(409, "an SD print is in progress; stop it first (M524)"); return; }
      if (file.empty()) {
        if (path.empty()) { response.error(400, "'path' (a host G-code file) or 'file' (a name on the card) is required"); return; }
        auto prepared = std::static_pointer_cast<PreparedCard>(request.prepared);
        if (!prepared) { response.error(400, "'" + path + "' could not be put on a card"); return; }
        if (prepared->entries.size() != 1) { response.error(400, "'path' must be a single file to print"); return; }
        std::string error;
        if (!insert_prepared_card(*prepared, error)) { response.error(400, error); return; }
        file = prepared->entries[0].short_name;
        writer.member("image", prepared->image);
        enqueue_gcode("M21");
      }
      // Marlin queues these in order: M21 mounts before M23 selects
      enqueue_gcode("M23 " + file);
      enqueue_gcode("M24");
      writer.member("file", file);
    #else
      response.error(400, "no SD support in this configuration; use \"via\":\"serial\"");
      return;
    #endif
  }
  else {
    response.error(400, "'via' must be \"sd\" or \"serial\"");
    return;
  }

  writer.member("serial_from", double(serial_from));
  writer.member("hint", std::string("poll GET /print for progress; GET /serial?since=serial_from for Marlin's replies"));
  writer.end_object();
  response.json(writer.str());
}

// GET /print -- progress of the SD print (from Marlin) and the serial host print.
void handle_get_print(const Request&, Response& response) {
  JsonWriter writer;
  writer.begin_object();

  writer.key("sd");
  writer.begin_object();
  #if HAS_MEDIA
    writer.member("mounted", card.isMounted());
    writer.member("file_open", card.isFileOpen());
    writer.member("printing", card.isPrinting());
    writer.member("paused", card.isPaused());
    writer.member("filename", std::string(card.isFileOpen() ? card.longest_filename() : ""));
    writer.member("percent", card.isFileOpen() ? double(card.percentDone()) : 0.0);
  #else
    writer.member("supported", false);
  #endif
  writer.end_object();

  writer.key("serial");
  host_print.serialize(writer);

  writer.end_object();
  response.json(writer.str());
}

// POST /print/control -- {"action": "pause" | "resume" | "abort"}
// Acts on the serial host print if one is running, otherwise on the SD print
// (M25 / M24 / M524).
void handle_post_print_control(const Request& request, Response& response) {
  JsonValue body;
  if (!body.parse(request.body)) { response.error(400, "body is not valid JSON"); return; }
  std::string action;
  if (!body.get_string("action", action)) { response.error(400, "'action' (pause, resume, abort) is required"); return; }

  std::string target;
  if (host_print.active()) {
    target = "serial";
    if (action == "pause") host_print.pause();
    else if (action == "resume") host_print.resume();
    else if (action == "abort") host_print.abort();
    else { response.error(400, "'action' must be pause, resume or abort"); return; }
  }
  else {
    #if HAS_MEDIA
      target = "sd";
      if (action == "pause") enqueue_gcode("M25");
      else if (action == "resume") enqueue_gcode("M24");
      else if (action == "abort") enqueue_gcode("M524");
      else { response.error(400, "'action' must be pause, resume or abort"); return; }
    #else
      response.error(409, "nothing is printing");
      return;
    #endif
  }

  JsonWriter writer;
  writer.begin_object();
  writer.member("ok", true);
  writer.member("target", target);
  writer.member("action", action);
  writer.end_object();
  response.json(writer.str());
}

} // namespace

void register_routes() {
  server.route("GET /ping", handle_get_ping);
  server.route("GET /kernel", handle_get_kernel);
  server.route("GET /state", handle_get_state);
  server.route("GET /components", handle_get_components);
  server.route("POST /kernel/control", handle_post_kernel_control, Affinity::Direct);

  // Serial log: own mutex, and must stay readable while the sim is frozen.
  server.route("GET /serial", handle_get_serial, Affinity::Direct);
  server.route("POST /serial", handle_post_serial, Affinity::Direct);

  // Simulation-thread routes: these touch Marlin/component state.
  // POST /gcode never blocks, so it needs no special budget.
  server.route("POST /gcode", handle_post_gcode);
  server.route("GET /idle", handle_get_idle);
  // Viewport camera: mailbox to the UI thread, so it works while the sim is frozen
  server.route("GET /view", handle_get_view, Affinity::Direct);
  server.route("POST /view", handle_post_view, Affinity::Direct);

  #if HAS_SIM_DISPLAY
    server.route("POST /screenshot", handle_post_screenshot);
    server.route("POST /capture/lcd", handle_post_screenshot);
    server.route("GET /displays", handle_get_displays);
    #if ENABLED(TOUCH_SCREEN)
      server.route("POST /touch", handle_post_touch);
      #if ENABLED(TFT_INTERFACE_SPI)
        server.route("POST /touch/rects", handle_post_touch_rects);
      #endif
    #endif
  #endif

  // Panel captures by short name. The Viewport is read on the UI thread, so its
  // handler waits on the server thread and doesn't need the simulation running.
  server.set_rewriter(canonicalize_request);
  server.route("POST /capture/viewport", handle_post_capture_viewport, Affinity::Direct);
  server.route_prefix("POST /capture/", handle_post_capture_other, Affinity::Direct);

  // Prefix route: any GET /state/<name>.
  server.route_prefix("GET /state/", handle_get_state_component);

  // Fault injection / sensor simulation by component name
  server.route_prefix("POST /component/", handle_post_component);

  // Printing from SD and from the built-in serial host
  // File I/O and image building run in preparers on the server thread, so the
  // simulated machine doesn't stall while a large G-code file is copied
  server.route("POST /sd/mount", handle_post_sd_mount);
  server.prepare("POST /sd/mount", prepare_card);
  server.route("POST /print", handle_post_print);
  server.prepare("POST /print", [](Request& request, Response& response) {
    return prepare_card(request, response) && prepare_host_file(request, response);
  });
  server.route("GET /print", handle_get_print);
  server.route("POST /print/control", handle_post_print_control);

  // The serial host writes into Marlin's receive buffer for its port
  host_print.set_sender([](int port, const std::string& line) {
    MSerialT* streams[] = { &serial_stream_0, &serial_stream_1, &serial_stream_2, &serial_stream_3 };
    streams[port]->receive_buffer.write((uint8_t*)line.data(), line.size());
  });
  host_print.set_ready([] { return marlin.isRunning() && !queue.has_commands_queued(); });
}

} // namespace agent

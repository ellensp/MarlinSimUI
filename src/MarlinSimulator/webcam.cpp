#include "webcam.h"

// BSD sockets, as in agent_server.cpp. On Windows SDLNet_Init() has already called WSAStartup.
#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  using socket_t = SOCKET;
  #define CLOSE_SOCKET closesocket
  #define BAD_SOCKET INVALID_SOCKET
  #define SEND_FLAGS 0
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <unistd.h>
  using socket_t = int;
  #define CLOSE_SOCKET ::close
  #define BAD_SOCKET (-1)
  #ifdef MSG_NOSIGNAL
    #define SEND_FLAGS MSG_NOSIGNAL
  #else
    #define SEND_FLAGS 0   // macOS: SO_NOSIGPIPE is set on each socket instead
  #endif
#endif

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_WRITE_NO_STDIO
#include <stb_image_write.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace webcam {

namespace {

using Clock = std::chrono::steady_clock;

constexpr int      jpeg_quality    = 80;
constexpr uint32_t max_width       = 1280;   // wider captures are decimated before encoding
constexpr int      frame_ms        = 200;    // at most 5 frames per second
constexpr int      demand_ms       = 2000;   // keep rendering this long after the last request
constexpr int      snapshot_wait_ms = 2000;
constexpr int      request_timeout_ms = 3000;

int64_t now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count();
}

struct Frame {
  std::vector<uint8_t> rgb;
  uint32_t width = 0, height = 0;
};

std::atomic_bool running { false };
std::atomic<uint16_t> listen_port { 0 };
std::atomic<uint16_t> fallback_port { default_port };
std::atomic<uint32_t> clients { 0 };
std::atomic<int64_t> last_demand { -1000000 };
std::atomic<int64_t> last_capture { -1000000 };
std::atomic_bool encoder_busy { false };

std::mutex state_mutex;                 // guards the lifecycle below
socket_t listener = BAD_SOCKET;
std::thread accept_thread;
std::thread encoder_thread;

std::mutex frame_mutex;                 // guards pending, latest, sequence
std::condition_variable frame_cv;
std::unique_ptr<Frame> pending;
std::shared_ptr<const std::string> latest;
uint64_t sequence = 0;

void demand() { last_demand = now_ms(); }

// Keep every `step`th pixel in each direction so a very wide Viewport stays cheap to encode.
void decimate(Frame& f) {
  if (f.width <= max_width) return;
  const uint32_t step = (f.width + max_width - 1) / max_width;
  const uint32_t w = f.width / step, h = f.height / step;
  std::vector<uint8_t> out(size_t(w) * h * 3);
  for (uint32_t y = 0; y < h; ++y)
    for (uint32_t x = 0; x < w; ++x)
      memcpy(&out[(size_t(y) * w + x) * 3], &f.rgb[(size_t(y) * step * f.width + size_t(x) * step) * 3], 3);
  f.rgb = std::move(out);
  f.width = w; f.height = h;
}

void encode_loop() {
  while (true) {
    std::unique_ptr<Frame> frame;
    {
      std::unique_lock lock(frame_mutex);
      frame_cv.wait(lock, [] { return pending || !running.load(); });
      if (!running.load()) return;
      frame = std::move(pending);
    }
    decimate(*frame);
    auto jpeg = std::make_shared<std::string>();
    stbi_write_jpg_to_func([](void* ctx, void* data, int size) {
      static_cast<std::string*>(ctx)->append(static_cast<const char*>(data), size_t(size));
    }, jpeg.get(), int(frame->width), int(frame->height), 3, frame->rgb.data(), jpeg_quality);
    {
      std::scoped_lock lock(frame_mutex);
      latest = std::move(jpeg);
      ++sequence;
    }
    encoder_busy = false;
    frame_cv.notify_all();
  }
}

bool send_all(socket_t s, const char* data, size_t size) {
  while (size) {
    const auto n = ::send(s, data, size, SEND_FLAGS);
    if (n <= 0) return false;
    data += n; size -= size_t(n);
  }
  return true;
}
bool send_all(socket_t s, const std::string& text) { return send_all(s, text.data(), text.size()); }

// Read the request head (up to the blank line). Returns the request-target of a GET, or "".
std::string read_request_target(socket_t s) {
  std::string head;
  const int64_t deadline = now_ms() + request_timeout_ms;
  char buf[1024];
  while (head.find("\r\n\r\n") == std::string::npos && head.size() < 8192) {
    if (!running.load() || now_ms() > deadline) return "";
    fd_set set; FD_ZERO(&set); FD_SET(s, &set);
    timeval tv { 0, 100000 };
    if (::select(int(s) + 1, &set, nullptr, nullptr, &tv) <= 0) continue;
    const auto n = ::recv(s, buf, sizeof(buf), 0);
    if (n <= 0) return "";
    head.append(buf, size_t(n));
  }
  if (head.compare(0, 4, "GET ") != 0) return "";
  const size_t end = head.find(' ', 4);
  return end == std::string::npos ? "" : head.substr(4, end - 4);
}

std::string http(const char* status, const char* type, const std::string& body, const char* extra = "") {
  return std::string("HTTP/1.1 ") + status + "\r\nContent-Type: " + type +
         "\r\nContent-Length: " + std::to_string(body.size()) +
         "\r\nCache-Control: no-store\r\nAccess-Control-Allow-Origin: *\r\n" + extra +
         "Connection: close\r\n\r\n" + body;
}

enum class Kind { page, snapshot, stream, none };

// mjpg-streamer's /?action=snapshot|stream is accepted too, since hosts default to it.
Kind classify(const std::string& target) {
  const size_t q = target.find('?');
  const std::string path = target.substr(0, q);
  const std::string query = q == std::string::npos ? "" : target.substr(q + 1);
  if (query.find("action=snapshot") != std::string::npos) return Kind::snapshot;
  if (query.find("action=stream") != std::string::npos) return Kind::stream;
  if (path == "/snapshot" || path == "/snapshot.jpg") return Kind::snapshot;
  if (path == "/stream" || path == "/stream.mjpg") return Kind::stream;
  if (path == "/" || path == "/index.html") return Kind::page;
  return Kind::none;
}

// Wait for a frame newer than `after`. Returns null on timeout or shutdown.
std::shared_ptr<const std::string> next_frame(uint64_t& after, int timeout_ms) {
  const int64_t deadline = now_ms() + timeout_ms;
  std::unique_lock lock(frame_mutex);
  while (running.load()) {
    demand();
    if (sequence > after && latest) { after = sequence; return latest; }
    const int64_t left = deadline - now_ms();
    if (left <= 0) return nullptr;
    frame_cv.wait_for(lock, std::chrono::milliseconds(std::min<int64_t>(left, 100)));
  }
  return nullptr;
}

void serve_client(socket_t s) {
  ++clients;
  const Kind kind = classify(read_request_target(s));
  if (kind == Kind::page) {
    send_all(s, http("200 OK", "text/html",
      "<!doctype html><title>Marlin Simulator</title><body style=\"margin:0;background:#222\">"
      "<img src=\"/stream\" style=\"max-width:100%\"></body>"));
  }
  else if (kind == Kind::snapshot) {
    uint64_t seen;
    { std::scoped_lock lock(frame_mutex); seen = sequence; }   // wait for a frame rendered after the request
    if (auto jpeg = next_frame(seen, snapshot_wait_ms))
      send_all(s, http("200 OK", "image/jpeg", *jpeg));
    else
      send_all(s, http("503 Service Unavailable", "text/plain", "No frame yet. Is the Viewport visible?\n"));
  }
  else if (kind == Kind::stream) {
    send_all(s, "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n"
                "Cache-Control: no-store\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n");
    uint64_t seen = 0;
    while (running.load()) {
      auto jpeg = next_frame(seen, 1000);
      if (!jpeg) continue;      // nothing new (Viewport hidden); keep the connection and keep asking
      if (!send_all(s, "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + std::to_string(jpeg->size()) + "\r\n\r\n") ||
          !send_all(s, *jpeg) || !send_all(s, "\r\n")) break;
    }
  }
  else {
    send_all(s, http("404 Not Found", "text/plain", "Try /snapshot or /stream\n"));
  }
  CLOSE_SOCKET(s);
  --clients;
}

void accept_loop(socket_t l) {
  while (running.load()) {
    fd_set set; FD_ZERO(&set); FD_SET(l, &set);
    timeval tv { 0, 100000 };
    if (::select(int(l) + 1, &set, nullptr, nullptr, &tv) <= 0) continue;
    socket_t s = ::accept(l, nullptr, nullptr);
    if (s == BAD_SOCKET) continue;
    #ifdef SO_NOSIGPIPE
      int one = 1;
      setsockopt(s, SOL_SOCKET, SO_NOSIGPIPE, (const char*)&one, sizeof(one));
    #endif
    std::thread(serve_client, s).detach();
  }
}

} // namespace

bool enable(uint16_t port) {
  std::scoped_lock lock(state_mutex);
  if (running.load()) return true;
  if (port) listen_port = port;
  else if (!listen_port) listen_port = fallback_port.load();

  socket_t l = ::socket(AF_INET, SOCK_STREAM, 0);
  if (l == BAD_SOCKET) { fprintf(stderr, "Webcam: socket() failed\n"); return false; }
  int reuse = 1;
  setsockopt(l, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));

  sockaddr_in address {};
  address.sin_family = AF_INET;
  address.sin_port = htons(listen_port.load());
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  if (::bind(l, (sockaddr*)&address, sizeof(address)) != 0 || ::listen(l, 16) != 0) {
    fprintf(stderr, "Webcam: can't listen on port %u (in use?)\n", unsigned(listen_port.load()));
    CLOSE_SOCKET(l);
    return false;
  }

  listener = l;
  running = true;
  encoder_busy = false;
  accept_thread = std::thread(accept_loop, l);
  encoder_thread = std::thread(encode_loop);
  printf("Webcam: http://0.0.0.0:%u/stream and /snapshot\n", unsigned(listen_port.load()));
  fflush(stdout);
  return true;
}

void disable() {
  std::scoped_lock lock(state_mutex);
  if (!running.load()) return;
  running = false;
  frame_cv.notify_all();
  if (accept_thread.joinable()) accept_thread.join();
  if (encoder_thread.joinable()) encoder_thread.join();
  CLOSE_SOCKET(listener);
  listener = BAD_SOCKET;
  // Client threads notice `running` within 100 ms; give them a moment to finish.
  for (int i = 0; i < 30 && clients.load(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::scoped_lock frames(frame_mutex);
  pending.reset(); latest.reset();
}

void shutdown() { disable(); }

bool enabled() { return running.load(); }
uint16_t port() { return listen_port.load() ? listen_port.load() : fallback_port.load(); }
void set_default_port(uint16_t p) { if (p) fallback_port = p; }
uint32_t client_count() { return clients.load(); }

bool wants_frame() {
  if (!running.load() || encoder_busy.load()) return false;
  const int64_t now = now_ms();
  return now - last_demand.load() < demand_ms && now - last_capture.load() >= frame_ms;
}

void submit_frame(std::vector<uint8_t>&& rgb, uint32_t width, uint32_t height) {
  if (!running.load() || !width || !height) return;
  last_capture = now_ms();
  auto frame = std::make_unique<Frame>();
  frame->rgb = std::move(rgb);
  frame->width = width; frame->height = height;
  encoder_busy = true;
  {
    std::scoped_lock lock(frame_mutex);
    pending = std::move(frame);
  }
  frame_cv.notify_all();
}

} // namespace webcam

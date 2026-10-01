#include "agent_server.h"
#include "json.h"
#include "request_log.h"

#include "../execution_control.h"

// BSD sockets rather than SDL_net: SDLNet_TCP_Open only creates a listening
// socket when the address is INADDR_ANY, so it cannot bind loopback-only. This
// endpoint can execute G-code, so binding 127.0.0.1 is a hard requirement.
// On Windows SDLNet_Init() has already called WSAStartup for us.
#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  using socket_t = SOCKET;
  #define CLOSE_SOCKET closesocket
  #define BAD_SOCKET INVALID_SOCKET
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  #include <fcntl.h>
  using socket_t = int;
  #define CLOSE_SOCKET ::close
  #define BAD_SOCKET (-1)
#endif

#include <cstdio>
#include <cstring>
#include <cctype>
#include <cstdlib>
#include <chrono>

namespace agent {

namespace {

constexpr uint32_t max_request_size = 65536;
constexpr uint32_t accept_timeout_us = 50000;   // 50ms select timeout
constexpr uint32_t request_timeout_ms = 5000;

// Stored as intptr_t in the header's void* slots.
inline socket_t to_socket(void* handle) { return socket_t(intptr_t(handle)); }
inline void* from_socket(socket_t socket) { return (void*)intptr_t(socket); }

const char* status_text(int code) {
  switch (code) {
    case 200: return "OK";
    case 204: return "No Content";
    case 400: return "Bad Request";
    case 404: return "Not Found";
    case 405: return "Method Not Allowed";
    case 413: return "Payload Too Large";
    case 500: return "Internal Server Error";
    case 503: return "Service Unavailable";
    default:  return "Unknown";
  }
}

std::string build_http_response(const Response& response) {
  std::string out = "HTTP/1.1 " + std::to_string(response.status) + " " + status_text(response.status) + "\r\n";
  out += "Content-Type: " + response.content_type + "\r\n";
  out += "Content-Length: " + std::to_string(response.body.size()) + "\r\n";
  out += "Cache-Control: no-store\r\n";
  out += "Connection: close\r\n\r\n";
  out += response.body;
  return out;
}

// Wait until `socket` is readable, or the timeout expires. Returns true if readable.
bool wait_readable(socket_t socket, uint32_t timeout_us) {
  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(socket, &read_set);

  timeval timeout {};
  timeout.tv_sec  = time_t(timeout_us / 1000000);
  timeout.tv_usec = suseconds_t(timeout_us % 1000000);

  int ready = select(int(socket) + 1, &read_set, nullptr, nullptr, &timeout);
  return ready > 0 && FD_ISSET(socket, &read_set);
}

void send_all(socket_t socket, const std::string& payload) {
  size_t sent = 0;
  while (sent < payload.size()) {
    #ifdef _WIN32
      int count = ::send(socket, payload.data() + sent, int(payload.size() - sent), 0);
    #else
      ssize_t count = ::send(socket, payload.data() + sent, payload.size() - sent, 0);
    #endif
    if (count <= 0) return;  // client went away; nothing useful to do
    sent += size_t(count);
  }
}

} // namespace

void Response::error(int code, const std::string& message) {
  status = code;
  JsonWriter writer;
  writer.begin_object();
  writer.member("error", message);
  writer.member("status", int64_t(code));
  writer.end_object();
  body = writer.str();
}

AgentServer::~AgentServer() {
  if (thread_active.load()) stop();
}

bool Request::query_param(const std::string& name, std::string& out) const {
  size_t pos = 0;
  while (pos < query.size()) {
    size_t amp = query.find('&', pos);
    if (amp == std::string::npos) amp = query.size();

    size_t eq = query.find('=', pos);
    if (eq != std::string::npos && eq < amp && query.compare(pos, eq - pos, name) == 0) {
      out = AgentServer::percent_decode(query.substr(eq + 1, amp - eq - 1));
      return true;
    }
    pos = amp + 1;
  }
  return false;
}

void AgentServer::route(const std::string& key, Handler handler, Affinity affinity, uint32_t timeout_ms) {
  handlers[key] = Route{ std::move(handler), affinity, nullptr, timeout_ms };
}

void AgentServer::route_prefix(const std::string& key, Handler handler, Affinity affinity) {
  prefix_handlers[key] = Route{ std::move(handler), affinity };
}

void AgentServer::prepare(const std::string& key, Preparer preparer) {
  auto it = handlers.find(key);
  if (it != handlers.end()) it->second.preparer = std::move(preparer);
}

std::string AgentServer::percent_decode(const std::string& text) {
  std::string out;
  out.reserve(text.size());

  for (size_t i = 0; i < text.size(); ++i) {
    if (text[i] == '%' && i + 2 < text.size() &&
        isxdigit((unsigned char)text[i + 1]) && isxdigit((unsigned char)text[i + 2])) {
      out += char(strtoul(text.substr(i + 1, 2).c_str(), nullptr, 16));
      i += 2;
    }
    else out += text[i];
  }
  return out;
}

bool AgentServer::start(uint16_t port) {
  socket_t listener = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listener == BAD_SOCKET) {
    fprintf(stderr, "AgentServer: socket() failed\n");
    return false;
  }

  int reuse = 1;
  setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));

  sockaddr_in address {};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);  // 127.0.0.1 only, by design

  if (::bind(listener, (sockaddr*)&address, sizeof(address)) != 0) {
    fprintf(stderr, "AgentServer: bind(127.0.0.1:%u) failed\n", port);
    CLOSE_SOCKET(listener);
    return false;
  }

  // Generous backlog: the accept loop is single-threaded and each request is
  // handled to completion before the next accept, so bursts of concurrent
  // clients must be absorbed by the kernel queue or connections get refused.
  if (::listen(listener, 64) != 0) {
    fprintf(stderr, "AgentServer: listen() failed\n");
    CLOSE_SOCKET(listener);
    return false;
  }

  listen_socket = from_socket(listener);
  listen_port = port;
  thread_active = true;
  server_thread = std::thread(&AgentServer::execute, this);

  printf("AgentServer: listening on http://127.0.0.1:%u\n", port);
  fflush(stdout);
  return true;
}

void AgentServer::stop() {
  if (!thread_active.load()) return;

  thread_active = false;
  if (server_thread.joinable()) server_thread.join();

  // Fail any request still queued so no client hangs on an unfulfilled promise.
  {
    std::scoped_lock lock(queue_mutex);
    while (!pending.empty()) {
      auto item = pending.front();
      pending.pop_front();
      Response response;
      response.error(503, "simulator shutting down");
      item->response.set_value(response);
    }
  }

  if (listen_socket) CLOSE_SOCKET(to_socket(listen_socket));
  listen_socket = nullptr;
}

/**
 * Read one complete HTTP request: headers until CRLFCRLF, then exactly
 * Content-Length body bytes. Returns false on malformed input, oversize
 * payload, timeout or disconnect.
 */
bool AgentServer::read_request(void* socket_handle, Request& request) {
  socket_t socket = to_socket(socket_handle);

  std::string raw;
  size_t header_end = std::string::npos;
  size_t content_length = 0;
  bool headers_done = false;
  char chunk[4096];

  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(request_timeout_ms);
  bool complete = false;

  while (thread_active.load() && std::chrono::steady_clock::now() < deadline) {
    if (!wait_readable(socket, accept_timeout_us)) continue;

    #ifdef _WIN32
      int length = ::recv(socket, chunk, int(sizeof(chunk)), 0);
    #else
      ssize_t length = ::recv(socket, chunk, sizeof(chunk), 0);
    #endif
    if (length <= 0) break;  // disconnect or error

    if (raw.size() + size_t(length) > max_request_size) break;
    raw.append(chunk, size_t(length));

    if (!headers_done) {
      header_end = raw.find("\r\n\r\n");
      if (header_end == std::string::npos) continue;
      headers_done = true;

      // Case-insensitive Content-Length lookup, header block only.
      std::string headers = raw.substr(0, header_end);
      std::string lower = headers;
      for (auto& c : lower) c = char(tolower((unsigned char)c));
      size_t at = lower.find("content-length:");
      if (at != std::string::npos) {
        content_length = size_t(strtoul(headers.c_str() + at + 15, nullptr, 10));
        if (content_length > max_request_size) break;
      }
    }

    if (headers_done && raw.size() >= header_end + 4 + content_length) {
      complete = true;
      break;
    }
  }

  if (!complete) return false;

  // Request line: METHOD SP TARGET SP HTTP/1.1
  size_t line_end = raw.find("\r\n");
  if (line_end == std::string::npos) return false;
  std::string line = raw.substr(0, line_end);

  size_t sp1 = line.find(' ');
  if (sp1 == std::string::npos) return false;
  size_t sp2 = line.find(' ', sp1 + 1);
  if (sp2 == std::string::npos) return false;

  std::string method = line.substr(0, sp1);
  std::string target = line.substr(sp1 + 1, sp2 - sp1 - 1);

  if (method == "GET") request.method = Method::GET;
  else if (method == "POST") request.method = Method::POST;
  else request.method = Method::UNSUPPORTED;

  size_t query_at = target.find('?');
  if (query_at == std::string::npos) request.path = target;
  else {
    request.path = target.substr(0, query_at);
    request.query = target.substr(query_at + 1);
  }

  request.body = raw.substr(header_end + 4, content_length);
  return true;
}

void AgentServer::execute() {
  #ifdef __APPLE__
    pthread_setname_np("agent_server");
  #elif !defined(_WIN32)
    pthread_setname_np(pthread_self(), "agent_server");
  #endif

  socket_t listener = to_socket(listen_socket);

  while (thread_active.load()) {
    if (!wait_readable(listener, accept_timeout_us)) continue;

    socket_t client = ::accept(listener, nullptr, nullptr);
    if (client == BAD_SOCKET) continue;

    Response response;
    Request request;

    const auto started = std::chrono::steady_clock::now();
    const Route* route = nullptr;

    if (!read_request(from_socket(client), request)) {
      response.error(400, "malformed or incomplete request");
    }
    else if (request.method == Method::UNSUPPORTED) {
      response.error(405, "only GET and POST are supported");
    }
    else {
      if (rewriter) rewriter(request);
      route = find_route(request);

      if (route == nullptr) {
        response.error(404, "no handler for " + request.path);
      }
      else if (route->affinity == Affinity::Direct) {
        // Runs here, on the server thread. Only valid for atomic-only
        // handlers, and essential for recovering a frozen simulation:
        // at realtime_scale == 0 the simulation thread never reaches
        // service(), so a queued unfreeze request could never be serviced.
        route->handler(request, response);
      }
      else if (route->preparer && !route->preparer(request, response)) {
        // The preparer answered (e.g. a bad path) without queueing
      }
      else {
        // Hand off to the simulation thread and wait. Shared ownership means a
        // timed-out request stays alive in the queue, so service() can still
        // fulfil the promise without a use-after-free.
        auto item = std::make_shared<PendingRequest>();
        item->request = request;
        auto future = item->response.get_future();

        {
          std::scoped_lock lock(queue_mutex);
          pending.push_back(item);
        }

        const uint32_t budget_ms = route->timeout_ms ? route->timeout_ms : request_timeout_ms;

        if (future.wait_for(std::chrono::milliseconds(budget_ms)) == std::future_status::ready)
          response = future.get();
        else
          response.error(503, "simulation thread did not service the request in time "
                              "(is realtime_scale 0? POST /kernel/control to resume)");
      }
    }

    send_all(client, build_http_response(response));
    CLOSE_SOCKET(client);

    // Record for the Agent Activity panel. Done after the response is sent so
    // logging never adds latency to the client, and captures the real status
    // including 404s and the 503 timeout path above.
    RequestRecord record;
    record.sim_seconds = Kernel::SimulationRuntime::seconds();
    record.method      = (request.method == Method::GET ? "GET" : "POST");
    record.path        = request.path;
    record.query       = request.query;
    record.status      = response.status;
    record.direct      = route && route->affinity == Affinity::Direct;
    record.duration_ms = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - started).count();
    request_log.append(record);
  }
}

const AgentServer::Route* AgentServer::find_route(const Request& request) const {
  std::string key = (request.method == Method::GET ? "GET " : "POST ") + request.path;

  auto it = handlers.find(key);
  if (it != handlers.end()) return &it->second;

  // Longest matching prefix wins. Iterating the map in reverse gives
  // descending key order, so the first match is the most specific.
  for (auto rit = prefix_handlers.rbegin(); rit != prefix_handlers.rend(); ++rit)
    if (key.compare(0, rit->first.size(), rit->first) == 0) return &rit->second;

  return nullptr;
}

void AgentServer::dispatch(const Request& request, Response& response) {
  const Route* route = find_route(request);
  if (route == nullptr) {
    response.error(404, "no handler for " + request.path);
    return;
  }

  route->handler(request, response);
}

void AgentServer::service() {
  if (!thread_active.load()) return;

  for (uint32_t serviced = 0; serviced < max_requests_per_service; ++serviced) {
    std::shared_ptr<PendingRequest> item;
    {
      std::scoped_lock lock(queue_mutex);
      if (pending.empty()) return;
      item = pending.front();
      pending.pop_front();
    }

    Response response;
    dispatch(item->request, response);
    item->response.set_value(response);
  }
}

} // namespace agent

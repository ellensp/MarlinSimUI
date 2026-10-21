#include <thread>
#include <atomic>
#include <cstring>
#include <cstdlib>

#include "application.h"
#include "execution_control.h"

#include "src/inc/MarlinConfig.h"

#include "RawSocketSerial.h"
#include "audio.h"
#include "agent/agent_interface.h"
#include "agent/serial_log.h"
#include "machine_model.h"
#include "thermal_fault_blink.h"

RawSocketSerial net_serial{};

std::atomic_bool main_finished = false;

void HAL_idletask() {
  Kernel::yield();
}

extern void setup();
extern void loop();
void marlin_loop() {
  static bool initialised = false;
  if (!initialised) {
    initialised = true;
    #ifdef MYSERIAL0
      MYSERIAL0.begin(BAUDRATE);
      SERIAL_FLUSHTX();
    #endif
    HAL_timer_init();
    setup();
  } else loop();
}

void simulation_main() {
  #ifdef __APPLE__
    pthread_setname_np("simulation_main");
  #else
    pthread_setname_np(pthread_self(), "simulation_main");
  #endif

  // Marlin Loop 500hz
  Kernel::Timers::timerInit(3, 1000000);
  Kernel::Timers::timerStart(3, 500);
  Kernel::Timers::timerEnable(3);
  Kernel::is_initialized(true);

  while(!main_finished) {
    try {
      Kernel::execute_loop();
    } catch (std::runtime_error& e) {
      // stack unrolled by exception in order to exit cleanly
      // todo: use a custom exception
      printf("Exception: %s\n", e.what());
      printf("Marlin thread terminated\n");
      main_finished = true;
    }
  }
}

struct CommandLineOptions {
  uint16_t serial_port = 8099;  // historical default for the raw G-code socket
  uint16_t agent_port  = 0;     // 0 = agent interface disabled
  bool audio_enabled   = true;
};

static void print_usage(const char* argv0) {
  printf("Usage: %s [options]\n\n", argv0);
  printf("  --serial-port <n>   TCP port for the raw G-code serial socket (default 8099)\n");
  printf("                      (also settable via MARLIN_SIM_SERIAL_PORT)\n");
  printf("  --agent-port <n>    Enable the HTTP agent control interface on 127.0.0.1:<n>\n");
  printf("                      (also settable via MARLIN_SIM_AGENT_PORT)\n");
  printf("  --no-audio          Disable audio/buzzer emulation\n");
  printf("  --machine <type>    Printer model in the Viewport: bedslinger, cube, delta, scara, belt\n");
  printf("                      (default: from the Marlin configuration; DELTA, SCARA and BELTPRINTER builds allow only their own)\n");
  printf("  --belt-angle <deg>  Belt printer gantry tilt from the belt, 1-89 (default %g)\n", MachineModel::belt_default_tilt);
  printf("  --help              Show this message\n");
}

// Returns false if the program should exit immediately.
static bool parse_command_line(int argc, char** argv, CommandLineOptions& options) {
  // Environment defaults, overridden by the command line
  auto env_port = [](const char* name, uint16_t& target) {
    if (const char* env = getenv(name)) {
      const unsigned long value = strtoul(env, nullptr, 10);
      if (value > 0 && value <= 65535) target = uint16_t(value);
      else fprintf(stderr, "%s: invalid port '%s' (ignored)\n", name, env);
    }
  };
  env_port("MARLIN_SIM_SERIAL_PORT", options.serial_port);
  env_port("MARLIN_SIM_AGENT_PORT", options.agent_port);

  for (int i = 1; i < argc; ++i) {
    const char* arg = argv[i];

    auto next_port = [&](uint16_t& target) {
      if (i + 1 >= argc) {
        fprintf(stderr, "%s requires a port number\n", arg);
        return false;
      }
      unsigned long value = strtoul(argv[++i], nullptr, 10);
      if (value == 0 || value > 65535) {
        fprintf(stderr, "%s: invalid port '%s'\n", arg, argv[i]);
        return false;
      }
      target = uint16_t(value);
      return true;
    };

    if (strcmp(arg, "--serial-port") == 0) {
      if (!next_port(options.serial_port)) return false;
    }
    else if (strcmp(arg, "--agent-port") == 0) {
      if (!next_port(options.agent_port)) return false;
    }
    else if (strcmp(arg, "--no-audio") == 0) {
      options.audio_enabled = false;
    }
    else if (strcmp(arg, "--machine") == 0) {
      if (i + 1 >= argc || (machine_type_option = machine_type_from_name(argv[++i])) == MACHINE_TYPE_COUNT) {
        fprintf(stderr, "--machine requires one of: bedslinger, cube, delta, scara, belt\n");
        return false;
      }
      if (!MachineModel::is_available(machine_type_option)) {
        fprintf(stderr, "--machine %s: not available for this build's kinematics (DELTA builds show only 'delta', SCARA builds only 'scara', BELTPRINTER builds only 'belt')\n", argv[i]);
        return false;
      }
    }
    else if (strcmp(arg, "--belt-angle") == 0) {
      char* end = nullptr;
      const float angle = i + 1 < argc ? strtof(argv[++i], &end) : 0.0f;
      if (end == nullptr || *end != '\0' || !(angle >= 1.0f && angle <= 89.0f)) {
        fprintf(stderr, "--belt-angle requires an angle in degrees from 1 to 89\n");
        return false;
      }
      belt_angle_option = angle;
    }
    else if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0) {
      print_usage(argv[0]);
      return false;
    }
    else {
      fprintf(stderr, "Unknown option: %s\n", arg);
      print_usage(argv[0]);
      return false;
    }
  }

  return true;
}

// Main code
int main(int argc, char** argv) {
  CommandLineOptions options;
  if (!parse_command_line(argc, argv, options)) return 0;

  uint32_t sdl_flags = options.audio_enabled ? SDL_INIT_AUDIO : 0;
  SDL_Init(sdl_flags);
  SDLNet_Init();

  if (options.audio_enabled) audio_init();

  // Listen before starting simulator loop to avoid
  // thread synchronization issues if listen_on_port fails
  net_serial.listen_on_port(options.serial_port);

  // Watch Marlin's output for thermal faults, to blink the offending heater.
  // Set before the simulation starts, whether or not the agent interface is on.
  agent::serial_log.set_observer([](uint8_t, const std::string& line) { thermal_fault_blink::observe(line); });

  if (options.agent_port) {
    agent::register_routes();
    agent::server.start(options.agent_port);  // non-fatal on failure
  }

  Application app;
  std::thread simulation_loop(simulation_main);

  while (app.active) {
    app.update();
    app.render();
    std::this_thread::yield();
  }

  main_finished = true;
  Kernel::quit_requested = true;
  simulation_loop.join();
  agent::server.stop();
  net_serial.stop();

  SDLNet_Quit();
  SDL_Quit();

  return 0;
}

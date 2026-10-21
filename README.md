# Marlin Simulator

**This is the `cutting-edge` branch, used for new development.** It adds features on
top of `master` that are still being refined, so expect changes and the occasional
rough edge. For a stable simulator use `master`.

MarlinSimUI runs Marlin firmware as a native desktop application, with simulated
steppers, heaters, endstops, displays and SD card, plus a 3D view of the printer.

## Building Marlin with the Simulator

### The easy way: Auto Build Marlin

1. Install [Visual Studio Code](https://code.visualstudio.com/) and the
   [Auto Build Marlin](https://marketplace.visualstudio.com/items?itemName=MarlinFirmware.auto-build)
   extension (it installs PlatformIO for you).
2. Get Marlin (`bugfix-2.1.x`) from [GitHub](https://github.com/MarlinFirmware/Marlin)
   and open the Marlin folder in VS Code.
3. Open Auto Build Marlin and click **Download** to open the Configuration
   Downloader. It backs up your current configuration first.
4. Select the **Simulator** example (type "Simulator" to search). Its
   `MOTHERBOARD` is `BOARD_SIMULATED`.
5. Click **Build** on the `simulator_…` environment for your OS, then **Run**.

### From the command line

Install the system libraries (SDL2, SDL2_net, OpenGL, GLM) listed at the top of
Marlin's `ini/native.ini`. For example:

```bash
sudo apt-get install build-essential libsdl2-dev libsdl2-net-dev libglm-dev   # Debian/Ubuntu
brew install gcc@14 glm mesa sdl2 sdl2_net                                     # macOS (Homebrew)
```

Copy the Simulator example from
[Configurations](https://github.com/MarlinFirmware/Configurations/tree/bugfix-2.1.x/config/examples/Simulator)
into `Marlin/`, then:

```bash
pio run -e simulator_linux_debug      # or simulator_macos_debug, simulator_windows
./.pio/build/simulator_linux_debug/debug/MarlinSimulator
```

Marlin's `ini/native.ini` fetches this branch
(`MarlinSimUI/archive/cutting-edge.zip`). To build against a local checkout,
point that `lib_deps` line at it:

```ini
MarlinSimUI=symlink:///path/to/MarlinSimUI
```

The simulator saves `eeprom.dat` and `imgui.ini` (window layout) in the directory
it starts from.

## What's new on cutting-edge

### Agent interface (HTTP/JSON)

A local HTTP endpoint lets a script, CI job or AI agent drive the simulator and
read its state. It reports the simulated hardware alongside what Marlin believes,
takes G-code, touches and camera moves, captures screenshots, and can freeze or
speed up simulated time.

```bash
MarlinSimulator --agent-port 8100 --no-audio &

curl -s http://127.0.0.1:8100/ping
curl -s -X POST http://127.0.0.1:8100/gcode -d '{"command":"G28"}'
curl -s http://127.0.0.1:8100/idle                        # repeat until "idle":true
curl -s http://127.0.0.1:8100/state/Bed%20Heater
curl -s -X POST http://127.0.0.1:8100/screenshot -d '{"path":"/tmp/lcd.png"}'
curl -s -X POST http://127.0.0.1:8100/capture/viewport -d '{"path":"/tmp/view.png"}'
curl -s -X POST http://127.0.0.1:8100/kernel/control -d '{"realtime_scale":10}'
```

- `POST /gcode` sends to serial stream 3, which Marlin only reads with
  `#define SERIAL_PORT_2 3` enabled in `Configuration.h` (commented out in the
  Simulator example). Without it, commands are accepted and ignored.
- The interface listens on 127.0.0.1 only, and is off unless `--agent-port` is
  given (or `MARLIN_SIM_AGENT_PORT` is set).
- Full reference: [docs/agent-interface.md](docs/agent-interface.md).

Command-line options:

| Option | Meaning |
| --- | --- |
| `--agent-port <n>` | Enable the agent interface on `127.0.0.1:<n>` (or `MARLIN_SIM_AGENT_PORT`). |
| `--serial-port <n>` | TCP port for the raw G-code socket (default 8099, or `MARLIN_SIM_SERIAL_PORT`). |
| `--no-audio` | Disable buzzer emulation. |
| `--machine <type>` | Viewport printer model: `bedslinger`, `cube`, `delta`, `scara` or `belt`. |
| `--belt-angle <deg>` | Belt printer gantry tilt from the belt, 1 to 89 degrees (default 45). |

### 3D printer models in the Viewport

The Viewport draws a whole printer that moves with the simulation, sized from
your configuration (bed size, Z height, delta geometry).

- **Bedslinger** (i3 style), **Cube** (head on top, bed moves in Z), **Delta**, **SCARA** (one arm over a Z bed)
  and **Belt Printer** (`BELTPRINTER`: a cube frame without bottom or back, tilted over two rollers and a belt).
  DELTA builds show the Delta, SCARA builds the SCARA, BELTPRINTER builds the Belt Printer; other builds offer the Bedslinger and the Cube.
- **Belt printer:** Marlin Y moves the nozzle up the tilted gantry (Y=0 is the tip on the belt) and Z feeds the belt
  toward the open front. The simulator draws the physical result of the G-code it receives, so G-code sliced for
  a belt printer (for example by OrcaSlicer's belt mode) prints upright. The print rides the belt away from the nozzle.
  X is mirrored, as in the slicer's belt profiles. The tilt is `--belt-angle`.
  Choose one in the **Printer** menu, with `--machine`, or with `POST /view`.
- **Multiple hotends:** one hotend per `HOTENDS`, placed at its tool offset. The
  model follows `M218`, `M501` and anything else that changes the offsets.
- **Temperature colors:** hotends go from light blue (cold) to pink (at target),
  and the bed from dark blue to dark red. The active hotend glows.
- **Bed markings:** the 0,0 origin (white circle with a cross), the
  `Z_SAFE_HOMING` point (yellow circle with an X), the probeable area (white
  dashed outline, following `M851`), the active tool's reach (light orange
  dashed outline of the software endstops, following the tool and `M218`;
  darker orange when `M211` turns them off) and the leveling mesh grid (light blue).
  Toggle with **Printer > Bed Markings**. On a Delta the reach also includes
  the physical limit of the arms: a solid light orange rounded triangle with its
  corners toward the towers.
  On a SCARA the software endstops are a box, solid light orange rings show the reach of the arms
  and the dead zone around the shoulder where they cross the bed, and dashed rings show the
  probe's own reach (it hangs from the forearm at the `M851` offset).
- **Printable volume (Delta):** **Printer > Printable Volume** shows a translucent
  volume of where the active tool can print, from the live `M665`/`M666`/`M218`
  geometry. The top is scooped toward each tower, where the carriages reach their
  endstops first.
- **Printer > Show Printer** hides the model, leaving the bed and the print.

### Camera

- **Turntable** (default) orbits a target point. **Fly** is a free camera.
  Switch in **Camera > Mode**.
- Turntable: left-drag to orbit, right-drag, middle-drag or Shift-drag to pan,
  mouse wheel or `+`/`-` to zoom, arrow keys to turn.
- Keys: `1` front, `3` right, `7` top, `0` three-quarter, `F` follow the nozzle,
  `R` reset. In either camera mode, `P` shows/hides the printer, `M` the bed
  markings and `V` the printable volume (Delta).
- Agents can set the view with `POST /view`
  (`{"preset":"iso"}`, `{"yaw":30,"pitch":20,"distance":400}`, `{"machine":"cube"}`,
  `{"markings":false}`).

### Other improvements

- The window opens maximized.
- Better touchscreen tap and drag handling.
- Fixes for building with no LCD, and with `CYRILLIC` characters.
- Panel capture by name: `POST /capture/<panel>` (`lcd`, `viewport`).

## Branches

- `master`: stable.
- `cutting-edge`: new development (this branch).
- `cutting-edge-more-printers`: `cutting-edge` plus a Prusa i3 demo model built
  from STL parts (`--machine i3`).

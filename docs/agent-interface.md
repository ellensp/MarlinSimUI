# MarlinSimUI Agent Interface

An HTTP/JSON control endpoint that lets a script, CI job, or AI agent drive the
simulator and observe machine state without a human at the window. It is the
canonical reference for the interface; firmware-side usage notes live in
Marlin's `Marlin/src/HAL/NATIVE_SIM/AGENTS.md`.

The three things it provides that the serial port cannot:

- **Ground truth.** Simulated stepper positions, heater physics, endstop pin
  levels, and touch/calibration state, reported next to what Marlin believes.
- **Stimulus beyond G-code.** Touchscreen taps now; pin and component fault
  injection are planned.
- **Time control.** Freeze, run at wall-clock, or fast-forward the simulation.

---

## 1. Quick start

```bash
# In the Marlin repo, with a Simulator config (see "Configuration" below)
pio run -e simulator_macos_debug           # or simulator_linux_debug, etc.
./.pio/build/simulator_macos_debug/debug/MarlinSimulator --agent-port 8100 --no-audio &

curl -s -m 5 http://127.0.0.1:8100/ping
curl -s -m 5 -X POST http://127.0.0.1:8100/gcode -d '{"command":"G28"}'
curl -s -m 5 http://127.0.0.1:8100/idle                    # repeat until "idle":true
curl -s -m 5 -X POST http://127.0.0.1:8100/screenshot -d '{"path":"/tmp/lcd.png"}'
```

Command-line options:

| Option | Meaning |
| --- | --- |
| `--agent-port <n>` | Enable the agent interface on `127.0.0.1:<n>` (or `MARLIN_SIM_AGENT_PORT`). Off by default. |
| `--serial-port <n>` | TCP port for the raw G-code socket (default 8099, or `MARLIN_SIM_SERIAL_PORT`). |
| `--no-audio` | Disable buzzer/audio emulation (useful for unattended runs). |
| `--machine <type>` | Viewport printer model: `bedslinger`, `cube`, `delta` or `scara`. Default follows the build's kinematics. DELTA builds accept only `delta`, SCARA builds only `scara`; other builds accept `bedslinger` and `cube`. |
| `--help` | Usage. |

The simulator does not exit on its own. Stop it with `POST /kernel/control
{"quit":true}` or by killing the process. The working directory holds
`eeprom.dat` and `imgui.ini`, so launch from a directory whose state you intend
to use.

### Configuration

- `POST /gcode` writes into serial stream 3, which Marlin only reads if a serial
  port maps to it. Enable `SERIAL_PORT_2 3` (commented out in the Simulator
  example config). Without it commands are accepted and silently ignored.
- `M111 S3` makes Marlin echo each command back on the port it arrived on, so
  agent-issued G-code is visible in the Serial Monitor(3) panel.
- Display screenshots and touch need the corresponding display enabled in the
  config (e.g. `TFT_COLOR_UI` for the ST7796 + XPT2046 pair).

---

## 2. Endpoints

All bodies are flat JSON objects. Every response is JSON; errors are
`{"error": "...", "status": N}` with the matching 4xx/5xx HTTP status.
Simulation-thread requests time out with 503 after 5 s of wall-clock time.

| Route | Thread | Purpose |
| --- | --- | --- |
| `GET /ping` | sim | Liveness. Proves the simulation thread is servicing requests. |
| `GET /kernel` | sim | Ticks, sim time, realtime scale, ISR timing error, timers. |
| `POST /kernel/control` | direct | `{"realtime_scale":N}` (0 freezes, 1 = wall clock, max 1000), `{"break":true}`, `{"quit":true}`. |
| `GET /components` | sim | Registry names usable with `/state/<name>`. |
| `GET /state` | sim | Kernel plus every component, one snapshot. |
| `GET /state/<name>` | sim | One component. Percent-encode names (`Endstop(X%20Min)`). |
| `GET /serial` | direct | Captured serial output (see 2.2). |
| `POST /serial` | direct | `{"clear":true}` / `{"max_lines":N}`. |
| `POST /gcode` | sim | `{"command":"G1 X10"}`. One line. Never blocks. |
| `GET /idle` | sim | `{"idle":bool, "rx_pending", "commands_queued", "blocks_queued", "planner_busy", "moves_queued"}`. |
| `GET /displays` | sim | Components that can be captured. |
| `POST /screenshot` | sim | `{"path":"/abs/file.png"[, "display":"<name>"]}`. Writes a PNG. |
| `POST /touch` | sim | `{"x":0..1, "y":0..1[, "hold_ms":N]}`. Taps or holds the touchscreen. |
| `POST /touch/rects` | sim | `{"show":bool}` or `{}` to toggle. Overlays Marlin's touch control rects on the TFT. |
| `POST /capture/<panel>` | varies | Capture a visual panel to PNG by short name, case-insensitive (see 2.7). |
| `GET /view` | direct | Current Viewport camera and printer model (see 2.6). |
| `POST /view` | direct | Set the Viewport camera and/or printer model. Applied on the next UI frame. |

**Thread** is the route's affinity. `sim` routes are queued and executed on the
simulation thread (safe to read Marlin and component state); if the simulation
does not service them within the budget the server answers 503. `direct` routes
run on the server thread and touch only atomics or their own mutex, so they keep
working while the simulation is frozen or wedged. That is how you unfreeze a
simulation that was set to `realtime_scale: 0`.

### 2.1 Waiting for work: the poll contract

`POST /gcode` returns immediately with `serial_from`, the serial-log sequence
number before the command was submitted. To wait for completion, poll
`GET /idle` until `idle` is true, then read `GET /serial?since=<serial_from>`.

Handlers never block by design. They run inside the simulation loop, which
cannot re-enter itself, so a handler that waited for the G-code queue to drain
would deadlock the simulation and, through the single accept loop, every route.

`rx_pending` is part of `idle` because right after `POST /gcode` the bytes are
still in the receive buffer while the queue and planner are empty.

### 2.2 Reading serial output

`GET /serial?since=<seq>&limit=<n>&stream=<0-3>` returns whole lines, each with
`seq`, `stream`, `sim_seconds`, `text`. Resume from the returned `next_seq`
(one past the last line returned) — not from `head_seq`, or a filtered or
limit-capped page silently skips lines. `oldest_seq` and `dropped` reveal
eviction from the bounded ring (default 10,000 lines). Clearing does not reset
sequence numbers.

### 2.3 Screenshots

`POST /screenshot` reads the device's own pixel buffer, not the GL texture, so
it works whether or not the panel is visible. RGB565 panels are expanded with
bit replication so full-scale white is 255. Always look at the image: a
well-formed PNG of the wrong screen passes every structural check.

### 2.4 Touch

`POST /touch` presses the first touch device at panel ratios `x`,`y`
(0,0 = top-left) and releases it after `hold_ms` of simulated time. The device
keeps the press until Marlin has polled it enough times to act on it (the TFT
touch code needs two consecutive polls), so a 0 ms tap is a reliable single
click. Hold longer than Marlin's repeat pre-delay to trigger key-repeat on
+/- controls, and longer than `TOUCH_SCREEN_HOLD_TO_CALIBRATE_MS` (2500 ms by
default) on an empty area of the status screen to open touch calibration.

To address a control, convert from the panel's pixel layout:
`x = px / TFT_WIDTH`, `y = py / TFT_HEIGHT`. Screenshot first to find it.

`GET /state/Touch` reports the raw coordinates, poll count, and the calibration
Marlin is currently applying (`cal_x`, `cal_y`, `offset_x`, `offset_y`,
`orientation`). A calibration of all zeros means every touch maps to (0,0) and
normal controls cannot fire.

`POST /touch/rects` shows (`{"show": true}`), hides (`{"show": false}`) or
toggles (`{}`) an overlay of Marlin's touch control rectangles on the SPI TFT:
a 2px white outline with a 10% white interior. The response gives the current
`count`. Marlin's simulator build sends the rectangles to the panel with the
custom command `0xF5` as the last stage of drawing each screen, so the overlay
always matches the controls Marlin will hit-test. It is composited over the
panel image (window and screenshots) without changing the panel's memory, so
it can be toggled at any time. The display's "Touch Rects" checkbox and
`POST /component/ST7796Device Display {"touch_rects": true}` do the same.
Use it to check that touch zones line up with what is drawn.

### 2.5 Component state

Components opt in by overriding `Component::serialize()`. Implemented:

| Component | Fields |
| --- | --- |
| Kinematic systems | Commanded position; per-effector position and stepper-derived position, in Marlin's native coordinates. On SCARA the shoulder is fixed in the viewport (centered behind the bed) and `SCARA_OFFSET` only sets how Marlin's coordinates map onto it, so `probe_tip` and the effectors are reported in Marlin's frame, and `joints` gives the ground-truth shoulder (`theta`, degrees from +X) and elbow (`psi`, degrees from the upper arm) angles, and a config whose offsets don't match the hardware puts the reach rings, limits and mesh off the plate instead of moving the arm. |
| `Heater` | Temperature, ambient, energy, sensor mode (`Normal`/`ForceMin`/`ForceMax`/`ForcePause`/`Fixed`), PWM duty/period, volts, resistance, pins; the thermal model in MPC units (`heater_power`, `heat_capacity`, `ambient_xfer`, `fan_xfer`, `fan_heater_loss`, `filament_heat_capacity_permm`, `sensor_responsiveness`), `sensor_reading`, `fan_pin`, `fan_fraction` and the live power terms `power_heater`/`power_ambient`/`power_fan`/`power_filament` (W), fan-coupled sensor error (`fan_sensor_offset`, `fan_sensor_ripple`, `fan_sensor_ripple_period`, `fan_sensor_noise`, last `sensor_error`). Settable: all model and sensor-error terms, `sensor_mode`, `sensor_temperature`, `element_ok`, `temperature`, `ambient_temperature`. |
| `EndStop` | Effective trigger, geometric trigger, manual override, pin level, pin. |
| `Touch` (XPT2046) | Raw x/y, dirty, held, polls, injected touches, calibration. |
| `ST7796Device Display` | Width, height, `touch_rects` (overlay shown), `touch_rect_count`. Settable: `touch_rects`. |

Others report `{}`. Prefer reporting simulated ground truth over Marlin's
belief — the difference between the two is usually the bug.

### 2.6 Viewport camera and printer model

`GET /view` returns `{"ok", "mode": "turntable"|"fly", "machine", "yaw",
"pitch", "distance", "x", "y", "z", "follow", "markings", "volume"}`. `x`,`y`,`z` is the camera
target.

`POST /view` switches to the Turntable camera and applies any of these keys
(all optional):

| Key | Meaning |
| --- | --- |
| `preset` | `front`, `right`, `back`, `left` (15° pitch), `top` or `iso` change the angle and keep the distance and target. `home` resets the whole view. |
| `yaw`, `pitch` | Degrees. |
| `distance` | mm from the target, > 0. |
| `x`, `y`, `z` | Camera target in Marlin coordinates (mm). All three or none. |
| `follow` | `true` keeps the nozzle at the view center. |
| `machine` | `bedslinger`, `cube`, `delta` or `scara`; same availability rule as `--machine`. |
| `markings` | `true`/`false` shows or hides the bed markings (origin, safe homing point, probeable area, tool reach, mesh grid), like Printer > Bed Markings. |
| `volume` | `true`/`false` shows or hides the translucent printable volume of the active tool, like Printer > Printable Volume. DELTA builds only; always `false` otherwise. |

The request is queued and applied on the next UI frame (the agent thread never
touches the camera), so it works while the simulation is frozen; poll
`GET /view` to confirm. Errors: 400 for bad JSON, a partial target,
`distance <= 0`, or an unknown/unavailable machine.

The Viewport is captured with `POST /capture/viewport` (2.7).

### 2.7 Panel capture by name

`POST /capture/<panel>` with `{"path":"/abs/file.png"}` writes a PNG of a panel
that renders visuals. The panel name is case-insensitive.

| Panel | Aliases | Captures | Thread |
| --- | --- | --- | --- |
| `lcd` | | Same as `POST /screenshot`, including the optional `"display"` key. Only in builds with a simulated display | sim |
| `viewport` | `vp` | The 3D Viewport at its current panel size, as last rendered | UI (the handler waits up to 3 s for the next frame; works while the simulation is frozen) |

The response matches `/screenshot`:
`{"ok", "path", "display", "width", "height", "bytes"}`.

Errors:
- 404 for any other panel name. Captures are added case by case, and only for
  panels that render visuals.
- 503 if no frame was rendered in time, or the Viewport has no image yet.
- 409 if a newer Viewport capture replaced this one.

Set up the shot first with `POST /view`. It's applied on the next frame, so it
lands before, or in the same frame as, the capture.

---

## 3. Using it for debugging

Typical loop for a firmware or UI change:

1. Build the simulator env and launch with `--agent-port`.
2. Put the machine in the state under test with `POST /gcode`, `POST /touch`,
   and `realtime_scale` to skip long heat-ups or moves.
3. Observe with `GET /state/<component>`, `GET /serial`, and `POST /screenshot`.
4. Compare ground truth with Marlin's reports (`M114`, `M105`, `M119`).
5. Rebuild and repeat with the same script so before and after are comparable.

Guidelines:

- **Script it.** A short Python or shell script that replays the scenario is
  the regression test for the fix. Keep it with the change.
- **Check what is actually loaded.** Delete or back up `eeprom.dat` to separate
  "stored settings" bugs from code bugs, and confirm settings after boot rather
  than trusting a decode of the file: init code can overwrite loaded values.
- **Verify the code path is linked.** Display and touch devices only link when
  the config selects them. `nm MarlinSimulator | grep <Device>` before trusting
  a green build.
- **Use wall-clock timeouts.** Simulation seconds scale with `realtime_scale`.
- **Leave a human's session alone.** A running simulator may be someone's
  interactive test. Launch your own instance on another port.

---

## 4. Extending the interface

- **New read-out:** override `serialize(agent::JsonWriter&)` on the component.
  Called on the simulation thread; keep it allocation-light and never block.
- **New display capture:** override `capture(rgb, width, height)` returning
  RGB888, row-major, top row first.
- **New input device:** override `inject_touch()` (or add a sibling virtual for
  other input kinds) on the component, and route it in `agent_interface.cpp`.
- **New route:** `server.route("METHOD /path", handler[, Affinity, timeout_ms])`
  in `register_routes()`. Use `Affinity::Direct` only for handlers that touch
  nothing but atomics or their own lock.

The server binds 127.0.0.1 only. Keep it that way: it executes G-code and will
eventually override pins.

---

## 5. Design notes and roadmap

**Why HTTP/JSON.** It works with `curl` and every language without a client
library, matches request/response agent work, is stateless, and reuses the
socket code already in the simulator. A hand-rolled JSON writer/parser avoids
adding a vendored dependency to an already slow native build. Newline-delimited
JSON over a raw socket would suit server-pushed events better but loses `curl`;
embedded scripting and log scraping were rejected as too heavy and too brittle
respectively.

**Threading contract.** The server thread parses HTTP and never touches Marlin
or component state. Simulation-affinity requests are queued with a promise and
drained by `Kernel::execute_loop()` at the same point as the TCP serial buffer,
at most `max_requests_per_service` per pass so a chatty client cannot starve the
stepper ISR. `execute_loop()` is re-entrant (`yield()`/`delayCycles()` call it
from inside ISRs), which is another reason handlers must not block.

**Planned, roughly in order:**

1. Fault injection: `POST /pin` (override a pin level) and
   `POST /component/<name>` (heater sensor fault, endstop force, runout).
2. `GET /events?since=<seq>` long-poll for endstop hits, target temperature
   reached, `kill()`/thermal runaway, print finished, status messages — so an
   agent can tell "still working" from "firmware halted" without polling state.
3. `--headless`: run the simulation thread and agent server without a window or
   GL context, for CI. Serial capture is currently gated on the Serial Monitor UI
   elements existing and must be decoupled first.
4. More input kinds: encoder rotation/click and hardware buttons for
   character/graphic LCDs, alongside touch.

Informed by prior art (Zephyr emul, MK404, Wokwi, QEMU qtest, Renode):

5. Per-device transport vs backend split: each device implements its bus face
   (SPI/I2C/UART) plus a backend face exposed as `GET/POST /component/<name>/...`.
6. Self-describing actions: `GET /actions` lists each component's named actions
   and faults with typed arguments.
7. Deterministic stepping: `POST /kernel/step {"ns":N}` or `{"until":"next_deadline"}`.
8. Touch press/move/release, optionally in raw controller coordinates so
   Marlin's calibration and rotation paths are exercised.
9. `POST /wait` on a serial line, component value threshold, or LCD text, with a
   sim-time timeout.
10. Named snapshots (EEPROM, SD, component state, sim time) and scenario files
    run by a CLI with CI exit codes.

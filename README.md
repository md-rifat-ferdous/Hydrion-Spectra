# DUBO — ROV Controller (Raspberry Pi)

Underwater ROV (Remotely Operated Vehicle) control system for Raspberry Pi (Raspberry Pi OS),
designed as a modular, professional control station. This file is the session handover document:
reading it tells you exactly what exists, what runs, and what to do next.

---

## 1. Project Goal

Build a modular ROV software stack that:

- Auto-starts on Raspberry Pi boot (via systemd).
- Provides a professional GUI control station.
- Streams live camera video.
- Drives a real 5-thruster layout via ESP32 ESC controller over UART.
- Provides real-time telemetry, sensors (IMU/depth/compass), failsafe, logging, mission control.

**Current milestone:** Phase 2 Part B is **DONE** — the ESP32 firmware protocol is fully implemented
in Python (the Pi side), the GUI is upgraded with live thruster/motion panels, E-STOP/RE-ARM, ESP32
UART diagnostics, and the system is tested end-to-end (74 unit tests pass, GUI boots clean with
simulated provider, ESP32 thread lifecycle verified).

**Bench state (2026-09-20):** a real ESP32 is wired to the Pi's `ttyAMA0` (UART0, GPIO14/15) at
460800 baud. The Pi now receives and decodes the ESP32's live STATUS telemetry (link alive, CRC clean);
throttle modes (soft 40% / medium 70% / high 100%) and press-and-hold 10% ramp are implemented. One
open item remains: **the physical Pi→ESP32 TX path is not yet verified** — the ESP32 has never emitted
an ACK, so its RX (UART0 / GPIO3, see §9) does not appear to be receiving the Pi's frames. See §9
Progress Log.

---

## 2. Project Structure

```
dubo/
├── app/                 # Entry point
│   └── main.py          # RUNS THE APP (launches GUI)
├── core/                # Shared framework
│   ├── base_module.py   # BaseModule class (implemented)
│   ├── application.py   # Application (boots Config+Logger+ServiceManager+Qt)
│   └── service_manager.py  # ServiceManager (module lifecycle conductor)
├── modules/             # Feature modules
│   ├── camera/          # camera_manager.py — REAL (captures + reads frames)
│   ├── sensors/         # sensor_manager.py — REAL (state + apply_motion, simulated provider)
│   ├── telemetry/       # telemetry_manager.py — REAL (JSON frames + heartbeat over a Link)
│   ├── controller/      # controller.py — REAL (MotionState merge: pad/keyboard/link/gamepad)
│   ├── config/          # config_manager.py — loads all YAML configs centrally
│   ├── logger/          # logger_manager.py — console + file logging
│   └── thrusters/       # 5-thruster model + ESP32 UART transport
│       ├── thruster_manager.py   # ThrusterManager + Esp32ThrusterProvider + SimulatedProvider
│       └── uart_transport.py     # ESP32 firmware-matched binary frame protocol
├── ui/                  # GUI (DUBO GCS "Dark Ocean" style)
│   ├── main_window.py   # MainWindow — app bar + nav + footer + dashboard + panels
│   ├── theme.py         # Design tokens (colors/fonts) + global QSS
│   ├── glass.py         # GlassPanel / StatusPill / NavButton / ValueRow widgets
│   ├── control_pad.py   # ControlPad3D — circular joystick + yaw ring
│   ├── sonar.py         # SonarPanel — top-right glass map
│   ├── hud.py           # HUD overlay — compass + crosshair + DEPTH/ALT + status chips
│   └── sections.py      # Nav pages — Sensors/Diagnostics/Logs/Settings/Operations
├── configs/             # YAML configs
│   ├── thrusters.yaml   # provider selection + ESP32 UART settings + motor map
│   ├── hud.yaml         # HUD settings (compass/crosshair/depth-alt/extra/status chips)
│   ├── gcs.yaml         # GCS shell (sonar + telemetry sim values)
│   ├── controller.yaml  # keymap + gamepad codemap
│   ├── camera.yaml      # camera device settings
│   ├── sensors.yaml     # sensor provider settings
│   └── network.yaml     # telemetry link settings
├── systemd/             # rov-controller.service — auto-start template (Step 6)
├── scripts/             # start.sh / stop.sh / restart.sh
├── tests/               # 74 unit tests (mixer + UART transport + provider)
├── logs/                # system.log / errors.log / mission.log (runtime)
└── .venv/               # Python 3.13 virtualenv with deps
```

---

## 3. Setup & Dependencies

Runs on **Raspberry Pi 5** (aarch64, Debian 13 trixie) or any Linux with a display.

Virtualenv: `.venv` (Python 3.13). Installed packages:

| Package        | Version |
|----------------|---------|
| opencv-python  | 5.0.0.93 |
| PySide6        | 6.11.2 |
| pillow         | 12.3.0 |
| PyYAML         | 6.0.3 |
| numpy          | 2.5.3 |
| pyserial       | 3.5    |

Activate / re-create:

```bash
cd /path/to/hydrion-spectra
python3 -m venv .venv
source .venv/bin/activate
pip install opencv-python PySide6 pillow PyYAML numpy pyserial
```

---

## 4. How to Run

```bash
source .venv/bin/activate
python app/main.py
```

Expected behavior:

1. A window **"DUBO - ROV Controller"** opens (dark navy "Dark Ocean" theme):
   - **Top app bar:** brand, pills (ROV / ESP32 / MODE / DEPTH / BATT / E-STOP / LINK / UP).
   - **Left nav rail** (expands on hover): Dashboard, Operations, Sensors, Manipulator, Diagnostics,
     Planner, AI Vision, Logs, Settings, and a LAUNCH MISSION button.
   - **Dashboard:** camera feed with overlays — LIVE indicator, sonar map, system telemetry, thruster
     meters (M1-M5), motion bars (SURGE/YAW/HEAVE), control dock (ARMED/REC/SNAP/LIGHTS/E-STOP/RE-ARM),
     circular motion controller, mission-log flyout.
   - **Bottom footer:** FPS / CPU / RAM / STORAGE / DEPTH / LATENCY / ESP32 / THRUST.
2. Drive the simulated ROV:
   - **Pad:** drag the circular stick — vertical = surge, horizontal = yaw — spring-return on release.
   - **Keyboard:** `W/S` forward/back, `A/D` + `Q/E` turn (yaw), `R/F` up/down (heave), `I/K` pitch,
     `J/L` roll (future), `Shift` boost, `Backspace` kill / E-STOP.
3. **E-STOP:** click the E-STOP button in the dock or press `Backspace` — all thrusters halt immediately,
   ESP32 receives ESTOP message. Press **RE-ARM** to resume (requires ESP32 acknowledgment).
4. Thruster meters update in real-time (100ms refresh), showing M1-M5 percentages and direction.
5. Telemetry frames are appended to `logs/mission.log`.
6. Closing the window releases the camera and stops all threads cleanly.

---

## 5. Implemented vs Placeholders

### Implemented (real code)

- **core/base_module.py** — `BaseModule` with `initialize/start/update/stop/health_check`.
- **core/service_manager.py** — `ServiceManager`: the conductor. `register(module)`, then
  `initialize_all/start_all/update_all/stop_all`. Each step is try/except-guarded.
- **core/application.py** — `Application`: boots Config + Logger, builds modules from config
  (CameraManager, SensorManager, TelemetryManager, ControllerModule, ThrusterManager), creates
  QApplication + MainWindow, drives update_all on a 100ms timer.
- **modules/config/config_manager.py** — `ConfigManager`: loads every `configs/*.yaml` centrally.
- **modules/logger/logger_manager.py** — `LoggerManager`: console + file logging to logs/.
- **modules/camera/camera_manager.py** — `CameraManager`: opens cv2.VideoCapture, reads frames.
- **modules/sensors/sensor_manager.py** — `SensorManager` with `SimulatedSensorProvider`. Produces
  `HudState` (x, y, depth, yaw_deg). `apply_motion()` integrates commanded motion into position.
- **modules/telemetry/telemetry_manager.py** — `TelemetryManager`: packs state into JSON frames,
  sends over a `Link`, emits heartbeats, buffers incoming commands.
- **modules/controller/controller.py** — `ControllerModule` + `MotionState` dataclass. Merges input
  sources (pad/keyboard/link) per axis (strongest wins). Handles link commands (FORWARD, LEFT, UP,
  STOP, ...) and `kill()` emergency stop. `LEFT`/`RIGHT` = yaw (no sway thruster).
- **modules/thrusters/thruster_manager.py** — 5-thruster model with:
  - `ThrusterId` enum (M1-M5).
  - Pure `mix()` function: M1/M4/M5 = heave, M2 = surge+yaw, M3 = surge-yaw. Clamps [-1,1].
  - `ThrusterProvider` ABC → `SimulatedThrusterProvider` + `Esp32ThrusterProvider`.
  - `Esp32ThrusterProvider`: 50Hz background thread, ESTOP/RE-ARM, safety gate, sequence sync,
    failsafe, reconnect-in-zero. All state exposed via `status()` for UI/diagnostics.
  - `ThrusterManager`: failsafe clamp, `get_provider_status()`, `emergency_stop()`/`clear_emergency_stop()`.
- **modules/thrusters/uart_transport.py** — ESP32 firmware-matched binary frame protocol:
  - Frame layout, message types, CRC-16/CCITT-FALSE, uint16 sequence, ACK/STATUS payload formats.
  - `build_motor_command()`, `build_estop()`, `parse_frame()`, `decode_ack()`, `decode_status()`.
- **ui/main_window.py** — MainWindow with:
  - App bar: ROV / ESP32 / MODE / DEPTH / BATT / E-STOP / LINK / UP pills.
  - Dashboard: thruster meters (M1-M5), motion bars (SURGE/YAW/HEAVE), E-STOP + RE-ARM dock buttons.
  - Toast notifications for E-STOP / RE-ARM.
  - 100ms fast refresh for live panels.
- **ui/control_pad.py** — ControlPad3D (circular joystick + yaw ring + reset()).
- **ui/sonar.py** — SonarPanel (glass map with ship, umbilical, ROV, sonar ping).
- **ui/hud.py** — HUD overlay: compass, crosshair, DEPTH/ALT pillars, **status chips** (mode / ESP32
  link / E-STOP drawn into the video frame, optional via config).
- **ui/sections.py** — Nav pages:
  - **Sensors:** depth/heading/position/pitch/roll + health pills (including ESP32).
  - **Operations:** system status + THRUSTERS panel (M1-M5 live values) + E-STOP/RE-ARM buttons.
  - **Diagnostics:** SUBSYSTEM HEALTH + COMPUTE CORE LOAD + UMBILICAL + **ESP32 UART LINK panel**
    (state/provider/tx/rx/crc/resync/ack-latency/seq/motors/last-error + E-STOP/RE-ARM buttons).
  - Logs, Settings — real. Navigation/Manipulator/Planner/AI Vision — placeholders.
- **configs/** — All YAML configs filled in with defaults.
- **systemd/** + **scripts/** — Auto-start on boot (Step 6).

### Manual control

| Input | What it drives | Status |
|-------|---------------|--------|
| `ControlPad3D` (mouse) | surge / yaw (+ yaw ring indicator) | live |
| Keyboard (`configs/controller.yaml` `keymap`) | surge / yaw / heave + boost + kill/E-STOP | live |
| Link commands (`FORWARD`, `LEFT`, ...) | same motion target | live |

Heave (up/down) is on the keyboard (**R/F**); the circular pad drives surge/yaw.

### 5-thruster motion model — canonical reference

| ID | Role | ESP32 GPIO | Mixer equation | Direction |
|----|------|:----------:|----------------|:---------:|
| `M1_FRONT_VERTICAL` | Front vertical | 25 | `heave` | configurable ±1 |
| `M2_MIDDLE_RIGHT_HORIZONTAL` | Middle right | 33 | `surge + yaw` | configurable ±1 |
| `M3_MIDDLE_LEFT_HORIZONTAL` | Middle left | 32 | `surge − yaw` | configurable ±1 |
| `M4_BACK_RIGHT_VERTICAL` | Back right | 27 | `heave` | configurable ±1 |
| `M5_BACK_LEFT_VERTICAL` | Back left | 26 | `heave` | configurable ±1 |

Supported axes: **surge, yaw, heave**. No sway. Pitch/roll kept for future, produce zero motor.

---

## 6. HUD Overlays (ui/hud.py + ui/sonar.py)

### Frame overlays (ui/hud.py)
- **Compass** (top-center): heading ± 20°.
- **Crosshair + pitch ladder** (center).
- **DEPTH pillar** (left-middle), **ALT pillar** (right-middle).
- **Status chips** (top-left, optional): shows MODE, ESP32 link state, E-STOP state drawn into the
  video feed. Configurable via `configs/hud.yaml` → `extra.enabled`.
- All panels toggleable via `configs/hud.yaml`.

### Sonar map (ui/sonar.py)
- Glass panel: ship, dashed umbilical, ROV triangle, animated sonar ping.

### Config reference (`configs/hud.yaml`)

| Key | Default |
|-----|---------|
| `enabled` | `true` |
| `panel_alpha` | `0.55` |
| `compass.enabled` | `true` |
| `crosshair.enabled` | `true` |
| `depth_alt.enabled` | `true` |
| `extra.enabled` | `true` (status chips) |

---

## 7. Hardware Integration (Simulation-first)

Every module is written against a clean interface. The ESP32 link is live on the bench (see §9
current state); sensors still return simulated values until hardware is attached by swapping the
provider — nothing else changes.

### ESP32 UART Protocol (firmware reference)

**The ESP32 firmware is the source of truth.** The Pi matches it exactly.

Frame layout (little-endian):

```
BYTE    FIELD        NOTES
0-1     HEADER       0xAA 0x55 (sync)
2       VERSION      0x01
3       TYPE         0x01 MOTOR_COMMAND (Pi->ESP32)
                     0x02 ESTOP        (Pi->ESP32)
                     0x81 ACK          (ESP32->Pi)
                     0x82 STATUS       (ESP32->Pi)
4       LENGTH       payload bytes
5-6     SEQUENCE     uint16 little-endian (monotonically increasing)
7..     PAYLOAD      type-specific (see below)
last-2  CRC16        CRC-16/CCITT-FALSE over ENTIRE frame INCLUDING the
                     0xAA 0x55 header (init=0xFFFF, poly=0x1021)
```

**Message types:**

| Type | Value | Direction | Payload |
|------|-------|-----------|---------|
| MOTOR_COMMAND | 0x01 | Pi→ESP32 | 5×int16 LE (M1..M5, -1000..+1000) + 1 FLAGS byte (must be ≤0x03) |
| ESTOP | 0x02 | Pi→ESP32 | empty (length=0); latches E-STOP on ESP32 |
| ACK | 0x81 | ESP32→Pi | STATUS(1) + ECHO_SEQ(2); total 5 bytes region but LENGTH=3 |
| STATUS | 0x82 | ESP32→Pi | FAILSAFE(1) + ESTOP(1) + M1..M5 int16(10); LENGTH=14, 12 meaningful |

**Firmware STATUS-length quirk (measured on the wire):** the ESP32's `sendStatus()` writes
`AA55 VERSION TYPE LENGTH=14 | SEQ(2) FAILSAFE(1) ESTOP(1) M1..M5(10) | CRC(2)` — the 2-byte sequence
is counted INSIDE LENGTH, so a STATUS frame is **21 bytes (5 + LENGTH + 2)**, not 7 + LENGTH + 2.
`parse_frame()` special-cases `MSG_STATUS` (head = 5) to match; other frame types are 7 + LENGTH + 2.
`decode_status()` parses the full 14-byte payload (`sequence`, `failsafe`, `estop`, `motors`).

**Throttle modes (2026-09-20):** UI throttle buttons map to limits — **soft 40%, medium 70%,
high 100%** (`THROTTLE_MODES`). The selected mode caps the ramp ceiling; press-and-hold keys climb
the setpoint by **fixed `THROTTLE_STEP = 0.10` per update** (release eases back down the same steps).
Ramp logic lives in `ThrusterManager._ramp_setpoints()`.

**ACK status codes:**
0=OK, 1=CRC error, 2=invalid payload length, 3=invalid motor value, 4=invalid flags,
5=old sequence, 6=unknown packet type, 7=E-STOP active.

**Sequence rules:**
- ESP32 `isNewerSequence` is int16-wraparound; only accepts strictly newer within ±32768.
- Fresh ESP32 starts at `lastSequence=0`; Pi must start at seq ≥ 1.
- Pi seeds from first STATUS: `pi_seq = esp32_lastSeq + 1`.
- Resyncs if `((pi_seq - esp_seq) & 0xFFFF) > 0x8000` (ESP32 restarted).
- ESTOP does NOT advance ESP32 lastSequence.

**E-STOP:**
- Pi sends TYPE=0x02 (ESTOP message), ESP32 latches `estopActive=true` forever.
- NO protocol-level re-arm in firmware — recovery requires ESP32 reset.
- Pi holds a safety gate (zeros) until ACK status 0 or STATUS shows `failsafe=0 && estop=0`.

**50Hz command stream:**
- Background daemon thread sends MOTOR_COMMAND at `command_rate_hz` (default 50).
- ESP32 250ms watchdog: no accepted command → ESCs stop automatically (failsafe).
- Pi link timeout: `heartbeat_timeout_ms` (default 1500ms, STATUS cadence 500ms).

**To get real values:** `thrusters.provider: esp32` and `thrusters.esp32.serial_port` point at the
Pi's on-board `/dev/ttyAMA0` (UART0, GPIO14/15). The app tolerates a missing device (reports
disconnected, retries reconnects, never crashes). The ESP32 uses its **UART0** pins — **RX0 =
GPIO3**, **TX0 = GPIO1** — so the Pi TX (GPIO14) must feed GPIO3 and the Pi RX (GPIO15) must come
from GPIO1, with common GND. GPIO1/GPIO3 also carry the USB-serial/programming interface: during
normal runtime UART0 is dedicated to the Pi (keep debug `Serial.println()` off UART0), and while
flashing over USB, disconnect the Pi UART wires if they interfere. Do not wire GPIO16/GPIO17
(UART2) or GPIO21/GPIO22 for this link.

### ESP32 GPIO Mapping (wiring, must not change)

| Thruster | ESP32 GPIO | Role |
|----------|:----------:|------|
| M1 | 25 | Front vertical |
| M2 | 33 | Middle right horizontal |
| M3 | 32 | Middle left horizontal |
| M4 | 27 | Back right vertical |
| M5 | 26 | Back left vertical |

ESC PWM: 50Hz, 1000-2000μs, 3D mode off.

---

## 8. Raspberry Pi Auto-Start (systemd) — Step 6

The ROV controller auto-starts on boot and restarts if it crashes.

- **Service name:** `rov-controller.service`
- **Deployment path:** `/home/riazafridi/Downloads/Hydrion-Spectra-main`
- **Unit template:** `systemd/rov-controller.service` (rendered by `scripts/start.sh`)
- **GUI environment:** `/etc/rov-controller.env` (DISPLAY/XAUTHORITY/QT_QPA_PLATFORM=xcb)
- **Behaviour:** starts after `graphical.target`; `Restart=on-failure`, `RestartSec=5`

### Commands

| Action | Command |
|--------|---------|
| Install + start | `bash scripts/start.sh` |
| Start / Stop / Restart | `sudo systemctl start/stop/restart rov-controller.service` |
| Enable / Disable at boot | `sudo systemctl enable/disable rov-controller.service` |
| Status | `systemctl status rov-controller.service` |
| Live logs | `journalctl -u rov-controller.service -f` |

---

## 9. Progress Log

### Current state

> **2026-09-20 bench session:** a real ESP32 is attached to the Pi's on-board UART (**`/dev/ttyAMA0`
> = UART0, GPIO14 TXD0 / GPIO15 RXD0**, 460800 baud). Everything below verified live:
>
> - ESP32→Pi **RX works**: 21-byte STATUS(0x82) frames every 500 ms are decoded cleanly
>   (link_alive=True, `rx_frames` increment, 0 CRC errors). CRC verified `0xDE64` matches
>   CRC-16/CCITT-FALSE over the full frame.
> - **Fixes applied this session:**
>   - `Esp32ThrusterProvider.initialize()` was never called from `ThrusterManager.initialize()` —
>     the 50Hz UART thread never ran, so nothing was ever transmitted (`tx_seq` stayed `None`).
>   - STATUS parser now handles the firmware's 21-byte framing (seq counted inside LENGTH).
>   - Throttle modes soft/medium/high (40/70/100%) + 10% press-and-hold ramp.
> - **Pi→ESP32 TX is electrically confirmed on the Pi side** (GPIO14 muxed `TXD0`, `out_waiting` drains)
>   and the emitted frames are **byte-perfect for the firmware** — the firmware's `readUART`/`processPacket`
>   state machine was re-implemented in Python and DUBO's real builders feed it an `ACK_OK` on every path.
> - **Open item (hardware):** the ESP32 has never produced a single ACK(0x81) and its STATUS always
>   shows `failsafe=1` / `lastSequence=0` since boot — i.e. it is receiving **nothing**. A 30-second
>   loopback short of GPIO14↔GPIO15 definitively splits "Pi TX dead" vs "ESP RX/firmware side". Verify
>   the physical GPIO14 (pin 8) → ESP32 GPIO3 (RX0) connection first (firmware moved to ESP32 UART0
>   GPIO1/GPIO3; **do not** use GPIO16/GPIO17).

| # | Step | Status |
|---|------|--------|
| 0 | GUI + camera + HUD | done |
| 1 | Config + Logger | done |
| 2 | ServiceManager / Application | done |
| 3 | Sensors / telemetry pipeline | done |
| 4 | Network / telemetry link | done |
| 5 | Thrusters / motion control | done |
| P1 | 5-thruster hardware model | done |
| 6 | systemd auto-start | done |
| P2A | ESP32 link (Pi side) | done |
| P2B | ESP32 firmware (Pi-side protocol matched) | done |
| P2B+ | Real-time UI upgrade (thrusters/motion/E-STOP/diagnostics) | done |
| P2C | Throttle modes + press-and-hold ramp | done |
| P2D | Live ESP32 on the bench: RX/STATUS decoded, TX pending | partly – RX done, **TX open** |

### Change record (most recent first)

- **2026-09-22 — Verification pass (98 tests passing, no refactor)**
  - Ran a full verification pass of the completed controller/UI work (no code changes unless a real bug
    was found). Verified end-to-end through the live window + controller + thruster loop: every key
    binding (W/S/A/D/Q/E/R/F, Shift boost, Backspace E-STOP), key-release-to-zero, the exact 10 % ramp
    trajectories and ceilings (SOFT 0.1→0.4 stop, MEDIUM 0.1→0.7 stop, HARD 0.1→1.0 stop), release
    ease-back, mixer (M1/M4/M5=heave, M2=surge+yaw, M3=surge−yaw), no sway/pitch/roll motor output,
    E-STOP→zero, RE-ARM→zero, and UART config (`/dev/ttyAMA0`, 460800, 8N1, no flow control).
  - **Bug found & fixed (one line):** `ThrusterManager.emergency_stop()` reset `_last = None`, so the
    first post-E-STOP update tick hit the seed guard and left the previous setpoint (`M2=1.0`) displayed
    for one 100 ms tick instead of zeroing immediately. Removed the unnecessary `_last = None` (dt is
    already clamped to ≤0.25 s); E-STOP now forces every setpoint to zero on the immediate next tick.
  - Static checks: no duplicate power state (single owner `ThrusterManager._throttle_mode`), no
    duplicate keyboard handler (single `eventFilter → _handle_key` path), no GUI-blocking `sleep()`
    (only the ESP32 provider's background thread), no unsafe non-zero startup command (ESP32 zero-gate
    `_gate` + thread-sent ESTOP), no direct serial access from UI/controller (serial confined to
    `uart_transport.py`).
  - `uart_transport.py` was not rewritten: the diff vs. the previous baseline is only the already-documented
    STATUS-frame length quirk fix (LENGTH includes the 2-byte sequence) + `decode_status` re-alignment;
    header/version/CRC and the existing binary protocol are unchanged.
  - Remaining (not claimed): physical Pi→ESP32 handshake (ACK 0x81, STATUS `failsafe=0`) after rewiring
    to GPIO3/GPIO1, motors OFF throughout.

- **2026-09-22 — ESP32 UART moved to UART0 (GPIO3/GPIO1) + keyboard/power verified (98 tests passing)**
  - **Hardware wiring change:** the ESP32 link now uses **UART0** pins — **RX0 = GPIO3**, **TX0 = GPIO1** —
    instead of UART2 (RX2/GPIO16, TX2/GPIO17). `configs/thrusters.yaml` now documents `esp32.rx_pin: 3` /
    `esp32.tx_pin: 1` (metadata; the Pi never drives these pins). Pi side unchanged: `/dev/ttyAMA0`
    (GPIO14 TXD0 → ESP32 GPIO3, GPIO15 RXD0 ← ESP32 GPIO1, common GND) at 460800 baud 8N1.
  - **UART0/USB note:** GPIO1/GPIO3 double as the ESP32 USB-serial interface — during runtime UART0 is
    dedicated to the Pi (keep debug `Serial.println()` off UART0); unplug the Pi UART wires while
    flashing over USB. Do not use GPIO16/17 or GPIO21/22.
  - **Keyboard verified (no logic regression found):** the full W/S/A/D/Q/E/R/F + Shift(boost) +
    Backspace(E-STOP) keymap (14 bindings) is intact in `ui/main_window.py` and driven through
    `ControllerModule`. Added `tests/test_controller_input.py` proving every binding, key-release →
    safe-zero, Shift boost and Backspace E-STOP. Hardened the GUI: `StrongFocus` + `focusOutEvent`/
    `_clear_pressed_keys()` so a held key can never stick after the window loses focus.
  - **Power modes verified:** SOFT 40 % / MEDIUM 70 % / HIGH 100 % ceilings with the fixed 10 %
    press-and-hold ramp (`THROTTLE_MODES`, `THROTTLE_STEP = 0.10`, `_ramp_setpoints`) — new
    `ThrottleRampTest` locks the exact 0→10→…→ceiling trajectories and release ease-back. Dashboard
    THROTTLE panel now shows `MODE: SOFT 40% MAX | POWER: 20%` live.
  - **Config tests:** `UartConfigTest` asserts `/dev/ttyAMA0`, 460800 baud, `rx_pin=3`/`tx_pin=1` and
    the unchanged M1..M5 GPIO mapping (25/33/32/27/26).
  - **Hardware left to verify (not claimed):** the physical Pi↔ESP32 UART handshake — ACK(0x81) receipt
    and STATUS `failsafe=0` — after re-wiring to GPIO3/GPIO1. Motors must stay powered OFF during this
    check (all setpoints start/remain zero until the ESP32 ACKs a zero command).

- **2026-09-20 — Live bench session: real ESP32 link characterized + fixes (74 tests passing)**
  - **Serial target:** moved from `/dev/ttyUSB0` to the Pi's on-board **`/dev/ttyAMA0`** (UART0,
    GPIO14 TXD0 / GPIO15 RXD0) in `configs/thrusters.yaml`; baud 460800. `pinctrl` + `dtparam=uart0=on`
    confirmed the mux; no other process holds the port.
  - **Bug fix:** `ThrusterManager.initialize()` now calls `provider.initialize()` when present
    (`modules/thrusters/thruster_manager.py`). Without it the `Esp32ThrusterProvider.initialize()`
    handshake (probe timeout, 50Hz thread start, `_seq_seeded` handling) never ran — earlier
    "configured" but silently-empty TX path.
  - **Protocol fix: 21-byte STATUS frames.** The firmware's `sendStatus()` counts the 2-byte sequence
    inside `LENGTH=14`, so a STATUS frame is `5 + LENGTH + 2 = 21` bytes, not `7 + LENGTH + 2 = 23`.
    `parse_frame()` now special-cases `MSG_STATUS` (head = 5); `decode_status()` parses the 14-byte
    payload (`sequence`, `failsafe`, `estop`, `motors`). Tests updated: `build_status()` now emits the
    exact firmware wire format (21 bytes).
  - **Throttle modes:** `THROTTLE_MODES = {"soft": 0.40, "medium": 0.70, "high": 1.0}` in
    `thruster_manager.py`, `THROTTLE_STEP = 0.10` fixed-step ramp (hold to climb, release to ease).
    UI (`ui/main_window.py`): buttons soft/medium/high, initial "high". Renamed old `hard`→`high`;
    `0.65`→`0.70` test fixture.
  - **On-wire verification:** received STATUS CRC `0xDE64` matched `crc16_ccitt` exactly; 13 clean
    STATUS frames decoded with 0 CRC/resync errors during a W-key press-hold harness.
  - **Software cross-check of TX:** the firmware's exact `readUART()` + `processPacket()` was
    re-implemented in Python (`/tmp/opencode/fw_sim_check.py`) and fed DUBO's real `build_motor_command`/
    `build_estop` frames — every path returns `ACK_OK` (seq gate, CRC, length all match). Combined with
    `out_waiting` draining, the Pi transmits valid bytes.
  - **Open:** ESP32 never ACKs and reports `lastSequence=0 / failsafe=1` since boot ⇒ its RX
    (UART0 RX0 / GPIO3) is not receiving. Physical check (GPIO14→GPIO3 wiring, then GPIO14↔GPIO15
    loopback) is the only remaining step before motors can be driven. (ESP32 now uses UART0
    GPIO1/GPIO3 — see §7 wiring.)

- **2026-09-14 — Phase 2 Part B+ (Step 4): Real-time 5-thruster control + UI upgrade**
  - **UART protocol rewrite:** `modules/thrusters/uart_transport.py` completely rewritten to match
    the ESP32 firmware byte-for-byte:
    - Message types: MOTOR_COMMAND=0x01, ESTOP=0x02, ACK=0x81, STATUS=0x82 (was: ACK=0x02,
      STATUS=0x04, HEARTBEAT=0x05, flag-bit ESTOP — all wrong).
    - Sequence is uint16 little-endian at bytes 5-6 (was: 1-byte at byte 5).
    - CRC covers the ENTIRE frame including the AA 55 header (was: body-only, header excluded).
    - ESTOP is a message TYPE (was: FLAGS bit0).
    - ACK: LENGTH=3 declared, 5-byte payload region (status + echo seq). STATUS: LENGTH=14,
      12 meaningful bytes (failsafe, estop, M1..M5 int16).
    - Motor command payload = 5×int16 + FLAGS byte (11 bytes).
    - `build_estop()`, `decode_ack()`, `decode_status()` added. MAX_FRAME_SIZE = 64.
    - CRC-16/CCITT-FALSE validated against standard check value (0x29B1 for "123456789").
  - **Esp32ThrusterProvider rewrite:** `modules/thrusters/thruster_manager.py`:
    - 50Hz background daemon thread (non-blocking pump + rate-limited send).
    - threading.RLock for shared state.
    - Safety gate: zeros until ESP32 ACKs a zero command (ACK status 0) or STATUS shows clear.
    - Latched E-STOP: sends ESTOP frames at 50Hz while active; RE-ARM sends zeros until accepted.
    - Sequence: starts at 1 (fresh ESP32 at 0 rejects equal), seeds from STATUS
      (`pi_seq = esp_seq + 1`), resyncs when gap > 0x8000.
    - Link alive timeout: 1500ms (STATUS cadence 500ms).
    - Reconnect-in-zero: never restores previous non-zero thrust.
    - `stop()` sends final zeroed command, then closes transport.
    - `status()` dict: all stats for UI (tx/rx/crc/resync/dropped/ack-latency/esp32-seq/motors/etc).
  - **ThrusterManager:** failsafe zero-clamp (esp32 provider + not link_alive → zero), get_provider_status(),
    stop() → provider.stop().
  - **configs/thrusters.yaml:** esp32 block with baudrate 460800, command_rate_hz 50,
    heartbeat_timeout_ms 1500, reconnect_interval_s 1.0. Provider selection documented.
  - **GUI upgrade (ui/main_window.py):**
    - App bar: ROV / ESP32 / MODE / DEPTH / BATT / E-STOP / LINK / UP pills.
    - Dashboard: thruster meters (M1-M5 bipolar bars with %), motion bars (SURGE/YAW/HEAVE).
    - Dock: E-STOP + RE-ARM buttons (alongside ARMED/REC/SNAP/LIGHTS).
    - Toast notifications for E-STOP / RE-ARM.
    - 100ms fast refresh timer for live panels.
    - Kill key (Backspace) now triggers E-STOP.
    - E-STOP blocks ARM toggle until RE-ARM.
  - **UI diagnostics (ui/sections.py):**
    - OperationsPage: THRUSTERS panel with M1-M5 live %, E-STOP/FAILSAFE state, controls.
    - DiagnosticsPage: ESP32 UART LINK panel with STATE/PROVIDER/TX/RX/CRC/RESYNC/MISSING ACK/
      ACK LATENCY/ESP32 SEQ/ESP32 MOTORS/LAST ERROR + E-STOP/RE-ARM buttons.
    - SensorsPage: ESP32 added to health pills.
  - **HUD (ui/hud.py):** optional status chips (mode/ESP32/E-STOP) drawn into the video frame.
    Configurable via `hud.extra.enabled`.
  - **core/application.py:** thrusters passed to MainWindow and wired into UI context.
  - **ui/control_pad.py:** added `reset()` method for E-STOP.
  - **Tests:** `tests/test_uart_transport.py` rewritten (72 tests total, all passing):
    - Protocol framing: header CRC-including, uint16 seq, motor command layout.
    - ACK/STATUS parsing: firmware-style payloads, statuses 0-7, STATUS 14-byte region.
    - Transport: open/send/pump/resync/partial-frame/close.
    - Provider: gate/rearm/estop/sequence-sync/stale-link/disconnect/stop-zeroes.
    - Manager: estop-reach/failsafe-zero/esp32-without-hardware.
  - **GUI smoke test:** Full Application boots clean offscreen with simulated provider.
  - **ESP32 provider lifecycle:** FailingSerial test: thread starts, sends frames, counts drops,
    status reports correctly, clean shutdown.

- **2026-09-13 — Phase 2 Part A: ESP32 UART link (Pi side)**
  - Initial implementation of `uart_transport.py` + `Esp32ThrusterProvider` (later rewritten in
    Step 4 above to match the actual firmware protocol).

- **2026-09-13 — Step 6: systemd auto-start**
  - Unit template + scripts for auto-start on boot. Verified lifecycle on the Pi.

- **2026-08-07 — Phase 1: 5-thruster hardware model**
  - ThrusterId enum, pure mix() function, ThrusterProvider ABC, SimulatedThrusterProvider.

- **2026-08-06 — UI redesign + Steps 2-5**
  - Dark Ocean glassmorphism shell, sensor pipeline, telemetry, controller, motion control.

---

## 10. Testing

### Run all tests

```bash
cd /path/to/dubo
source .venv/bin/activate
python -m unittest discover -s tests -v
```

### What's tested (74 tests)

**test_thruster_mixer.py (28 tests):**
- 5-thruster mixer: neutral, forward/backward, yaw left/right, up/down, combined axes.
- Saturation clamping, unsupported axes (sway/pitch/roll), direction flip, five-output invariant.
- ThrusterManager: forward setpoints, emergency stop, clear stop, fallback provider, GPIO config.
- Throttle modes: soft 40%, medium 70%, high 100%; 10% press-and-hold ramp.

**test_uart_transport.py (46 tests):**
- Protocol framing: header layout, uint16 sequence, motor command 11-byte payload, CRC covers header.
- CRC validation: CCITT-FALSE check value (0x29B1), bad CRC rejection, header-modified CRC.
- Roundtrip: build → parse, parse incomplete frame, parse oversized length.
- ACK/STATUS: firmware-style payloads, all 8 status codes, STATUS 21-byte wire format (firmware
  LENGTH=14 counts the 2-byte sequence), full 14-byte payload decode (seq/failsafe/estop/motors).
- Transport: open/close/rate-limited reconnect, send/write/drop, pump/resync through garbage,
  partial frame buffering across reads.
- Provider: safety gate (zeros until ACK), ESTOP message type (not flag), rearm handshake,
  sequence increment, stale link → failsafe, disconnect → no crash, stop zeroed, thread lifecycle.
- Manager: ESP32 without hardware, estop reaches provider, failsafe zeroes outputs.

### Headless smoke test

```bash
QT_QPA_PLATFORM=offscreen .venv/bin/python - <<'EOF'
from core.application import Application
from ui.theme import apply_theme
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer
class SmokeApp(Application):
    def run(self):
        self.initialize()
        self.qt_app = QApplication(self.argv)
        apply_theme(self.qt_app)
        self.main_window = self.create_window()
        self.main_window.show()
        QTimer.singleShot(500,                                                                                                                                                                                                                                                                                                                                                       
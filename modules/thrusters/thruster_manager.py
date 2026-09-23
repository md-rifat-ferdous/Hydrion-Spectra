"""5-thruster thrust model for the DUBO ROV (Phase 1 + Phase 2).

Actual hardware layout (ESP32 GPIO refer to the low-level ESC controller;
the Raspberry Pi never drives these pins — metadata only):

    M1  Front Vertical            -> ESP32 GPIO 25
    M2  Middle Right Horizontal   -> ESP32 GPIO 33
    M3  Middle Left Horizontal    -> ESP32 GPIO 32
    M4  Back Right Vertical       -> ESP32 GPIO 27
    M5  Back Left Vertical        -> ESP32 GPIO 26

Supported motion (Phase 1): surge, yaw, heave.
Sway, pitch and roll are NOT mixed into the motor output (no sway thruster;
pitch/roll stabilization is future work).

Phase 2: Esp32ThrusterProvider pushes the already-mixed M1..M5 setpoints to
an ESP32 ESC controller over UART (never surge/yaw/heave). The provider stays
behind the ThrusterProvider abstraction; ``configs/thrusters.yaml`` keeps
``provider: simulated`` as the default. A background thread maintains a stable
command stream (default 50 Hz) so the ESP32 watchdog is fed without blocking
the Qt event loop. The UART protocol matches the ESP32 firmware exactly (see
``uart_transport.py``): MOTOR_COMMAND 0x01 / ESTOP 0x02 / ACK 0x81 / STATUS
0x82, uint16 sequence, CRC over the whole frame including the header.

Safety model:
  - all motion starts at zero (gate until the ESP32 ACKs a zero command);
  - E-STOP is a latched ESTOP message that halts the ESP32 immediately;
  - re-arm requires an explicit action and an acknowledged zero command;
  - reconnect never restores previous non-zero thrust (gate re-asserts);
  - motor values are clamped to [-1, 1] at every boundary;
  - UART I/O stays on the provider thread; nothing blocks the GUI.
"""

import logging
import os
import time
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import IntEnum, auto

from core.base_module import BaseModule
from modules.controller.controller import MotionState
from modules.thrusters.uart_transport import (
    ACK_ESTOP,
    ACK_NAMES,
    ACK_OK,
    MSG_ACK,
    MSG_ESTOP,
    MSG_STATUS,
    UartTransport,
    build_estop,
    build_motor_command,
    decode_ack,
    decode_status,
    motor_values_to_int16,
)


class ThrusterId(IntEnum):
    """Stable identifiers for the five physical thrusters."""
    M1_FRONT_VERTICAL = auto()
    M2_MIDDLE_RIGHT_HORIZONTAL = auto()
    M3_MIDDLE_LEFT_HORIZONTAL = auto()
    M4_BACK_RIGHT_VERTICAL = auto()
    M5_BACK_LEFT_VERTICAL = auto()


DEFAULT_GPIO = {
    ThrusterId.M1_FRONT_VERTICAL: 25,
    ThrusterId.M2_MIDDLE_RIGHT_HORIZONTAL: 33,
    ThrusterId.M3_MIDDLE_LEFT_HORIZONTAL: 32,
    ThrusterId.M4_BACK_RIGHT_VERTICAL: 27,
    ThrusterId.M5_BACK_LEFT_VERTICAL: 26,
}

SUPPORTED_AXES = ("surge", "yaw", "heave")

# Dashboard throttle limiter: the max fraction of command each mode allows.
#   soft   = 40 % of the commanded thrust
#   medium = 70 %
#   high   = 100 % (full)
THROTTLE_MODES = {"soft": 0.40, "medium": 0.70, "high": 1.0}
DEFAULT_THROTTLE_MODE = "soft"
DEFAULT_THROTTLE_RAMP = 1.0  # fraction of full range per second
THROTTLE_STEP = 0.10         # fixed 10 % climb/ease step per control tick

# Focused per-tick pipeline trace: KEY INPUT / MIXER / POWER-RAMP / APPLIED.
# Off by default; set HYDRION_TRACE_PIPELINE=1 to watch one key press flow
# through the whole chain (e.g.  W ->  surge=1.00 -> M2/M3 = +1.00 in the
# mixer -> +0.10, +0.20, ... ramped setpoints). Remove the flag and the
# ``_trace_pipeline`` method once the investigation is done.
TRACE_PIPELINE = os.environ.get("HYDRION_TRACE_PIPELINE", "") == "1"

_ROLES = {
    ThrusterId.M1_FRONT_VERTICAL: "FRONT VERTICAL",
    ThrusterId.M2_MIDDLE_RIGHT_HORIZONTAL: "RIGHT HORIZONTAL",
    ThrusterId.M3_MIDDLE_LEFT_HORIZONTAL: "LEFT HORIZONTAL",
    ThrusterId.M4_BACK_RIGHT_VERTICAL: "BACK RIGHT VERTICAL",
    ThrusterId.M5_BACK_LEFT_VERTICAL: "BACK LEFT VERTICAL",
}


def role_of(thruster_id):
    return _ROLES.get(thruster_id, "?")


def motor_values(setpoints):
    """Return M1..M5 in ThrusterId order as a list of clamped floats."""
    return [max(-1.0, min(1.0, float(setpoints.get(t, 0.0)))) for t in ThrusterId]


def mix(motion=None, surge=0.0, yaw=0.0, heave=0.0, directions=None):
    """Pure 5-thruster mixer.

    Returns exactly five motor outputs keyed by ``ThrusterId``, each clamped
    to [-1.0, 1.0]. Only the supported axes (surge/yaw/heave) are consumed;
    sway/pitch/roll inputs can never produce a motor command.

    The optional ``directions`` mapping flips the physical sign of a motor
    (configurable per-motor, default +1) so reversed mounts are corrected at
    config time, not by guessing in code.
    """
    if motion is not None:
        surge = float(getattr(motion, "surge", surge))
        yaw = float(getattr(motion, "yaw", yaw))
        heave = float(getattr(motion, "heave", heave))
    raw = (
        (ThrusterId.M1_FRONT_VERTICAL, heave),
        (ThrusterId.M2_MIDDLE_RIGHT_HORIZONTAL, surge + yaw),
        (ThrusterId.M3_MIDDLE_LEFT_HORIZONTAL, surge - yaw),
        (ThrusterId.M4_BACK_RIGHT_VERTICAL, heave),
        (ThrusterId.M5_BACK_LEFT_VERTICAL, heave),
    )
    output = {}
    for thruster, value in raw:
        direction = (directions or {}).get(thruster, 1)
        output[thruster] = max(-1.0, min(1.0, value * direction))
    return output


@dataclass
class ThrusterConfig:
    """Per-motor hardware configuration loaded from ``configs/thrusters.yaml``.

    ``directions``: physical motor sign (+1/-1) — verified on the real ROV.
    ``gpio``: ESP32 pin (metadata only; the Pi never drives these pins).
    """

    directions: dict
    gpio: dict

    @classmethod
    def from_dict(cls, data):
        data = data or {}
        directions = {}
        gpio = {}
        for thruster in ThrusterId:
            entry = data.get(thruster.name) or {}
            directions[thruster] = int(entry.get("direction", 1))
            gpio[thruster] = int(entry.get("gpio", DEFAULT_GPIO.get(thruster, 0)))
        return cls(directions=directions, gpio=gpio)


class ThrusterProvider(ABC):
    """Interface every thruster provider must match.

    SimulatedThrusterProvider runs by default; Esp32ThrusterProvider drives
    the real hardware. ControllerModule and the UI only ever talk to
    ThrusterManager, so swapping the provider never touches them.
    """

    @abstractmethod
    def setpoints(self, motion):
        """Return {ThrusterId: float} motor commands for a MotionState."""

    @abstractmethod
    def apply(self, motion, dt, setpoints=None):
        """Apply the motion to the vehicle (simulated now, hardware later).

        ``setpoints`` carries the already-mixed 5-motor commands computed by
        ``setpoints()`` so hardware providers can transmit them directly.
        """


class SimulatedThrusterProvider(ThrusterProvider):
    """Converts motion setpoints into 5-motor commands and applies the
    supported motion axes (surge/heave/yaw) to the simulated vehicle state."""

    def __init__(self, config=None, sensors=None):
        self.config = config or {}
        self.sensors = sensors
        self.speed_mps = self.config.get("speed_mps", 0.8)
        self.turn_rate_degps = self.config.get("turn_rate_degps", 60.0)
        self.thruster_config = ThrusterConfig.from_dict(self.config.get("motors"))

    def setpoints(self, motion):
        return mix(motion, directions=self.thruster_config.directions)

    def apply(self, motion, dt, setpoints=None):
        if self.sensors is None:
            return
        speed = self.speed_mps * (2.0 if motion.boost else 1.0)
        self.sensors.apply_motion(
            surge=motion.surge * speed,
            sway=0.0,
            heave=motion.heave * speed,
            yaw_rate=motion.yaw * self.turn_rate_degps,
            dt=dt,
        )


class Esp32ThrusterProvider(ThrusterProvider):
    """Drives the ESP32 ESC controller over UART (Phase 2, real hardware).

    Receives the five **already-mixed** motor setpoints from ThrusterManager
    and pushes them to the ESP32 as MOTOR_COMMAND frames (normalized [-1,1],
    re-clamped before transmit — never surge/yaw/heave). A daemon thread keeps
    the command stream at ``command_rate_hz`` (default 50 Hz) so the ESP32's
    250 ms watchdog is always fed, without blocking the Qt event loop. The
    ESP32 replies with ACK / STATUS frames which the provider parses and
    exposes through :meth:`status`.

    Safety:
      - Missing hardware is never an error: the provider stays disconnected,
        retries reconnects and reports diagnostics via ``status()``.
      - On every (re)connect the provider re-asserts its zero-gate: only a
        zeroed MOTOR_COMMAND acknowledged by the ESP32 (ACK status 0, or a
        STATUS showing the ESP32 failsafe clear) lifts the gate. Previous
        non-zero thrust is never restored automatically.
      - E-STOP is a latched ESTOP message (firmware halts all ESCs). Re-arm
        is explicit: a zeroed command is streamed until the ESP32 accepts it
        again (after an ESP32 reset), then normal motion resumes.
      - The 16-bit sequence is kept strictly ahead of the ESP32's last
        acknowledged sequence and re-synced if the ESP32 restarts.
    """

    def __init__(self, config=None, sensors=None, transport=None):
        self.config = config or {}
        self.sensors = sensors
        self.logger = logging.getLogger("Thrusters.Esp32")
        self.thruster_config = ThrusterConfig.from_dict(self.config.get("motors"))
        self.esp32 = self.config.get("esp32") or {}
        if transport is None:
            transport = UartTransport(self.esp32, logger=self.logger)
        self.transport = transport
        self._lock = threading.RLock()
        # Start at seq 1: a freshly booted ESP32 is at lastSequence 0 and only
        # accepts strictly-newer sequences, so seq 0 would be rejected forever.
        self._seq = 1
        self._seq_seeded = False
        self._estop = False
        self._rearm = False          # re-arming: only zeroed commands allowed
        self._gate = True            # safety gate: zeros until ESP32 ACKs
        self._pending = [0.0] * 5
        self._last_send = 0.0
        self._last_warn = 0.0
        self._failsafe = True        # Pi-side failsafe (command not confirmed)
        self._failsafe_logged = None
        self._command_interval = 1.0 / max(1, float(self.esp32.get("command_rate_hz", 50)))
        self._link_timeout = (
            float(self.esp32.get("heartbeat_timeout_ms", 1500)) / 1000.0
        )
        # Link / ack state surfaced via status() for the diagnostics UI.
        self._last_rx_at = None
        self._last_rx_info = None
        self._last_tx_seq = None
        self._last_tx_at = None
        self._ack_pending = None      # (sequence, sent_monotonic)
        self._last_ack_seq = None
        self._last_ack_status = None
        self._last_ack_latency_s = None
        self._missing_acks = 0
        self._link_stale_logged = False
        # ESP32-reported state (from STATUS frames).
        self._esp32_seq = None
        self._esp32_failsafe = None
        self._esp32_estop = None
        self._esp32_motors = None
        # Background command stream.
        self._running = False
        self._thread = None

    # -- lifecycle ---------------------------------------------------------

    def initialize(self):
        """Best-effort open + start the 50 Hz command thread."""
        if self.transport is not None:
            self.transport.open()
        if self.transport is not None and self._thread is None:
            self._running = True
            self._thread = threading.Thread(
                target=self._run_loop, daemon=True, name="esp32-uart"
            )
            self._thread.start()
            self.logger.info(
                "ESP32 command stream: %.0f Hz", 1.0 / max(1e-6, self._command_interval)
            )
        return self.transport is not None

    def is_connected(self):
        return self.transport is not None and self.transport.is_open()

    def is_link_alive(self):
        if not self.is_connected() or self._last_rx_at is None:
            return False
        return (time.time() - self._last_rx_at) < self._link_timeout

    def setpoints(self, motion):
        return mix(motion, directions=self.thruster_config.directions)

    def apply(self, motion, dt, setpoints=None):
        """Apply the already-mixed 5-motor setpoints to the ESP32."""
        if setpoints is None:
            setpoints = self.setpoints(motion)
        values = [max(-1.0, min(1.0, float(setpoints[t]))) for t in ThrusterId]
        with self._lock:
            self._pending = values
            self._pump()
            self._transmit_due()

    def notify_estop(self, active):
        """Manager e-stop hook: latch/clear ESTOP and push a frame immediately."""
        with self._lock:
            self._estop = bool(active)
            if active:
                self._rearm = False
                self._gate = True
                self._failsafe = True
                self._send_frame(build_estop(self._seq))
                self.logger.warning("ESP32 provider: E-STOP ASSERTED (ESTOP frame sent)")
            else:
                self._rearm = bool(active) is False
                self._log_failsafe(False)  # pi clamp released by the manager below
                self._send_frame(build_motor_command(self._seq, [0.0] * 5))
                self.logger.warning(
                    "ESP32 provider: E-STOP CLEARED -> re-arm handshake (zeros until ESP32 ACKs)"
                )

    def estop_keepalive(self, dt, setpoints):
        """Compatibility helper while e-stopped (thread also re-issues frames)."""
        with self._lock:
            self._pump()
            self._transmit_due()

    def stop(self):
        """Send a final zeroed command and close the transport cleanly."""
        if self._thread is not None:
            self._running = False
            self._thread.join(timeout=1.5)
            self._thread = None
        with self._lock:
            if self.transport is not None:
                if self.transport.is_open():
                    frame = build_motor_command(self._seq, [0.0] * 5)
                    try:
                        self.transport.send(frame)
                    except Exception:
                        pass
                self.transport.close()
        self.logger.info("ESP32 provider stopped (zeroed command sent)")

    # -- background thread -------------------------------------------------

    def _run_loop(self):
        while self._running:
            tick0 = time.monotonic()
            try:
                with self._lock:
                    self._pump()
                    self._transmit_due()
                self._check_link()
            except Exception:  # pragma: no cover - never let the loop die
                with self._lock:
                    self.logger.exception("ESP32 UART loop error")
            elapsed = time.monotonic() - tick0
            time.sleep(max(0.0, self._command_interval - elapsed))

    # -- sending -----------------------------------------------------------

    def _transmit_due(self):
        """Rate-limited transmit of the current command (assumes lock held)."""
        now = time.monotonic()
        if now - self._last_send < self._command_interval:
            return
        self._last_send = now
        if self._estop:
            frame = build_estop(self._seq)
        elif self._gate or self._rearm:
            frame = build_motor_command(self._seq, [0.0] * 5)
        else:
            frame = build_motor_command(self._seq, self._pending)
        self._send_frame(frame)

    def _send_frame(self, frame):
        """Build-frame sequence bookkeeping + transmit (assumes lock held)."""
        if self.transport is None:
            return
        if not self.transport.is_open():
            self.transport.open()
        if not self.transport.is_open():
            self.transport.stats["dropped_frames"] += 1
            self._log_drop()
            return
        try:
            seq = struct_seq(frame)
        except Exception:
            seq = self._seq
        if self.transport.send(frame):
            self._last_tx_seq = seq
            self._last_tx_at = time.time()
            self._last_send = time.monotonic()
            self._ack_pending = (seq, time.time())
            self._seq = (self._seq + 1) & 0xFFFF

    def _log_drop(self):
        now = time.monotonic()
        if now - self._last_warn < 5.0:
            return
        self._last_warn = now
        error = (
            self.transport.stats.get("last_error")
            if self.transport is not None
            else "transport unavailable"
        )
        self.logger.warning("ESP32 not connected (%s); motor frames dropped", error)

    # -- receiving ---------------------------------------------------------

    def _pump(self):
        if self.transport is None or not self.transport.is_open():
            return
        self.transport.pump()
        now = time.time()
        for packet in self.transport.read_packets():
            self._last_rx_at = now
            if self._link_stale_logged:
                self._link_stale_logged = False
                self.logger.info("ESP32 UART link recovered")
            self._last_rx_info = {
                "mtype": packet.mtype,
                "sequence": packet.sequence,
                "ts": now,
            }
            if packet.mtype == MSG_ACK:
                self._handle_ack(packet, now)
            elif packet.mtype == MSG_STATUS:
                self._handle_status(packet, now)

    def _handle_ack(self, packet, now):
        try:
            status, echo = decode_ack(packet.payload)
        except ValueError:
            self._missing_acks += 1
            return
        self._last_ack_seq = packet.sequence
        self._last_ack_status = status
        if self._ack_pending and packet.sequence == self._ack_pending[0]:
            self._last_ack_latency_s = now - self._ack_pending[1]
        if status == ACK_OK:
            # The ESP32 accepted the command: engage motion / re-arm is done.
            was_safe = self._gate or self._rearm
            self._gate = False
            self._rearm = False
            self._failsafe = False
            if not self._seq_seeded:
                self._seq_seeded = True
            self._log_failsafe(False)
            if was_safe:
                self.logger.info(
                    "ESP32 engaged: zero handshake ACKed (seq=%d)", packet.sequence
                )
        else:
            self._missing_acks += 1
            self.logger.warning(
                "ESP32 ACK status %d (%s) seq=%d",
                status,
                ACK_NAMES.get(status, "?"),
                packet.sequence,
            )

    def _handle_status(self, packet, now):
        try:
            info = decode_status(packet.payload)
        except ValueError:
            self.logger.warning("ESP32 bad STATUS frame (len=%d)", packet.length)
            self.transport.stats["invalid_packets"] += 1
            return
        self._esp32_seq = packet.sequence
        self._esp32_failsafe = info["failsafe"]
        self._esp32_estop = info["estop"]
        self._esp32_motors = info["motors"]
        if not self._seq_seeded:
            # Seed so the NEXT command is strictly newer than the ESP32's last:
            # the firmware only accepts sequences that are newer than lastSequence.
            self._seq = (self._esp32_seq + 1) & 0xFFFF
            self._seq_seeded = True
            self.logger.info("ESP32 sequence synced to seq=%d", self._esp32_seq)
        elif ((self._seq - self._esp32_seq) & 0xFFFF) > 0x8000:
            # ESP32 restarted (sequence reset) while we kept counting: re-sync.
            self.logger.info(
                "ESP32 restarted (last seq=%d); re-syncing Pi sequence", self._esp32_seq
            )
            self._seq = (self._esp32_seq + 1) & 0xFFFF
        if not info["estop"] and not info["failsafe"]:
            # The ESP32 is receiving commands again (its lastStatus) -> safe.
            was_safe = self._gate or self._rearm
            self._gate = False
            self._rearm = False
            self._failsafe = False
            self._log_failsafe(False)
            if was_safe:
                self.logger.info("ESP32 link engaged")
        elif info["failsafe"]:
            self._failsafe = True
            self._gate = True
            self._log_failsafe(True)
        if info["estop"]:
            self.logger.warning("ESP32 reports E-STOP ACTIVE (holding zero)")

    def _check_link(self):
        if not self.is_connected():
            if self._failsafe is not True:
                with self._lock:
                    self._failsafe = True
                    self._gate = True
                self._log_failsafe(True)
            return
        if self._last_rx_at is None:
            return
        stale = (time.time() - self._last_rx_at) > self._link_timeout
        if stale:
            if not self._link_stale_logged:
                self._link_stale_logged = True
                self.logger.warning(
                    "ESP32 UART link STALE: no frame for %.0f ms",
                    self._link_timeout * 1000,
                )
            if self._failsafe is not True:
                with self._lock:
                    self._failsafe = True
                    self._gate = True
                self._log_failsafe(True)

    def _log_failsafe(self, active):
        if active == self._failsafe_logged:
            return
        self._failsafe_logged = active
        if active:
            self.logger.error("FAILSAFE ACTIVE: motor outputs clamped to zero")
        else:
            self.logger.info("FAILSAFE CLEARED: motor outputs re-enabled")

    # -- diagnostics -------------------------------------------------------

    def status(self):
        """Expose all provider + link state for the diagnostics UI."""
        transport = self.transport
        now = time.time()
        stats = transport.stats if transport is not None else {}
        base = dict(stats)
        base.update(
            provider="esp32",
            connected=self.is_connected(),
            link_alive=self.is_link_alive(),
            estop=self._estop,
            rearming=self._rearm,
            gate=self._gate,
            failsafe=self._failsafe,
            last_error=stats.get("last_error"),
            command_rate_hz=1.0 / max(1e-6, self._command_interval),
            link_timeout_s=self._link_timeout,
            last_tx_seq=self._last_tx_seq,
            last_tx_at=self._last_tx_at,
            last_rx_at=self._last_rx_at,
            last_rx_info=self._last_rx_info,
            last_ack_seq=self._last_ack_seq,
            last_ack_status=self._last_ack_status,
            last_ack_latency_s=self._last_ack_latency_s,
            missing_acks=self._missing_acks,
            esp32_seq=self._esp32_seq,
            esp32_failsafe=self._esp32_failsafe,
            esp32_estop=self._esp32_estop,
            esp32_motors=self._esp32_motors,
        )
        return base


def struct_seq(frame):
    """Extract the uint16 sequence from a built frame (bytes 5-6)."""
    return (frame[5] | (frame[6] << 8)) & 0xFFFF


class ThrusterManager(BaseModule):
    """Splits a motion target across the 5-thruster layout via a provider."""

    def __init__(self, config=None, sensors=None):
        super().__init__("Thrusters")
        self.config = config or {}
        self.sensors = sensors
        self.provider = None
        self.provider_name = None
        self.thruster_config = ThrusterConfig.from_dict(self.config.get("motors"))
        self._motion = MotionState()
        self._last = None
        self._estop = False
        self._failsafe_shown = None
        self._throttle_mode = str(self.config.get("throttle_mode", DEFAULT_THROTTLE_MODE)).lower()
        if self._throttle_mode not in THROTTLE_MODES:
            self._throttle_mode = DEFAULT_THROTTLE_MODE
        self._throttle_ramp = float(self.config.get("throttle_ramp_per_sec", DEFAULT_THROTTLE_RAMP))
        self.last_setpoints = {thruster: 0.0 for thruster in ThrusterId}

    def initialize(self):
        super().initialize()
        name = self.config.get("provider", "simulated")
        providers = {
            "simulated": SimulatedThrusterProvider,
            "esp32": Esp32ThrusterProvider,
        }
        provider_cls = providers.get(name)
        if provider_cls is None:
            self.logger.warning("Unknown thruster provider '%s'; using simulated", name)
            provider_cls = SimulatedThrusterProvider
            name = "simulated"
        self.provider = provider_cls(self.config, sensors=self.sensors)
        self.provider_name = name
        provider_initialize = getattr(self.provider, "initialize", None)
        if callable(provider_initialize):
            provider_initialize()
        self.logger.info("Thruster provider: %s (5-motor layout)", name)
        return True

    def set_motion(self, motion):
        self._motion = motion

    @property
    def is_esp32(self):
        return self.provider_name == "esp32"

    @property
    def throttle_mode(self):
        return self._throttle_mode

    @property
    def throttle_limit(self):
        """Max commanded fraction allowed by the active throttle mode."""
        return THROTTLE_MODES[self._throttle_mode]

    @property
    def current_power(self):
        """Peak absolute value of the current ramped motor setpoints (0..1)."""
        return max((abs(v) for v in self.last_setpoints.values()), default=0.0)

    def set_throttle_mode(self, name):
        """Select soft/medium/high throttle limit from the dashboard."""
        name = str(name).lower()
        if name not in THROTTLE_MODES:
            self.logger.warning("Unknown throttle mode '%s'; keeping '%s'", name, self._throttle_mode)
            return
        self._throttle_mode = name
        self.logger.info("Throttle mode -> %s (max %d%%)", name, int(self.throttle_limit * 100))

    def _ramp_setpoints(self, target, dt):
        """Move current setpoints toward ``target`` in fixed 10 % steps.

        Every control tick each thruster climbs (or eases back) by one
        THROTTLE_STEP toward the commanded value instead of snapping to
        full thrust, so holding a key builds up 10 % at a time and
        releasing lets it ease back the same way.
        """
        limit = self.throttle_limit
        rate_step = max(0.0, self._throttle_ramp) * max(0.1, dt)
        step = max(THROTTLE_STEP, rate_step)
        prev = self.last_setpoints
        ramped = {}
        for thruster in ThrusterId:
            current = prev.get(thruster, 0.0)
            desired = max(-limit, min(limit, float(target.get(thruster, 0.0))))
            delta = desired - current
            if abs(delta) <= step:
                next_val = desired
            else:
                next_val = current + (step if delta > 0 else -step)
            ramped[thruster] = max(-1.0, min(1.0, next_val))
        return ramped

    def emergency_stop(self):
        """Independent software e-stop; the ESP32 provider additionally issues
        a latched ESTOP frame so the ESC controller halts."""
        self._estop = True
        self.logger.warning("EMERGENCY STOP: all thrusters zeroed")
        notify = getattr(self.provider, "notify_estop", None)
        if callable(notify):
            notify(True)

    def clear_emergency_stop(self):
        """Explcit re-arm: clears the Pi latch and starts the zero handshake."""
        self._estop = False
        notify = getattr(self.provider, "notify_estop", None)
        if callable(notify):
            notify(False)

    def update(self):
        if not self.running or self.provider is None:
            return
        now = time.monotonic()
        if self._last is None:
            self._last = now
            return
        dt = min(now - self._last, 0.25)
        self._last = now
        if self._estop:
            self.last_setpoints = {thruster: 0.0 for thruster in ThrusterId}
            keepalive = getattr(self.provider, "estop_keepalive", None)
            if callable(keepalive):
                keepalive(dt, self.last_setpoints)
            return

        mixed = self.provider.setpoints(self._motion)
        self.last_setpoints = self._ramp_setpoints(mixed, dt)

        link_ok = self._link_ok()
        failsafe_active = bool(self.is_esp32) and not link_ok
        if failsafe_active:
            # Internal ramped setpoints stay intact: ``last_setpoints`` IS the
            # real command state (the GUI meters and diagnostics read it and it
            # must not be flattened to zero just because the link is unhappy).
            # What the provider is allowed to APPLY, however, is zeroed until
            # the ESP32 link is alive again. The ESP32 provider also keeps its
            # own wire-level gate closed (only zeros hit the bus) while it is
            # gated, re-arming, or E-STOPped, so the physical motors stay safe.
            applied = {thruster: 0.0 for thruster in ThrusterId}
            if self._failsafe_shown is not True:
                self._failsafe_shown = True
                self.logger.error(
                    "FAILSAFE: ESP32 link not alive (%s); motor outputs zeroed",
                    self._link_reason(),
                )
        else:
            applied = self.last_setpoints
            if self._failsafe_shown is True:
                self._failsafe_shown = False
                self.logger.info("FAILSAFE CLEARED: motor outputs re-enabled")

        if any(abs(v) > 0.01 for v in self.last_setpoints.values()):
            self.logger.debug("Setpoints: %s", self.last_setpoints)
        self._trace_pipeline(self._motion, mixed, applied)
        self.provider.apply(self._motion, dt, setpoints=applied)

    def _trace_pipeline(self, motion, mixed, applied):
        """Per-tick INPUT / MIXER / POWER-RAMP / APPLIED trace for debugging."""
        if not TRACE_PIPELINE:
            return

        def num(value):
            return "+%.2f" % value if value >= 0 else "%.2f" % value

        self.logger.info(
            "INPUT:      surge=%+.2f yaw=%+.2f heave=%+.2f (sway/pitch/roll not mixed)",
            motion.surge, motion.yaw, motion.heave,
        )
        self.logger.info(
            "MIXER:      M1=%s M2=%s M3=%s M4=%s M5=%s",
            *[num(mixed[t]) for t in ThrusterId],
        )
        self.logger.info(
            "POWER/RAMP: M1=%s M2=%s M3=%s M4=%s M5=%s",
            *[num(self.last_setpoints[t]) for t in ThrusterId],
        )
        self.logger.info(
            "APPLIED:    M1=%s M2=%s M3=%s M4=%s M5=%s",
            *[num(applied[t]) for t in ThrusterId],
        )

    def _link_ok(self):
        if not self.is_esp32:
            return True
        alive = getattr(self.provider, "is_link_alive", None)
        if callable(alive):
            return bool(alive())
        return False

    def _link_reason(self):
        status = self.get_provider_status()
        if status is None:
            return "no provider"
        if not status.get("connected"):
            return "disconnected"
        if not status.get("link_alive"):
            return "link stale"
        return "unknown"

    def get_provider_status(self):
        """Return the active provider's diagnostics dict (None for simulated)."""
        status = getattr(self.provider, "status", None)
        if callable(status):
            return status()
        return None

    def stop(self):
        super().stop()
        provider_stop = getattr(self.provider, "stop", None)
        if callable(provider_stop):
            provider_stop()

    def health_check(self):
        if self.provider is None:
            return False
        if self.is_esp32:
            alive = getattr(self.provider, "is_link_alive", None)
            if callable(alive):
                return bool(alive())
            return False
        return True
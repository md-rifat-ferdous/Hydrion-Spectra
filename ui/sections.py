"""Workspace pages for the GCS nav rail sections."""

from collections import deque
import os

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from ui.glass import GlassPanel, SectionPage, StatusPill, ValueRow, caps_label


class SensorsPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx
        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)

        nav_panel = GlassPanel("CORE NAVIGATION")
        self.depth = ValueRow("DEPTH", "--", highlight=True)
        self.heading = ValueRow("HEADING", "--")
        self.pos_x = ValueRow("POSITION X", "--")
        self.pos_y = ValueRow("POSITION Y", "--")
        for r in (self.depth, self.heading, self.pos_x, self.pos_y):
            nav_panel.add_row(r)

        att_panel = GlassPanel("ATTITUDE")
        self.pitch = ValueRow("PITCH", "--")
        self.roll = ValueRow("ROLL", "--")
        self.boost = ValueRow("BOOST", "OFF")
        for r in (self.pitch, self.roll, self.boost):
            att_panel.add_row(r)

        power = GlassPanel("POWER")
        self.bus_v = ValueRow("BUS VOLTAGE", "--")
        self.bus_i = ValueRow("TOTAL CURRENT", "--")
        self.temp = ValueRow("INT TEMP", "--")
        self.leak = ValueRow("LEAK SENSOR", "SAFE")
        for r in (self.bus_v, self.bus_i, self.temp, self.leak):
            power.add_row(r)

        self.health = GlassPanel("SENSOR HEALTH")
        hl = QHBoxLayout()
        hl.setSpacing(8)
        self._health_pills = {}
        for name in ("IMU", "DEPTH", "CAMERA", "LINK", "ESP32"):
            pill = StatusPill(name, "info")
            self._health_pills[name] = pill
            hl.addWidget(pill)
        hl.addStretch(1)
        health_w = QWidget()
        health_w.setLayout(hl)
        self.health.add_row(health_w)

        page.add(nav_panel)
        page.add(att_panel)
        page.add(power)
        page.add(self.health)

    def update(self):
        st = self.ctx["sensors"]
        state = st.get_state() if st is not None else None
        if state is None:
            return
        self.depth.set_value(f"{state.depth:5.2f} m")
        self.heading.set_value(f"{state.yaw_deg:03.0f} deg")
        self.pos_x.set_value(f"{state.x:5.2f} m")
        self.pos_y.set_value(f"{state.y:5.2f} m")
        ctrl = self.ctx["controller"]
        if ctrl is not None:
            m = ctrl.last_motion
            self.pitch.set_value(f"{m.pitch * 60:+.0f}")
            self.roll.set_value(f"{m.roll * 60:+.0f}")
            self.boost.set_value("ON" if m.boost else "OFF")
        for name, pill in self._health_pills.items():
            if name == "ESP32":
                thrusters = self.ctx.get("thrusters")
                status = thrusters.get_provider_status() if thrusters is not None else None
                if status is None:
                    pill.set_status("ok")
                elif status.get("link_alive") and status.get("connected"):
                    pill.set_status("ok")
                elif status.get("connected"):
                    pill.set_status("warn")
                else:
                    pill.set_status("bad")
            else:
                pill.set_status("ok")


class DiagnosticsPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx
        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)

        self.health_panel = GlassPanel("SUBSYSTEM HEALTH")
        self.health_grid = QGridLayout()
        self.health_grid.setSpacing(8)
        self.health_panel.body().addLayout(self.health_grid)
        self._health_pills = {}

        perf = GlassPanel("COMPUTE CORE LOAD")
        self.fps = ValueRow("FPS", "--")
        self.cpu = ValueRow("CPU LOAD", "--")
        self.ram = ValueRow("MEMORY ALLOC", "--")
        self.uptime = ValueRow("UPTIME", "--")
        for r in (self.fps, self.cpu, self.ram, self.uptime):
            perf.add_row(r)

        umb = GlassPanel("UMBILICAL")
        self.latency = ValueRow("LATENCY", "--")
        self.link = ValueRow("LINK STATE", "--")
        for r in (self.latency, self.link):
            umb.add_row(r)

        self.esp32_panel = GlassPanel("ESP32 THRUSTER LINK (UART)")
        self.esp32_state = ValueRow("STATE", "--", highlight=True)
        self.esp32_provider = ValueRow("PROVIDER", "--")
        self.esp32_estop = ValueRow("ESTOP", "--")
        self.esp32_failsafe = ValueRow("FAILSAFE", "--")
        self.esp32_rearm = ValueRow("RE-ARM", "--")
        self.esp32_tx = ValueRow("TX FRAMES", "--")
        self.esp32_rx = ValueRow("RX FRAMES", "--")
        self.esp32_crc = ValueRow("CRC ERRORS", "--")
        self.esp32_resync = ValueRow("RESYNCS", "--")
        self.esp32_missing = ValueRow("MISSING ACKS", "--")
        self.esp32_latency = ValueRow("ACK LATENCY", "--")
        self.esp32_seq = ValueRow("ESP32 SEQ", "--")
        self.esp32_motors = ValueRow("ESP32 MOTORS", "--")
        self.esp32_error = ValueRow("LAST ERROR", "--")
        for r in (self.esp32_state, self.esp32_provider, self.esp32_estop,
                  self.esp32_failsafe, self.esp32_rearm, self.esp32_tx,
                  self.esp32_rx, self.esp32_crc, self.esp32_resync,
                  self.esp32_missing, self.esp32_latency, self.esp32_seq,
                  self.esp32_motors, self.esp32_error):
            self.esp32_panel.add_row(r)
        self.esp32_controls = _ThrusterControls(ctx)
        self.esp32_panel.add_row(self.esp32_controls)

        page.add(self.health_panel)
        page.add(perf)
        page.add(umb)
        page.add(self.esp32_panel)

    def update(self):
        sm = self.ctx["service_manager"]
        if sm is None:
            return
        health = sm.health_check()
        names = set(list(self._health_pills) + list(health))
        col = 0
        for name in sorted(names):
            if name not in self._health_pills:
                pill = StatusPill(name, "info")
                self.health_grid.addWidget(pill, 0, col)
                self._health_pills[name] = pill
                col += 1
            ok = health.get(name)
            pill = self._health_pills[name]
            pill.set_status("ok" if ok else ("bad" if ok is False else "info"))
        self.fps.set_value(self.ctx.get("fps")())
        cpu, ram = self._load()
        self.cpu.set_value(cpu)
        self.ram.set_value(ram)
        self.uptime.set_value(self.ctx.get("uptime")())

        self._update_esp32()

    def _update_esp32(self):
        thrusters = self.ctx.get("thrusters")
        status = thrusters.get_provider_status() if thrusters is not None else None
        if status is None:
            self.esp32_state.set_value("SIMULATED")
            self.esp32_provider.set_value("simulated")
            self.esp32_controls.setEnabled(False)
            return
        self.esp32_controls.setEnabled(True)
        connected = status.get("connected")
        alive = status.get("link_alive")
        if connected and alive:
            state, state_color = "CONNECTED", "#64ffda"
        elif connected:
            state, state_color = "STALE", "#ffd364"
        else:
            state, state_color = "OFFLINE", "#ffb4ab"
        self.esp32_state.val.setStyleSheet(f"font-family: 'JetBrains Mono'; font-size: 12px; color: {state_color};")
        self.esp32_state.set_value(state)
        self.esp32_provider.set_value(status.get("provider", "--"))
        self.esp32_estop.set_value("ACTIVE" if status.get("estop") else "CLEAR")
        self.esp32_failsafe.set_value("ACTIVE" if status.get("failsafe") else "CLEAR")
        self.esp32_rearm.set_value("PENDING" if status.get("rearming") else "DONE")
        self.esp32_tx.set_value(status.get("tx_frames", 0))
        self.esp32_rx.set_value(status.get("rx_frames", 0))
        self.esp32_crc.set_value(status.get("crc_errors", 0))
        self.esp32_resync.set_value(status.get("resyncs", 0))
        self.esp32_missing.set_value(status.get("missing_acks", 0))
        latency = status.get("last_ack_latency_s")
        self.esp32_latency.set_value(
            f"{latency * 1000:.0f} ms" if latency is not None else "--")
        esp_seq = status.get("esp32_seq")
        self.esp32_seq.set_value(str(esp_seq) if esp_seq is not None else "--")
        motors = status.get("esp32_motors")
        self.esp32_motors.set_value(
            " ".join(f"{v:+d}" for v in motors) if motors else "--")
        error = status.get("last_error")
        self.esp32_error.set_value(error if error else "OK")

    @staticmethod
    def _load():
        try:
            import psutil

            return f"{psutil.cpu_percent():.0f}%", f"{psutil.virtual_memory().percent:.0f}%"
        except Exception:
            return "--", "--"


class LogsPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx
        self.mission_path = os.path.join(ctx["log_dir"], "mission.log")
        self.system_path = os.path.join(ctx["log_dir"], "system.log")

        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)

        mission = GlassPanel("MISSION LOG (TELEMETRY)")
        self.mission_box = QTextEdit()
        self.mission_box.setReadOnly(True)
        mission.add_row(self.mission_box)

        system = GlassPanel("SYSTEM LOG")
        self.system_box = QTextEdit()
        self.system_box.setReadOnly(True)
        system.add_row(self.system_box)

        refresh = QPushButton("REFRESH LOGS")
        refresh.clicked.connect(self.update)
        page.add(mission)
        page.add(system)
        page.add(refresh)

    @staticmethod
    def _tail(path, n=80):
        try:
            with open(path) as f:
                return "".join(deque(f, n))
        except FileNotFoundError:
            return "(no log file yet)\n"

    def update(self):
        self.mission_box.setPlainText(self._tail(self.mission_path))
        self.system_box.setPlainText(self._tail(self.system_path))


class SettingsPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)
        cm = ctx["config_manager"]
        if cm is None:
            page.add(GlassPanel("CONFIG"))
            return
        for section in cm.sections():
            data = cm.get_section(section) or {}
            panel = GlassPanel(section.upper())
            for k, v in list(data.items())[:14]:
                panel.add_row(ValueRow(k.replace("_", " ").upper(), str(v)))
            page.add(panel)


class OperationsPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        self.ctx = ctx
        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)

        status = GlassPanel("SYSTEM STATUS")
        self.mode = ValueRow("MODE", "MANUAL")
        self.link_pill_row = QWidget()
        row = QHBoxLayout(self.link_pill_row)
        row.setContentsMargins(0, 0, 0, 0)
        self.link_pill = StatusPill("LINK", "info")
        self.armed_pill = StatusPill("DISARMED", "warn")
        row.addWidget(self.link_pill)
        row.addWidget(self.armed_pill)
        row.addStretch(1)
        status.add_row(self.mode)
        status.add_row(self.link_pill_row)

        cmd = GlassPanel("COMMAND CHANNEL")
        self.cmd_input = QTextEdit()
        self.cmd_input.setMaximumHeight(64)
        self.cmd_input.setPlaceholderText('e.g. FORWARD 0.5  (space-separated command)')
        send = QPushButton("SEND COMMAND")
        send.clicked.connect(self._send)
        cmd.add_row(self.cmd_input)
        cmd.add_row(send)

        thrust = GlassPanel("THRUSTERS (5-MOTOR)")
        self._thrust_rows = {}
        for name in ("M1_FRONT_VERTICAL", "M2_MIDDLE_RIGHT_HORIZONTAL",
                     "M3_MIDDLE_LEFT_HORIZONTAL", "M4_BACK_RIGHT_VERTICAL",
                     "M5_BACK_LEFT_VERTICAL"):
            short = name.split("_")[0]
            row = ValueRow(short, "0%")
            self._thrust_rows[name] = row
            thrust.add_row(row)
        self.thrust_estop = ValueRow("E-STOP STATE", "--")
        self.thrust_failsafe = ValueRow("FAILSAFE", "--")
        thrust.add_row(self.thrust_estop)
        thrust.add_row(self.thrust_failsafe)
        thrust.add_row(ThrusterControlsWidget(ctx))

        page.add(status)
        page.add(cmd)
        page.add(thrust)

    def _send(self):
        text = self.cmd_input.toPlainText().strip()
        telemetry = self.ctx.get("telemetry")
        if text and telemetry is not None:
            telemetry.inject_command(text)

    def update(self):
        armed = self.ctx.get("armed")()
        self.armed_pill.setText("ARMED" if armed else "DISARMED")
        self.armed_pill.set_status("ok" if armed else "warn")
        self.link_pill.setText("LINK OK" if self.ctx.get("link_ok")() else "LINK DOWN")
        self.link_pill.set_status("ok" if self.ctx.get("link_ok")() else "bad")
        thrusters = self.ctx.get("thrusters")
        if thrusters is not None:
            setpoints = thrusters.last_setpoints
            for name, row in self._thrust_rows.items():
                key = next((t for t in setpoints if t.name == name), None)
                value = setpoints.get(key, 0.0) if key is not None else 0.0
                row.set_value(f"{int(value * 100):+d}%")
            provider = thrusters.get_provider_status()
            self.thrust_estop.set_value(
                "ACTIVE" if (provider or {}).get("estop") else "CLEAR")
            self.thrust_failsafe.set_value(
                "ACTIVE" if (provider or {}).get("failsafe") else "CLEAR")


class NavigationPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)

        hold = GlassPanel("HOLD CONTROLS (FUTURE)")
        self.depth_hold = QCheckBox("DEPTH HOLD")
        self.depth_set = QSpinBox()
        self.depth_set.setRange(0, 500)
        self.depth_set.setValue(10)
        self.depth_set.setSuffix(" m")
        dh = QWidget()
        row = QHBoxLayout(dh)
        row.addWidget(self.depth_hold)
        row.addWidget(self.depth_set)
        row.addStretch(1)
        self.heading_hold = QCheckBox("HEADING HOLD")
        self.hdg_set = QSpinBox()
        self.hdg_set.setRange(0, 359)
        self.hdg_set.setValue(0)
        self.hdg_set.setSuffix(" deg")
        hh = QWidget()
        row2 = QHBoxLayout(hh)
        row2.addWidget(self.heading_hold)
        row2.addWidget(self.hdg_set)
        row2.addStretch(1)
        hold.add_row(dh)
        hold.add_row(hh)
        page.add(hold)
        wp = GlassPanel("WAYPOINTS")
        wp.add_row(caps_label("No waypoints yet"))
        page.add(wp)


class ManipulatorPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)
        arm = GlassPanel("MANIPULATOR ARM (FUTURE)")
        btns = QWidget()
        row = QHBoxLayout(btns)
        open_b = QPushButton("CLAW OPEN")
        close_b = QPushButton("CLAW CLOSE")
        open_b.setEnabled(False)
        close_b.setEnabled(False)
        row.addWidget(open_b)
        row.addWidget(close_b)
        row.addStretch(1)
        arm.add_row(btns)
        arm.add_row(ValueRow("JAW POSITION", "--"))
        hyd = GlassPanel("HYDRAULICS")
        hyd.add_row(ValueRow("PRESSURE", "--"))
        hyd.add_row(ValueRow("TEMP", "--"))
        hyd.add_row(ValueRow("FLOW RATE", "--"))
        page.add(arm)
        page.add(hyd)


class AIVisionPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)
        det = GlassPanel("OBJECT DETECTION (FUTURE)")
        enable = QCheckBox("ENABLE DETECTION")
        enable.setEnabled(False)
        det.add_row(enable)
        det.add_row(ValueRow("MODEL STATUS", "NOT LOADED"))
        det.add_row(ValueRow("INFERENCE FPS", "--"))
        page.add(det)
        results = GlassPanel("DETECTIONS")
        results.add_row(ValueRow("PIPELINE JOINT", "92%"))
        results.add_row(ValueRow("L. PERTUSA", "88%"))
        results.add_row(ValueRow("UNKNOWN OBJ", "64%"))
        page.add(results)


class PlannerPage(QWidget):
    def __init__(self, ctx):
        super().__init__()
        page = SectionPage(self)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)
        page.add(GlassPanel("MISSION BLOCKS (FUTURE)"))
        seq = GlassPanel("SEQUENCE LOGIC")
        seq.add_row(caps_label("No mission loaded"))
        page.add(seq)
        val = GlassPanel("PLAN VALIDATION")
        val.add_row(ValueRow("STATUS", "--"))
        page.add(val)


class ThrusterControlsWidget(QWidget):
    """E-STOP / RE-ARM buttons wired to the thruster manager."""

    def __init__(self, ctx, parent=None):
        super().__init__(parent)
        self.ctx = ctx
        self.estop_btn = QPushButton(" E-STOP ")
        self.estop_btn.setObjectName("Danger")
        self.estop_btn.clicked.connect(self._estop)
        self.rearm_btn = QPushButton(" RE-ARM ")
        self.rearm_btn.clicked.connect(self._rearm)
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.addWidget(self.estop_btn)
        row.addWidget(self.rearm_btn)
        row.addStretch(1)

    def _estop(self):
        thrusters = self.ctx.get("thrusters")
        if thrusters is not None:
            thrusters.emergency_stop()
        controller = self.ctx.get("controller")
        if controller is not None:
            controller.kill()

    def _rearm(self):
        thrusters = self.ctx.get("thrusters")
        if thrusters is not None:
            thrusters.clear_emergency_stop()
        controller = self.ctx.get("controller")
        if controller is not None:
            controller.recover()


_ThrusterControls = ThrusterControlsWidget


def build(ctx):
    """Return {key: (title, glyph, widget)} for every non-dashboard section."""
    pages = {}
    for key, cls in (
        ("operations", OperationsPage),
        ("navigation", NavigationPage),
        ("sensors", SensorsPage),
        ("manipulator", ManipulatorPage),
        ("diagnostics", DiagnosticsPage),
        ("planner", PlannerPage),
        ("ai_vision", AIVisionPage),
        ("logs", LogsPage),
        ("settings", SettingsPage),
    ):
        pages[key] = cls(ctx)
    return pages

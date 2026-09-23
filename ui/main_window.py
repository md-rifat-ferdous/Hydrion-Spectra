import os
import shutil
import time

import cv2
import numpy as np
from PySide6.QtCore import QEvent, QSize, Qt, QTimer
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QSpinBox,
    QSizePolicy,
    QStackedWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from modules.controller.controller import MotionState
from modules.thrusters.thruster_manager import ThrusterId
from ui.control_pad import ControlPad3D
from ui.glass import GlassPanel, NavButton, StatusPill, caps_label
from ui.sections import build as build_sections
from ui.sonar import SonarPanel
from ui.theme import C, FONT_DATA

NAV_ITEMS = [
    ("dashboard", "Dashboard", "\u2302"),
    ("operations", "Operations", "\u25ce"),
    ("navigation", "Navigation", "\u2794"),
    ("sensors", "Sensors", "\u2733"),
    ("manipulator", "Manipulator", "\u2699"),
    ("diagnostics", "Diagnostics", "\u25a4"),
    ("planner", "Planner", "\u25ba"),
    ("ai_vision", "AI Vision", "\u2726"),
    ("logs", "Logs", "\u2630"),
    ("settings", "Settings", "\u25c8"),
]

_TYPING_WIDGETS = (QLineEdit, QTextEdit, QSpinBox, QComboBox)

# Dashboard sidebar sizing/shaping. Sidebars keep a controlled width so the
# central camera viewport always keeps the largest share of the window.
_SIDEBAR_W = 262
_DASH_MARGIN = 10
_DASH_SPACING = 10


class MainWindow(QMainWindow):
    def __init__(self, camera_manager, overlay=None, hud_provider=None,
                 controller=None, keymap=None, service_manager=None,
                 config_manager=None, telemetry=None, log_dir="logs",
                 gcs_config=None, thrusters=None):
        super().__init__()
        self.camera = camera_manager
        self.overlay = overlay
        self.hud_provider = hud_provider
        self.controller = controller
        self.thrusters = thrusters
        self.telemetry = telemetry
        self.log_dir = log_dir
        self.gcs = gcs_config or {}
        self.key_speed = 1.0
        self._pressed_actions = set()
        self._key_actions = self._build_keymap(keymap or {})
        self._armed = True
        self._estop_active = False
        self._recording = False
        self._writer = None
        self._lights = False
        self._frames = 0
        self._fps = 0.0
        self._fps_t = time.time()
        self._boot_t = time.time()
        self._cam_size = (0, 0)

        self.setWindowTitle("DUBO - ROV Controller")
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.resize(1440, 820)

        self._ctx = {
            "service_manager": service_manager,
            "config_manager": config_manager,
            "sensors": hud_provider,
            "controller": controller,
            "thrusters": thrusters,
            "telemetry": telemetry,
            "log_dir": log_dir,
            "fps": lambda: f"{self._fps:.0f}",
            "uptime": self._uptime_str,
            "armed": lambda: self._armed,
            "estop": lambda: self._estop_active,
            "link_ok": lambda: self._link_ok(),
        }

        self._build_ui()
        self._build_sections()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_camera)
        self.timer.start(33)

        self.refresh = QTimer(self)
        self.refresh.timeout.connect(self._refresh_static)
        self.refresh.start(1000)

        self.fast = QTimer(self)
        self.fast.timeout.connect(self._refresh_fast)
        self.fast.start(100)

        QApplication.instance().installEventFilter(self)

    def _build_keymap(self, keymap):
        actions = {}
        for action, name in keymap.items():
            if name is None:
                continue
            key = getattr(Qt.Key, f"Key_{name}", None)
            if key is not None:
                actions[key] = action
        return actions

    def _link_ok(self):
        if self.telemetry is None:
            return False
        return getattr(self.telemetry, "link", None) is not None

    def _uptime_str(self):
        t = int(time.time() - self._boot_t)
        return f"{t // 3600:02d}:{t % 3600 // 60:02d}"

    def _build_ui(self):
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        root.addWidget(self._build_appbar())
        body = QWidget()
        body_lay = QHBoxLayout(body)
        body_lay.setContentsMargins(0, 0, 0, 0)
        body_lay.setSpacing(0)
        self.rail = self._build_rail()
        body_lay.addWidget(self.rail)
        self.stack = QStackedWidget()
        body_lay.addWidget(self.stack, 1)
        root.addWidget(body, 1)
        root.addWidget(self._build_footer())

        self.setCentralWidget(central)

    def _build_appbar(self):
        bar = QFrame()
        bar.setFixedHeight(46)
        bar.setStyleSheet(
            f"background: rgba(4, 19, 41, 0.85); border-bottom: 1px solid "
            f"rgba(100, 255, 218, 0.22);"
        )
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(20, 0, 16, 0)
        brand = QLabel("DUBO")
        brand.setObjectName("Brand")
        lay.addWidget(brand)
        workspace = QLabel("WORKSPACE")
        workspace.setStyleSheet(
            f"color: {C['primary']}; font-weight: 700; font-size: 12px; "
            f"letter-spacing: 2px; border-bottom: 2px solid {C['primary']};"
        )
        lay.addSpacing(28)
        lay.addWidget(workspace)
        lay.addSpacing(28)

        self.rov_pill = StatusPill("ROV --", "info")
        self.esp32_pill = StatusPill("ESP32 --", "info")
        self.batt_pill = StatusPill("BATT --", "info")
        self.link_pill = StatusPill("LINK --", "info")
        self.uptime_pill = StatusPill("UP 00:00", "info")
        self.mode_pill = StatusPill("MANUAL", "ok")
        self.depth_pill = StatusPill("DEPTH --", "info")
        self.estop_pill = StatusPill("E-STOP READY", "ok")
        for pill in (self.rov_pill, self.esp32_pill, self.mode_pill,
                     self.depth_pill, self.batt_pill, self.estop_pill,
                     self.link_pill, self.uptime_pill):
            lay.addWidget(pill, 1, Qt.AlignCenter)
        avatar = QLabel("P")
        avatar.setFixedSize(30, 30)
        avatar.setAlignment(Qt.AlignCenter)
        avatar.setStyleSheet(
            f"background: {C['surface_high']}; color: {C['primary']}; "
            f"border: 1px solid rgba(100, 255, 218, 0.3); border-radius: 15px;"
        )
        lay.addWidget(avatar, 1, Qt.AlignCenter)
        return bar

    def _build_rail(self):
        rail = _RailFrame()
        rail.expand = self.set_expanded
        self.rail = rail
        rail.setObjectName("Rail")
        rail.setStyleSheet(
            f"QFrame#Rail {{ background: rgba(17, 32, 54, 0.72); "
            f"border-right: 1px solid rgba(100, 255, 218, 0.22); }}"
        )
        rail.setMinimumWidth(56)
        rail.setMaximumWidth(208)
        self._rail_lay = QVBoxLayout(rail)
        self._rail_lay.setContentsMargins(8, 12, 8, 10)
        self._rail_lay.setSpacing(6)

        self.rail_logo = QLabel("\u25a6")  # system glyph
        self.rail_logo.setFixedHeight(34)
        self.rail_logo.setAlignment(Qt.AlignCenter)
        self.rail_logo.setStyleSheet(
            f"background: {C['surface_highest']}; color: {C['primary']}; "
            f"border: 1px solid rgba(100, 255, 218, 0.3); border-radius: 17px;"
        )
        self._rail_lay.addWidget(self.rail_logo, 0, Qt.AlignHCenter)

        self.rail_btns = {}
        for key, title, glyph in NAV_ITEMS:
            btn = NavButton(glyph, title)
            btn.clicked.connect(lambda _=False, k=key: self._set_section(k))
            self._rail_lay.addWidget(btn)
            self.rail_btns[key] = btn

        self._rail_lay.addStretch(1)

        self.launch_btn = QPushButton("\u25ba  LAUNCH MISSION")
        self.launch_btn.setObjectName("Danger")
        self.launch_btn.setCursor(Qt.PointingHandCursor)
        self.launch_btn.clicked.connect(self._launch_mission)
        self._rail_lay.addWidget(self.launch_btn)

        self.set_expanded(False)
        return rail

    def set_expanded(self, expanded):
        self._collapsed = not expanded
        for btn in self.rail_btns.values():
            btn.set_collapsed(not expanded)
        self.launch_btn.setText("\u25ba  LAUNCH MISSION" if expanded else "\u25ba")
        self.rail.set_expanded_state(expanded)

    def _build_footer(self):
        bar = QFrame()
        bar.setFixedHeight(30)
        bar.setStyleSheet(
            f"background: rgba(1, 14, 36, 0.9); border-top: 1px solid "
            f"rgba(100, 255, 218, 0.15);"
        )
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(16, 0, 16, 0)
        cr = QLabel("(c) 2024 HYDRION OFFSHORE SYSTEMS")
        cr.setStyleSheet(f"color: {C['secondary']}; font-size: 11px; letter-spacing: 1px;")
        lay.addWidget(cr)
        lay.addStretch(1)
        self.foot = {}
        for name in ("FPS", "CPU", "RAM", "STORAGE", "DEPTH", "LATENCY",
                     "ESP32", "THRUST"):
            lab = QLabel(f"{name}: --")
            lab.setStyleSheet(
                f"color: {C['on_surface_variant']}; font-size: 11px; "
                f"font-family: {FONT_DATA};"
            )
            lay.addWidget(lab)
            self.foot[name] = lab
            lay.addSpacing(14)
        return bar

    def _build_sections(self):
        pages = build_sections(self._ctx)
        self._pages = {}
        for key, title, glyph in NAV_ITEMS:
            if key == "dashboard":
                widget = self._build_dashboard()
            else:
                widget = pages[key]
            self._pages[key] = widget
            self.stack.addWidget(widget)
        self.rail_btns["dashboard"].setChecked(True)

    def _build_dashboard(self):
        container = QWidget()
        container.setStyleSheet("background: #000000;")
        self.camera_container = container

        # Full-bleed live-feed backdrop, exactly like the original DUBO HUD.
        # The translucent glass deck floats over it, so the camera stays
        # visible through the panels. (Plain QWidget hosts underneath are the
        # only thing that can hide it: the global `QWidget { background: }`
        # rule paints them opaque navy, so every host gets an explicit
        # transparent background below.)
        cam_lay = QVBoxLayout(container)
        cam_lay.setContentsMargins(0, 0, 0, 0)
        self.camera_view = QLabel("LIVE CAMERA FEED")
        self.camera_view.setAlignment(Qt.AlignCenter)
        self.camera_view.setMinimumSize(1, 1)
        self.camera_view.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.camera_view.setStyleSheet("background: #000; color: #556; font-weight: bold;")
        cam_lay.addWidget(self.camera_view)

        # Transparent glass deck: holds the layout, paints nothing itself.
        deck = QWidget(container)
        deck.setStyleSheet("background: transparent;")
        self._deck = deck
        root = QVBoxLayout(deck)
        root.setContentsMargins(_DASH_MARGIN, _DASH_MARGIN, _DASH_MARGIN, _DASH_MARGIN)
        root.setSpacing(_DASH_SPACING)

        # -- central camera focus zone (transparent; camera shows through) --
        center = QWidget()
        center.setObjectName("CameraViewport")
        center.setStyleSheet("background: transparent;")
        center.setMinimumSize(200, 200)
        self._viewport = center

        # -- left sidebar: telemetry / thrusters / motion ------------------
        left_host = QWidget()
        left_host.setFixedWidth(_SIDEBAR_W)
        left_host.setStyleSheet("background: transparent;")
        left = QVBoxLayout(left_host)
        left.setContentsMargins(0, 0, 0, 0)
        left.setSpacing(_DASH_SPACING)

        self.telemetry_panel = GlassPanel("SYSTEM TELEMETRY")
        self.telemetry_panel._values = {}
        tcfg = self.gcs.get("telemetry", {})
        for label, value in (
            ("BUS VOLTAGE", f"{tcfg.get('bus_voltage', 48.2)} V"),
            ("TOTAL CURR", f"{tcfg.get('total_current', 12.4)} A"),
            ("INT TEMP", f"{tcfg.get('int_temp', 24.0)} C"),
        ):
            row = _ValueRowCompact(label, value)
            self.telemetry_panel._values[label] = row
            self.telemetry_panel.add_row(row)
        leak = _ValueRowCompact("LEAK SENSOR", tcfg.get("leak_status", "SAFE"), pill=True)
        self.telemetry_panel._values["LEAK SENSOR"] = leak
        self.telemetry_panel.add_row(leak)
        left.addWidget(self.telemetry_panel)

        self.thruster_panel = GlassPanel("THRUSTERS M1-M5")
        self.thruster_panel._values = {}
        motor_rows = {}
        for thruster in ThrusterId:
            row = _MotorRow(thruster)
            motor_rows[thruster] = row
            self.thruster_panel.add_row(row)
            self.thruster_panel._values[thruster.name] = row
        self._motor_rows = motor_rows
        left.addWidget(self.thruster_panel)

        self.motion_panel = GlassPanel("MOTION SURGE / YAW / HEAVE")
        self.motion_rows = {}
        for axis in ("surge", "yaw", "heave"):
            row = _AxisRow(axis)
            self.motion_rows[axis] = row
            self.motion_panel.add_row(row)
        left.addWidget(self.motion_panel)

        left.addStretch(1)

        # -- right sidebar: sonar / throttle / control pad -----------------
        right_host = QWidget()
        right_host.setFixedWidth(_SIDEBAR_W)
        right_host.setStyleSheet("background: transparent;")
        right = QVBoxLayout(right_host)
        right.setContentsMargins(0, 0, 0, 0)
        right.setSpacing(_DASH_SPACING)

        self.sonar = SonarPanel(size=self.gcs.get("sonar", {}).get("size", 220),
                                range_m=self.gcs.get("sonar", {}).get("range_m", 6.0),
                                grid_lines=self.gcs.get("sonar", {}).get("grid_lines", 6))
        right.addWidget(self.sonar, 0, Qt.AlignHCenter)

        self.throttle_panel = GlassPanel("THROTTLE MODE")
        throttle_lay = QHBoxLayout()
        throttle_lay.setContentsMargins(0, 0, 0, 0)
        throttle_lay.setSpacing(6)
        initial = "high"
        if self.thrusters is not None:
            initial = self.thrusters.throttle_mode
        self.throttle_buttons = {}
        for name, pct in (("soft", 40), ("medium", 70), ("high", 100)):
            btn = QPushButton(f"{name.upper()}\n{pct}%")
            btn.setCheckable(True)
            btn.setCursor(Qt.PointingHandCursor)
            btn.setMinimumHeight(40)
            btn.clicked.connect(lambda _=False, n=name: self._set_throttle(n))
            btn.setChecked(name == initial)
            self.throttle_buttons[name] = btn
            throttle_lay.addWidget(btn, 1)
        self.throttle_panel.body().addLayout(throttle_lay)
        self.power_value = QLabel("")
        self.power_value.setStyleSheet(
            f"font-family: {FONT_DATA}; font-size: 11px; color: {C['primary']};"
        )
        self.power_value.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        self.throttle_panel.body().addWidget(self.power_value)
        self._refresh_power_readout()
        right.addWidget(self.throttle_panel)

        self.pad = ControlPad3D()
        if self.controller is not None:
            self.pad.motionChanged.connect(
                lambda motion: self._pad_motion(motion)
            )
        right.addWidget(self.pad, 0, Qt.AlignHCenter)

        self.heave_hint = QLabel("HEAVE: R / F")
        self.heave_hint.setStyleSheet(
            f"background: rgba(17, 32, 54, 0.55); color: {C['on_surface_variant']}; "
            f"border: 1px solid {C['outline_variant']}; border-radius: 8px; "
            f"padding: 2px 8px; font-size: 11px;"
        )
        right.addWidget(self.heave_hint, 0, Qt.AlignHCenter)

        right.addStretch(1)

        # -- assemble main row -------------------------------------------------
        page = QHBoxLayout()
        page.setSpacing(_DASH_SPACING)
        page.addWidget(left_host)
        page.addWidget(center, 1)
        page.addWidget(right_host)
        root.addLayout(page, 1)

        # -- bottom dock bar ----------------------------------------------------
        self.estop_btn = _DockButton("\u26d4", "E-STOP", danger=True)
        self.estop_btn.clicked.connect(self._estop)
        self.rearm_btn = _DockButton("\u27f3", "RE-ARM")
        self.rearm_btn.clicked.connect(self._rearm)
        self.dock = self._build_dock(container)
        self.dock.setStyleSheet("background: transparent;")

        bottom = QHBoxLayout()
        bottom.setSpacing(_DASH_SPACING)
        bottom.addStretch(1)
        bottom.addWidget(self.dock, 0, Qt.AlignCenter)
        bottom.addStretch(1)
        root.addLayout(bottom)

        # -- floating overlays: LIVE badge, mission-log flyout, status toast ---
        self.live = QLabel()
        self.live.setText(" \u25cf LIVE: CH-1 MAIN CAM")
        self.live.setStyleSheet(
            f"background: rgba(17, 32, 54, 0.55); color: {C['on_surface']}; "
            f"border: 1px solid {C['outline_variant']}; border-radius: 8px; "
            f"padding: 3px 10px; font-family: {FONT_DATA}; font-size: 12px;"
        )
        self.live.setParent(container)
        self.live.adjustSize()
        self.live.show()

        self.flyout = self._build_flyout(container)

        self._status_toast = QLabel("")
        self._status_toast.setStyleSheet(
            f"background: rgba(17, 32, 54, 0.9); color: {C['on_surface']}; "
            f"border: 1px solid {C['primary']}; border-radius: 10px; "
            f"padding: 8px 14px; font-weight: 700; font-size: 13px;"
        )
        self._status_toast.setParent(container)
        self._status_toast.adjustSize()
        self._status_toast.hide()

        self._reposition_overlays()
        return container

    def _build_flyout(self, parent):
        tab = QPushButton("\u00bb")
        tab.setFixedSize(22, 60)
        tab.setObjectName("Ghost")
        tab.setParent(parent)
        panel = GlassPanel("MISSION LOG", parent=parent)
        panel.setFixedSize(230, 220)
        self.log_body = QLabel()
        self.log_body.setWordWrap(True)
        self.log_body.setTextFormat(Qt.PlainText)
        self.log_body.setStyleSheet(
            f"color: {C['on_surface_variant']}; font-family: {FONT_DATA}; "
            f"font-size: 10px;"
        )
        panel.add_row(self.log_body)
        panel.hide()

        def toggle():
            panel.setVisible(not panel.isVisible())
            panel.raise_()

        tab.clicked.connect(toggle)
        self._flyout_panel = panel
        self._flyout_tab = tab
        return tab

    def _build_dock(self, parent):
        dock = QWidget(parent)
        lay = QHBoxLayout(dock)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)

        self.armed_btn = _DockButton("\u25c9", "ARMED", danger=True)
        self.armed_btn.clicked.connect(self._toggle_armed)
        self.rec_btn = _DockButton("\u25cf", "REC")
        self.rec_btn.clicked.connect(self._toggle_rec)
        self.snap_btn = _DockButton("\u25a3", "SNAP")
        self.snap_btn.clicked.connect(self._snap)
        self.light_btn = _DockButton("\u2600", "LIGHTS")
        self.light_btn.clicked.connect(self._toggle_lights)
        for b in (self.armed_btn, self.rec_btn, self.snap_btn, self.light_btn,
                  self.estop_btn, self.rearm_btn):
            lay.addWidget(b)
        dock.adjustSize()
        dock.show()
        self._refresh_armed()
        return dock

    def _refresh_armed(self):
        self.armed_btn.set_active(self._armed)
        self.armed_btn.set_label("ARMED" if self._armed else "DISARMED")
        self.mode_pill.setText("MANUAL" if self._armed else "DISARMED")
        self.mode_pill.set_status("ok" if self._armed else "warn")
        self.estop_pill.setText("E-STOP ACTIVE" if self._estop_active else "E-STOP READY")
        self.estop_pill.set_status("bad" if self._estop_active else "ok")
        self.rov_pill.setText("ROV E-STOP" if self._estop_active
                              else ("ROV ARMED" if self._armed else "ROV DISARMED"))
        self.rov_pill.set_status("bad" if self._estop_active else
                                 ("ok" if self._armed else "warn"))
        self.estop_btn.set_active(self._estop_active)
        self.rearm_btn.set_active(False)

    def _estop(self):
        self._estop_active = True
        self._armed = False
        if self.controller is not None:
            self.controller.kill()
            self.controller.set_input("keyboard", MotionState())
            self.controller.set_input("pad", MotionState())
        if self.thrusters is not None:
            self.thrusters.emergency_stop()
        self.pad.set_yaw(0.0)
        self.pad.reset()
        self._refresh_armed()
        self.status_message("EMERGENCY STOP ENGAGED - all thrusters halted", 3000)

    def _rearm(self):
        self._estop_active = False
        if self.thrusters is not None:
            self.thrusters.clear_emergency_stop()
        if self.controller is not None:
            self.controller.recover()
        self._armed = True
        self.estop_btn.set_active(False)
        self._refresh_armed()
        self.status_message("RE-ARMED - thrusters handshake in progress", 3000)

    def _set_throttle(self, name):
        for key, btn in self.throttle_buttons.items():
            btn.setChecked(key == name)
        if self.thrusters is not None:
            self.thrusters.set_throttle_mode(name)
            limit = self.thrusters.throttle_limit
        else:
            limit = {"soft": 0.40, "medium": 0.70, "high": 1.0}[name]
        self._refresh_power_readout()
        self.status_message(f"THROTTLE: {name.upper()} — {int(limit * 100)}% MAX", 1500)

    def status_message(self, text, ms=2000):
        if hasattr(self, "_status_toast"):
            self._status_toast.setText(text)
            self._status_toast.adjustSize()
            self._status_toast.show()
            from PySide6.QtCore import QTimer as _T
            _T.singleShot(ms, self._status_toast.hide)

    def _refresh_power_readout(self):
        """Mode + live power readout under the throttle-mode buttons.

        Shows the selected SOFT/MEDIUM/HARD ceiling and the current ACTIVE
        (ramped, already-limited) power, e.g. ``MODE: SOFT 40% MAX | POWER 20%``.
        """
        if not hasattr(self, "power_value"):
            return
        limit = None
        if self.thrusters is not None:
            limit = self.thrusters.throttle_limit
            peak = max(abs(v) for v in self.thrusters.last_setpoints.values())
        else:
            peak = 0.0
        if limit is None:
            text = "POWER: --"
        else:
            mode = self.thrusters.throttle_mode if self.thrusters is not None else "soft"
            text = f"MODE: {mode.upper()} {int(limit * 100)}% MAX | POWER: {int(round(peak * 100))}%"
        self.power_value.setText(text)

    def _toggle_armed(self):
        if self._estop_active:
            self.status_message("E-STOP ACTIVE - press RE-ARM first", 2500)
            return
        self._armed = not self._armed
        if self.controller is not None:
            if self._armed:
                self.controller.recover()
            else:
                self.controller.kill()
            self.controller.set_input("keyboard", MotionState())
        self.pad.set_yaw(0.0)
        self._refresh_armed()

    def _toggle_rec(self):
        self._recording = not self._recording
        self.rec_btn.set_active(self._recording)
        if self._recording:
            self._start_recording()
        else:
            self._stop_recording()

    def _start_recording(self):
        if self._writer is not None:
            return
        try:
            path = os.path.join(self.log_dir, "recordings")
            os.makedirs(path, exist_ok=True)
            fname = time.strftime("rec_%Y%m%d_%H%M%S.avi")
            w, h = self._cam_size or (640, 480)
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            self._writer = cv2.VideoWriter(
                os.path.join(path, fname), fourcc, 30.0, (w, h)
            )
            if not self._writer.isOpened():
                self._writer = None
        except Exception:
            self._writer = None

    def _stop_recording(self):
        if self._writer is not None:
            self._writer.release()
            self._writer = None

    def _toggle_lights(self):
        self._lights = not self._lights
        self.light_btn.set_active(self._lights)

    def _snap(self):
        path = os.path.join(self.log_dir, "snapshots")
        os.makedirs(path, exist_ok=True)
        pix = self.camera_view.pixmap()
        if pix is not None:
            fname = time.strftime("%Y%m%d_%H%M%S.png")
            pix.save(os.path.join(path, fname))

    def _launch_mission(self):
        self.status_message("MISSION LAUNCH - placeholder", 2000)

    def _pad_motion(self, motion):
        if self.controller is not None and self._armed:
            self.controller.set_input("pad", motion)

    def _reposition_overlays(self):
        """Keep the glass deck over the full-bleed camera and anchor the three
        floating overlays (LIVE badge, mission-log flyout, status toast) to the
        central focus zone. Everything else lives in Qt layouts, so nothing can
        overlap unless the window is squeezed below the layout minimums.
        """
        cw = self.camera_container.width()
        ch = self.camera_container.height()
        if hasattr(self, "_deck"):
            self._deck.setGeometry(0, 0, cw, ch)
            self._deck.raise_()
        vp = self._viewport
        vx = vp.mapTo(self.camera_container, vp.rect().topLeft()).x()
        vy = vp.mapTo(self.camera_container, vp.rect().topLeft()).y()
        vw, vh = vp.width(), vp.height()
        m = _DASH_MARGIN
        self.live.move(vx + m, vy + m)
        self._flyout_tab.move(vx, vy + max(0, vh // 2 - 30))
        fy = self._flyout_tab.y() + 2
        self._flyout_panel.move(vx + 20, fy)
        for w in (self.live, self._flyout_tab, self._flyout_panel,
                  self._status_toast):
            w.raise_()
        self._status_toast.move(
            vx + max(0, (vw - self._status_toast.width()) // 2),
            vy + max(0, vh - self._status_toast.height() - 16))

    def _set_section(self, key):
        self.stack.setCurrentWidget(self._pages[key])
        for k, btn in self.rail_btns.items():
            btn.setChecked(k == key)
        page = self._pages[key]
        upd = getattr(page, "update", None)
        if callable(upd):
            upd()

    def _axis(self, action_pos, action_neg):
        return ((action_pos in self._pressed_actions) - (action_neg in self._pressed_actions)) \
            * self.key_speed

    def _keyboard_state(self):
        # A/D (left/right) and Q/E (yaw_left/yaw_right) both turn the ROV;
        # sway is unsupported on the 5-thruster layout. Pitch/roll keys stay
        # for future stabilization (the 5-thruster mixer ignores them).
        yaw = float(np.clip(
            self._axis("yaw_right", "yaw_left") + self._axis("right", "left"),
            -1.0, 1.0,
        ))
        return MotionState(
            surge=self._axis("forward", "back"),
            sway=0.0,
            heave=self._axis("up", "down"),
            yaw=yaw,
            pitch=self._axis("pitch_up", "pitch_down"),
            roll=self._axis("roll_right", "roll_left"),
            boost="boost" in self._pressed_actions,
        )

    def _handle_key(self, key, pressed):
        action = self._key_actions.get(key)
        if action is None:
            return
        if action == "kill":
            if pressed:
                self._estop()
            return
        if pressed:
            self._pressed_actions.add(action)
        else:
            self._pressed_actions.discard(action)
        if self.controller is not None:
            state = self._keyboard_state() if self._armed else MotionState()
            self.controller.set_input("keyboard", state)
            self.pad.set_yaw(state.yaw)

    def eventFilter(self, obj, event):
        if obj is self.camera_container and event.type() == QEvent.Type.Resize:
            self._reposition_overlays()
        if event.type() in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease):
            focus = QApplication.focusWidget()
            if focus is None or not isinstance(focus, _TYPING_WIDGETS):
                self._handle_key(event.key(), event.type() == QEvent.Type.KeyPress)
        return super().eventFilter(obj, event)

    def _clear_pressed_keys(self):
        """Drop every held key and zero the keyboard input (release safety)."""
        if not self._pressed_actions:
            return
        self._pressed_actions.clear()
        if self.controller is not None:
            self.controller.set_input("keyboard", MotionState())
            self.pad.set_yaw(0.0)

    def focusOutEvent(self, event):
        # Safety: if the ROV window loses OS focus while a movement key is held,
        # clear every pressed action so motion eases back to zero instead of
        # sticking while the operator works in another window.
        self._clear_pressed_keys()
        super().focusOutEvent(event)

    def _update_camera(self):
        frame = self.camera.read_frame() if self.camera is not None else None
        if frame is None:
            return
        if self.hud_provider is not None:
            state = self.hud_provider.get_state()
            self.sonar.set_state(state)
            if self.overlay is not None:
                frame = self.overlay.render(frame, state, extra=self._hud_extra())
        if self._lights:
            frame = np.clip(frame.astype(np.int16) + 36, 0, 255).astype(np.uint8)

        if self._writer is not None:
            self._writer.write(frame)

        self._frames += 1
        now = time.time()
        if now - self._fps_t >= 1.0:
            self._fps = self._frames / (now - self._fps_t)
            self._frames = 0
            self._fps_t = now

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        self._cam_size = (w, h)
        image = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        pix = QPixmap.fromImage(image)
        vs = self.camera_view.size()
        if vs.isValid() and not vs.isEmpty():
            pix = pix.scaled(vs, Qt.KeepAspectRatioByExpanding,
                             Qt.SmoothTransformation)
            if pix.width() > vs.width() or pix.height() > vs.height():
                x = (pix.width() - vs.width()) // 2
                y = (pix.height() - vs.height()) // 2
                pix = pix.copy(x, y, vs.width(), vs.height())
        self.camera_view.setPixmap(pix)

    def _refresh_static(self):
        tcfg = self.gcs.get("telemetry", {})
        self.batt_pill.setText(f"BATT {tcfg.get('bus_voltage', 48.2)}V")
        self.uptime_pill.setText(f"UP {self._uptime_str()}")
        link_ok = self._link_ok()
        self.link_pill.setText("LINK OK" if link_ok else "LINK DOWN")
        self.link_pill.set_status("ok" if link_ok else "bad")
        self.foot["FPS"].setText(f"FPS: {self._fps:.0f}")
        cpu, ram = _load_percent()
        self.foot["CPU"].setText(f"CPU: {cpu}")
        self.foot["RAM"].setText(f"RAM: {ram}")
        disk = "--"
        try:
            usage = shutil.disk_usage(".")
            disk = f"{usage.used / usage.total * 100:.0f}%"
        except OSError:
            pass
        self.foot["STORAGE"].setText(f"STORAGE: {disk}")
        self.foot["DEPTH"].setText(
            f"DEPTH: {self.hud_provider.get_state().depth:.1f}m"
            if self.hud_provider is not None else "DEPTH: --")
        self.foot["LATENCY"].setText(
            f"LATENCY: {self.gcs.get('latency_ms', 12)}ms")
        if self.thrusters is not None:
            status = self.thrusters.get_provider_status()
            if status is not None:
                self.foot["ESP32"].setText(
                    f"ESP32: {status.get('provider', '--').upper()} "
                    f"{'OK' if status.get('link_alive') else ('STALE' if status.get('connected') else 'OFF')}")
            else:
                self.foot["ESP32"].setText("ESP32: SIMULATED")
            self.foot["THRUST"].setText(
                f"THRUST: {_thrust_level(self.thrusters.last_setpoints)}")

        if self._flyout_panel.isVisible():
            self.log_body.setText(_tail(self.log_dir, "mission.log", 8))

        page = self._pages[self._current_section_key()]
        upd = getattr(page, "update", None)
        if callable(upd):
            upd()

    def _refresh_fast(self):
        """100 ms panel refresh: thrusters, motion, link/estop pills."""
        state = self.hud_provider.get_state() if self.hud_provider is not None else None
        if state is not None:
            self.depth_pill.setText(f"DEPTH {state.depth:.1f}m")
            self.depth_pill.set_status("info")
        else:
            self.depth_pill.setText("DEPTH --")

        esp_status = "info"
        esp_text = "ESP32 --"
        if self.thrusters is not None:
            status = self.thrusters.get_provider_status()
            if status is None:
                esp_status, esp_text = "info", "ESP32 SIM"
            elif status.get("link_alive") and status.get("connected"):
                esp_status, esp_text = "ok", "ESP32 OK"
            elif status.get("connected"):
                esp_status, esp_text = "warn", "ESP32 STALE"
            else:
                esp_status, esp_text = "bad", "ESP32 OFF"
        self.esp32_pill.setText(esp_text)
        self.esp32_pill.set_status(esp_status)

        if self.thrusters is not None:
            self._refresh_power_readout()
            for thruster, row in self._motor_rows.items():
                row.set_value(self.thrusters.last_setpoints.get(thruster, 0.0))
            motion = self.controller.last_motion if self.controller is not None else MotionState()
            for axis in ("surge", "yaw", "heave"):
                value = getattr(motion, axis)
                self.motion_rows[axis].set_value(value)
        elif self.controller is not None:
            motion = self.controller.last_motion
            for axis in ("surge", "yaw", "heave"):
                self.motion_rows[axis].set_value(getattr(motion, axis))

    def _current_section_key(self):
        for key, widget in self._pages.items():
            if widget is self.stack.currentWidget():
                return key
        return "dashboard"

    def _hud_extra(self):
        extra = {"mode": "MANUAL", "armed": self._armed, "estop": self._estop_active}
        if self.thrusters is not None:
            status = self.thrusters.get_provider_status()
            if status is None:
                extra["esp32"] = "SIM"
            elif status.get("link_alive") and status.get("connected"):
                extra["esp32"] = "OK"
            elif status.get("connected"):
                extra["esp32"] = "STALE"
            else:
                extra["esp32"] = "OFF"
            extra["thrust"] = self.thrusters.last_setpoints
        else:
            extra["esp32"] = "OFF"
        return extra

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._reposition_overlays()
        self._update_camera()

    def showEvent(self, event):
        super().showEvent(event)
        QTimer.singleShot(0, self._reposition_overlays)

    def closeEvent(self, event):
        self._stop_recording()
        super().closeEvent(event)


def _load_percent():
    try:
        import psutil

        return f"{psutil.cpu_percent():.0f}%", f"{psutil.virtual_memory().percent:.0f}%"
    except Exception:
        return "--", "--"


def _tail(log_dir, name, n=8):
    from collections import deque

    path = os.path.join(log_dir, name)
    try:
        with open(path) as f:
            return "".join(deque(f, n))
    except FileNotFoundError:
        return "(no log yet)"


class _RailFrame(QFrame):
    """Nav rail that expands on hover, collapses on leave.

    The width is applied synchronously via ``sizeHint``/``resize`` instead of
    ``setFixedWidth`` so the layout minimum stays constant (the window never
    grows past the screen when the rail expands).
    """

    expand = None

    def __init__(self, parent=None):
        super().__init__(parent)
        self._rail_width = 56

    def set_expanded_state(self, expanded):
        self._rail_width = 208 if expanded else 56
        self.resize(self._rail_width, self.height())
        self.updateGeometry()

    def sizeHint(self):
        s = super().sizeHint()
        return QSize(self._rail_width, s.height())

    def minimumSizeHint(self):
        return QSize(56, 0)

    def enterEvent(self, event):
        if self.expand is not None:
            self.expand(True)
        super().enterEvent(event)

    def leaveEvent(self, event):
        if self.expand is not None:
            self.expand(False)
        super().leaveEvent(event)


class _ValueRowCompact(QWidget):
    def __init__(self, label, value, pill=False):
        super().__init__()
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 4, 8, 4)
        lay.setSpacing(8)
        lab = caps_label(label)
        self.val = QLabel(value)
        self.val.setStyleSheet(
            f"font-family: {FONT_DATA}; font-size: 12px; color: {C['primary']};"
        )
        lay.addWidget(lab)
        lay.addStretch(1)
        lay.addWidget(self.val)
        self.setStyleSheet(
            f"background: rgba(17, 32, 54, 0.6); border-top: 1px solid "
            f"rgba(100, 255, 218, 0.5); border-radius: 6px;"
        )

    def set_value(self, v):
        self.val.setText(str(v))


class _DockButton(QWidget):
    def __init__(self, glyph, label, danger=False):
        super().__init__()
        self._danger = danger
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)
        self.btn = QPushButton(glyph)
        self.btn.setFixedSize(42, 42)
        self.btn.setCursor(Qt.PointingHandCursor)
        self.lbl = QLabel(label)
        self.lbl.setAlignment(Qt.AlignCenter)
        self.lbl.setStyleSheet(
            f"color: {C['on_surface_variant']}; font-size: 9px; font-weight: 700;"
            f"letter-spacing: 1px;"
        )
        lay.addWidget(self.btn, 0, Qt.AlignHCenter)
        lay.addWidget(self.lbl)
        self.clicked = self.btn.clicked

    def set_active(self, active):
        if self._danger:
            if active:
                bg = "rgba(255, 180, 171, 0.30)"
                border = C['error']
                color = C['error']
            else:
                bg = "rgba(255, 180, 171, 0.08)"
                border = "rgba(255, 180, 171, 0.4)"
                color = C['on_error_container']
        else:
            bg = f"rgba(100, 255, 218, 0.15)" if active else C['surface']
            border = f"{C['primary']}" if active else "rgba(100, 255, 218, 0.2)"
            color = C['primary'] if active else C['on_surface_variant']
        self.btn.setStyleSheet(
            f"background: {bg}; color: {color}; border: 1px solid {border}; "
            f"border-radius: 21px; font-size: 16px;"
        )

    def set_label(self, text):
        self.lbl.setText(text)


class _CenterMeter(QWidget):
    """Bipolar meter that animates motor/thruster output."""

    def __init__(self, low=-1.0, high=1.0, parent=None):
        super().__init__(parent)
        self.low = low
        self.high = high
        self.value = 0.0
        self.setFixedHeight(14)
        self.setMinimumWidth(90)
        self._active = False

    def set_value(self, v):
        v = max(self.low, min(self.high, float(v)))
        self._active = abs(v) > 0.03
        if abs(v - self.value) > 1e-3:
            self.value = v
            self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        h = self.height()
        track_y = (h - 4) // 2
        center_x = w // 2

        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#0a1730"))
        p.drawRoundedRect(0, track_y, w, 4, 2, 2)

        p.setPen(QPen(QColor(100, 255, 218, 80), 1))
        p.drawLine(center_x, track_y + 1, center_x, track_y + 3)

        if abs(self.value) >= 1e-3:
            frac = (self.value - self.low) / max(1e-9, self.high - self.low) - 0.5
            fill_w = int(abs(frac) * w * 0.48)
            fill_x = center_x if frac < 0 else center_x + 1
            color = QColor("#ffd364" if frac < 0 else "#64ffda")
            p.setBrush(color)
            p.drawRoundedRect(fill_x, track_y, fill_w, 4, 2, 2)
        p.end()


class _MotorRow(QWidget):
    def __init__(self, thruster, parent=None):
        super().__init__(parent)
        self.thruster = thruster
        from modules.thrusters.thruster_manager import _ROLES
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 4, 8, 4)
        lay.setSpacing(8)
        role = _ROLES.get(thruster, "")
        parts = role.split()
        first = " ".join(parts[:2])
        rest = parts[2] if len(parts) > 2 else ""
        short = str(thruster.name).split("_")[0]
        text = f"{short} {first}\n{rest}" if rest else f"{short} {first}"
        self.label = QLabel(text)
        self.label.setStyleSheet(
            f"color: {C['on_surface_variant']}; font-weight: 700; font-size: 10px; "
            f"letter-spacing: 1px; line-height: 1.2;"
        )
        self.label.setMinimumWidth(92)
        lay.addWidget(self.label)
        self.meter = _CenterMeter()
        lay.addWidget(self.meter, 1)
        self.value = QLabel("0%")
        self.value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.value.setMinimumWidth(38)
        self.value.setStyleSheet(
            f"font-family: {FONT_DATA}; font-size: 12px; color: {C['primary']};"
        )
        lay.addWidget(self.value)
        self.setStyleSheet(
            "background: rgba(17, 32, 54, 0.6); border-top: 1px solid "
            "rgba(100, 255, 218, 0.5); border-radius: 6px;"
        )

    def set_value(self, v):
        self.meter.set_value(v)
        self.value.setText(f"{int(v * 100):+d}%")
        color = C['error'] if abs(v) > 0.95 else (C['secondary'] if abs(v) < 0.02 else C['primary'])
        self.value.setStyleSheet(f"font-family: {FONT_DATA}; font-size: 12px; color: {color};")


class _AxisRow(QWidget):
    def __init__(self, name, parent=None):
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 4, 8, 4)
        lay.setSpacing(8)
        self.label = QLabel(name.upper())
        self.label.setStyleSheet(
            f"color: {C['on_surface_variant']}; font-weight: 700; font-size: 10px; "
            f"letter-spacing: 1px;"
        )
        self.label.setMinimumWidth(70)
        lay.addWidget(self.label)
        self.meter = _CenterMeter()
        lay.addWidget(self.meter, 1)
        self.value = QLabel("0%")
        self.value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.value.setMinimumWidth(38)
        self.value.setStyleSheet(
            f"font-family: {FONT_DATA}; font-size: 12px; color: {C['secondary']};"
        )
        lay.addWidget(self.value)
        self.setStyleSheet(
            "background: rgba(17, 32, 54, 0.6); border-top: 1px solid "
            "rgba(100, 255, 218, 0.5); border-radius: 6px;"
        )

    def set_value(self, v):
        self.meter.set_value(v)
        self.value.setText(f"{int(v * 100):+d}%")
        color = C['error'] if abs(v) > 0.95 else (C['secondary'] if abs(v) < 0.02 else C['primary'])
        self.value.setStyleSheet(f"font-family: {FONT_DATA}; font-size: 12px; color: {color};")


def _thrust_level(setpoints):
    peak = max(abs(v) for v in setpoints.values()) if setpoints else 0.0
    return f"{int(peak * 100):d}%" if peak > 0.01 else "IDLE"

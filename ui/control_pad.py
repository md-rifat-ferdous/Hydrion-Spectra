"""Circular 3D motion controller (DUBO style).

Stick = surge/yaw (the 5-thruster ROV has no sway). The dashed outer ring
spins to indicate commanded yaw, updated both while dragging and externally
via `set_yaw`. Heave stays on the keyboard (R/F).
"""

import math

from PySide6.QtCore import QPointF, Qt, Signal, QTimer
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget

from modules.controller.controller import MotionState


class ControlPad3D(QWidget):
    """Glass joystick disc with an animated yaw ring."""

    motionChanged = Signal(object)
    yawChanged = Signal(float)

    SIZE = 148
    OUTER = 66
    RADIUS = 46
    KNOB = 15

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(self.SIZE, self.SIZE)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._joy = QPointF()
        self._drag = False
        self._ring_angle = 0.0
        self._yaw_cmd = 0.0

        self._anim = QTimer(self)
        self._anim.timeout.connect(self._tick)
        self._anim.start(50)

    def set_yaw(self, value):
        self._yaw_cmd = max(-1.0, min(1.0, value))

    def reset(self):
        self._joy = QPointF()
        self._yaw_cmd = 0.0
        self.update()

    def _tick(self):
        if self._yaw_cmd:
            self._ring_angle += 1.6 * self._yaw_cmd
            self.update()

    def _emit(self):
        yaw = float(self._joy.x())
        self._yaw_cmd = yaw
        self.update()
        self.motionChanged.emit(
            MotionState(surge=-self._joy.y(), yaw=yaw)
        )

    def _set_joy(self, pos):
        dx = (pos.x() - self.SIZE / 2) / self.RADIUS
        dy = (pos.y() - self.SIZE / 2) / self.RADIUS
        length = math.hypot(dx, dy)
        if length > 1.0:
            dx, dy = dx / length, dy / length
        self._joy = QPointF(dx, dy)
        self._emit()
        self.update()

    def mousePressEvent(self, event):
        c = QPointF(self.SIZE / 2, self.SIZE / 2)
        if (event.position() - c).manhattanLength() <= self.RADIUS + 12:
            self._drag = True
            self._set_joy(event.position())
        event.accept()

    def mouseMoveEvent(self, event):
        if self._drag:
            self._set_joy(event.position())
        event.accept()

    def mouseReleaseEvent(self, event):
        if self._drag:
            self._drag = False
            self._joy = QPointF()
            self._emit()
            self.update()
        event.accept()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        cx = cy = self.SIZE / 2

        p.setBrush(QColor(17, 34, 64, 185))
        p.setPen(QPen(QColor(100, 255, 218, 60), 1))
        p.drawEllipse(QPointF(cx, cy), self.OUTER, self.OUTER)
        p.setPen(QPen(QColor(100, 255, 218, 140), 1))
        p.drawEllipse(QPointF(cx, cy), self.OUTER - 3, self.OUTER - 3)

        p.save()
        p.translate(cx, cy)
        p.rotate(self._ring_angle)
        dash = QPen(QColor(100, 255, 218, 90), 1)
        dash.setDashPattern([4, 5])
        p.setPen(dash)
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(0, 0), self.OUTER - 9, self.OUTER - 9)
        p.restore()

        p.setBrush(QColor(39, 53, 76, 220))
        p.setPen(QPen(QColor(100, 255, 218, 80), 1))
        p.drawEllipse(QPointF(cx, cy), self.RADIUS, self.RADIUS)

        p.setPen(QPen(QColor(100, 255, 218, 110), 1))
        p.drawLine(int(cx - self.RADIUS), int(cy), int(cx + self.RADIUS), int(cy))
        p.drawLine(int(cx), int(cy - self.RADIUS), int(cx), int(cy + self.RADIUS))

        knob = self._joy * (self.RADIUS - self.KNOB - 4)
        p.setBrush(QColor(100, 255, 218, 70))
        p.setPen(QPen(QColor(100, 255, 218, 220), 2))
        p.drawEllipse(QPointF(cx + knob.x(), cy + knob.y()), self.KNOB, self.KNOB)

        p.setPen(QPen(QColor(186, 202, 195, 200), 1))
        small = p.font()
        small.setPointSize(8)
        small.setBold(True)
        p.setFont(small)
        p.drawText(0, 10, self.SIZE, 14, Qt.AlignHCenter, "FWD")
        p.drawText(0, self.SIZE - 24, self.SIZE, 14, Qt.AlignHCenter, "REV")
        p.drawText(8, 0, 14, self.SIZE, Qt.AlignVCenter, "L")
        p.drawText(self.SIZE - 22, 0, 14, self.SIZE, Qt.AlignVCenter, "R")
        p.end()

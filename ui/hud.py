"""HUD overlays drawn into the video frame (DUBO style).

Top-center: heading compass. Center: crosshair + pitch ladder. Left-middle:
depth pillar. Right-middle: altitude pillar. (The sonar map lives in
`ui/sonar.py` as a Qt widget so it stays aligned with the other panels.)
"""

import cv2
import numpy as np

from modules.sensors.sensor_manager import HudState

CYAN = (218, 255, 100)          # BGR of #64FFDA
BLUE = (255, 199, 173)          # BGR of #adc7ff
ON_SURFACE = (255, 227, 214)    # BGR of #d6e3ff
VARIANT = (195, 202, 186)       # BGR of #bacac3
PANEL_BG = (64, 34, 17)         # BGR of #112240


class HudOverlay:
    def __init__(self, config):
        cfg = config.get("hud", {})
        self.enabled = cfg.get("enabled", True)
        self.panel_alpha = cfg.get("panel_alpha", 0.55)
        self.crosshair_enabled = cfg.get("crosshair", {}).get("enabled", True)
        self.compass_enabled = cfg.get("compass", {}).get("enabled", True)
        self.depth_alt_enabled = cfg.get("depth_alt", {}).get("enabled", True)
        self.extra_enabled = cfg.get("extra", {}).get("enabled", True)

        colors = cfg.get("colors", {})
        self._c = {
            "cyan": self._bgr(colors.get("primary", [100, 255, 218])),
            "blue": self._bgr(colors.get("secondary", [173, 199, 255])),
            "text": self._bgr(colors.get("on_surface", [214, 227, 255])),
            "variant": self._bgr(colors.get("on_surface_variant", [186, 202, 195])),
            "bg": self._bgr(colors.get("bg", [17, 34, 64])),
            "bg_high": self._bgr(colors.get("bg_high", [39, 53, 76])),
        }

    @staticmethod
    def _bgr(rgb):
        return tuple(int(v) for v in reversed(rgb[:3]))

    def update(self, state):
        pass

    def _text(self, frame, text, org, color=None, scale=0.45, thickness=1):
        cv2.putText(frame, text, (org[0] + 1, org[1] + 1),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 1,
                    cv2.LINE_AA)
        cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    color or self._c["text"], thickness, cv2.LINE_AA)

    @staticmethod
    def _rounded_mask(h, w, x0, y0, x1, y1, r):
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.rectangle(mask, (x0 + r, y0), (x1 - r, y1), 255, -1)
        cv2.rectangle(mask, (x0, y0 + r), (x1, y1 - r), 255, -1)
        cv2.circle(mask, (x0 + r, y0 + r), r, 255, -1)
        cv2.circle(mask, (x1 - r, y0 + r), r, 255, -1)
        cv2.circle(mask, (x0 + r, y1 - r), r, 255, -1)
        cv2.circle(mask, (x1 - r, y1 - r), r, 255, -1)
        return mask

    def _panel(self, frame, x0, y0, x1, y1, radius=8, alpha=None, border=None):
        h, w = frame.shape[:2]
        x0, x1 = sorted((max(0, x0), min(w, x1)))
        y0, y1 = sorted((max(0, y0), min(h, y1)))
        if x1 - x0 < 4 or y1 - y0 < 4:
            return
        mask = self._rounded_mask(h, w, x0, y0, x1, y1, radius)
        blend = cv2.addWeighted(frame, 1 - (alpha or self.panel_alpha),
                                np.full((h, w, 3), self._c["bg"], dtype=np.uint8),
                                alpha or self.panel_alpha, 0)
        frame[mask > 0] = blend[mask > 0]
        erode = cv2.erode(mask, np.ones((3, 3), np.uint8))
        edge = mask - erode
        frame[edge > 0] = border or self._c["cyan"]

    def render(self, frame, state, extra=None):
        if not self.enabled or frame is None:
            return frame
        h, w = frame.shape[:2]
        if h < 300 or w < 400:
            return frame
        if self.compass_enabled:
            self._draw_compass(frame, state, w)
        if self.crosshair_enabled:
            self._draw_crosshair(frame, h, w)
        if self.depth_alt_enabled:
            self._draw_depth(frame, state, h)
            self._draw_alt(frame, state, h, w)
        if self.extra_enabled and extra:
            self._draw_status_chips(frame, extra, w)
        return frame

    def _draw_compass(self, frame, state, w):
        cx = w // 2
        vals = [state.yaw_deg + d for d in (-20, -10, 0, 10, 20)]
        widths = [46, 42, 46, 42, 46]
        total = sum(widths) + 40
        x0 = cx - total // 2
        y0 = 10
        self._panel(frame, x0, y0, x0 + total, y0 + 30, radius=10)
        x = x0 + 10
        self._text(frame, "HDG", (x, y0 + 19), self._c["variant"], 0.4, 1)
        x += 34
        for i, v in enumerate(vals):
            hdg = v % 360
            if i == 2:
                self._text(frame, f"{hdg:03.0f}", (x, y0 + 21),
                           self._c["text"], 0.55, 2)
            else:
                self._text(frame, f"{hdg:03.0f}", (x, y0 + 19),
                           self._c["variant"], 0.4, 1)
            x += widths[i]

    def _draw_crosshair(self, frame, h, w):
        cx, cy = w // 2, h // 2
        r = 44
        cv2.circle(frame, (cx, cy), r, self._c["cyan"], 1, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), 2, self._c["cyan"], -1, cv2.LINE_AA)
        cv2.line(frame, (cx, cy - r + 8), (cx, cy - r + 24), self._c["cyan"], 1)
        cv2.line(frame, (cx, cy + r - 8), (cx, cy + r - 24), self._c["cyan"], 1)
        cv2.line(frame, (cx - r + 8, cy), (cx - r + 24, cy), self._c["cyan"], 1)
        cv2.line(frame, (cx + r - 8, cy), (cx + r - 24, cy), self._c["cyan"], 1)
        for off, label in ((-24, "+10"), (24, "-10")):
            y = cy - off
            cv2.line(frame, (cx - 42, y), (cx - 30, y), self._c["cyan"], 1)
            cv2.line(frame, (cx + 42, y), (cx + 30, y), self._c["cyan"], 1)
            if off < 0:
                cv2.line(frame, (cx - 34, y), (cx - 34, y), self._c["cyan"], 1)
            self._text(frame, label, (cx - 62, y + 5), self._c["cyan"], 0.35, 1)

    def _draw_depth(self, frame, state, h):
        yc = h // 2
        x0 = 10
        pw, ph = 84, 130
        self._panel(frame, x0, yc - ph // 2, x0 + pw, yc + ph // 2, radius=10)
        self._text(frame, "DEPTH", (x0 + 18, yc - 38), self._c["variant"], 0.4, 1)
        self._text(frame, f"{state.depth:5.1f}", (x0 + 20, yc + 4),
                   self._c["cyan"], 0.6, 2)
        self._text(frame, "m", (x0 + 36, yc + 30), self._c["variant"], 0.4, 1)

    def _draw_alt(self, frame, state, h, w):
        yc = h // 2
        pw, ph = 84, 130
        x1 = w - 10
        y0 = yc - ph // 2
        self._panel(frame, x1 - pw, y0, x1, y0 + ph, radius=10)
        self._text(frame, "ALT", (x1 - pw + 28, y0 + 24), self._c["variant"], 0.4, 1)
        self._text(frame, f"{state.alt:4.1f}", (x1 - pw + 24, y0 + 52), self._c["blue"], 0.6, 2)
        self._text(frame, "m", (x1 - pw + 34, y0 + 78), self._c["variant"], 0.4, 1)

    def _draw_status_chips(self, frame, extra, w):
        import cv2 as _cv2
        esp32 = extra.get("esp32", "OFF").upper()
        estop = "E-STOP" if extra.get("estop") else "NORMAL"
        mode = extra.get("mode", "MANUAL")
        parts = [f"{mode}"]
        if extra.get("armed"):
            parts.append("ARMED")
        parts.append(f"ESP {esp32}")
        parts.append(estop)
        text = "  ".join(parts)
        pw = 40 + len(text) * 8
        x0, y0 = 10, 52
        self._panel(frame, x0, y0, min(w - 10, x0 + pw), y0 + 22, radius=8, alpha=0.7)
        color = (0, 0, 255) if extra.get("estop") else self._c["cyan"]
        self._text(frame, text, (x0 + 10, y0 + 16), color, 0.38, 1)


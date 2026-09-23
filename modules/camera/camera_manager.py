import time

import cv2

from core.base_module import BaseModule


class CameraManager(BaseModule):
    def __init__(self, config=None):
        super().__init__("Camera")
        self.config = config or {}
        self.capture = None
        self._last_open_attempt = 0.0

    def _open_capture(self):
        device = self.config.get("device", 0)
        width = self.config.get("width", 1280)
        height = self.config.get("height", 720)
        fps = self.config.get("fps", 30)

        self.capture = cv2.VideoCapture(device)
        if not self.capture.isOpened():
            self.logger.error("Failed to open camera device%s",
                              f" ({device})" if device != 0 else "")
            self.capture = None
            return False

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, fps)
        self.logger.info("Camera opened (device %s)", device)
        return True

    def initialize(self):
        super().initialize()
        if self._open_capture():
            return True

        # Prefer keeping the sim/app alive: if no camera is present right now
        # (e.g. USB cam unplugged at boot) we report healthy and let
        # read_frame() keep retrying until a camera appears.
        self._last_open_attempt = time.monotonic()
        self.logger.warning(
            "Camera not connected yet; feed will appear when it is plugged in"
        )
        return True

    def read_frame(self):
        if self.capture is None:
            interval = float(self.config.get("reconnect_interval_s", 2.0))
            now = time.monotonic()
            if now - self._last_open_attempt < interval:
                return None
            self._last_open_attempt = now
            if not self._open_capture():
                return None
        ok, frame = self.capture.read()
        if not ok:
            self.capture = None
            self.logger.warning("Camera frame read failed; retrying on next tick")
            return None
        return frame

    def stop(self):
        super().stop()
        if self.capture is not None:
            self.capture.release()
            self.capture = None
            self.logger.info("Camera released")

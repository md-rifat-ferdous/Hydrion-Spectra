"""Keyboard control path tests for the ROV Auto UI.

Verifies the full keyboard -> ControllerModule key/button path exactly as the
GUI wires it (ui/main_window.py `_build_keymap` + `_handle_key`):

    W/S  surge forward/back
    A/D  yaw left/right   (5-thruster layout: LEFT/RIGHT turn, no sway)
    Q/E  yaw alternative, already supported by the controller
    R/F  heave up/down
    Shift boost
    Backspace kill / E-STOP

It also proves a held-then-released key returns the command safely to zero
(the gradual power ramp itself lives in ThrusterManager and is tested in
tests/test_thruster_mixer.py).

Runs headless (QT_QPA_PLATFORM=offscreen); skipped if Qt is unavailable.

Run from the project root:

    python -m unittest tests.test_controller_input -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from modules.config.config_manager import ConfigManager
from modules.controller.controller import ControllerModule, MotionState
from modules.thrusters.thruster_manager import ThrusterManager

try:
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import QApplication
except ImportError:  # pragma: no cover - Qt not installed
    Qt = None
    QApplication = None

THRUSTER_CFG = {
    "provider": "simulated",
    "throttle_mode": "high",
    "motors": {
        "M1_FRONT_VERTICAL": {"gpio": 25, "direction": 1},
        "M2_MIDDLE_RIGHT_HORIZONTAL": {"gpio": 33, "direction": 1},
        "M3_MIDDLE_LEFT_HORIZONTAL": {"gpio": 32, "direction": 1},
        "M4_BACK_RIGHT_VERTICAL": {"gpio": 27, "direction": 1},
        "M5_BACK_LEFT_VERTICAL": {"gpio": 26, "direction": 1},
    },
}

EXPECTED_BINDINGS = {
    "forward": "W",
    "back": "S",
    "left": "A",
    "right": "D",
    "up": "R",
    "down": "F",
    "yaw_left": "Q",
    "yaw_right": "E",
    "boost": "Shift",
    "kill": "Backspace",
}


@unittest.skipUnless(Qt is not None and QApplication is not None,
                     "PySide6 not installed")
class KeyboardControlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        cm = ConfigManager()
        cm.initialize()
        cls.keymap = cm.get_section("controller").get("keymap", {})
        cls.controller = ControllerModule({"enabled": True, "key_speed": 1.0})
        cls.controller.start()
        cls.thrusters = ThrusterManager(dict(THRUSTER_CFG), sensors=None)
        cls.thrusters.initialize()
        cls.thrusters.start()
        from ui.main_window import MainWindow

        cls.win = MainWindow(
            camera_manager=None,
            controller=cls.controller,
            keymap=cls.keymap,
            thrusters=cls.thrusters,
            gcs_config={},
        )

    @classmethod
    def tearDownClass(cls):
        cls.win.close()
        cls.thrusters.stop()

    def _code(self, name):
        return getattr(Qt.Key, f"Key_{name}")

    def _keyboard(self):
        return self.controller._inputs.get("keyboard", MotionState())

    def _press(self, name):
        self.win._handle_key(self._code(name), True)

    def _release(self, name):
        self.win._handle_key(self._code(name), False)

    def test_keymap_loads_all_expected_bindings(self):
        for action, name in EXPECTED_BINDINGS.items():
            self.assertEqual(self.keymap.get(action), name)
            self.assertEqual(
                self.win._key_actions.get(self._code(name)), action
            )
        # Q/E yaw alternatives live in controller.yaml too.
        self.assertEqual(self.keymap.get("yaw_left"), "Q")
        self.assertEqual(self.keymap.get("yaw_right"), "E")

    def test_w_surge_forward(self):
        self._press("W")
        self.assertEqual(self._keyboard().surge, 1.0)
        self._release("W")

    def test_s_surge_backward(self):
        self._press("S")
        self.assertEqual(self._keyboard().surge, -1.0)
        self._release("S")

    def test_a_yaw_left(self):
        self._press("A")
        self.assertEqual(self._keyboard().yaw, -1.0)
        self.assertEqual(self._keyboard().surge, 0.0)
        self._release("A")

    def test_d_yaw_right(self):
        self._press("D")
        self.assertEqual(self._keyboard().yaw, 1.0)
        self._release("D")

    def test_q_and_e_yaw_alternatives(self):
        self._press("Q")
        self.assertEqual(self._keyboard().yaw, -1.0)
        self._release("Q")
        self._press("E")
        self.assertEqual(self._keyboard().yaw, 1.0)
        self._release("E")

    def test_r_heave_up(self):
        self._press("R")
        self.assertEqual(self._keyboard().heave, 1.0)
        self._release("R")

    def test_f_heave_down(self):
        self._press("F")
        self.assertEqual(self._keyboard().heave, -1.0)
        self._release("F")

    def test_key_release_returns_to_zero(self):
        self._press("W")
        self._press("A")
        self._press("R")
        state = self._keyboard()
        self.assertEqual((state.surge, state.yaw, state.heave), (1.0, -1.0, 1.0))
        self._release("W")
        self._release("A")
        self._release("R")
        state = self._keyboard()
        self.assertEqual((state.surge, state.yaw, state.heave), (0.0, 0.0, 0.0))

    def test_shift_boost(self):
        self._press("W")
        self._press("Shift")
        self._press("D")
        self.assertTrue(self._keyboard().boost)
        self.assertEqual(self._keyboard().yaw, 1.0)
        self._release("Shift")
        self._release("D")
        self._release("W")

    def test_backspace_estop(self):
        self.assertEqual(self.win._estop_active, False)
        self.win._handle_key(self._code("Backspace"), True)
        self.assertEqual(self.win._estop_active, True)
        self.assertTrue(self.controller._killed)
        # All thrusters zeroed by the emergency stop.
        self.thrusters.update()
        self.assertTrue(all(v == 0.0 for v in self.thrusters.last_setpoints.values()))
        # RE-ARM restores control (mirrors the dock RE-ARM button).
        self.win._rearm()
        self.assertEqual(self.win._estop_active, False)

    def test_controller_merges_keyboard_into_last_motion(self):
        self._press("R")
        self.controller.update()
        self.assertEqual(self.controller.last_motion.heave, 1.0)
        self._release("R")


@unittest.skipUnless(Qt is not None and QApplication is not None,
                     "PySide6 not installed")
class KeyboardSafetyTest(unittest.TestCase):
    """Focus-loss must clear held keys so motion cannot stick."""

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        cls.controller = ControllerModule({"enabled": True, "key_speed": 1.0})
        keymap = {
            "forward": "W",
            "back": "S",
            "left": "A",
            "right": "D",
            "up": "R",
            "down": "F",
            "yaw_left": "Q",
            "yaw_right": "E",
            "boost": "Shift",
            "kill": "Backspace",
        }
        from ui.main_window import MainWindow

        cls.win = MainWindow.__new__(MainWindow)
        cls.win.key_speed = 1.0
        cls.win._pressed_actions = set()
        cls.win._key_actions = cls.win._build_keymap(keymap)
        cls.win.controller = cls.controller
        cls.win._armed = True
        cls.win.pad = type("FakePad", (), {"set_yaw": lambda self, v: None})()
        cls.win._estop_active = False

    def test_focus_loss_clears_pressed_actions(self):
        self.win._handle_key(Qt.Key.Key_W, True)
        self.win._handle_key(Qt.Key.Key_A, True)
        self.assertIn("forward", self.win._pressed_actions)
        # Simulate the ROV window losing OS focus.
        self.win._clear_pressed_keys()
        self.assertEqual(self.win._pressed_actions, set())
        state = self.controller._inputs.get("keyboard")
        self.assertEqual((state.surge, state.yaw), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
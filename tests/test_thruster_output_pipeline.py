"""W-key -> thruster output pipeline regression tests (DUBO / Hydrion-Spectra).

Locks the exact bug reported: pressing W moved SURGE to +100% in the GUI while
M1..M5 stayed at 0%. These tests drive the REAL keyboard path (MainWindow
``_handle_key``) -> ControllerModule -> ThrusterManager -> mixer -> power/ramp,
and assert the internal thruster setpoints (what the GUI meters show) actually
climb. Safety rails tested too: E-STOP forces all five to zero and releasing a
key eases the outputs back to zero.

Run from the project root:

    python -m unittest tests.test_thruster_output_pipeline -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from modules.controller.controller import ControllerModule, MotionState
from modules.thrusters.thruster_manager import ThrusterId, ThrusterManager

try:
    from PySide6.QtCore import Qt
    from PySide6.QtWidgets import QApplication
except ImportError:  # pragma: no cover - Qt not installed
    Qt = None
    QApplication = None

M1 = ThrusterId.M1_FRONT_VERTICAL
M2 = ThrusterId.M2_MIDDLE_RIGHT_HORIZONTAL
M3 = ThrusterId.M3_MIDDLE_LEFT_HORIZONTAL
M4 = ThrusterId.M4_BACK_RIGHT_VERTICAL
M5 = ThrusterId.M5_BACK_LEFT_VERTICAL
ALL = (M1, M2, M3, M4, M5)

THRUSTER_CFG = {
    "provider": "simulated",
    "throttle_mode": "soft",
    "throttle_ramp_per_sec": 1.0,
    "motors": {
        "M1_FRONT_VERTICAL": {"gpio": 25, "direction": 1},
        "M2_MIDDLE_RIGHT_HORIZONTAL": {"gpio": 33, "direction": 1},
        "M3_MIDDLE_LEFT_HORIZONTAL": {"gpio": 32, "direction": 1},
        "M4_BACK_RIGHT_VERTICAL": {"gpio": 27, "direction": 1},
        "M5_BACK_LEFT_VERTICAL": {"gpio": 26, "direction": 1},
    },
}

_ZERO = {t: 0.0 for t in ALL}


@unittest.skipUnless(Qt is not None and QApplication is not None,
                     "PySide6 not installed")
class ThrusterOutputPipelineTest(unittest.TestCase):
    """Keyboard W/S/A/D/Q/E/R/F + release + E-STOP -> real ramped setpoints."""

    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        from modules.config.config_manager import ConfigManager
        from ui.main_window import MainWindow

        cm = ConfigManager()
        cm.initialize()
        cls.keymap = cm.get_section("controller").get("keymap", {})
        cls.MainWindow = MainWindow

    def setUp(self):
        self.thrusters = ThrusterManager(dict(THRUSTER_CFG), sensors=None)
        self.thrusters.initialize()
        self.thrusters.start()
        self.controller = ControllerModule(
            {"enabled": True, "key_speed": 1.0}, thrusters=self.thrusters
        )
        self.controller.start()
        self.win = self.MainWindow(
            camera_manager=None,
            controller=self.controller,
            keymap=self.keymap,
            thrusters=self.thrusters,
            gcs_config={},
        )

    def tearDown(self):
        self.win.close()
        self.thrusters.stop()
        self.controller.stop()

    def _press(self, name):
        self.win._handle_key(getattr(Qt.Key, f"Key_{name}"), True)

    def _release(self, name):
        self.win._handle_key(getattr(Qt.Key, f"Key_{name}"), False)

    def _tick(self, n=1):
        """One ServiceManager-style update cycle (controller first, thrusters)."""
        for _ in range(n):
            self.controller.update()
            self.thrusters.update()

    # TEST 1 + 2 + 3 -------------------------------------------------------

    def test_w_hold_ramps_m2_m3_and_keeps_verticals_at_zero(self):
        # TEST 2 (soft ramp trajectory): 0.0 -> 0.1 -> 0.2 -> 0.3 -> 0.4 -> hold.
        self._press("W")
        self.controller.update()  # keyboard is merged + pushed into thrusters
        seq = []
        for _ in range(7):
            self.thrusters.update()
            seq.append(round(self.thrusters.last_setpoints[M2], 6))
        self._release("W")
        self.assertEqual(seq, [0.0, 0.1, 0.2, 0.3, 0.4, 0.4, 0.4])

        # TEST 1: W eventually produces real (non-zero) M2/M3 setpoints.
        self.assertGreater(self.thrusters.last_setpoints[M2], 0.0)
        self.assertGreater(self.thrusters.last_setpoints[M3], 0.0)

        # TEST 3: pure surge never touches the vertical thrusters.
        self.assertEqual(self.thrusters.last_setpoints[M1], 0.0)
        self.assertEqual(self.thrusters.last_setpoints[M4], 0.0)
        self.assertEqual(self.thrusters.last_setpoints[M5], 0.0)

    # TEST 4 ---------------------------------------------------------------

    def test_s_produces_negative_m2_m3(self):
        self._press("S")
        self.controller.update()
        for _ in range(6):
            self.thrusters.update()
        self._release("S")
        self.assertLess(self.thrusters.last_setpoints[M2], 0.0)
        self.assertLess(self.thrusters.last_setpoints[M3], 0.0)
        self.assertEqual(self.thrusters.last_setpoints[M1], 0.0)

    # TEST 5 ---------------------------------------------------------------

    def test_release_w_returns_outputs_to_zero(self):
        self._press("W")
        self.controller.update()
        for _ in range(6):  # climb to the 0.40 soft ceiling
            self.thrusters.update()
        self.assertAlmostEqual(self.thrusters.last_setpoints[M2], 0.40, places=6)

        self._release("W")
        self._tick()  # controller now pushes a zeroed MotionState
        for _ in range(6):  # ease back down 0.4 -> ... -> 0.0
            self.thrusters.update()
        self.assertTrue(all(abs(v) <= 1e-6 for v in self.thrusters.last_setpoints.values()))

    # TEST 6 ---------------------------------------------------------------

    def test_estop_forces_all_five_outputs_to_zero(self):
        self._press("W")
        self.controller.update()
        for _ in range(5):
            self.thrusters.update()
        self.assertGreater(self.thrusters.last_setpoints[M2], 0.0)

        self.win._estop()  # same path as Backspace / dock E-STOP button
        self.thrusters.update()
        self.assertEqual(self.thrusters.last_setpoints, _ZERO)
        for _ in range(3):
            self.thrusters.update()
        self.assertEqual(self.thrusters.last_setpoints, _ZERO)

    # requirement 7: yaw + heave directions --------------------------------

    def test_a_yaw_left_gives_differential_m2_m3(self):
        self._press("A")
        self.controller.update()
        for _ in range(6):
            self.thrusters.update()
        self._release("A")
        self.assertLess(self.thrusters.last_setpoints[M2], 0.0)
        self.assertGreater(self.thrusters.last_setpoints[M3], 0.0)
        self.assertEqual(self.thrusters.last_setpoints[M1], 0.0)

    def test_d_yaw_right_gives_differential_m2_m3(self):
        self._press("D")
        self.controller.update()
        for _ in range(6):
            self.thrusters.update()
        self._release("D")
        self.assertGreater(self.thrusters.last_setpoints[M2], 0.0)
        self.assertLess(self.thrusters.last_setpoints[M3], 0.0)

    def test_q_yaw_left_alternative(self):
        self._press("Q")
        self.controller.update()
        for _ in range(6):
            self.thrusters.update()
        self._release("Q")
        self.assertLess(self.thrusters.last_setpoints[M2], 0.0)
        self.assertGreater(self.thrusters.last_setpoints[M3], 0.0)

    def test_e_yaw_right_alternative(self):
        self._press("E")
        self.controller.update()
        for _ in range(6):
            self.thrusters.update()
        self._release("E")
        self.assertGreater(self.thrusters.last_setpoints[M2], 0.0)
        self.assertLess(self.thrusters.last_setpoints[M3], 0.0)

    def test_r_heave_up_drives_m1_m4_m5_positive(self):
        self._press("R")
        self.controller.update()
        for _ in range(6):
            self.thrusters.update()
        self._release("R")
        for thruster in (M1, M4, M5):
            self.assertGreater(self.thrusters.last_setpoints[thruster], 0.0)
        self.assertEqual(self.thrusters.last_setpoints[M2], 0.0)
        self.assertEqual(self.thrusters.last_setpoints[M3], 0.0)

    def test_f_heave_down_drives_m1_m4_m5_negative(self):
        self._press("F")
        self.controller.update()
        for _ in range(6):
            self.thrusters.update()
        self._release("F")
        for thruster in (M1, M4, M5):
            self.assertLess(self.thrusters.last_setpoints[thruster], 0.0)
        self.assertEqual(self.thrusters.last_setpoints[M2], 0.0)
        self.assertEqual(self.thrusters.last_setpoints[M3], 0.0)

    # final requirement: the GUI meters must move, not just SURGE ----------

    def test_gui_meters_reflect_ramped_m2_m3_not_raw_surge(self):
        self._press("W")
        self.controller.update()
        for _ in range(7):
            self.thrusters.update()
        self.win._refresh_fast()  # the 100 ms dashboard panel update
        self.assertEqual(self.win._motor_rows[M2].value.text(), "+40%")
        self.assertEqual(self.win._motor_rows[M3].value.text(), "+40%")
        self.assertEqual(self.win._motor_rows[M1].value.text(), "+0%")
        # SURGE shows the requested +100%, which is expected and distinct.
        self.assertEqual(self.win.motion_rows["surge"].value.text(), "+100%")
        self._release("W")


if __name__ == "__main__":
    unittest.main(verbosity=2)
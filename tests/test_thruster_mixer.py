"""Unit tests for the 5-thruster mixer and ThrusterManager (Phase 1).

Run from the project root:

    python -m unittest tests.test_thruster_mixer -v
    # or, if pytest is available:
    python -m pytest tests/test_thruster_mixer.py -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from modules.controller.controller import MotionState
from modules.thrusters.thruster_manager import (
    ThrusterId,
    ThrusterManager,
    mix,
)

M1 = ThrusterId.M1_FRONT_VERTICAL
M2 = ThrusterId.M2_MIDDLE_RIGHT_HORIZONTAL
M3 = ThrusterId.M3_MIDDLE_LEFT_HORIZONTAL
M4 = ThrusterId.M4_BACK_RIGHT_VERTICAL
M5 = ThrusterId.M5_BACK_LEFT_VERTICAL

_ALL_IDS = (M1, M2, M3, M4, M5)
_ZERO = {t: 0.0 for t in _ALL_IDS}


class FiveThrusterMixerTest(unittest.TestCase):
    def assertMix(self, expected, **kwargs):
        result = mix(**kwargs)
        self.assertEqual(result, expected)

    def test_neutral(self):
        self.assertMix(_ZERO)
        self.assertMix(_ZERO, surge=0.0, yaw=0.0, heave=0.0)

    def test_stop_is_five_zeros(self):
        result = mix(MotionState(surge=1.0, yaw=1.0, heave=1.0))
        self.assertNotEqual(result, _ZERO)
        self.assertMix(_ZERO, surge=0.0, yaw=0.0, heave=0.0)

    def test_forward(self):
        self.assertMix({M1: 0.0, M2: 1.0, M3: 1.0, M4: 0.0, M5: 0.0}, surge=1)

    def test_forward_from_motion(self):
        result = mix(MotionState(surge=1.0))
        self.assertEqual(result, {M1: 0.0, M2: 1.0, M3: 1.0, M4: 0.0, M5: 0.0})

    def test_backward(self):
        self.assertMix({M1: 0.0, M2: -1.0, M3: -1.0, M4: 0.0, M5: 0.0}, surge=-1)

    def test_yaw_left(self):
        self.assertMix({M1: 0.0, M2: -1.0, M3: 1.0, M4: 0.0, M5: 0.0}, yaw=-1)

    def test_yaw_right(self):
        self.assertMix({M1: 0.0, M2: 1.0, M3: -1.0, M4: 0.0, M5: 0.0}, yaw=1)

    def test_up(self):
        self.assertMix({M1: 1.0, M2: 0.0, M3: 0.0, M4: 1.0, M5: 1.0}, heave=1)

    def test_down(self):
        self.assertMix({M1: -1.0, M2: 0.0, M3: 0.0, M4: -1.0, M5: -1.0}, heave=-1)

    def test_forward_yaw(self):
        self.assertMix({M1: 0.0, M2: 1.0, M3: 0.0, M4: 0.0, M5: 0.0}, surge=1, yaw=1)

    def test_forward_up(self):
        self.assertMix({M1: 1.0, M2: 1.0, M3: 1.0, M4: 1.0, M5: 1.0}, surge=1, heave=1)

    def test_yaw_up(self):
        self.assertMix({M1: 1.0, M2: 1.0, M3: -1.0, M4: 1.0, M5: 1.0}, yaw=1, heave=1)

    def test_forward_yaw_up(self):
        self.assertMix({M1: 1.0, M2: 1.0, M3: 0.0, M4: 1.0, M5: 1.0}, surge=1, yaw=1, heave=1)

    def test_saturation_clamping(self):
        for kwargs in (
            dict(surge=1.0, yaw=1.0),
            dict(surge=0.7, yaw=0.7),
            dict(surge=1.0, yaw=1.0, heave=-1.0),
            dict(surge=-1.0, yaw=-1.0, heave=1.0),
        ):
            result = mix(**kwargs)
            self.assertEqual(len(result), 5)
            for thruster in _ALL_IDS:
                self.assertGreaterEqual(result[thruster], -1.0)
                self.assertLessEqual(result[thruster], 1.0)

    def test_unsupported_sway_produces_no_thrust(self):
        self.assertEqual(mix(MotionState(sway=1.0)), _ZERO)

    def test_unsupported_sway_from_motion(self):
        self.assertEqual(mix(MotionState(sway=1.0)), _ZERO)

    def test_unsupported_pitch_produces_no_thrust(self):
        self.assertEqual(mix(MotionState(pitch=1.0)), _ZERO)

    def test_unsupported_roll_produces_no_thrust(self):
        self.assertEqual(mix(MotionState(roll=1.0)), _ZERO)

    def test_always_returns_exactly_five_motor_outputs(self):
        result = mix(MotionState(surge=0.3, yaw=-0.4, heave=0.2))
        self.assertEqual(len(result), 5)
        self.assertEqual(set(result.keys()), set(_ALL_IDS))

    def test_direction_flip(self):
        flip = {M2: -1, M3: -1}
        result = mix(surge=1.0, directions=flip)
        self.assertEqual(result[M2], -1.0)
        self.assertEqual(result[M3], -1.0)
        self.assertEqual(result[M1], 0.0)

    def test_no_directions_is_all_positive(self):
        result = mix(surge=1.0)
        self.assertEqual(result[M2], 1.0)
        self.assertEqual(result[M3], 1.0)


class ThrusterManagerTest(unittest.TestCase):
    def _manager(self):
        config = {
            "provider": "simulated",
            "motors": {
                "M1_FRONT_VERTICAL": {"gpio": 25, "direction": 1},
                "M2_MIDDLE_RIGHT_HORIZONTAL": {"gpio": 33, "direction": 1},
            },
        }
        mgr = ThrusterManager(config, sensors=None)
        self.assertTrue(mgr.initialize())
        mgr.start()
        return mgr

    def test_forward_setpoints(self):
        mgr = self._manager()
        mgr.set_throttle_mode("high")
        mgr.set_motion(MotionState(surge=1.0))
        for _ in range(15):
            mgr.update()
        self.assertEqual(mgr.last_setpoints[M2], 1.0)
        self.assertEqual(mgr.last_setpoints[M3], 1.0)
        self.assertEqual(mgr.last_setpoints[M1], 0.0)

    def test_throttle_mode_limits_max_output(self):
        mgr = self._manager()
        mgr.set_throttle_mode("soft")
        self.assertEqual(mgr.throttle_limit, 0.40)
        mgr.set_motion(MotionState(surge=1.0))
        for _ in range(30):
            mgr.update()
        self.assertAlmostEqual(mgr.last_setpoints[M2], 0.40, places=6)

        mgr.set_throttle_mode("medium")
        self.assertEqual(mgr.throttle_limit, 0.70)
        for _ in range(30):
            mgr.update()
        self.assertAlmostEqual(mgr.last_setpoints[M2], 0.70, places=6)

    def test_throttle_ramps_gradually_not_insta(self):
        mgr = self._manager()
        mgr.set_throttle_mode("high")
        mgr.set_motion(MotionState(surge=1.0))
        mgr.update()  # first tick just seeds the timer
        mgr.update()
        self.assertGreater(mgr.last_setpoints[M2], 0.0)
        self.assertLess(mgr.last_setpoints[M2], 1.0)
        for _ in range(20):
            mgr.update()
        self.assertEqual(mgr.last_setpoints[M2], 1.0)

    def test_emergency_stop_zeroes_all_motors(self):
        mgr = self._manager()
        mgr.set_motion(MotionState(surge=1.0, heave=1.0))
        mgr.emergency_stop()
        mgr.update()
        mgr.update()
        self.assertEqual(mgr.last_setpoints, _ZERO)

    def test_emergency_stop_cleared(self):
        mgr = self._manager()
        mgr.emergency_stop()
        mgr.clear_emergency_stop()
        mgr.set_throttle_mode("high")
        mgr.set_motion(MotionState(surge=1.0))
        for _ in range(15):
            mgr.update()
        self.assertEqual(mgr.last_setpoints[M2], 1.0)

    def test_unknown_provider_falls_back_to_simulated(self):
        mgr = ThrusterManager({"provider": "bogus"}, sensors=None)
        self.assertTrue(mgr.initialize())
        self.assertTrue(mgr.health_check())

    def test_gpio_metadata_loaded(self):
        mgr = self._manager()
        self.assertEqual(mgr.thruster_config.gpio[M1], 25)
        self.assertEqual(mgr.thruster_config.gpio[M2], 33)
        self.assertEqual(mgr.thruster_config.directions[M3], 1)


class ThrottleRampTest(unittest.TestCase):
    """Press-and-hold gradual ramp: 0 -> 10% -> 20% -> ... -> mode max.

    The mode is a CEILING, not an instant jump: holding a movement key climbs
    by one THROTTLE_STEP (10%) per control tick and stops exactly at the
    selected mode's maximum. Releasing the key eases back down the same steps.
    """

    _CFG = {
        "provider": "simulated",
        "throttle_mode": "soft",
        "motors": {
            "M1_FRONT_VERTICAL": {"gpio": 25, "direction": 1},
            "M2_MIDDLE_RIGHT_HORIZONTAL": {"gpio": 33, "direction": 1},
            "M3_MIDDLE_LEFT_HORIZONTAL": {"gpio": 32, "direction": 1},
            "M4_BACK_RIGHT_VERTICAL": {"gpio": 27, "direction": 1},
            "M5_BACK_LEFT_VERTICAL": {"gpio": 26, "direction": 1},
        },
    }

    def _mgr(self, mode):
        cfg = dict(self._CFG)
        cfg["throttle_mode"] = mode
        mgr = ThrusterManager(cfg, sensors=None)
        self.assertTrue(mgr.initialize())
        mgr.start()
        mgr.set_throttle_mode(mode)
        return mgr

    def _trajectory(self, mode, ticks):
        """Return (manager, list of M2 setpoints per tick while surge=1 held)."""
        mgr = self._mgr(mode)
        mgr.set_motion(MotionState(surge=1.0))
        mgr.update()  # first tick just seeds the ramp timer
        seq = []
        for _ in range(ticks):
            mgr.update()
            seq.append(mgr.last_setpoints[M2])
        return mgr, seq

    def test_soft_ramps_10pct_and_stops_at_40(self):
        mgr, seq = self._trajectory("soft", 12)
        self.assertEqual(mgr.throttle_limit, 0.40)
        # 0 -> 10 -> 20 -> 30 -> 40, then holds at 40.
        self.assertAlmostEqual(seq[0], 0.10, places=6)
        self.assertAlmostEqual(seq[1], 0.20, places=6)
        self.assertAlmostEqual(seq[2], 0.30, places=6)
        self.assertAlmostEqual(seq[3], 0.40, places=6)
        self.assertAlmostEqual(seq[4], 0.40, places=6)
        self.assertAlmostEqual(seq[11], 0.40, places=6)
        self.assertLessEqual(max(seq), 0.40 + 1e-6)  # never above the ceiling

    def test_medium_ramps_10pct_and_stops_at_70(self):
        mgr, seq = self._trajectory("medium", 12)
        self.assertEqual(mgr.throttle_limit, 0.70)
        steps = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70]
        for i, want in enumerate(steps):
            self.assertAlmostEqual(seq[i], want, places=6)
        self.assertAlmostEqual(seq[11], 0.70, places=6)
        self.assertLessEqual(max(seq), 0.70 + 1e-6)

    def test_hard_ramps_10pct_and_stops_at_100(self):
        mgr, seq = self._trajectory("high", 16)
        self.assertEqual(mgr.throttle_limit, 1.00)
        steps = [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00]
        for i, want in enumerate(steps):
            self.assertAlmostEqual(seq[i], want, places=6)
        self.assertAlmostEqual(seq[14], 1.00, places=6)
        self.assertAlmostEqual(seq[15], 1.00, places=6)
        self.assertLessEqual(max(seq), 1.00 + 1e-6)

    def test_soft_never_exceeds_40_over_many_ticks(self):
        _, seq = self._trajectory("soft", 40)
        self.assertLessEqual(max(seq), 0.40 + 1e-6)
        self.assertEqual(seq[-1], seq[-2])  # held flat at the ceiling

    def test_every_step_is_exactly_10_pct(self):
        for mode in ("soft", "medium", "high"):
            _, seq = self._trajectory(mode, 12)
            deltas = [round(b - a, 6) for a, b in zip(seq, seq[1:])]
            # Steps are 0.10 until the ceiling, then 0.00.
            nontrivial = [d for d in deltas if abs(d) > 1e-6]
            self.assertTrue(all(abs(d - 0.10) < 1e-6 for d in nontrivial), deltas)

    def test_key_release_eases_back_toward_zero(self):
        mgr, _ = self._trajectory("high", 16)
        self.assertAlmostEqual(mgr.last_setpoints[M2], 1.00, places=6)
        mgr.set_motion(MotionState())  # key released -> command returns to zero
        for _ in range(12):
            mgr.update()
        self.assertAlmostEqual(mgr.last_setpoints[M2], 0.00, places=6)
        self.assertTrue(all(abs(v) <= 1e-6 for v in mgr.last_setpoints.values()))

    def test_mode_switch_caps_current_power_gradually(self):
        mgr, _ = self._trajectory("high", 16)
        self.assertAlmostEqual(mgr.last_setpoints[M2], 1.00, places=6)
        mgr.set_throttle_mode("soft")  # HARD -> SOFT while surge still held
        for _ in range(7):  # 1.0 -> 0.9 -> ... -> 0.4
            mgr.update()
        self.assertAlmostEqual(mgr.last_setpoints[M2], 0.40, places=6)
        self.assertLessEqual(max(abs(v) for v in mgr.last_setpoints.values()), 0.40 + 1e-6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
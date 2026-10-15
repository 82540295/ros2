"""Offline unit tests for teleop planning helpers (no ROS, no simulator)."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from waypoint.coords import body_velocity_to_world_ned  # noqa: E402
from waypoint.planner import manual_active, twist_is_release  # noqa: E402


class TestBodyVelocityToWorldNed(unittest.TestCase):
    def test_yaw_zero_forward(self):
        # FLU forward -> world ENU east -> NED (0, 1, 0)
        v = body_velocity_to_world_ned((1.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
        np.testing.assert_allclose(v, (0.0, 1.0, 0.0), atol=1e-12)

    def test_yaw_zero_ascend(self):
        # FLU up -> world ENU up -> NED (0, 0, -1)
        v = body_velocity_to_world_ned((0.0, 0.0, 1.0), (1.0, 0.0, 0.0, 0.0))
        np.testing.assert_allclose(v, (0.0, 0.0, -1.0), atol=1e-12)

    def test_yaw_90deg_forward(self):
        # 90 deg CCW yaw in ENU: FLU forward -> world ENU north -> NED (1, 0, 0)
        q = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
        v = body_velocity_to_world_ned((1.0, 0.0, 0.0), q)
        np.testing.assert_allclose(v, (1.0, 0.0, 0.0), atol=1e-9)


class TestManualMode(unittest.TestCase):
    def test_no_command(self):
        self.assertFalse(manual_active(10.0, None))

    def test_fresh_command(self):
        self.assertTrue(manual_active(10.0, 9.6, timeout_s=1.5))

    def test_expired_command(self):
        self.assertFalse(manual_active(10.0, 8.4, timeout_s=1.5))

    def test_release_marker(self):
        self.assertTrue(twist_is_release(0.0, 0.0, 0.0))
        self.assertTrue(twist_is_release(1e-9, 0.0, -1e-9))
        self.assertFalse(twist_is_release(1.5, 0.0, 0.0))


if __name__ == '__main__':
    unittest.main()

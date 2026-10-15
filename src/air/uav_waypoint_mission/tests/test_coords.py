"""Offline unit tests for the coordinate helpers (no ROS, no simulator)."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from waypoint.coords import (enu_flu_orientation, enu_to_ned, ned_angular_to_flu,  # noqa: E402
                             ned_to_enu, ned_velocity_to_flu)


class TestPointConversion(unittest.TestCase):
    def test_ned_to_enu(self):
        # 3 m north, 4 m east, 2 m down -> ENU (4 east, 3 north, -2 up)
        self.assertEqual(ned_to_enu([3.0, 4.0, 2.0]), (4.0, 3.0, -2.0))

    def test_enu_to_ned(self):
        self.assertEqual(enu_to_ned([4.0, 3.0, -2.0]), (3.0, 4.0, 2.0))

    def test_roundtrip(self):
        p = (1.5, -2.25, 0.75)
        self.assertEqual(enu_to_ned(ned_to_enu(p)), p)


class TestVelocityConversion(unittest.TestCase):
    def test_yaw_zero_forward(self):
        v = ned_velocity_to_flu([1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(v, [1.0, 0.0, 0.0], atol=1e-12)

    def test_yaw_90deg_east_forward(self):
        q = [math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)]
        v = ned_velocity_to_flu([0.0, 1.0, 0.0], q)
        np.testing.assert_allclose(v, [1.0, 0.0, 0.0], atol=1e-9)

    def test_descent_is_negative_up(self):
        v = ned_velocity_to_flu([0.0, 0.0, 1.0], [1.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(v, [0.0, 0.0, -1.0], atol=1e-12)

    def test_angular_sign_flip(self):
        w = ned_angular_to_flu([1.0, -2.0, 3.0])
        np.testing.assert_allclose(w, [1.0, 2.0, -3.0], atol=1e-12)


class TestOrientation(unittest.TestCase):
    def test_identity_quaternion_stays_normalized(self):
        q = enu_flu_orientation([1.0, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(float(np.linalg.norm(q)), 1.0, places=9)


if __name__ == '__main__':
    unittest.main()

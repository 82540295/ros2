"""Offline unit tests for the demo simulation (no ROS, no simulator, no matplotlib)."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from waypoint.demo import run_demo  # noqa: E402
from waypoint.mission import State, load_mission  # noqa: E402


class TestRunDemo(unittest.TestCase):
    def test_repo_mission_completes(self):
        m = load_mission(ROOT / 'config' / 'mission.yaml')
        trail, timeline, ctl = run_demo(m)
        self.assertIs(ctl.state, State.LANDED)
        self.assertLess(timeline[-1][0], 600.0)
        states = [s for _, s in timeline]
        for expected in ('armed', 'taking_off', 'waypoint', 'landing', 'landed'):
            self.assertIn(expected, states)
        self.assertLess(states.index('taking_off'), states.index('waypoint'))
        self.assertLess(states.index('waypoint'), states.index('landing'))

    def test_waypoints_visited_in_order(self):
        m = load_mission(ROOT / 'config' / 'mission.yaml')
        trail, _, _ = run_demo(m)
        idx = 0
        for w in m.waypoints:
            while idx < len(trail) and abs(trail[idx][0] - w.x) + abs(trail[idx][1] - w.y) > 0.6:
                idx += 1
            self.assertLess(idx, len(trail), 'waypoint %s never reached' % w.name)

    def test_lands_near_home(self):
        m = load_mission(ROOT / 'config' / 'mission.yaml')
        trail, _, _ = run_demo(m)
        x, y, z = trail[-1]
        self.assertLess((x * x + y * y) ** 0.5, 0.6)
        self.assertLess(z, 0.3)


if __name__ == '__main__':
    unittest.main()

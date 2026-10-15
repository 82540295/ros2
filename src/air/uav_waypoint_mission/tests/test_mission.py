"""Offline unit tests for the pure mission core (no ROS, no simulator)."""
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from waypoint.mission import (Mission, MissionController, State, Waypoint, load_mission)  # noqa: E402


def mission(n=2, takeoff=3.0, timeout=60.0):
    wps = [Waypoint(name='wp%d' % i, x=5.0 * (i + 1), y=0.0, z=float(takeoff)) for i in range(n)]
    return Mission(name='unit', waypoints=wps, takeoff_altitude_m=takeoff,
                   cruise_speed_mps=2.0, land_speed_mps=0.5, waypoint_timeout_s=timeout)


def drive(ctl, dt=0.1, max_steps=2000):
    """Fly the controller with an ideal drone that reaches each command target."""
    pos = (0.0, 0.0, 0.0)
    trail = [ctl.state]
    for _ in range(max_steps):
        cmd = ctl.tick(pos, (0.0, 0.0, 0.0), dt)
        trail.append(ctl.state)
        if cmd.target is not None:
            pos = cmd.target
        if ctl.state in (State.LANDED, State.FAILED):
            break
    return trail


class TestLoadMission(unittest.TestCase):
    def test_repo_config_loads(self):
        m = load_mission(ROOT / 'config' / 'mission.yaml')
        self.assertEqual(m.name, 'town01_square')
        self.assertEqual(len(m.waypoints), 4)
        self.assertAlmostEqual(m.takeoff_altitude_m, 3.0)
        for w in m.waypoints:
            self.assertGreater(w.z, 0.0)

    def test_empty_waypoints_rejected(self):
        with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
            f.write(textwrap.dedent('''
                mission:
                  name: empty
                  waypoints: []
            '''))
            path = f.name
        with self.assertRaises(ValueError):
            load_mission(path)


class TestStateMachine(unittest.TestCase):
    def test_full_mission_sequence(self):
        ctl = MissionController(mission(n=2))
        self.assertEqual(ctl.state, State.IDLE)
        ctl.arm()
        self.assertEqual(ctl.state, State.ARMED)
        ctl.go()
        self.assertEqual(ctl.state, State.TAKING_OFF)
        trail = drive(ctl)
        self.assertIn(State.WAYPOINT, trail)
        self.assertIn(State.LANDING, trail)
        self.assertEqual(ctl.state, State.LANDED)
        self.assertEqual(ctl.waypoint_index, 2)

    def test_go_requires_armed(self):
        ctl = MissionController(mission(n=1))
        ctl.go()
        self.assertEqual(ctl.state, State.IDLE)

    def test_takeoff_command_targets_home_altitude(self):
        ctl = MissionController(mission(takeoff=3.0))
        ctl.arm()
        ctl.go()
        cmd = ctl.tick((0, 0, 0), (0, 0, 0), 0.1)
        self.assertEqual(cmd.kind, 'takeoff')
        self.assertEqual(cmd.target, (0.0, 0.0, 3.0))
        self.assertAlmostEqual(cmd.speed, 2.0)

    def test_waypoint_timeout_fails(self):
        ctl = MissionController(mission(n=1, timeout=1.0))
        ctl.arm()
        ctl.go()
        pos = (0.0, 0.0, 0.0)
        for _ in range(300):
            cmd = ctl.tick(pos, (0, 0, 0), 0.1)
            if cmd.kind == 'takeoff':
                pos = (0.0, 0.0, 3.0)  # reach takeoff altitude, then refuse to move
            if ctl.state is State.FAILED:
                break
        self.assertEqual(ctl.state, State.FAILED)
        self.assertEqual(ctl.last_command.kind, 'stop')

    def test_abort_lands_vertically(self):
        ctl = MissionController(mission(n=3))
        ctl.arm()
        ctl.go()
        pos = (0.0, 0.0, 3.0)
        ctl.tick(pos, (0, 0, 0), 0.1)  # takeoff altitude reached -> waypoint 0
        self.assertEqual(ctl.state, State.WAYPOINT)
        ctl.abort()
        self.assertEqual(ctl.state, State.LANDING)
        cmd = ctl.tick(pos, (0, 0, 0), 0.1)
        self.assertEqual(cmd.kind, 'land')
        self.assertEqual(cmd.target, (0.0, 0.0, 0.0))
        ctl.tick((0.0, 0.0, 0.0), (0, 0, 0), 0.1)
        self.assertEqual(ctl.state, State.LANDED)


if __name__ == '__main__':
    unittest.main()

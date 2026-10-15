"""Offline unit tests for the AirSim action planner (no ROS, no simulator)."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from waypoint.mission import Command, Mission, MissionController, State, Waypoint  # noqa: E402
from waypoint.planner import mission_finished, plan_airsim_command  # noqa: E402


def mission(n=2, takeoff=3.0):
    wps = [Waypoint(name='wp%d' % i, x=5.0 * (i + 1), y=0.0, z=float(takeoff)) for i in range(n)]
    return Mission(name='unit', waypoints=wps, takeoff_altitude_m=takeoff, land_speed_mps=0.5)


class TestPlan(unittest.TestCase):
    def test_takeoff_maps_to_ned(self):
        ctl = MissionController(mission())
        ctl.arm()
        ctl.go()
        cmd = ctl.tick((0, 0, 0), (0, 0, 0), 0.1)
        action, target, speed = plan_airsim_command(cmd)
        self.assertEqual(action, 'move')
        self.assertEqual(target, (0.0, 0.0, -3.0))  # ENU (0,0,3) -> NED (0,0,-3)
        self.assertAlmostEqual(speed, 2.0)

    def test_waypoint_maps_to_ned(self):
        ctl = MissionController(mission())
        ctl.arm()
        ctl.go()
        pos = (0.0, 0.0, 3.0)
        ctl.tick(pos, (0, 0, 0), 0.1)  # takeoff altitude reached -> waypoint 0
        cmd = ctl.tick(pos, (0, 0, 0), 0.1)
        action, target, _ = plan_airsim_command(cmd)
        self.assertEqual(action, 'move')
        self.assertEqual(target, (0.0, 5.0, -3.0))  # ENU (5,0,3) -> NED (0,5,-3)

    def test_land_targets_ground(self):
        ctl = MissionController(mission(n=1))
        ctl.arm()
        ctl.go()
        ctl.tick((0.0, 0.0, 3.0), (0, 0, 0), 0.1)  # enter waypoint 0
        ctl.tick((5.0, 0.0, 3.0), (0, 0, 0), 0.1)  # waypoint reached -> begin landing
        action, target, speed = plan_airsim_command(ctl.last_command)
        self.assertEqual(action, 'move')
        self.assertEqual(target, (0.0, 5.0, 0.0))
        self.assertAlmostEqual(speed, 0.5)

    def test_hold_and_stop(self):
        self.assertEqual(plan_airsim_command(Command(kind='hold')), ('hold', None, 0.0))
        self.assertEqual(plan_airsim_command(Command(kind='stop')), ('stop', None, 0.0))

    def test_full_flow_actions(self):
        ctl = MissionController(mission(n=2))
        ctl.arm()
        ctl.go()
        pos = (0.0, 0.0, 0.0)
        actions = []
        for _ in range(200):
            cmd = ctl.tick(pos, (0, 0, 0), 0.1)
            actions.append(plan_airsim_command(cmd)[0])
            if cmd.target is not None:
                pos = cmd.target
            if mission_finished(ctl.state):
                break
        self.assertEqual(ctl.state, State.LANDED)
        self.assertTrue(all(a == 'move' for a in actions[:-1]))
        self.assertEqual(actions[-1], 'hold')


if __name__ == '__main__':
    unittest.main()

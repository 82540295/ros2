#!/usr/bin/env python3
"""Single module entry point; execute from a normal Python 3.8+ interpreter.

Stages:
    doctor  check the local Python environment
    test    run the offline unit tests (no ROS / no simulator required)
    build   colcon-build this package inside the ros2 workspace
    demo    (later commit) preview the mission without a simulator
    live    (later commit) run the mission against a live AirSim/CarlaAir
"""
import argparse
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
WS = ROOT.parents[2]  # .../ros2 workspace root


def doctor():
    print('python: %s (%s)' % (sys.version.split()[0], sys.executable))
    ok = sys.version_info >= (3, 8)
    try:
        import yaml  # noqa: F401
        yaml_ok = True
    except ImportError:
        yaml_ok = False
    print('pyyaml: %s' % ('ok' if yaml_ok else 'MISSING (pip install pyyaml)'))
    if ok and yaml_ok:
        print('doctor: OK')
        return 0
    return 1


def test():
    os.chdir(ROOT)
    return subprocess.call([sys.executable, '-m', 'unittest', 'discover', '-s', 'tests', '-v'])


def build():
    return subprocess.call(['colcon', 'build', '--packages-select', 'uav_waypoint_mission'], cwd=str(WS))


def main():
    p = argparse.ArgumentParser(description='uav_waypoint_mission entry point')
    p.add_argument('stage', choices=['doctor', 'test', 'build', 'demo', 'live'])
    a = p.parse_args()
    if a.stage == 'doctor':
        return doctor()
    if a.stage == 'test':
        return test()
    if a.stage == 'build':
        return build()
    print("stage '%s' is not available yet: the simulator-facing ROS2 nodes are added "
          'in a later commit of this module; see README.md for the plan.' % a.stage, file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())

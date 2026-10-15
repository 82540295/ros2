"""Pure mission core: waypoint mission model and a deterministic flight state machine.

This module has no ROS or AirSim dependencies, so it can be unit-tested offline:

    python3 main.py test

Frames: all positions are ENU (x east, y north, z up) in metres, relative to the
drone takeoff point (the simulator spawn location). The ROS2/AirSim adapter added
in a later commit converts ENU commands to AirSim NED and feeds the simulator.
"""
from dataclasses import dataclass
import enum
import math
from pathlib import Path

import yaml


class State(enum.Enum):
    IDLE = 'idle'
    ARMED = 'armed'
    TAKING_OFF = 'taking_off'
    WAYPOINT = 'waypoint'
    LANDING = 'landing'
    LANDED = 'landed'
    FAILED = 'failed'


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    z: float


@dataclass
class Mission:
    name: str
    waypoints: list
    takeoff_altitude_m: float = 3.0
    cruise_speed_mps: float = 2.0
    land_speed_mps: float = 0.5
    waypoint_timeout_s: float = 60.0

    @classmethod
    def from_file(cls, path):
        data = yaml.safe_load(Path(path).read_text())
        m = data['mission']
        wps = [Waypoint(name=str(w['name']), x=float(w['x']), y=float(w['y']), z=float(w['z']))
               for w in m['waypoints']]
        if not wps:
            raise ValueError('mission %r defines no waypoints' % m.get('name'))
        return cls(name=str(m.get('name', 'unnamed')), waypoints=wps,
                   takeoff_altitude_m=float(m.get('takeoff_altitude_m', 3.0)),
                   cruise_speed_mps=float(m.get('cruise_speed_mps', 2.0)),
                   land_speed_mps=float(m.get('land_speed_mps', 0.5)),
                   waypoint_timeout_s=float(m.get('waypoint_timeout_s', 60.0)))


@dataclass
class Command:
    kind: str            # 'hold' | 'takeoff' | 'move' | 'land' | 'stop'
    target: tuple = None  # ENU (x, y, z) metres
    speed: float = 0.0


class MissionController:
    """Deterministic mission state machine (pure Python, offline-testable).

    Drive it with `tick(pos, vel, dt)`; it returns the Command the drone should
    follow. Lifecycle: IDLE -> arm() -> ARMED -> go() -> TAKING_OFF ->
    WAYPOINT (per waypoint) -> LANDING -> LANDED. `abort()` lands from anywhere
    mid-flight. A waypoint that is not reached in time moves the mission to
    FAILED.
    """

    def __init__(self, mission, pos_tol_m=0.5, z_tol_m=0.3):
        self.mission = mission
        self.pos_tol_m = pos_tol_m
        self.z_tol_m = z_tol_m
        self.state = State.IDLE
        self.waypoint_index = 0
        self._in_state_s = 0.0
        self._land_target = None
        self.last_command = Command(kind='stop')

    # -- control API -----------------------------------------------------
    def arm(self):
        if self.state is State.IDLE:
            self._goto(State.ARMED, Command(kind='hold'))
        return self.state

    def go(self):
        if self.state is State.ARMED:
            self._goto(State.TAKING_OFF,
                       Command(kind='takeoff', target=(0.0, 0.0, self.mission.takeoff_altitude_m),
                               speed=self.mission.cruise_speed_mps))
        return self.state

    def abort(self):
        if self.state in (State.TAKING_OFF, State.WAYPOINT):
            x, y, _ = self._last_pos
            self._land_target = (x, y, 0.0)
            self._goto(State.LANDING, Command(kind='land', target=self._land_target,
                                              speed=self.mission.land_speed_mps))
        return self.state

    def reset(self):
        self.state = State.IDLE
        self.waypoint_index = 0
        self._in_state_s = 0.0
        self._land_target = None
        self.last_command = Command(kind='stop')

    # -- driving ---------------------------------------------------------
    def tick(self, pos, vel, dt):
        x, y, z = (float(v) for v in pos)
        self._last_pos = (x, y, z)
        self._in_state_s += float(dt)

        if self.state in (State.IDLE, State.ARMED, State.LANDED):
            self.last_command = Command(kind='hold')
        elif self.state is State.FAILED:
            self.last_command = Command(kind='stop')
        elif self.state is State.TAKING_OFF:
            self.last_command = Command(kind='takeoff',
                                        target=(0.0, 0.0, self.mission.takeoff_altitude_m),
                                        speed=self.mission.cruise_speed_mps)
            if abs(z - self.mission.takeoff_altitude_m) < self.z_tol_m and math.hypot(x, y) < self.pos_tol_m:
                self._enter_waypoint(0)
        elif self.state is State.WAYPOINT:
            wp = self.mission.waypoints[self.waypoint_index]
            self.last_command = Command(kind='move', target=(wp.x, wp.y, wp.z),
                                        speed=self.mission.cruise_speed_mps)
            if self._in_state_s > self.mission.waypoint_timeout_s:
                self._goto(State.FAILED, Command(kind='stop'))
            elif math.hypot(x - wp.x, y - wp.y) < self.pos_tol_m and abs(z - wp.z) < self.z_tol_m:
                self.waypoint_index += 1
                if self.waypoint_index < len(self.mission.waypoints):
                    self._enter_waypoint(self.waypoint_index)
                else:
                    self._begin_landing(x, y)
        elif self.state is State.LANDING:
            tx, ty, tz = self._land_target
            self.last_command = Command(kind='land', target=(tx, ty, tz),
                                        speed=self.mission.land_speed_mps)
            if z <= self.z_tol_m and math.hypot(x - tx, y - ty) < self.pos_tol_m:
                self._goto(State.LANDED, Command(kind='hold'))
        return self.last_command

    # -- helpers ---------------------------------------------------------
    def _goto(self, state, command):
        self.state = state
        self._in_state_s = 0.0
        self.last_command = command

    def _enter_waypoint(self, index):
        wp = self.mission.waypoints[index]
        self.waypoint_index = index
        self._goto(State.WAYPOINT, Command(kind='move', target=(wp.x, wp.y, wp.z),
                                           speed=self.mission.cruise_speed_mps))

    def _begin_landing(self, x, y):
        self._land_target = (x, y, 0.0)
        self._goto(State.LANDING, Command(kind='land', target=self._land_target,
                                          speed=self.mission.land_speed_mps))


def load_mission(path):
    """Load a Mission from a YAML file (see config/mission.yaml for the schema)."""
    return Mission.from_file(path)

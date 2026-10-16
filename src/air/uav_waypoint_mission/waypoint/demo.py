"""Offline mission demo: fly the state machine with a simulated point-mass
drone (no ROS / no simulator required) and visualise the trajectory.

    python3 main.py demo

The drone model is velocity-limited and idealised; the demo exists to show the
mission state machine driving a full takeoff -> waypoints -> landing flight,
not flight dynamics. matplotlib is optional - without it the demo prints the
state timeline only.
"""
from pathlib import Path

from .mission import Command, MissionController, State


class SimulatedDrone:
    """Velocity-limited point-mass drone in ENU coordinates."""

    def __init__(self, pos=(0.0, 0.0, 0.0), max_speed=2.0):
        self.pos = tuple(float(v) for v in pos)
        self.vel = (0.0, 0.0, 0.0)
        self.max_speed = max_speed

    def step(self, cmd, dt):
        if cmd.kind in ('stop', 'hold') or cmd.target is None:
            self.vel = (0.0, 0.0, 0.0)
            return self.pos
        speed = min(max(float(cmd.speed), 0.1), self.max_speed)
        delta = [cmd.target[i] - self.pos[i] for i in range(3)]
        dist = sum(d * d for d in delta) ** 0.5
        if dist < 1e-9:
            self.vel = (0.0, 0.0, 0.0)
            return self.pos
        step_len = min(speed * dt, dist)
        direction = [d / dist for d in delta]
        self.pos = tuple(self.pos[i] + direction[i] * step_len for i in range(3))
        self.vel = tuple(direction[i] * (step_len / dt) for i in range(3))
        return self.pos


def run_demo(mission, dt=0.05, max_time=600.0):
    """Fly the mission with the simulated drone.

    Returns (trail, timeline, controller): trail is the ENU position history,
    timeline lists (t_s, state_name) state transitions.
    """
    ctl = MissionController(mission)
    drone = SimulatedDrone(max_speed=mission.cruise_speed_mps)
    ctl.arm()
    timeline = [(0.0, ctl.state.value)]
    ctl.go()
    timeline.append((0.0, ctl.state.value))
    trail = [drone.pos]
    t = 0.0
    while t < max_time:
        cmd = ctl.tick(drone.pos, drone.vel, dt)
        drone.step(cmd, dt)
        t += dt
        trail.append(drone.pos)
        if ctl.state.value != timeline[-1][1]:
            timeline.append((round(t, 2), ctl.state.value))
        if ctl.state in (State.LANDED, State.FAILED):
            break
    return trail, timeline, ctl


def plot_trajectory(trail, mission, out_path):
    """Render the 3D trajectory with waypoints; requires matplotlib."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot([p[0] for p in trail], [p[1] for p in trail], [p[2] for p in trail],
            lw=1.5, color='tab:blue', label='flight path')
    ax.scatter([0], [0], [0], color='green', s=45, label='home (takeoff)')
    for w in mission.waypoints:
        ax.scatter([w.x], [w.y], [w.z], color='red', s=35)
        ax.text(w.x, w.y, w.z + 0.3, w.name, fontsize=8)
    last = mission.waypoints[-1]
    ax.scatter([last.x], [last.y], [0], color='blue', marker='s', s=50, label='landing')
    ax.set_xlabel('E east [m]')
    ax.set_ylabel('N north [m]')
    ax.set_zlabel('U up [m]')
    ax.set_title('Waypoint mission "%s" - offline demo' % mission.name)
    ax.legend(loc='upper left', fontsize=8)
    fig.tight_layout()
    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    return str(out)

"""Map pure mission commands to concrete AirSim actions (no ROS/AirSim imports).

The ROS2 mission controller node feeds every state-machine Command through
`plan_airsim_command` and executes the returned action; keeping the mapping pure
makes the full flight logic testable offline.
"""
from .coords import enu_to_ned
from .mission import State


def plan_airsim_command(cmd):
    """Return (action, target_ned_or_None, velocity_mps).

    action is 'move' (send moveOnPositionAsync), 'hold' (no command - the
    vehicle keeps its last setpoint) or 'stop' (mission failed; no command).
    """
    if cmd.kind in ('takeoff', 'move', 'land'):
        return 'move', enu_to_ned(cmd.target), float(cmd.speed)
    if cmd.kind == 'hold':
        return 'hold', None, 0.0
    return 'stop', None, 0.0


def mission_finished(state):
    """True once the mission reaches a terminal state."""
    return state in (State.LANDED, State.FAILED)

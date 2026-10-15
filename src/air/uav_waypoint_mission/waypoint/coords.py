"""Pure coordinate conversion helpers (NED <-> ENU / FRD <-> FLU).

No ROS or AirSim imports, so these can be unit-tested offline. AirSim reports
NED/FRD (x north, y east, z down; body axes forward-right-down); ROS2 topics in
this module use the ENU map frame (x east, y north, z up) with FLU body axes
(forward-left-up).
"""
import numpy as np


def ned_to_enu(p):
    """(x_north, y_east, z_down) -> (x_east, y_north, z_up)."""
    return float(p[1]), float(p[0]), float(-p[2])


def enu_to_ned(p):
    """(x_east, y_north, z_up) -> (x_north, y_east, z_down)."""
    return float(p[1]), float(p[0]), float(-p[2])


def quat_product(a, b):
    """Hamilton product of quaternions (w, x, y, z)."""
    w, x, y, z = a
    v, i, j, k = b
    return [w * v - x * i - y * j - z * k,
            w * i + x * v + y * k - z * j,
            w * j - x * k + y * v + z * i,
            w * k + x * j - y * i + z * v]


def enu_flu_orientation(q_ned):
    """NED body quaternion (w,x,y,z) -> ENU-FLU quaternion (w,x,y,z).

    R_ENU_FLU = R_ENU_NED * R_NED_FRD * R_FRD_FLU.
    """
    return quat_product(quat_product([0, 2 ** -0.5, 2 ** -0.5, 0], q_ned), [0, 1, 0, 0])


def dcm_from_quat(q):
    """Body -> world direction-cosine matrix from a quaternion (w, x, y, z)."""
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def ned_velocity_to_flu(v_ned, q_ned):
    """Body velocity in NED-FRD -> body velocity in ENU-FLU (linear)."""
    return (dcm_from_quat(q_ned).T @ np.asarray(v_ned, dtype=float)) * np.array([1, -1, -1])


def body_velocity_to_world_ned(v_flu, q_enu_flu):
    """Body-frame FLU velocity -> world NED velocity (for moveByVelocityAsync)."""
    return enu_to_ned(tuple(dcm_from_quat(q_enu_flu) @ np.asarray(v_flu, dtype=float)))


def ned_angular_to_flu(w_ned):
    """Body angular velocity in NED-FRD -> ENU-FLU (sign flip only)."""
    return np.asarray(w_ned, dtype=float) * np.array([1, -1, -1])

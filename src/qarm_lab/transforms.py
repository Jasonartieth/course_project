"""Small rigid-transform helpers shared by the model / collision tools (URDF conventions)."""

import numpy as np


def rpy_to_R(r, p, y):
    """URDF rpy -> rotation matrix, R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def R_to_rpy(R):
    """Rotation matrix -> URDF rpy (inverse of rpy_to_R; at pitch = +-pi/2 the yaw is set to 0)."""
    pitch = np.arcsin(-np.clip(R[2, 0], -1, 1))
    if abs(np.cos(pitch)) > 1e-9:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll, yaw = np.arctan2(-R[1, 2], R[1, 1]), 0.0
    return np.array([roll, pitch, yaw])


def axis_angle_R(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


def homog(R=np.eye(3), t=(0, 0, 0)):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def origin_T(el):
    """4x4 transform and (xyz, rpy) tuples of a URDF element's <origin> child (identity if absent)."""
    o = el.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    return homog(rpy_to_R(*rpy), xyz), (tuple(xyz), tuple(rpy))

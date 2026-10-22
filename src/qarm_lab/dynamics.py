"""Static (gravity) model of the QArm in DH frames, using the official body parameters from spec.

Gravity torque tau_g(q) = dV/dq with V(q) = g * sum_i m_i * z_i(q), z_i = height of body i's centre of mass.
V is linear in the parameters (m_i * c_i, m_i), so tau_g(q) = Y(q) @ theta with
theta = [m1*c1x, m1*c1y, m1*c1z, m1, ..., m4*c4x, m4*c4y, m4*c4z, m4]  (16 entries, c_i in DH frame i).
Y is used by the gravity identification (only some linear combinations of theta are identifiable).
"""

import numpy as np

from . import kinematics, spec

G = 9.81


def theta_official():
    th = []
    for m, c in zip(spec.DH_BODY_MASS, spec.DH_BODY_COM):
        th += list(m * c) + [m]
    return np.array(th)


def _height_rows(q):
    """Row vectors r_i(q) (length 4) with z_com_i = r_i @ [m c_i, m] / m, i.e. [R_i[2,:], p_i[2]]."""
    return [np.concatenate([T[2, :3], [T[2, 3]]]) for T in kinematics.fk_frames(q)]


def gravity_regressor(q, eps=1e-6):
    """Y(q) (4 x 16): tau_g = Y @ theta, via central differences of the height rows."""
    q = np.asarray(q, float)
    Y = np.zeros((4, 16))
    for j in range(4):
        d = np.zeros(4)
        d[j] = eps
        rp, rm = _height_rows(q + d), _height_rows(q - d)
        for i in range(4):
            Y[j, 4 * i:4 * i + 4] = G * (rp[i] - rm[i]) / (2 * eps)
    return Y


def gravity_torque(q, theta=None):
    """Joint torque needed to hold pose q against gravity [N*m] (sign: torque the motor must apply)."""
    return gravity_regressor(q) @ (theta_official() if theta is None else theta)

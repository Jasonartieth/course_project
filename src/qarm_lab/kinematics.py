"""QArm kinematics in the official standard-DH convention (Spong), matching Quanser's
`hal.products.qarm.QArmUtilities` (same frames, same joint-angle offsets).

phi  : joint angles as read/commanded on the robot ([base, shoulder, elbow, wrist])
theta: DH angles, theta = phi + [0, BETA - pi/2, -BETA, 0]

Frame 0 is the base frame (z up, origin on the base-yaw axis at table level), frame 4 is the TCP
(LAMBDA3 = 0.40 m from the elbow along the forearm; the wrist only rolls about that axis, so it
changes TCP orientation but never its position).
"""

import numpy as np

from . import spec

_DH_A = np.array([0.0, spec.LAMBDA2, 0.0, 0.0])
_DH_ALPHA = np.array([-np.pi / 2, 0.0, -np.pi / 2, 0.0])
_DH_D = np.array([spec.LAMBDA1, 0.0, 0.0, spec.LAMBDA3])
_THETA_OFFSET = np.array([0.0, spec.BETA - np.pi / 2, -spec.BETA, 0.0])


def dh_transform(a, alpha, d, theta):
    """Standard-DH homogeneous transform from frame i to frame i-1."""
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def fk_frames(phi):
    """Return [T01, T02, T03, T04] (each 4x4, frame i in base frame) for joint angles phi."""
    theta = np.asarray(phi, dtype=float) + _THETA_OFFSET
    T = np.eye(4)
    frames = []
    for i in range(4):
        T = T @ dh_transform(_DH_A[i], _DH_ALPHA[i], _DH_D[i], theta[i])
        frames.append(T)
    return frames


def fk(phi):
    """TCP position (3,) and orientation (3x3) in the base frame."""
    T04 = fk_frames(phi)[-1]
    return T04[:3, 3].copy(), T04[:3, :3].copy()


def ik(p, gamma=0.0, phi_prev=None, elbow_up=True):
    """Analytic inverse kinematics.

    p        : desired TCP position (x, y, z) in the base frame.
    gamma    : desired wrist roll angle (passed through as phi[3]).
    phi_prev : if given, the solution (of the up to 4 branches) closest to it is returned.
    elbow_up : branch preference when phi_prev is None (QArm "elbow up" = elbow joint angle >= -BETA side).

    Returns (phi, ok) where ok is False if the point is unreachable or every branch violates the
    joint limits (phi is then the closest-to-limits candidate, or None if unreachable).
    """
    x, y, z = map(float, p)
    candidates = []
    r_xy = np.hypot(x, y)
    for base_flip in (False, True):
        phi1 = np.arctan2(y, x) + (np.pi if base_flip else 0.0)
        phi1 = (phi1 + np.pi) % (2 * np.pi) - np.pi
        r = -r_xy if base_flip else r_xy          # signed horizontal reach in the arm plane
        s = z - spec.LAMBDA1
        # planar 2-link: shoulder->elbow length LAMBDA2, elbow->TCP length LAMBDA3
        c3 = (r * r + s * s - spec.LAMBDA2 ** 2 - spec.LAMBDA3 ** 2) / (2 * spec.LAMBDA2 * spec.LAMBDA3)
        if abs(c3) > 1.0 + 1e-9:
            continue
        c3 = np.clip(c3, -1.0, 1.0)
        for sgn in (1.0, -1.0):
            psi = sgn * np.arccos(c3)             # angle between upper-arm line and forearm line
            # angle of the upper-arm line measured from vertical (z) toward +r
            k1 = spec.LAMBDA2 + spec.LAMBDA3 * np.cos(psi)
            k2 = spec.LAMBDA3 * np.sin(psi)
            ang_tcp = np.arctan2(r, s)            # direction shoulder->TCP from vertical
            ang_upper = ang_tcp - np.arctan2(k2, k1)
            # Map planar angles to QArm joint angles (home: upper-arm line tilted by BETA from vertical,
            # forearm horizontal). Verified against fk() below rather than derived by hand.
            phi2 = ang_upper - spec.BETA
            phi3 = psi - (np.pi / 2 - spec.BETA)
            cand = np.array([phi1, phi2, phi3, gamma])
            if np.linalg.norm(fk(cand)[0] - np.array([x, y, z])) < 1e-6:
                candidates.append(cand)
    if not candidates:
        return None, False
    in_lim = [c for c in candidates if within_limits(c)]
    pool = in_lim if in_lim else candidates
    if phi_prev is not None:
        best = min(pool, key=lambda c: np.linalg.norm(_wrap(c - np.asarray(phi_prev))))
    else:
        # elbow-up: forearm above the upper-arm line -> larger phi3 for the non-flipped base
        pool_nf = [c for c in pool if abs(_wrap(c[0] - np.arctan2(y, x))) < 1e-9] or pool
        best = max(pool_nf, key=lambda c: c[2]) if elbow_up else min(pool_nf, key=lambda c: c[2])
    return best, bool(in_lim)


def jacobian(phi, eps=1e-6):
    """Numerical 3x4 position Jacobian of the TCP (d p / d phi)."""
    phi = np.asarray(phi, dtype=float)
    J = np.zeros((3, 4))
    p0 = fk(phi)[0]
    for i in range(4):
        d = np.zeros(4)
        d[i] = eps
        J[:, i] = (fk(phi + d)[0] - p0) / eps
    return J


def within_limits(phi, margin=0.0):
    phi = np.asarray(phi, dtype=float)
    return bool(np.all(phi >= spec.Q_MIN + margin) and np.all(phi <= spec.Q_MAX - margin))


def _wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi

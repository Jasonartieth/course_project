"""Emulation of the Dynamixel velocity-based trapezoidal profile that QArm position mode applies to every goal.

On the real arm (pal defaults) each joint follows a profile limited to profile_velocity = pi/2 rad/s and
profile_acceleration = pi/3 rad/s^2 toward the latest goal position; the servo's PID then tracks that profile.
The simulator feeds goals through this limiter before its PD controller, and the runner logs the limiter output
as `q_ref` for both backends so tracking errors are measured against what the servo is actually asked to do.
Whether the real servo's profile matches this emulation is one of the things the step tests check.

Guarantee (tests/test_core.py): |dq| <= v_max always; |ddq| <= a_max on every step except the arrival step, where
a residual velocity of at most 2*a_max*dt is zeroed (0.24 deg/s at 500 Hz).
"""

import numpy as np

from . import spec


class ProfileLimiter:
    def __init__(self, v_max=spec.DQ_MAX, a_max=spec.DDQ_MAX):
        self.v_max = np.asarray(v_max, dtype=float)
        self.a_max = np.asarray(a_max, dtype=float)
        self.q = np.zeros_like(self.v_max)
        self.dq = np.zeros_like(self.v_max)

    def reset(self, q, dq=None):
        self.q = np.array(q, dtype=float)
        self.dq = np.zeros_like(self.q) if dq is None else np.array(dq, dtype=float)

    def step(self, goal, dt):
        """Advance by dt toward `goal`; returns (q_ref, dq_ref) copies."""
        e = np.asarray(goal, dtype=float) - self.q
        # fastest velocity from which we can still stop at the goal, capped by v_max. Discrete-time braking law:
        # decelerating by a*dt per step from v covers v^2/(2a) + v*dt/2, so v = -a*dt/2 + sqrt((a*dt/2)^2 + 2a|e|)
        # (the continuous sqrt(2a|e|) brakes too late and ends with a velocity jump at the goal).
        h = 0.5 * self.a_max * dt
        v_des = np.sign(e) * np.minimum(self.v_max, -h + np.sqrt(h * h + 2.0 * self.a_max * np.abs(e)))
        dv = np.clip(v_des - self.dq, -self.a_max * dt, self.a_max * dt)
        self.dq = self.dq + dv
        step = self.dq * dt
        # do not overshoot: snap onto the goal when this step would cross it
        cross = (np.abs(step) >= np.abs(e)) & (np.sign(step) == np.sign(e))
        self.q = np.where(cross, self.q + e, self.q + step)
        self.dq = np.where(cross, 0.0, self.dq)
        return self.q.copy(), self.dq.copy()


def filter_path(q_goal, dt, q0=None):
    """Run a whole goal sequence (N,4) through the limiter; returns the reference path (N,4)."""
    q_goal = np.asarray(q_goal, dtype=float)
    lim = ProfileLimiter()
    lim.reset(q_goal[0] if q0 is None else q0)
    return np.array([lim.step(g, dt)[0] for g in q_goal])

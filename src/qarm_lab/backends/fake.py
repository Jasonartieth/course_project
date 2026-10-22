"""Dependency-free stand-in for the arm (tests and Mac-side dry runs of the runner/safety logic).

Joint motion = Dynamixel profile emulation + first-order lag; current = official gravity torque / KT_NOMINAL
plus noise. It is NOT a physics simulation; use the Genesis backend for that.
"""

import numpy as np

from .. import dynamics, spec
from ..profile import ProfileLimiter
from .base import Backend, State


class FakeBackend(Backend):
    name = "fake"

    def __init__(self, rate_hz=500.0, q0=spec.SLEEP_POSE, lag_s=0.03, noise_a=0.02, seed=0):
        self.rate_hz = float(rate_hz)
        self.q = np.array(q0, dtype=float)
        self.dq = np.zeros(4)
        self.lag_s = lag_s
        self.noise_a = noise_a
        self.rng = np.random.default_rng(seed)
        self.lim = ProfileLimiter()
        self.t = 0.0
        self.grip = 0.1

    def open(self):
        self.lim.reset(self.q)
        return self

    def _state(self):
        cur = dynamics.gravity_torque(self.q) / spec.KT_NOMINAL + self.rng.normal(0, self.noise_a, 4)
        return State(self.t, np.r_[self.q, self.grip], np.r_[self.dq, 0.0], np.r_[cur, 0.05],
                     np.zeros(5), np.full(5, 30.0))

    def read(self):
        return self._state()

    def step(self, q_goal, grip):
        dt = 1.0 / self.rate_hz
        q_ref, _ = self.lim.step(q_goal, dt)
        a = dt / (self.lag_s + dt)
        q_new = self.q + a * (q_ref - self.q)
        self.dq = (q_new - self.q) / dt
        self.q = q_new
        self.grip += (float(np.clip(grip, spec.GRIP_MIN, spec.GRIP_MAX)) - self.grip) * a
        self.t += dt
        return self._state()

    def close(self):
        pass

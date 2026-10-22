"""Dynamixel position-mode servo model used by the simulator (and fitted by real2sim).

Per joint, at the control rate:
    q_ref, dq_ref = profile(goal)                                  (velocity-based trapezoid, see profile.py)
    pwm   = clip(kp * e + ki * integral(e) + kd * (dq_ref - dq), -1, 1),   e = q_ref - q
    tau   = tau_stall * pwm - b * dq - friction,   b = tau_stall / omega_no_load   (back-EMF of a PWM drive)
    current = (tau_stall * pwm - b * dq) / Kt
The simulator applies tau_stall * pwm as an actuator force and puts b (damping), armature and frictionloss into
the physics engine's joint model so they are integrated implicitly.

Parameter status: tau_stall, omega_no_load from the ROBOTIS datasheets; kp, ki from pal's PID write (units
assumed to be PWM fraction per rad, unverified); kd, armature, frictionloss are estimates. All of them are
written to/loaded from JSON so identified values (real2sim) replace the defaults without code changes.
"""

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from . import spec


def _arr(v):
    return field(default_factory=lambda: np.array(v, dtype=float))


@dataclass
class ServoParams:
    kp: np.ndarray = _arr([spec.PAL_PID["P"]] * 4)          # PWM fraction / rad
    ki: np.ndarray = _arr([spec.PAL_PID["I"]] * 4)          # PWM fraction / (rad*s)
    kd: np.ndarray = _arr([0.0] * 4)                         # PWM fraction / (rad/s)
    tau_stall: np.ndarray = _arr(spec.TAU_INST)              # N*m at the joint (shoulder = 2 motors)
    omega_nl: np.ndarray = _arr(spec.NO_LOAD_SPEED)          # rad/s
    armature: np.ndarray = _arr([0.02, 0.04, 0.02, 0.01])    # kg*m^2 reflected rotor inertia (ESTIMATE: J_rotor*N^2)
    frictionloss: np.ndarray = _arr([0.10, 0.20, 0.10, 0.05])  # N*m Coulomb (ESTIMATE)
    v_max: np.ndarray = _arr(spec.DQ_MAX)                    # profile velocity
    a_max: np.ndarray = _arr(spec.DDQ_MAX)                   # profile acceleration
    note: str = "defaults 2026-10-22 (datasheet + estimates, not identified)"

    @property
    def damping(self):
        return self.tau_stall / self.omega_nl

    def save(self, path):
        d = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in asdict(self).items()}
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(d, indent=2))

    @staticmethod
    def load(path):
        d = json.loads(Path(path).read_text())
        return ServoParams(**{k: (np.array(v, float) if isinstance(v, list) else v) for k, v in d.items()})


class ServoPID:
    """Discrete PID producing the PWM command (integral clamped to avoid wind-up)."""

    def __init__(self, p: ServoParams):
        self.p = p
        self.integ = np.zeros(4)

    def reset(self):
        self.integ[:] = 0.0

    def pwm(self, q_ref, dq_ref, q, dq, dt):
        e = q_ref - q
        self.integ = np.clip(self.integ + e * dt, -1.0 / np.maximum(self.p.ki, 1e-9), 1.0 / np.maximum(self.p.ki, 1e-9))
        return np.clip(self.p.kp * e + self.p.ki * self.integ + self.p.kd * (dq_ref - dq), -1.0, 1.0)

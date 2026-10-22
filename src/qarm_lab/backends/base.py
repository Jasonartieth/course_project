"""Common backend interface.

Every backend runs at a fixed control rate. `step(q_goal, grip)` sends one position-mode command and returns
the state measured in that control period; on the real arm it blocks until the next hardware-clocked sample,
in simulation it advances physics by 1/rate_hz.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class State:
    t: float                 # backend time [s] (wall clock on hardware, sim time in simulation)
    q: np.ndarray            # (5,) joint positions [rad] + gripper position (0 open .. 1 closed)
    dq: np.ndarray           # (5,) joint speeds [rad/s] + gripper speed
    current: np.ndarray      # (5,) motor currents [A] (simulation: torque / KT_NOMINAL)
    pwm: np.ndarray          # (5,) PWM duty -1..1 (simulation: 0)
    temp: np.ndarray         # (5,) motor temperatures [degC] (simulation: constant)


class Backend:
    name = "base"
    rate_hz = 500.0

    def open(self):
        raise NotImplementedError

    def read(self) -> State:
        raise NotImplementedError

    def step(self, q_goal, grip) -> State:
        raise NotImplementedError

    def set_led(self, rgb):
        """Base LED colour (real arm only): (0,0,1) idle, (0,1,0) running, (1,0,0) aborted."""

    def close(self):
        raise NotImplementedError

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()
        return False

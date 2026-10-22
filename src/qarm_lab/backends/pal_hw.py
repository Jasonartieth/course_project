"""Real QArm through Quanser's `pal.products.qarm.QArm` (position mode, hardware-clocked reads).

Requirements on the machine wired to the arm: Quanser SDK (`quanser.hardware`, from apt
python3-quanser-apis on Ubuntu) and Quanser_Academic_Resources/0_libraries/python on PYTHONPATH (`pal`).

Notes on pal behaviour this wrapper compensates for:
  * pal prints HILError messages instead of raising -> we check `status` and the sanity of every reading.
  * pal's measJoint* attributes alias internal buffers -> every State holds copies.
  * readMode=1 creates a hardware-clocked reader task at `rate_hz`; read_std()/read_write_std() block until
    the next sample, which paces our control loop.
  * opening the card writes no position goal; pal sets the position PID gains (P 8.89, I 0.012, D 10.23).
"""

import time

import numpy as np

from .base import Backend, State


class PalQArmBackend(Backend):
    name = "pal"

    def __init__(self, rate_hz=500.0, device_id=0):
        self.rate_hz = float(rate_hz)
        self.device_id = device_id
        self.arm = None
        self.led = np.array([0.0, 0.0, 1.0])
        self._t0 = None

    def open(self):
        from pal.products.qarm import QArm   # imported here so other machines don't need the SDK
        self.arm = QArm(hardware=1, readMode=1, frequency=int(self.rate_hz), deviceId=self.device_id)
        if not self.arm.status:
            raise RuntimeError("QArm card did not open (see the HIL error printed above): USB cable, power, "
                               "another program holding the arm, or the Quanser SDK not installed?")
        self._t0 = time.perf_counter()
        s = self.read()
        if not np.all(np.isfinite(s.q)) or np.all(s.temp == 0):
            raise RuntimeError(f"implausible first reading from the arm: {s}")
        return self

    def _state(self):
        a = self.arm
        return State(t=time.perf_counter() - self._t0,
                     q=np.array(a.measJointPosition, dtype=float, copy=True),
                     dq=np.array(a.measJointSpeed, dtype=float, copy=True),
                     current=np.array(a.measJointCurrent, dtype=float, copy=True),
                     pwm=np.array(a.measJointPWM, dtype=float, copy=True),
                     temp=np.array(a.measJointTemperature, dtype=float, copy=True))

    def read(self):
        self.arm.read_std()
        return self._state()

    def step(self, q_goal, grip):
        # pal clips the gripper to [0.1, 0.9]; pass a float (assigning a 1-element array into its buffer
        # is deprecated in NumPy 2)
        self.arm.read_write_std(phiCMD=np.asarray(q_goal, dtype=np.float64), gprCMD=float(grip),
                                baseLED=self.led)
        return self._state()

    def set_led(self, rgb):
        self.led = np.asarray(rgb, dtype=np.float64)

    def close(self):
        if self.arm is not None:
            self.arm.terminate()
            self.arm = None

"""Genesis simulation of the QArm with the Dynamixel servo model (servo.py) on the 4 arm joints.

Default model: models/qarm_v2/qarm_with_gripper.urdf (aligned gripper, box/cylinder/sphere collisions that
enclose the visual meshes, finger limits at first contact). The v1 model (models/qarm/urdf/qarm_gripper.urdf,
floating finger parts) can still be passed explicitly with urdf=...
Arm joints: torque control (tau_stall * pwm from our PID); back-EMF damping, armature and Coulomb friction are
set as Genesis joint properties. Fingers: Genesis position control to grip * spec.GRIPPER_FINGER_CLOSE.
Physics runs at `physics_hz` (default 1 kHz); the servo/PID updates at `rate_hz` with zero-order hold.
Runs as fast as possible (no real-time pacing); State.t is simulation time.
"""

from pathlib import Path

import numpy as np

from .. import spec
from ..profile import ProfileLimiter
from ..servo import ServoParams, ServoPID
from .base import Backend, State

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_URDF = ROOT / "models" / "qarm_v2" / "qarm_with_gripper.urdf"
ARM_JOINTS = ("YAW", "SHOULDER", "ELBOW", "WRIST")

_GS_INITIALISED = False


class GenesisBackend(Backend):
    name = "genesis"

    def __init__(self, rate_hz=500.0, physics_hz=1000.0, urdf=DEFAULT_URDF, params=None, q0=spec.SLEEP_POSE,
                 device="cpu", temp_c=30.0, show_viewer=False):
        self.rate_hz = float(rate_hz)
        self.n_sub = int(round(physics_hz / rate_hz))
        assert self.n_sub >= 1 and abs(self.n_sub * rate_hz - physics_hz) < 1e-6, "physics_hz must be k * rate_hz"
        self.urdf = str(urdf)
        self.params = params if isinstance(params, ServoParams) else (ServoParams.load(params) if params else ServoParams())
        self.q0 = np.array(q0, dtype=float)
        self.device = device
        self.temp_c = temp_c
        self.show_viewer = show_viewer
        self.t = 0.0
        self.grip = spec.GRIP_MIN
        self.last_pwm = np.zeros(4)

    # ------------------------------------------------------------------ setup
    def open(self):
        global _GS_INITIALISED
        import genesis as gs
        if not _GS_INITIALISED:
            gs.init(backend=gs.gpu if self.device == "gpu" else gs.cpu, logging_level="warning")
            _GS_INITIALISED = True
        self.gs = gs
        self.scene = gs.Scene(sim_options=gs.options.SimOptions(dt=1.0 / (self.rate_hz * self.n_sub)),
                              show_viewer=self.show_viewer)
        self.scene.add_entity(gs.morphs.Plane())
        self.arm = self.scene.add_entity(gs.morphs.URDF(file=self.urdf, fixed=True))
        self.scene.build()
        jmap = {j.name: j for j in self.arm.joints}
        self.arm_dofs = [jmap[n].dofs_idx_local[0] for n in ARM_JOINTS]
        self.finger_names = [n for n in spec.GRIPPER_FINGER_CLOSE if n in jmap]
        self.finger_dofs = [jmap[n].dofs_idx_local[0] for n in self.finger_names]
        self.finger_close = np.array([spec.GRIPPER_FINGER_CLOSE[n] for n in self.finger_names])
        p = self.params
        self.arm.set_dofs_kp(np.zeros(4), self.arm_dofs)
        self.arm.set_dofs_kv(np.zeros(4), self.arm_dofs)
        self.arm.set_dofs_damping(p.damping, self.arm_dofs)
        self.arm.set_dofs_armature(p.armature, self.arm_dofs)
        self.arm.set_dofs_frictionloss(p.frictionloss, self.arm_dofs)
        self.arm.set_dofs_force_range(-p.tau_stall, p.tau_stall, self.arm_dofs)
        if self.finger_dofs:
            self.arm.set_dofs_kp(np.full(len(self.finger_dofs), 2.0), self.finger_dofs)
            self.arm.set_dofs_kv(np.full(len(self.finger_dofs), 0.05), self.finger_dofs)
        self.profile = ProfileLimiter(p.v_max, p.a_max)
        self.pid = ServoPID(p)
        self.reset(self.q0)
        return self

    def reset(self, q0):
        q0 = np.asarray(q0, float)
        self.arm.set_dofs_position(q0, self.arm_dofs)
        self.arm.set_dofs_velocity(np.zeros(4), self.arm_dofs)
        if self.finger_dofs:
            self.arm.set_dofs_position(self.grip * self.finger_close, self.finger_dofs)
        self.profile.reset(q0)
        self.pid.reset()
        self.t = 0.0
        # servo holds the reset pose until the first command arrives
        self._goal = q0.copy()

    # ------------------------------------------------------------------ I/O
    def _qdq(self):
        q = np.asarray(self.arm.get_dofs_position(self.arm_dofs).cpu(), dtype=float).reshape(-1)
        dq = np.asarray(self.arm.get_dofs_velocity(self.arm_dofs).cpu(), dtype=float).reshape(-1)
        return q, dq

    def _state(self, q, dq):
        p = self.params
        cur = (p.tau_stall * self.last_pwm - p.damping * dq) / spec.KT_NOMINAL
        return State(self.t, np.r_[q, self.grip], np.r_[dq, 0.0], np.r_[cur, 0.0],
                     np.r_[self.last_pwm, 0.0], np.full(5, self.temp_c))

    def read(self):
        return self._state(*self._qdq())

    def step(self, q_goal, grip):
        dt = 1.0 / self.rate_hz
        q_ref, dq_ref = self.profile.step(np.asarray(q_goal, float), dt)
        q, dq = self._qdq()
        self.last_pwm = self.pid.pwm(q_ref, dq_ref, q, dq, dt)
        self.arm.control_dofs_force(self.params.tau_stall * self.last_pwm, self.arm_dofs)
        g = float(np.clip(grip, spec.GRIP_MIN, spec.GRIP_MAX))
        self.grip += (g - self.grip) * min(1.0, dt / 0.2)
        if self.finger_dofs:
            self.arm.control_dofs_position(self.grip * self.finger_close, self.finger_dofs)
        for _ in range(self.n_sub):
            self.scene.step()
        self.t += dt
        return self._state(*self._qdq())

    def tcp_position(self):
        """TCP in the base frame from the simulated END-EFFECTOR pose (for checks/plots)."""
        link = self.arm.get_link("END-EFFECTOR")
        pos = np.asarray(link.get_pos().cpu(), float).reshape(-1)
        w, x, y, z = np.asarray(link.get_quat().cpu(), float).reshape(-1)
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return pos + R @ spec.URDF_EE_TO_TCP

    def close(self):
        pass

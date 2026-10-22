"""Fast unit tests for the dependency-free core (run anywhere: `pytest tests`)."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from qarm_lab import dynamics, kinematics, profile, spec  # noqa: E402
from qarm_lab import trajectory as T  # noqa: E402
from qarm_lab.backends import make_backend  # noqa: E402
from qarm_lab.runner import run_trajectory  # noqa: E402


def test_fk_home_matches_manual():
    p, _ = kinematics.fk(spec.HOME_POSE)
    assert np.allclose(p, [0.45, 0.0, 0.49], atol=1e-9)


def test_ik_roundtrip():
    rng = np.random.default_rng(1)
    for _ in range(300):
        q = rng.uniform(spec.Q_MIN + 0.05, spec.Q_MAX - 0.05)
        q2, ok = kinematics.ik(kinematics.fk(q)[0], q[3], q)
        assert q2 is not None and ok
        assert np.allclose(kinematics.fk(q2)[0], kinematics.fk(q)[0], atol=1e-9)


def test_gravity_regressor_matches_numeric_potential():
    q = np.array([0.3, 0.4, -0.5, 0.2])
    eps = 1e-6

    def V(x):
        return sum(m * dynamics.G * (T_[:3, :3] @ c + T_[:3, 3])[2]
                   for m, c, T_ in zip(spec.DH_BODY_MASS, spec.DH_BODY_COM, kinematics.fk_frames(x)))
    num = np.array([(V(q + eps * e) - V(q - eps * e)) / (2 * eps) for e in np.eye(4)])
    assert np.allclose(dynamics.gravity_torque(q), num, atol=1e-6)


@pytest.mark.parametrize("dt", [0.002, 0.01])
def test_profile_respects_limits_and_reaches_goal(dt):
    goal = np.array([1.0, -0.5, 0.2, 2.0])
    lim = profile.ProfileLimiter()
    lim.reset(np.zeros(4))
    prev_v = np.zeros(4)
    for _ in range(int(6 / dt)):
        q, v = lim.step(goal, dt)
        assert np.all(np.abs(v) <= spec.DQ_MAX + 1e-9)
        over = np.abs(v - prev_v) > spec.DDQ_MAX * dt + 1e-9
        # only allowed on the arrival step, and only to zero a residual velocity <= 2 * a_max * dt
        assert np.all(q[over] == goal[over])
        assert np.all(np.abs(prev_v[over]) <= 2 * spec.DDQ_MAX[over] * dt + 1e-9)
        prev_v = v
    assert np.allclose(q, goal)


@pytest.mark.parametrize("tr", [T.joint_steps(1), T.sine_sweep(2), T.fourier_excitation(3), T.hold(spec.HOME_POSE, 2)])
def test_generated_trajectories_pass_checks(tr):
    assert T.check_trajectory(tr).ok


def test_check_rejects_table_collision():
    q_low = np.array([0.0, np.deg2rad(60), np.deg2rad(60), 0.0])     # arm pitched down into the table
    assert not T.check_trajectory(T.min_jerk(spec.HOME_POSE, q_low)).ok


def test_runner_with_fake_backend_and_safety_trip(tmp_path):
    be = make_backend("fake")
    be.open()
    _, abort = run_trajectory(be, T.joint_steps(0, hold_s=0.5), out_dir=tmp_path / "ok", verbose=False)
    assert abort is None and (tmp_path / "ok" / "log.npz").exists()

    class Hot(type(be)):
        def _state(self):
            s = super()._state()
            s.temp[2] = 60.0
            return s
    hot = Hot()
    hot.open()
    _, abort = run_trajectory(hot, T.joint_steps(0, hold_s=0.5), verbose=False)
    assert abort and "temperature" in abort

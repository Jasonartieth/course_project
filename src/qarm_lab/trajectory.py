"""Joint-space trajectories: container, generators and offline safety checks.

A Trajectory is a time-stamped sequence of joint *goal* positions (what we send in position mode) plus a
gripper command. It is stored at its own sample rate; the runner interpolates it to the control rate, so the
same file runs at 500 Hz on the real arm and at any rate in simulation.

Trajectories flagged meta["goal_steps"] = True contain deliberate goal jumps (step-response tests); for those
the velocity/acceleration limits are checked on the profile-filtered path (what the servo will execute)
instead of on the raw goals.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import kinematics, profile, spec

DEFAULT_DT = 0.01        # storage resolution (100 Hz); the runner interpolates


@dataclass
class Trajectory:
    name: str
    t: np.ndarray                 # (N,)  seconds, starts at 0
    q: np.ndarray                 # (N,4) joint goals [rad]
    grip: np.ndarray              # (N,)  gripper command, 0 = open .. 1 = closed (pal clips to [0.1, 0.9])
    meta: dict = field(default_factory=dict)

    @property
    def duration(self):
        return float(self.t[-1])

    def sample(self, t):
        """Linear interpolation of (q, grip) at time t (clamped to the ends)."""
        q = np.array([np.interp(t, self.t, self.q[:, j]) for j in range(4)])
        return q, float(np.interp(t, self.t, self.grip))

    def save(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, t=self.t, q=self.q, grip=self.grip,
                            meta=np.array(json.dumps({"name": self.name, **self.meta})))

    @staticmethod
    def load(path):
        z = np.load(path, allow_pickle=False)
        meta = json.loads(str(z["meta"]))
        return Trajectory(meta.pop("name"), z["t"], z["q"], z["grip"], meta)


# ------------------------------------------------------------------------------------------- generators
def _traj(name, q, grip, dt, **meta):
    q = np.atleast_2d(np.asarray(q, dtype=float))
    grip = np.broadcast_to(np.asarray(grip, dtype=float), (len(q),)).copy()
    return Trajectory(name, np.arange(len(q)) * dt, q, grip, meta)


def hold(q, duration, grip=0.1, dt=DEFAULT_DT, name="hold"):
    n = int(round(duration / dt)) + 1
    return _traj(name, np.tile(np.asarray(q, float), (n, 1)), grip, dt)


def min_jerk_duration(q0, q1, speed_frac=0.5):
    """Shortest quintic (min-jerk) duration keeping peak |dq| <= frac*DQ_MAX and |ddq| <= frac*DDQ_MAX."""
    d = np.abs(np.asarray(q1, float) - np.asarray(q0, float))
    t_v = 1.875 * d / (speed_frac * spec.DQ_MAX)
    t_a = np.sqrt(5.7735 * d / (speed_frac * spec.DDQ_MAX))
    return float(max(np.max(t_v), np.max(t_a), 0.5))


def min_jerk(q0, q1, duration=None, speed_frac=0.5, grip=0.1, dt=DEFAULT_DT, name="move"):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    T = duration or min_jerk_duration(q0, q1, speed_frac)
    s = np.linspace(0.0, 1.0, int(round(T / dt)) + 1)
    blend = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    return _traj(name, q0 + np.outer(blend, q1 - q0), grip, dt)


def concat(name, parts, **meta):
    """Join trajectories end to start (each part's first sample is dropped after the first part)."""
    dt = parts[0].t[1] - parts[0].t[0]
    q = np.vstack([parts[0].q] + [p.q[1:] for p in parts[1:]])
    grip = np.concatenate([parts[0].grip] + [p.grip[1:] for p in parts[1:]])
    merged = {}
    for p in parts:
        merged.update(p.meta)
    merged.update(meta)
    return _traj(name, q, grip, dt, **merged)


def joint_steps(joint, center=spec.HOME_POSE, steps=(0.1, 0.0, -0.1, 0.0, 0.2, 0.0, -0.2, 0.0),
                hold_s=2.5, grip=0.1, dt=DEFAULT_DT):
    """Goal steps on one joint around `center` (step-response / profile identification)."""
    center = np.asarray(center, float)
    segs = []
    for s in (0.0,) + tuple(steps):
        q = center.copy()
        q[joint] += s
        segs.append(np.tile(q, (int(round(hold_s / dt)), 1)))
    q = np.vstack(segs + [segs[-1][-1:]])
    return _traj(f"steps_{spec.JOINT_NAMES[joint]}", q, grip, dt, goal_steps=True, joint=joint,
                 steps=list(steps), hold_s=hold_s)


def sine_sweep(joint, center=spec.HOME_POSE, freqs=(0.1, 0.2, 0.3, 0.45), cycles=3, amp_max=0.25,
               frac=0.8, grip=0.1, dt=DEFAULT_DT):
    """Sinusoids of increasing frequency on one joint, each with a raised-cosine fade in/out of one cycle so the
    goal starts and ends at rest. Amplitude per frequency respects frac * (DQ_MAX, DDQ_MAX)."""
    center = np.asarray(center, float)
    out, amps = [center[None, :]], []
    for f in freqs:
        w = 2 * np.pi * f
        A = min(amp_max, frac * min(spec.DQ_MAX[joint] / w, spec.DDQ_MAX[joint] / w ** 2))
        # the fade envelope adds acceleration ~ A*w*w_env; keep margin by shrinking A a little more
        A *= 0.85
        amps.append(A)
        T = cycles / f
        t = np.arange(0.0, T, dt)
        env = np.ones_like(t)
        ramp = 1.0 / f
        env[t < ramp] = 0.5 - 0.5 * np.cos(np.pi * t[t < ramp] / ramp)
        env[t > T - ramp] = 0.5 - 0.5 * np.cos(np.pi * (T - t[t > T - ramp]) / ramp)
        q = np.tile(center, (len(t), 1))
        q[:, joint] += A * env * np.sin(w * t)
        out += [q, np.tile(center, (int(1.0 / dt), 1))]          # 1 s rest between frequencies
    return _traj(f"sine_{spec.JOINT_NAMES[joint]}", np.vstack(out), grip, dt, joint=joint,
                 freqs=list(freqs), amps=[float(a) for a in amps], cycles=cycles)


def static_poses(poses, hold_s=4.0, speed_frac=0.4, grip=0.1, dt=DEFAULT_DT, name="gravity_poses"):
    """Visit each pose with a min-jerk move and hold it (gravity / static torque identification).
    meta["hold_windows"] lists (t_start, t_end, pose_index) of every hold for the analysis scripts."""
    parts, windows, t = [], [], 0.0
    prev = np.asarray(poses[0], float)
    parts.append(hold(prev, hold_s, grip, dt))
    windows.append((0.0, hold_s, 0))
    t = hold_s
    for i, p in enumerate(poses[1:], start=1):
        mv = min_jerk(prev, p, speed_frac=speed_frac, grip=grip, dt=dt)
        h = hold(p, hold_s, grip, dt)
        parts += [mv, h]
        t += mv.duration
        windows.append((t, t + hold_s, i))
        t += hold_s
        prev = np.asarray(p, float)
    return concat(name, parts, hold_windows=windows, poses=np.asarray(poses).tolist())


def fourier_excitation(seed, center=spec.HOME_POSE, f0=0.05, n_harm=5, periods=2, frac=0.7, span=None,
                       fade_s=2.5, grip=0.1, dt=DEFAULT_DT, name=None):
    """Band-limited periodic excitation (finite Fourier series, Swevers et al.) for dynamic identification.
    Random coefficients (seeded) are scaled so the path stays within `span` (default: half of each joint's
    usable range around `center`) and below frac * (DQ_MAX, DDQ_MAX); a raised-cosine fade makes the
    trajectory start and end at rest at `center`."""
    rng = np.random.default_rng(seed)
    center = np.asarray(center, float)
    w0 = 2 * np.pi * f0
    T = periods / f0
    t = np.arange(0.0, T, dt)
    if span is None:
        span = 0.5 * np.minimum(center - (spec.Q_MIN + spec.LIMIT_MARGIN), (spec.Q_MAX - spec.LIMIT_MARGIN) - center)
    q = np.zeros((len(t), 4))
    for j in range(4):
        a, b = rng.normal(size=n_harm), rng.normal(size=n_harm)
        l = np.arange(1, n_harm + 1)
        q[:, j] = (a / (w0 * l)) @ np.sin(np.outer(l, w0 * t)) - (b / (w0 * l)) @ np.cos(np.outer(l, w0 * t))
        q[:, j] -= q[:, j].mean()
    env = np.ones_like(t)
    env[t < fade_s] = 0.5 - 0.5 * np.cos(np.pi * t[t < fade_s] / fade_s)
    env[t > T - fade_s] = 0.5 - 0.5 * np.cos(np.pi * (T - t[t > T - fade_s]) / fade_s)
    dev = q * env[:, None]
    dev -= dev[0]                                            # start exactly at center
    # scale each joint into position span, then all joints together into velocity/acceleration bounds
    dev *= np.minimum(1.0, span / np.maximum(np.abs(dev).max(0), 1e-9))
    for _ in range(20):
        dq = np.gradient(dev, dt, axis=0)
        ddq = np.gradient(dq, dt, axis=0)
        s = min(1.0, np.min(frac * spec.DQ_MAX / np.maximum(np.abs(dq).max(0), 1e-9)),
                np.min(frac * spec.DDQ_MAX / np.maximum(np.abs(ddq).max(0), 1e-9)))
        if s >= 0.999:
            break
        dev *= s
    return _traj(name or f"fourier_seed{seed}", center + dev, grip, dt, seed=seed, f0=f0, n_harm=n_harm,
                 periods=periods, frac=frac)


def cartesian_waypoints(points, gamma=0.0, seg_s=3.0, pause_s=1.0, grip=0.1, dt=DEFAULT_DT, name="cartesian"):
    """Straight-line TCP segments between `points` (min-jerk timing along each segment), joint goals via IK."""
    qs, phi_prev = [], None
    pts = [np.asarray(p, float) for p in points]
    for k in range(len(pts)):
        p0, p1 = pts[k], pts[(k + 1) % len(pts)] if k + 1 < len(pts) else pts[k]
        n = int(round(seg_s / dt)) + 1
        s = np.linspace(0, 1, n)
        blend = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
        for b in blend:
            phi, ok = kinematics.ik(p0 + b * (p1 - p0), gamma, phi_prev)
            if phi is None or not ok:
                raise ValueError(f"IK failed or out of limits near {p0 + b * (p1 - p0)}")
            qs.append(phi)
            phi_prev = phi
        qs += [qs[-1]] * int(round(pause_s / dt))
        if k + 1 == len(pts):
            break
    return _traj(name, np.array(qs), grip, dt, waypoints=[p.tolist() for p in pts])


# ------------------------------------------------------------------------------------------- checks
@dataclass
class CheckResult:
    ok: bool
    problems: list
    stats: dict


def workspace_points(phi):
    """Points that must stay clear of the table/base: elbow, wrist (DH), TCP."""
    T = kinematics.fk_frames(phi)
    elbow = T[1][:3, 3]
    tcp = T[3][:3, 3]
    wrist = elbow + (tcp - elbow) * (spec.L4 / spec.LAMBDA3)
    return {"elbow": elbow, "wrist": wrist, "tcp": tcp}


def check_trajectory(traj, margin=spec.LIMIT_MARGIN, vel_frac=1.0, acc_frac=1.0, min_z=spec.TCP_MIN_Z,
                     base_keepout=(0.22, 0.20), tol=1.02, limit_tol=1e-9):
    """Offline checks before anything is sent to the arm. Returns CheckResult(ok, problems, stats).

    - joint goals inside [Q_MIN + margin - limit_tol, Q_MAX - margin + limit_tol]
      (limit_tol > 0 only for approach moves, which start wherever the arm rests, possibly on a limit)
    - velocity/acceleration <= frac * limits on the raw goals; for goal-step trajectories only the velocity of
      the profile-filtered path is checked (the profile itself bounds the acceleration)
    - elbow / wrist / TCP above min_z, and TCP outside the cylinder (r < base_keepout[0], z < base_keepout[1])
      around the base, evaluated on the profile-filtered path
    - gripper command in [0, 1]
    """
    problems = []
    q, dt = traj.q, float(traj.t[1] - traj.t[0])
    lo, hi = spec.Q_MIN + margin, spec.Q_MAX - margin
    for j in range(4):
        if q[:, j].min() < lo[j] - limit_tol or q[:, j].max() > hi[j] + limit_tol:
            problems.append(f"joint {spec.JOINT_NAMES[j]} range [{q[:, j].min():.5f}, {q[:, j].max():.5f}] "
                            f"outside [{lo[j]:.5f}, {hi[j]:.5f}] (margin {np.rad2deg(margin):.0f} deg)")
    steps = bool(traj.meta.get("goal_steps"))
    q_exec = profile.filter_path(q, dt) if steps else q
    dq = np.gradient(q_exec, dt, axis=0)
    ddq = np.gradient(dq, dt, axis=0)
    for j in range(4):
        if np.abs(dq[:, j]).max() > vel_frac * spec.DQ_MAX[j] * tol:
            problems.append(f"joint {spec.JOINT_NAMES[j]} |dq| {np.abs(dq[:, j]).max():.3f} > {vel_frac:.2f}*limit")
        # goal-step trajectories: the profile bounds acceleration by construction; finite differences of its
        # snap-onto-goal step would only report a numerical spike, so acceleration is not checked for them
        if not steps and np.abs(ddq[:, j]).max() > acc_frac * spec.DDQ_MAX[j] * tol:
            problems.append(f"joint {spec.JOINT_NAMES[j]} |ddq| {np.abs(ddq[:, j]).max():.3f} > {acc_frac:.2f}*limit")
    zmin = {"elbow": np.inf, "wrist": np.inf, "tcp": np.inf}
    keepout_hit = 0
    for phi in q_exec[:: max(1, int(0.02 / dt))]:
        pts = workspace_points(phi)
        for k, p in pts.items():
            zmin[k] = min(zmin[k], p[2])
        r = np.hypot(*pts["tcp"][:2])
        if r < base_keepout[0] and pts["tcp"][2] < base_keepout[1]:
            keepout_hit += 1
    for k, z in zmin.items():
        if z < min_z:
            problems.append(f"{k} goes down to z = {z:.3f} m < {min_z:.3f} m")
    if keepout_hit:
        problems.append(f"TCP enters the base keep-out cylinder in {keepout_hit} samples")
    if traj.grip.min() < 0 or traj.grip.max() > 1:
        problems.append("gripper command outside [0, 1]")
    stats = {"duration_s": round(traj.duration, 2),
             "max_abs_dq": np.abs(dq).max(0).round(3).tolist(),
             "max_abs_ddq": np.abs(ddq).max(0).round(3).tolist(),
             "q_min_deg": np.rad2deg(q.min(0)).round(1).tolist(),
             "q_max_deg": np.rad2deg(q.max(0)).round(1).tolist(),
             "z_min": {k: round(float(v), 3) for k, v in zmin.items()}}
    return CheckResult(not problems, problems, stats)

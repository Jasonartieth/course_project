"""Backend-agnostic trajectory execution: approach -> settle -> trajectory -> tail, with logging and safety.

    approach  min-jerk move from the *measured* pose to the trajectory start at `approach_speed` x limits
    settle    hold the start pose `settle_s`
    traj      stream the trajectory (interpolated to the control rate)
    tail      hold the final pose `tail_s`
    abort     on a safety trip or Ctrl+C: hold the last measured pose for 1 s, log the reason, return

The first command is always the measured pose, so the arm never jumps when the loop starts. The runner sends
raw goals; the servo (or the simulator) applies its own profile. The runner's own ProfileLimiter output is
logged as q_ref and used by the safety monitor.
"""

import datetime as dt
import hashlib
import time
from pathlib import Path

import numpy as np

from . import spec
from .logio import PHASES, RunLog
from .profile import ProfileLimiter
from .safety import SafetyConfig, SafetyMonitor
from .trajectory import check_trajectory, min_jerk, workspace_points


class TrajectoryRejected(RuntimeError):
    pass


def traj_hash(traj):
    h = hashlib.sha1()
    for a in (traj.t, traj.q, traj.grip):
        h.update(np.ascontiguousarray(a, dtype=np.float64).tobytes())
    return h.hexdigest()[:12]


def _approach_ok(app, q_start):
    """Approach starts wherever the arm is (possibly folded near the base), so only require that it never goes
    lower than where it starts (minus 1 cm) or below TCP_MIN_Z, whichever is lower."""
    z0 = min(p[2] for p in workspace_points(q_start).values())
    res = check_trajectory(app, margin=0.0, min_z=min(spec.TCP_MIN_Z, z0 - 0.01), base_keepout=(0.0, 0.0))
    return res


def run_trajectory(backend, traj, out_dir=None, approach_speed=0.25, settle_s=1.0, tail_s=1.0,
                   safety=SafetyConfig(), notes="", verbose=True, check_kwargs=None):
    """Execute `traj` on an *opened* backend. Returns (RunLog, abort_reason or None). Saves to out_dir if given."""
    res = check_trajectory(traj, **(check_kwargs or {}))
    if not res.ok:
        raise TrajectoryRejected(f"{traj.name}: " + "; ".join(res.problems))

    rate = backend.rate_hz
    dtc = 1.0 / rate
    s = backend.read()
    q_start = s.q[:4].copy()
    app = min_jerk(q_start, traj.q[0], speed_frac=approach_speed, grip=float(traj.grip[0]), name="approach")
    app_res = _approach_ok(app, q_start)
    if not app_res.ok:
        raise TrajectoryRejected("approach path unsafe: " + "; ".join(app_res.problems) +
                                 " -> move the arm to a pose closer to the trajectory start first")

    segments = [("approach", app.duration), ("settle", settle_s), ("traj", traj.duration), ("tail", tail_s)]
    bounds = np.cumsum([d for _, d in segments])
    n_steps = int(np.ceil(bounds[-1] * rate)) + 1

    def goal_at(t):
        if t <= bounds[0]:
            return "approach", *app.sample(t)
        if t <= bounds[1]:
            return "settle", traj.q[0], float(traj.grip[0])
        if t <= bounds[2]:
            return "traj", *traj.sample(t - bounds[1])
        return "tail", traj.q[-1], float(traj.grip[-1])

    log = RunLog()
    log.meta.update({"trajectory": traj.name, "trajectory_hash": traj_hash(traj), "trajectory_meta": traj.meta,
                     "backend": backend.name, "rate_hz": rate, "approach_s": app.duration, "settle_s": settle_s,
                     "tail_s": tail_s, "q_start_measured": q_start.tolist(), "notes": notes,
                     "started": dt.datetime.now().isoformat(timespec="seconds"), "safety": vars(safety)})
    lim = ProfileLimiter()
    lim.reset(q_start)
    mon = SafetyMonitor(safety)
    abort, k, last_print = None, 0, -1e9
    backend.set_led((0.0, 1.0, 0.0))
    t_wall0 = time.perf_counter()
    t_prev = t_wall0
    try:
        for k in range(n_steps):
            t = k * dtc
            phase, q_goal, grip = goal_at(t)
            q_ref, dq_ref = lim.step(q_goal, dtc)
            s = backend.step(q_goal, grip)
            now = time.perf_counter()
            log.append(t=t, phase=PHASES[phase], loop_dt=now - t_prev, q_cmd=q_goal, grip_cmd=grip, q_ref=q_ref,
                       dq_ref=dq_ref, q=s.q, dq=s.dq, current=s.current, pwm=s.pwm, temp=s.temp)
            t_prev = now
            abort = mon.update(t, s, q_ref, dq_ref)
            if abort:
                break
            if verbose and t - last_print >= 2.0:
                last_print = t
                print(f"[{traj.name}] t={t:6.1f}/{bounds[-1]:.1f}s {phase:8s} q={np.rad2deg(s.q[:4]).round(1)} "
                      f"I={s.current[:4].round(2)} T={s.temp.max():.0f}C")
    except KeyboardInterrupt:
        abort = "KeyboardInterrupt (user stop)"
    if abort:
        backend.set_led((1.0, 0.0, 0.0))
        print(f"!!! ABORT [{traj.name}]: {abort} -> holding the measured pose for 1 s")
        q_hold = s.q[:4].copy()
        for i in range(int(rate)):
            s = backend.step(q_hold, s.q[4] if np.isfinite(s.q[4]) else spec.GRIP_MIN)
            log.append(t=(k + 1 + i) * dtc, phase=PHASES["abort_hold"], loop_dt=0.0, q_cmd=q_hold, grip_cmd=s.q[4],
                       q_ref=q_hold, dq_ref=np.zeros(4), q=s.q, dq=s.dq, current=s.current, pwm=s.pwm, temp=s.temp)
    else:
        backend.set_led((0.0, 0.0, 1.0))
    wall = time.perf_counter() - t_wall0
    arr = log.arrays()
    ldt = arr["loop_dt"][1:]
    log.meta.update({"aborted": abort, "finished": dt.datetime.now().isoformat(timespec="seconds"),
                     "wall_time_s": round(wall, 3), "sim_or_nominal_time_s": round(k * dtc, 3),
                     "loop_dt_ms": {"median": round(float(np.median(ldt)) * 1e3, 3) if len(ldt) else None,
                                    "p99": round(float(np.percentile(ldt, 99)) * 1e3, 3) if len(ldt) else None,
                                    "max": round(float(ldt.max()) * 1e3, 3) if len(ldt) else None}})
    if out_dir is not None:
        log.save(out_dir)
        if verbose:
            print(f"saved {Path(out_dir)}  ({'ABORTED: ' + abort if abort else 'completed'})")
    return log, abort

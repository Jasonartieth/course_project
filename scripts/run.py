"""Run trajectories on the real arm (pal) or in simulation (genesis / fake). One invocation = one session.

Examples
  # simulation dry run of the whole hardware-day set (4090, env qarm)
  python scripts/run.py --backend genesis --set data/trajectories/hwday_20261023
  # real arm: first contact, no motion at all (read only, 5 s)
  python scripts/run.py --backend pal --read-only 5
  # real arm: engage the servos where the arm is and hold 5 s
  python scripts/run.py --backend pal --hold-here 5
  # real arm: one trajectory
  python scripts/run.py --backend pal --traj data/trajectories/hwday_20261023/02_steps_shoulder.npz

Output: <out-root>/<YYYYmmdd_HHMMSS>_<backend>[_<tag>]/<NN_name>/{log.npz, meta.json, plot.png} and session.json
(out-root defaults: data/real for pal, results/sim_runs for simulation). Nothing is ever overwritten.

Stopping: Ctrl+C, SIGTERM (`kill <pid>`) and SIGHUP (SSH connection lost) all abort the current trajectory the
same way: hold the measured pose for 1 s, save the log, close the arm. Run real sessions inside tmux anyway.
"""

import argparse
import datetime as dt
import json
import signal
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from qarm_lab import spec  # noqa: E402
from qarm_lab import trajectory as T  # noqa: E402
from qarm_lab.backends import make_backend  # noqa: E402
from qarm_lab.logio import RunLog  # noqa: E402
from qarm_lab.plots import plot_run, summarize  # noqa: E402
from qarm_lab.runner import TrajectoryRejected, run_trajectory  # noqa: E402
from qarm_lab.safety import SafetyConfig  # noqa: E402


def read_only(backend, seconds, out_dir):
    log = RunLog()
    n = int(seconds * backend.rate_hz)
    nan4 = np.full(4, np.nan)
    for k in range(n):
        s = backend.read()
        log.append(t=s.t, phase=-1, loop_dt=0.0, q_cmd=nan4, grip_cmd=np.nan, q_ref=nan4, dq_ref=nan4,
                   q=s.q, dq=s.dq, current=s.current, pwm=s.pwm, temp=s.temp)
    a = log.arrays()
    log.meta.update({"trajectory": "read_only", "backend": backend.name, "rate_hz": backend.rate_hz,
                     "seconds": seconds, "aborted": None})
    log.save(out_dir)
    print(f"read-only {seconds} s, {n} samples -> {out_dir}")
    print("  q [deg]   mean", np.rad2deg(a["q"][:, :4].mean(0)).round(2), " std", np.rad2deg(a["q"][:, :4].std(0)).round(3))
    print("  I [A]     mean", a["current"][:, :4].mean(0).round(3), " std", a["current"][:, :4].std(0).round(3))
    print("  temp [C]  ", a["temp"][-1].round(1), "| gripper", a["q"][-1, 4].round(3))
    dt_s = np.diff(a["t"])
    print(f"  sample period ms: median {np.median(dt_s) * 1e3:.3f}  p99 {np.percentile(dt_s, 99) * 1e3:.3f}")


def _stop_like_ctrl_c(signum, _frame):
    # SSH drop (SIGHUP) or `kill <pid>` (SIGTERM) must take the same safe path as Ctrl+C: the runner catches
    # KeyboardInterrupt, holds the measured pose, saves the log and the session closes the arm.
    raise KeyboardInterrupt(f"signal {signal.Signals(signum).name}")


def main():
    for sig in (signal.SIGHUP, signal.SIGTERM):
        signal.signal(sig, _stop_like_ctrl_c)
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", required=True, choices=["pal", "genesis", "fake"])
    ap.add_argument("--traj", nargs="*", default=[], help="trajectory .npz files, run in the given order")
    ap.add_argument("--set", help="run every NN_*.npz of this directory in order (excluding 99_park unless --park)")
    ap.add_argument("--park", action="store_true", help="also run the set's 99_park at the end")
    ap.add_argument("--read-only", type=float, metavar="S", help="only read the arm for S seconds (no commands)")
    ap.add_argument("--hold-here", type=float, metavar="S", help="engage and hold the measured pose for S seconds")
    ap.add_argument("--rate", type=float, default=500.0)
    ap.add_argument("--device", default="cpu", help="genesis: cpu | gpu")
    ap.add_argument("--params", help="genesis: ServoParams JSON (identified values)")
    ap.add_argument("--q0", default="sleep", help="genesis/fake start pose: sleep | home | park")
    ap.add_argument("--approach-speed", type=float, default=0.25)
    ap.add_argument("--out-root")
    ap.add_argument("--tag", default="")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--check-only", action="store_true", help="only run the offline checks")
    a = ap.parse_args()

    files = [Path(f) for f in a.traj]
    if a.set:
        files += sorted(p for p in Path(a.set).glob("[0-9][0-9]_*.npz") if a.park or not p.name.startswith("99_"))
    trajs = [(f.stem, T.Trajectory.load(f)) for f in files]
    for stem, tr in trajs:
        r = T.check_trajectory(tr)
        print(f"check {stem:28s} {'OK ' if r.ok else 'FAIL'} {r.stats['duration_s']:7.1f} s  {'; '.join(r.problems)}")
        if not r.ok:
            sys.exit(f"refusing to run: {stem} failed the checks")
    if a.check_only:
        return

    q0 = {"sleep": spec.SLEEP_POSE, "home": spec.HOME_POSE,
          "park": np.array([0.0, np.deg2rad(-80), np.deg2rad(70), 0.0])}[a.q0]
    kw = {"rate_hz": a.rate}
    if a.backend == "genesis":
        kw.update(device=a.device, params=a.params, q0=q0)
    elif a.backend == "fake":
        kw.update(q0=q0)
    backend = make_backend(a.backend, **kw)
    out_root = Path(a.out_root) if a.out_root else ROOT / ("data/real" if a.backend == "pal" else "results/sim_runs")
    session = out_root / f"{dt.datetime.now():%Y%m%d_%H%M%S}_{a.backend}{'_' + a.tag if a.tag else ''}"
    session.mkdir(parents=True, exist_ok=False)
    summary = {"argv": sys.argv, "runs": []}
    backend.open()
    try:
        if a.read_only:
            read_only(backend, a.read_only, session / "read_only")
        if a.hold_here:
            s = backend.read()
            tr = T.hold(s.q[:4], a.hold_here, grip=float(np.clip(s.q[4], spec.GRIP_MIN, spec.GRIP_MAX)), name="hold_here")
            trajs.insert(0, ("hold_here", tr))
        for stem, tr in trajs:
            run_dir = session / stem
            ck = None
            if stem == "hold_here":       # holding wherever the arm already is: no workspace margins
                ck = dict(margin=0.0, limit_tol=SafetyConfig().pos_tol, min_z=-1.0, base_keepout=(0.0, 0.0))
            try:
                _, abort = run_trajectory(backend, tr, out_dir=run_dir, approach_speed=a.approach_speed,
                                          check_kwargs=ck)
            except TrajectoryRejected as e:
                print(f"REJECTED {stem}: {e}")
                summary["runs"].append({"trajectory": stem, "rejected": str(e)})
                break
            if not a.no_plot:
                plot_run(run_dir)
            summary["runs"].append(summarize(run_dir))
            print(json.dumps(summary["runs"][-1]))
            if abort:
                print("stopping the session after an abort")
                break
    finally:
        backend.close()
        (session / "session.json").write_text(json.dumps(summary, indent=2))
        print(f"session -> {session}")


if __name__ == "__main__":
    main()

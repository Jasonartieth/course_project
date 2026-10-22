"""Replay a recorded run (normally from the real arm) in Genesis and compare: the real2sim check.

The simulator starts from the recorded first measured pose and receives exactly the recorded goal sequence
(q_cmd, grip_cmd) at the recorded control rate, so both logs share one timeline. Output:
  <out>/log.npz, meta.json        simulated run (same format as every run)
  <out>/compare.png               overlay real vs sim (position, tracking error, current)
  <out>/metrics.json              per-joint RMSE / max of (q_sim - q_real) and (I_sim - I_real), trajectory phase

Usage (4090):  python scripts/sim/replay_in_sim.py data/real/<session>/<run> [--params sim_params.json]
               [--out results/replay_<ts>/<run>]
"""

import argparse
import datetime as dt
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from qarm_lab.backends import make_backend  # noqa: E402
from qarm_lab.logio import PHASES, RunLog, load_run  # noqa: E402
from qarm_lab.plots import plot_run  # noqa: E402
from qarm_lab.profile import ProfileLimiter  # noqa: E402


def replay(real_dir, out_dir, params=None, device="cpu"):
    r, meta = load_run(real_dir)
    ok = np.isfinite(r["q_cmd"]).all(1)
    if not ok.any():
        raise SystemExit(f"{real_dir} has no commands (read-only run?)")
    rate = float(meta["rate_hz"])
    be = make_backend("genesis", rate_hz=rate, params=params, q0=r["q"][0, :4], device=device)
    be.open()
    lim = ProfileLimiter(be.params.v_max, be.params.a_max)
    lim.reset(r["q"][0, :4])
    log = RunLog()
    for k in np.flatnonzero(ok):
        q_ref, dq_ref = lim.step(r["q_cmd"][k], 1.0 / rate)
        s = be.step(r["q_cmd"][k], r["grip_cmd"][k])
        log.append(t=r["t"][k], phase=r["phase"][k], loop_dt=0.0, q_cmd=r["q_cmd"][k], grip_cmd=r["grip_cmd"][k],
                   q_ref=q_ref, dq_ref=dq_ref, q=s.q, dq=s.dq, current=s.current, pwm=s.pwm, temp=s.temp)
    log.meta.update({"trajectory": meta.get("trajectory"), "backend": "genesis(replay)", "rate_hz": rate,
                     "replay_of": str(real_dir), "params": str(params) if params else "default",
                     "aborted": None, "created": dt.datetime.now().isoformat(timespec="seconds")})
    log.save(out_dir)
    s_arr = log.arrays()
    m = s_arr["phase"] == PHASES["traj"]
    rm = r["phase"][ok] == PHASES["traj"]
    dq = s_arr["q"][m, :4] - r["q"][ok][rm, :4]
    di = s_arr["current"][m, :4] - r["current"][ok][rm, :4]
    metrics = {"q_rmse_deg": np.rad2deg(np.sqrt((dq ** 2).mean(0))).round(3).tolist(),
               "q_max_deg": np.rad2deg(np.abs(dq).max(0)).round(3).tolist(),
               "current_rmse_A": np.sqrt((di ** 2).mean(0)).round(3).tolist(),
               "current_max_A": np.abs(di).max(0).round(3).tolist(),
               "n_samples": int(m.sum())}
    (Path(out_dir) / "metrics.json").write_text(json.dumps(metrics, indent=2))
    plot_run(real_dir, other_dir=out_dir, labels=("real", "sim"), out_name="compare_tmp.png")
    Path(real_dir, "compare_tmp.png").replace(Path(out_dir) / "compare.png")
    return metrics


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--params")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--out")
    a = ap.parse_args()
    out = Path(a.out) if a.out else ROOT / "results" / f"replay_{dt.datetime.now():%Y%m%d_%H%M%S}" / Path(a.run_dir).name
    m = replay(a.run_dir, out, a.params, a.device)
    print(json.dumps(m, indent=2))
    print("->", out)


if __name__ == "__main__":
    main()

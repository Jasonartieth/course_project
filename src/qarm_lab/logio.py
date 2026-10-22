"""Run logs shared by the real-arm and simulated backends (identical format -> identical analysis).

A run directory contains:
  log.npz    per-control-step arrays (see FIELDS)
  meta.json  backend, trajectory name/hash, rates, abort reason, git commit, timestamps, notes
"""

import json
import subprocess
from pathlib import Path

import numpy as np

# name -> width (1 = scalar per step). q/dq/current/pwm/temp are 5 wide: 4 joints + gripper (pal order).
FIELDS = {
    "t": 1, "phase": 1, "loop_dt": 1,
    "q_cmd": 4, "grip_cmd": 1, "q_ref": 4, "dq_ref": 4,
    "q": 5, "dq": 5, "current": 5, "pwm": 5, "temp": 5,
}
PHASES = {"approach": 0, "settle": 1, "traj": 2, "tail": 3, "abort_hold": 4}


class RunLog:
    def __init__(self):
        self.rows = {k: [] for k in FIELDS}
        self.meta = {}

    def append(self, **values):
        for k in FIELDS:
            self.rows[k].append(np.array(values[k], dtype=float, copy=True))   # copy: pal reuses its buffers

    def arrays(self):
        return {k: np.array(v) for k, v in self.rows.items()}

    def save(self, out_dir):
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(out / "log.npz", **self.arrays())
        self.meta.setdefault("git_commit", git_commit())
        (out / "meta.json").write_text(json.dumps(self.meta, indent=2, default=str))
        return out


def load_run(run_dir):
    run_dir = Path(run_dir)
    z = np.load(run_dir / "log.npz")
    meta = json.loads((run_dir / "meta.json").read_text())
    return {k: z[k] for k in z.files}, meta


def git_commit():
    try:
        return subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=Path(__file__).parent,
                                       text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return "unknown"

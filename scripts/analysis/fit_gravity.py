"""Gravity / torque-constant identification from the static-pose experiment (09_gravity_poses).

For every hold window the mean measured pose and current over the last `--avg` seconds are taken.
  * Without a payload run: tau = Kt * I with Kt = KT_NOMINAL; the gravity parameters theta (dynamics.py) are
    fitted by ridge regression toward the official values (only some combinations are identifiable; the SVD of
    the stacked regressor is reported).
  * With a payload run (same trajectory, known mass held at the TCP): the current difference at each pose is
    caused by a known torque tau_p = J_tcp^T [0, 0, m g], which calibrates Kt per joint first.
Static friction biases hold currents (depends on approach direction); the residuals show how large that is.

Usage:  python scripts/analysis/fit_gravity.py <run_dir> [--payload-run <run_dir> --payload-kg 0.35]
Output: results/sysid_<ts>/gravity_fit.json, gravity_fit.png
"""

import argparse
import datetime as dt
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from qarm_lab import dynamics, kinematics, spec  # noqa: E402
from qarm_lab.logio import PHASES, load_run  # noqa: E402

FIT_JOINTS = [1, 2]          # shoulder, elbow carry the gravity load (base: none; wrist: tiny)


def hold_means(run_dir, avg_s):
    r, meta = load_run(run_dir)
    windows = meta["trajectory_meta"]["hold_windows"]
    t_traj0 = r["t"][np.argmax(r["phase"] == PHASES["traj"])]
    qs, cur = [], []
    for t0, t1, idx in windows:
        m = (r["t"] >= t_traj0 + t1 - avg_s) & (r["t"] < t_traj0 + t1) & (r["phase"] == PHASES["traj"])
        if m.sum() < 10:
            continue
        qs.append(r["q"][m, :4].mean(0))
        cur.append(r["current"][m, :4].mean(0))
    return np.array(qs), np.array(cur), meta


def payload_torque(q, mass):
    return kinematics.jacobian(q).T @ np.array([0.0, 0.0, mass * dynamics.G])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--payload-run")
    ap.add_argument("--payload-kg", type=float)
    ap.add_argument("--avg", type=float, default=2.0)
    ap.add_argument("--ridge", type=float, default=1e-2, help="weight pulling theta toward the official values")
    a = ap.parse_args()

    q, I, meta = hold_means(a.run_dir, a.avg)
    kt = spec.KT_NOMINAL.copy()
    out = {"run": a.run_dir, "n_poses": len(q), "kt_source": "nominal (stall torque / stall current)"}
    if a.payload_run:
        qp, Ip, _ = hold_means(a.payload_run, a.avg)
        n = min(len(q), len(qp))
        tau_p = np.array([payload_torque(x, a.payload_kg) for x in qp[:n]])
        dI = Ip[:n] - I[:n]
        for j in FIT_JOINTS:
            kt[j] = float(tau_p[:, j] @ tau_p[:, j] / (tau_p[:, j] @ dI[:, j]))
        out.update(kt_source=f"payload {a.payload_kg} kg ({a.payload_run})")
    out["kt"] = kt.round(4).tolist()

    tau = I * kt
    Y = np.vstack([dynamics.gravity_regressor(x)[FIT_JOINTS] for x in q])
    b = np.concatenate([t[FIT_JOINTS] for t in tau])
    th0 = dynamics.theta_official()
    lam = a.ridge * np.linalg.norm(Y, 2) ** 2
    th = np.linalg.solve(Y.T @ Y + lam * np.eye(16), Y.T @ b + lam * th0)
    sv = np.linalg.svd(Y, compute_uv=False)
    pred_off = np.vstack([dynamics.gravity_torque(x)[FIT_JOINTS] for x in q])
    pred_fit = np.vstack([dynamics.gravity_torque(x, th)[FIT_JOINTS] for x in q])
    meas = tau[:, FIT_JOINTS]
    out.update({
        "rms_resid_official_Nm": np.sqrt(((meas - pred_off) ** 2).mean(0)).round(4).tolist(),
        "rms_resid_fitted_Nm": np.sqrt(((meas - pred_fit) ** 2).mean(0)).round(4).tolist(),
        "singular_values": sv.round(4).tolist(),
        "theta_official": th0.round(5).tolist(), "theta_fitted": th.round(5).tolist(),
        "joints": [spec.JOINT_NAMES[j] for j in FIT_JOINTS]})
    res_dir = ROOT / "results" / f"sysid_{dt.datetime.now():%Y%m%d_%H%M%S}"
    res_dir.mkdir(parents=True)
    (res_dir / "gravity_fit.json").write_text(json.dumps(out, indent=2))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, len(FIT_JOINTS), figsize=(12, 4.5))
    for ax, k in zip(axs, range(len(FIT_JOINTS))):
        ax.plot(meas[:, k], "ko", ms=4, label="measured (Kt * I)")
        ax.plot(pred_off[:, k], "b.-", lw=0.8, label="official params")
        ax.plot(pred_fit[:, k], "r.--", lw=0.8, label="fitted")
        ax.set_title(f"{spec.JOINT_NAMES[FIT_JOINTS[k]]}: hold torque per pose")
        ax.set_xlabel("pose #")
        ax.set_ylabel("N*m")
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(res_dir / "gravity_fit.png", dpi=90)
    print(json.dumps({k: out[k] for k in ("n_poses", "kt", "kt_source", "rms_resid_official_Nm", "rms_resid_fitted_Nm")}))
    print("->", res_dir)


if __name__ == "__main__":
    main()

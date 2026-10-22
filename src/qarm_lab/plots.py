"""Plots for run logs (real and simulated share the format). matplotlib is imported lazily."""

from pathlib import Path

import numpy as np

from . import spec
from .logio import PHASES, load_run


def plot_run(run_dir, other_dir=None, labels=("run", "other"), out_name="plot.png"):
    """Per joint: position (cmd / ref / measured), tracking error, current and PWM. If `other_dir` is given
    (e.g. sim vs real of the same trajectory) its measured position/current are overlaid, aligned on the start of
    the trajectory phase."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    a, meta = load_run(run_dir)
    t0 = a["t"][np.argmax(a["phase"] == PHASES["traj"])] if (a["phase"] == PHASES["traj"]).any() else 0.0
    t = a["t"] - t0
    b = None
    if other_dir is not None:
        b, _ = load_run(other_dir)
        tb = b["t"] - (b["t"][np.argmax(b["phase"] == PHASES["traj"])] if (b["phase"] == PHASES["traj"]).any() else 0)
    fig, axs = plt.subplots(4, 3, figsize=(18, 12), sharex=True)
    for j in range(4):
        ax = axs[j, 0]
        ax.plot(t, np.rad2deg(a["q_cmd"][:, j]), lw=0.8, label="goal (cmd)")
        ax.plot(t, np.rad2deg(a["q_ref"][:, j]), lw=0.8, ls="--", label="profile ref")
        ax.plot(t, np.rad2deg(a["q"][:, j]), lw=1.0, label=f"measured ({labels[0]})")
        if b is not None:
            ax.plot(tb, np.rad2deg(b["q"][:, j]), lw=1.0, alpha=0.8, label=f"measured ({labels[1]})")
        ax.set_ylabel(f"{spec.JOINT_NAMES[j]} [deg]")
        ax = axs[j, 1]
        ax.plot(t, np.rad2deg(a["q"][:, j] - a["q_ref"][:, j]), lw=0.8, label=labels[0])
        if b is not None:
            ax.plot(tb, np.rad2deg(b["q"][:, j] - b["q_ref"][:, j]), lw=0.8, alpha=0.8, label=labels[1])
        ax.set_ylabel("q - q_ref [deg]")
        ax = axs[j, 2]
        ax.plot(t, a["current"][:, j], lw=0.8, label=f"current {labels[0]} [A]")
        if b is not None:
            ax.plot(tb, b["current"][:, j], lw=0.8, alpha=0.8, label=f"current {labels[1]} [A]")
        ax.plot(t, a["pwm"][:, j] * spec.I_INST[j], lw=0.6, alpha=0.6, label=f"pwm x I_inst ({labels[0]})")
        ax.axhline(spec.I_CONT[j], color="r", lw=0.5, ls=":")
        ax.axhline(-spec.I_CONT[j], color="r", lw=0.5, ls=":")
        ax.set_ylabel("[A]")
    for ax in axs[0]:
        ax.legend(fontsize=7, loc="upper right")
    for ax in axs[-1]:
        ax.set_xlabel("t from trajectory start [s]")
    for k, v in PHASES.items():
        if k in ("traj",):
            continue
    abort = meta.get("aborted")
    fig.suptitle(f"{meta.get('trajectory')} | backend {meta.get('backend')} | rate {meta.get('rate_hz')} Hz | "
                 f"{'ABORTED: ' + abort if abort else 'completed'} | loop dt ms {meta.get('loop_dt_ms')}", fontsize=10)
    fig.tight_layout()
    out = Path(run_dir) / out_name
    fig.savefig(out, dpi=80)
    plt.close(fig)
    return out


def summarize(run_dir):
    a, meta = load_run(run_dir)
    m = a["phase"] == PHASES["traj"]
    if not m.any():
        m = np.ones(len(a["t"]), bool)
    err = np.rad2deg(np.abs(a["q"][m, :4] - a["q_ref"][m])).max(0)
    return {"trajectory": meta.get("trajectory"), "aborted": meta.get("aborted"),
            "max_track_err_deg": err.round(2).tolist(),
            "max_abs_current_A": np.abs(a["current"][m, :4]).max(0).round(2).tolist(),
            "max_temp_C": float(np.nanmax(a["temp"])), "loop_dt_ms": meta.get("loop_dt_ms")}

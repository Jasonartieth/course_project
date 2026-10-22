"""Smoke test of the Genesis backend: link masses, static hold at SLEEP, TCP vs DH FK, one step trajectory.

Usage (4090, env `qarm`):  python scripts/sim/smoke_genesis.py [--device cpu|gpu] [--params sim_params.json]
"""

import argparse
import tempfile
import time

import numpy as np

from qarm_lab import kinematics as K
from qarm_lab import spec
from qarm_lab import trajectory as T
from qarm_lab.backends import make_backend
from qarm_lab.runner import run_trajectory


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--params", default=None)
    a = ap.parse_args()
    be = make_backend("genesis", rate_hz=500, q0=spec.SLEEP_POSE, device=a.device, params=a.params)
    be.open()
    masses = np.asarray(be.arm.get_links_mass().cpu()).reshape(-1)
    print("link masses [kg]:", {l.name: round(float(m), 4) for l, m in zip(be.arm.links, masses)})

    t0 = time.time()
    for _ in range(1000):
        s = be.step(spec.SLEEP_POSE, 0.1)
    print(f"hold SLEEP 2 s: drift [deg] {np.rad2deg(s.q[:4] - spec.SLEEP_POSE).round(3)}  I [A] {s.current[:4].round(3)}"
          f"  pwm {s.pwm[:4].round(3)}  (wall {time.time() - t0:.1f} s)")
    print("TCP sim / DH @SLEEP:", be.tcp_position().round(4), K.fk(s.q[:4])[0].round(4))

    t0 = time.time()
    log, abort = run_trajectory(be, T.joint_steps(1), out_dir=tempfile.mkdtemp(), verbose=False)
    r = log.arrays()
    m = r["phase"] == 2
    err = np.abs(r["q"][m, :4] - r["q_ref"][m]).max(0)
    print(f"steps_shoulder: abort={abort} sim {r['t'][-1]:.1f} s in wall {time.time() - t0:.1f} s")
    print(f"  max|q - q_ref| [deg] {np.rad2deg(err).round(2)}  max|I| [A] {np.abs(r['current'][m, :4]).max(0).round(2)}"
          f"  max|pwm| {np.abs(r['pwm'][m, :4]).max(0).round(2)}")
    print("TCP sim / DH @end:", be.tcp_position().round(4), K.fk(r["q"][-1, :4])[0].round(4))


if __name__ == "__main__":
    main()

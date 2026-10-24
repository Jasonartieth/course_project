"""Physics sanity check of a QArm-with-gripper URDF in Genesis with collisions ON and gravity ON.

  1. hold: PD-hold HOME for 4 s while the gripper cycles open -> closed -> open twice (recommended mapping from
     gripper_report.json next to the URDF, else --close). Pass: states finite, arm tracking error < 2 deg.
  2. drop: three 3 cm / 20 g cubes are dropped from 6 cm above the palm (gripper body, merged into END-EFFECTOR
     by Genesis), finger A and finger B of the open gripper. Pass: every cube touches the link(s) it was dropped
     on, i.e. the collision geometry is really there. (A cube dropped *between* the open fingers can legitimately
     fall through the ~36 mm gap, so that is not used as a test.)

Usage (4090):  python scripts/model/verify_qarm_genesis.py <urdf> [--close a1,a2,b1,b2] [--device cpu]
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from qarm_lab import spec  # noqa: E402

ARM = ("YAW", "SHOULDER", "ELBOW", "WRIST")
FINGERS = ("Joint1A", "Joint2A", "Joint1B", "Joint2B")
DT = 0.002


def quat_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def link_point(arm, name, local):
    link = arm.get_link(name)
    p = link.get_pos().cpu().numpy().reshape(-1)
    R = quat_R(link.get_quat().cpu().numpy().reshape(-1))
    return p + R @ np.asarray(local, float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf")
    ap.add_argument("--close")
    ap.add_argument("--device", default="cpu")
    a = ap.parse_args()
    urdf = Path(a.urdf).resolve()
    if a.close:
        close = np.array([float(x) for x in a.close.split(",")])
    else:
        close = np.array(json.loads((urdf.parent / "gripper_report.json").read_text())
                         ["recommended_mapping"]["coefficients_at_grip_1"])

    import genesis as gs
    gs.init(backend=gs.gpu if a.device == "gpu" else gs.cpu, logging_level="warning")
    scene = gs.Scene(sim_options=gs.options.SimOptions(dt=DT, substeps=2), show_viewer=False)
    scene.add_entity(gs.morphs.Plane())
    arm = scene.add_entity(gs.morphs.URDF(file=str(urdf), fixed=True))
    cubes = {k: scene.add_entity(gs.morphs.Box(size=(0.03, 0.03, 0.03), pos=(1.5, 0.3 * i, 0.015)))
             for i, k in enumerate(("palm", "fingerA", "fingerB"))}
    scene.build()
    for c in cubes.values():
        c.set_mass(0.02)
    arm_dofs = [arm.get_joint(n).dofs_idx_local[0] for n in ARM]
    f_dofs = [arm.get_joint(n).dofs_idx_local[0] for n in FINGERS]
    dofs = arm_dofs + f_dofs
    arm.set_dofs_kp(np.r_[np.full(3, 400.0), 50.0, np.full(4, 2.0)], dofs)
    arm.set_dofs_kv(np.r_[np.full(3, 30.0), 3.0, np.full(4, 0.1)], dofs)
    caps = np.r_[spec.TAU_INST, np.full(4, spec.GRIP_TAU_INST)]
    arm.set_dofs_force_range(-caps, caps, dofs)
    arm.set_dofs_position(np.r_[spec.HOME_POSE, np.zeros(4)], dofs)

    # 1) hold + gripper cycles
    err_max, finite = 0.0, True
    n = int(4.0 / DT)
    for k in range(n):
        g = 0.5 - 0.5 * np.cos(2 * np.pi * k / (n / 2))
        arm.control_dofs_position(np.r_[spec.HOME_POSE, g * close], dofs)
        scene.step()
        q = arm.get_dofs_position(dofs).cpu().numpy().reshape(-1)
        if not np.isfinite(q).all():
            finite = False
            break
        if k > 500:
            err_max = max(err_max, float(np.abs(q[:4] - spec.HOME_POSE).max()))
    hold = {"finite": finite, "arm_err_max_deg": round(float(np.rad2deg(err_max)), 3),
            "pass": bool(finite and np.rad2deg(err_max) < 2.0)}

    # 2) drops onto palm / finger A / finger B of the open gripper
    arm.control_dofs_position(np.r_[spec.HOME_POSE, np.zeros(4)], dofs)
    for _ in range(int(1.0 / DT)):
        scene.step()
    targets = {"palm": (link_point(arm, "END-EFFECTOR", (0, 0, 0.06)), {"END-EFFECTOR"}),
               "fingerA": (link_point(arm, "Link1A", (0.05, 0, 0)), {"Link1A", "Link2A"}),
               "fingerB": (link_point(arm, "Link1B", (0.05, 0, 0)), {"Link1B", "Link2B"})}
    for k, (p, _) in targets.items():
        cubes[k].set_pos(p + np.array([0, 0, 0.06]))
        cubes[k].set_dofs_velocity(np.zeros(6))
    idx2name = {l.idx: l.name for l in arm.links}
    touched = {k: set() for k in cubes}
    for _ in range(int(1.5 / DT)):
        scene.step()
        for k, c in cubes.items():
            con = arm.get_contacts(with_entity=c)
            for la, lb in zip(np.asarray(con["link_a"]).reshape(-1), np.asarray(con["link_b"]).reshape(-1)):
                for li in (int(la), int(lb)):
                    if li in idx2name:
                        touched[k].add(idx2name[li])
    drop = {}
    for k, (p, want) in targets.items():
        z = float(cubes[k].get_pos().cpu().numpy().reshape(-1)[2])
        drop[k] = {"dropped_above": np.round(p, 3).tolist(), "touched_links": sorted(touched[k]),
                   "final_z_m": round(z, 3), "pass": bool(touched[k] & want)}
    ok = hold["pass"] and all(d["pass"] for d in drop.values())
    print(json.dumps({"urdf": str(urdf), "close": close.tolist(), "hold": hold, "drop": drop, "pass": ok}, indent=1))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

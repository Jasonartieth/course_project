"""Physics sanity check of a QArm-with-gripper URDF in Genesis with collisions ON and gravity ON.

  1. hold: PD-hold HOME for 2 s while the gripper cycles open -> closed -> open twice (recommended mapping from
     gripper_report.json next to the URDF, else --close). Pass: states finite, arm tracking error < 2 deg.
  2. drop: a 3 cm, 20 g cube is dropped from 6 cm above the open fingers. Pass: after 1.5 s it is still above the
     fingers (z > 0.40 m) instead of having fallen through the collision geometry to the floor.

Usage (4090):  python scripts/model/verify_qarm_genesis.py <urdf> [--close a1,a2,b1,b2] [--device cpu]
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from qarm_lab import kinematics, spec  # noqa: E402

ARM = ("YAW", "SHOULDER", "ELBOW", "WRIST")
FINGERS = ("Joint1A", "Joint2A", "Joint1B", "Joint2B")


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
    scene = gs.Scene(sim_options=gs.options.SimOptions(dt=0.002, substeps=2), show_viewer=False)
    scene.add_entity(gs.morphs.Plane())
    arm = scene.add_entity(gs.morphs.URDF(file=str(urdf), fixed=True))
    tcp = kinematics.fk(spec.HOME_POSE)[0]
    cube = scene.add_entity(gs.morphs.Box(size=(0.03, 0.03, 0.03), pos=(tcp[0] - 0.03, tcp[1], tcp[2] + 0.06)))
    scene.build()
    cube.set_mass(0.02)
    arm_dofs = [arm.get_joint(n).dofs_idx_local[0] for n in ARM]
    f_dofs = [arm.get_joint(n).dofs_idx_local[0] for n in FINGERS]
    dofs = arm_dofs + f_dofs
    arm.set_dofs_kp(np.r_[np.full(3, 400.0), 50.0, np.full(4, 2.0)], dofs)
    arm.set_dofs_kv(np.r_[np.full(3, 30.0), 3.0, np.full(4, 0.1)], dofs)
    caps = np.r_[spec.TAU_INST, np.full(4, spec.GRIP_TAU_INST)]
    arm.set_dofs_force_range(-caps, caps, dofs)
    arm.set_dofs_position(np.r_[spec.HOME_POSE, np.zeros(4)], dofs)
    cube_frozen = cube.get_pos().clone()

    # 1) hold + gripper cycles (cube kept out of the way: reset to its start each step until the drop test)
    err_max, grip_ok = 0.0, True
    n = int(4.0 / 0.002)
    for k in range(n):
        g = 0.5 - 0.5 * np.cos(2 * np.pi * k / (n / 2))
        arm.control_dofs_position(np.r_[spec.HOME_POSE, g * close], dofs)
        cube.set_pos(cube_frozen)
        cube.set_dofs_velocity(np.zeros(6))
        scene.step()
        q = arm.get_dofs_position(dofs).cpu().numpy().reshape(-1)
        if not np.isfinite(q).all():
            grip_ok = False
            break
        if k > 500:
            err_max = max(err_max, float(np.abs(q[:4] - spec.HOME_POSE).max()))
    hold = {"finite": grip_ok, "arm_err_max_deg": round(np.rad2deg(err_max), 3),
            "pass": bool(grip_ok and np.rad2deg(err_max) < 2.0)}

    # 2) drop the cube onto the open fingers
    arm.control_dofs_position(np.r_[spec.HOME_POSE, np.zeros(4)], dofs)
    for _ in range(int(1.5 / 0.002)):
        scene.step()
    z = float(cube.get_pos().cpu().numpy().reshape(-1)[2])
    drop = {"cube_final_z_m": round(z, 4), "tcp_z_m": round(float(tcp[2]), 4), "pass": bool(z > 0.40)}
    res = {"urdf": str(urdf), "close": close.tolist(), "hold": hold, "drop": drop}
    print(json.dumps(res, indent=1))
    sys.exit(0 if hold["pass"] and drop["pass"] else 1)


if __name__ == "__main__":
    main()

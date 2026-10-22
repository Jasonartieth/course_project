"""Render sanity-check images of the generated QArm URDF (gripper mount, finger closing direction, TCP).

Runs on the Mac with the MuJoCo package from env/requirements-sim.txt (MuJoCo is only used as a URDF viewer
here; the simulator of record is Genesis on the 4090). Markers: red sphere = tcp link origin,
green sphere = END-EFFECTOR origin (DH wrist point is 12 mm further along the wrist axis), blue = wrist axis.

Usage:  python scripts/model/render_model_check.py [--urdf models/qarm/urdf/qarm_gripper.urdf]
Output: results/model_check_<timestamp>/*.png
"""

import argparse
import datetime as dt
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts" / "model"))
import build_qarm_urdf as B  # noqa: E402

from qarm_lab.spec import GRIPPER_FINGER_CLOSE as FINGER_CLOSE  # noqa: E402


def load(urdf_path):
    # MuJoCo's URDF parser drops <visual> geoms unless discardvisual="false" is set *before* parsing, and fuses
    # fixed-joint links (tcp, gripper_base) unless fusestatic="false" -> inject a <mujoco> block into the text.
    urdf_path = Path(urdf_path).resolve()
    xml = urdf_path.read_text()
    inject = (f'<mujoco><compiler meshdir="{urdf_path.parent}/" discardvisual="false" fusestatic="false" '
              f'strippath="false"/></mujoco>')
    i = xml.index(">", xml.index("<robot")) + 1
    spec_ = mujoco.MjSpec.from_string(xml[:i] + inject + xml[i:])
    spec_.visual.global_.offwidth, spec_.visual.global_.offheight = 960, 720
    marks = {"tcp": [1, 0, 0, 1], "END-EFFECTOR": [0, 1, 0, 1]}
    for body in spec_.bodies:
        if body.name in marks:
            body.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.006, 0, 0], rgba=marks[body.name],
                          contype=0, conaffinity=0, group=0)
        if body.name == "END-EFFECTOR":
            body.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[0, 0, -0.05, 0, 0, 0.2], size=[0.0015, 0, 0],
                          rgba=[0, 0, 1, 0.6], contype=0, conaffinity=0, group=0)
    spec_.worldbody.add_light(pos=[0.5, -0.5, 1.5], dir=[-0.3, 0.3, -1])
    spec_.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1, 1, 0.01], rgba=[0.85, 0.85, 0.85, 1],
                             contype=0, conaffinity=0)
    # hide the convex-hull collision geoms (their mesh files live in meshes/collision/)
    col_meshes = {me.name for me in spec_.meshes if "/collision/" in me.file}
    for g in spec_.geoms:
        if g.meshname in col_meshes:
            g.rgba = [0, 0, 0, 0]
    return spec_.compile()


def set_q(m, d, q_arm, grip=0.0):
    names = list(B.ARM_JOINTS) + list(FINGER_CLOSE)
    vals = list(q_arm) + [grip * FINGER_CLOSE[n] for n in FINGER_CLOSE]
    for n, v in zip(names, vals):
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        if jid >= 0:
            d.qpos[m.jnt_qposadr[jid]] = v
    mujoco.mj_forward(m, d)


def shot(m, d, r, lookat, dist, az, el):
    cam = mujoco.MjvCamera()
    cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = lookat, dist, az, el
    r.update_scene(d, camera=cam)
    return r.render()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=str(ROOT / "models/qarm/urdf/qarm_gripper.urdf"))
    args = ap.parse_args()
    out = ROOT / "results" / f"model_check_{dt.datetime.now():%Y%m%d_%H%M%S}"
    out.mkdir(parents=True, exist_ok=True)
    m = load(args.urdf)
    d = mujoco.MjData(m)
    r = mujoco.Renderer(m, 720, 960)
    tcp_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "tcp")
    assert tcp_id >= 0, "tcp body missing (fused away?)"

    panels = []
    for grip in (0.0, 1.0):
        set_q(m, d, [0, 0, 0, 0], grip)
        tcp = d.xpos[tcp_id].copy()
        panels += [
            (f"home, grip={grip:.0f}: full arm", shot(m, d, r, [0.25, 0, 0.3], 1.3, 135, -20)),
            (f"grip={grip:.0f}: wrist, side view (along -y)", shot(m, d, r, tcp - [0.08, 0, 0], 0.32, 90, 0)),
            (f"grip={grip:.0f}: wrist, top view", shot(m, d, r, tcp - [0.08, 0, 0], 0.32, 90, -89)),
            (f"grip={grip:.0f}: wrist, front view (along -x)", shot(m, d, r, tcp, 0.30, 180, -10)),
        ]
    fig, axs = plt.subplots(2, 4, figsize=(22, 9))
    for ax, (title, img) in zip(axs.flat, panels):
        ax.imshow(img)
        ax.set_title(title, fontsize=10)
        ax.axis("off")
    fig.suptitle(f"{Path(args.urdf).name} | red = tcp, green = END-EFFECTOR origin, blue = wrist axis | "
                 f"mount xyz={B.GRIPPER_MOUNT_XYZ}", fontsize=11)
    fig.tight_layout()
    fig.savefig(out / "model_check.png", dpi=80)
    set_q(m, d, [0, 0, 0, 0])
    print("tcp @home (MuJoCo):", np.round(d.xpos[tcp_id], 4), "| expected (DH): [0.45 0. 0.49]")
    print("saved", out / "model_check.png")


if __name__ == "__main__":
    main()

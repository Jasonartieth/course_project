"""Audit how Genesis actually sees a QArm URDF's collision geometry.

Reports (Genesis 1.4.x, default URDF morph options, i.e. what a plain gs.morphs.URDF(file=...) gets):
  * parser warnings (legacy-parser fallback, dubious centres of mass, neutral self-collision pairs filtered out)
  * per link: number of collision geoms, faces, convexity, AABB size (Genesis convexifies + decimates meshes)
  * self-collision link pairs over: the neutral pose, a gripper open->close sweep at HOME, and random poses
  * render: visual geometry vs the collision geometry Genesis uses (full arm, gripper open, gripper closed)

Usage (4090, env qarm):
  python scripts/model/collision_audit.py <urdf> [--tag name] [--samples 400] [--device cpu]
Output: results/collision_audit_<timestamp>_<tag>/{report.json, log.txt, visual_vs_collision.png}
"""

import argparse
import datetime as dt
import io
import json
import re
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from qarm_lab import spec  # noqa: E402

ARM_JOINTS = ("YAW", "SHOULDER", "ELBOW", "WRIST")
COL_OFFSET_Y = 1.0          # the collision-mode copy of the robot is placed 1 m away along +y


class Tee(io.TextIOBase):
    def __init__(self, *streams):
        self.streams = streams

    def write(self, s):
        for st in self.streams:
            st.write(s)
        return len(s)

    def flush(self):
        for st in self.streams:
            st.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--samples", type=int, default=400)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--close", help="finger angles at grip=1 as 'Joint1A,Joint2A,Joint1B,Joint2B' (rad); default: "
                                    "gripper_report.json next to the URDF if present, else spec.GRIPPER_FINGER_CLOSE")
    a = ap.parse_args()
    urdf = Path(a.urdf).resolve()
    tag = a.tag or urdf.stem
    out = ROOT / "results" / f"collision_audit_{dt.datetime.now():%Y%m%d_%H%M%S}_{tag}"
    out.mkdir(parents=True)
    log_f = open(out / "log.txt", "w")
    sys.stdout = Tee(sys.__stdout__, log_f)
    sys.stderr = Tee(sys.__stderr__, log_f)

    import genesis as gs
    gs.init(backend=gs.gpu if a.device == "gpu" else gs.cpu, logging_level="warning")
    scene = gs.Scene(sim_options=gs.options.SimOptions(dt=0.002, gravity=(0, 0, 0)), show_viewer=False,
                     renderer=gs.renderers.Rasterizer())
    t0 = time.time()
    arm = scene.add_entity(gs.morphs.URDF(file=str(urdf), fixed=True), vis_mode="visual")
    arm_c = None
    if not a.no_render:
        arm_c = scene.add_entity(gs.morphs.URDF(file=str(urdf), fixed=True, pos=(0, COL_OFFSET_Y, 0)),
                                 vis_mode="collision")
    cams = {}
    if not a.no_render:
        for name, y0 in (("visual", 0.0), ("collision", COL_OFFSET_Y)):
            cams[name, "arm"] = scene.add_camera(res=(640, 480), pos=(0.95, y0 - 0.85, 0.75),
                                                 lookat=(0.2, y0, 0.3), fov=40, GUI=False)
            cams[name, "grip_side"] = scene.add_camera(res=(640, 480), pos=(0.62, y0 - 0.30, 0.58),
                                                       lookat=(0.47, y0, 0.48), fov=40, GUI=False)
            cams[name, "grip_top"] = scene.add_camera(res=(640, 480), pos=(0.50, y0 - 0.02, 0.80),
                                                      lookat=(0.48, y0, 0.48), fov=40, GUI=False)
    scene.build()
    build_s = time.time() - t0

    jmap = {j.name: j for j in arm.joints}
    arm_dofs = [jmap[n].dofs_idx_local[0] for n in ARM_JOINTS]
    close_map = dict(spec.GRIPPER_FINGER_CLOSE)
    grip_report = urdf.parent / "gripper_report.json"
    if a.close:
        close_map = dict(zip(("Joint1A", "Joint2A", "Joint1B", "Joint2B"), map(float, a.close.split(","))))
    elif grip_report.exists():
        rec = json.loads(grip_report.read_text())["recommended_mapping"]
        close_map = dict(zip(rec["joints"], rec["coefficients_at_grip_1"]))
    fingers = [n for n in close_map if n in jmap]
    finger_dofs = [jmap[n].dofs_idx_local[0] for n in fingers]
    close = np.array([close_map[n] for n in fingers])
    geom2link = {g.idx: l.name for l in arm.links for g in l.geoms}

    def set_q(q_arm, grip, ent=arm):
        q = np.zeros(ent.n_dofs)
        q[arm_dofs] = q_arm
        if finger_dofs:
            q[finger_dofs] = grip * close
        ent.set_dofs_position(q)

    def pairs_now():
        res = arm.detect_collision()
        return {tuple(sorted((geom2link.get(int(i), f"g{i}"), geom2link.get(int(j), f"g{j}")))) for i, j in res}

    links = []
    for l in arm.links:
        boxes = [np.asarray(g.get_AABB().cpu() if hasattr(g.get_AABB(), "cpu") else g.get_AABB()).reshape(2, 3)
                 for g in l.geoms]
        links.append({"link": l.name, "n_collision_geoms": l.n_geoms, "n_visual_geoms": l.n_vgeoms,
                      "collision_faces": [int(g.n_faces) for g in l.geoms],
                      "convex": [bool(g.is_convex) for g in l.geoms],
                      "aabb_size_mm": [((b[1] - b[0]) * 1e3).round(1).tolist() for b in boxes]})

    report = {"urdf": str(urdf), "genesis": gs.__version__, "build_s": round(build_s, 2), "links": links,
              "finger_close_at_grip_1": dict(zip(fingers, close.tolist()))}
    set_q(np.zeros(4), 0.0)
    report["contacts_neutral"] = sorted(map(list, pairs_now()))
    sweep = Counter()
    for g in np.linspace(0, 1, 21):
        set_q(np.zeros(4), g)
        for p in pairs_now():
            sweep[p] += 1
    report["contacts_gripper_sweep_home"] = {f"{p[0]}|{p[1]}": c for p, c in sweep.items()}
    rng = np.random.default_rng(0)
    rand = Counter()
    for _ in range(a.samples):
        set_q(rng.uniform(spec.Q_MIN, spec.Q_MAX), rng.uniform(0, 1))
        for p in pairs_now():
            rand[p] += 1
    report["contacts_random_poses"] = {f"{p[0]}|{p[1]}": c for p, c in rand.most_common()}
    report["random_samples"] = a.samples

    if not a.no_render:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        imgs = {}
        for grip in (0.0, 1.0):
            set_q(np.zeros(4), grip)
            set_q(np.zeros(4), grip, ent=arm_c)
            scene.visualizer.update()
            for (mode, view), cam in cams.items():
                if view == "arm" and grip == 1.0:
                    continue
                rgb = cam.render(rgb=True)[0]
                imgs[mode, view, grip] = np.asarray(rgb.cpu() if hasattr(rgb, "cpu") else rgb)
        cols = [("arm", 0.0), ("grip_side", 0.0), ("grip_top", 0.0), ("grip_side", 1.0), ("grip_top", 1.0)]
        fig, axs = plt.subplots(2, len(cols), figsize=(4.2 * len(cols), 7))
        for r, mode in enumerate(("visual", "collision")):
            for c, (view, grip) in enumerate(cols):
                axs[r, c].imshow(imgs[mode, view, grip])
                axs[r, c].set_title(f"{mode} | {view} | grip={grip:.0f}", fontsize=9)
                axs[r, c].axis("off")
        fig.suptitle(f"{tag}: visual geometry (top) vs collision geometry used by Genesis (bottom)", fontsize=11)
        fig.tight_layout()
        fig.savefig(out / "visual_vs_collision.png", dpi=80)

    sys.stdout.flush()
    text = (out / "log.txt").read_text()
    report["warnings"] = {
        "legacy_parser_fallback": "Falling back to legacy URDF parser" in text,
        "dubious_com_links": sorted(set(re.findall(r"Link '([^']+)' has dubious center of mass", text))),
        "neutral_pairs_filtered": re.findall(r"self-collision for the neutral configuration \(qpos0\): ([^.]*)\.", text),
    }
    (out / "report.json").write_text(json.dumps(report, indent=2))
    sys.stdout = sys.__stdout__
    print(json.dumps({k: report[k] for k in ("build_s", "warnings", "contacts_neutral",
                                             "contacts_gripper_sweep_home", "contacts_random_poses")}, indent=1))
    print("->", out)


if __name__ == "__main__":
    main()

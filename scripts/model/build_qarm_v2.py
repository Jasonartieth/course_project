"""Build the QArm-with-gripper asset set v2 (for LuoShaoyinn/mmic-project assets/qarm-community).

Sources: the official Quanser URDFs and meshes in models/qarm_official (BSD-3; byte-identical to the meshes
mmic ships) plus the re-aligned gripper frames from mmic-project commit 2b8ab95.

What the generated model contains:
  * arm: real inertias (official "Dynamic Parameters", via build_qarm_urdf.build_arm), manual joint limits,
    effort = instantaneous torque, velocity = pi/2, `tcp` link 0.162 m beyond END-EFFECTOR
  * gripper: mmic's mount / joint origins / joint axes / visual origins (the official gripper URDF frames leave
    the finger parts floating); finger joint limits from geometry: fully open at 0, closing stops where the two
    fingers' visual meshes first touch; token finger masses with the COM at each part's geometric centre
    (the rotating wrist + gripper mass is in END-EFFECTOR, official body B4)
  * collision: ONLY boxes / cylinders / spheres (qarm_lab.collision_primitives), fitted to the full-resolution
    official meshes so that every visual vertex is inside the collision geometry (larger, never smaller)
  * visual: official meshes decimated to <= 50k faces (models/qarm/meshes/visual) -> Genesis' primary parser
    accepts them (MuJoCo's STL limit is 200k faces) and loading is fast
  * named materials (Genesis' URDF parser rejects <material> without a name)

Usage:  python scripts/model/build_qarm_v2.py [--out models/qarm_v2]
Output: <out>/qarm_with_gripper.urdf, meshes/visual/*.stl, collision_report.json, gripper_report.json
"""

import argparse
import hashlib
import json
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts" / "model"))
import build_qarm_urdf as B  # noqa: E402
from qarm_lab import collision_primitives as CP  # noqa: E402
from qarm_lab import spec  # noqa: E402

PI = np.pi
OFFICIAL = ROOT / "models" / "qarm_official"
VISUAL_SRC = ROOT / "models" / "qarm" / "meshes" / "visual"

# mesh name in the output -> (official STL, decimated visual STL, primitive budget)
MESHES = {
    "base_link": ("qarm/meshes/base_link.STL", "qarm__base_link__vis.stl", 6),
    "YAW": ("qarm/meshes/YAW.STL", "qarm__YAW__vis.stl", 4),
    "BICEP": ("qarm/meshes/BICEP.STL", "qarm__BICEP__vis.stl", 6),
    "FOREARM": ("qarm/meshes/FOREARM.STL", "qarm__FOREARM__vis.stl", 8),
    "END-EFFECTOR": ("qarm/meshes/END-EFFECTOR.STL", "qarm__END-EFFECTOR__vis.stl", 2),
    "gripper_base_link": ("qarm_gripper/meshes/base_link.STL", "qarm_gripper__base_link__vis.stl", 8),
    "LINK1A": ("qarm_gripper/meshes/Link1A.STL", "qarm_gripper__Link1A__vis.stl", 4),
    "LINK1B": ("qarm_gripper/meshes/Link1B.STL", "qarm_gripper__Link1B__vis.stl", 4),
    "LINK2A": ("qarm_gripper/meshes/Link2A.STL", "qarm_gripper__Link2A__vis.stl", 3),
    "LINK2B": ("qarm_gripper/meshes/Link2B.STL", "qarm_gripper__Link2B__vis.stl", 3),
}
ARM_LINK_MESH = {"base_link": "base_link", "YAW": "YAW", "BICEP": "BICEP", "FOREARM": "FOREARM",
                 "END-EFFECTOR": "END-EFFECTOR"}

# ---- gripper frames from mmic-project 2b8ab95 (xyz, rpy); joint axes; visual origins in the link frame
G_MOUNT = ((0, 0, 0), (PI / 2, 0, PI / 2))                    # END-EFFECTOR -> gripper_base_link
G_LINKS = {   # link: (parent, joint, joint_origin, axis, mesh, visual_origin)
    "Link1A": ("gripper_base_link", "Joint1A", ((-0.0305, 0.0515, 0), (0, 0, PI / 2)), (0, 0, 1), "LINK1A",
               ((0.0353, 0, 0), (-PI / 2, PI, 0))),
    "Link2A": ("Link1A", "Joint2A", ((0.07, 0, 0), (0, 0, 0)), (0, 0, 1), "LINK2A",
               ((0.02, 0.01, -0.015), (-PI / 2, PI, 0))),
    "Link1B": ("gripper_base_link", "Joint1B", ((0.0305, 0.0515, 0), (0, 0, PI / 2)), (0, 0, 1), "LINK1B",
               ((0.0353, 0, 0.017), (-PI / 2, 0, PI))),
    "Link2B": ("Link1B", "Joint2B", ((0.07, 0, 0), (0, 0, 0)), (0, 0, -1), "LINK2B",
               ((0.02, -0.01, 0.015), (-PI / 2, 0, PI))),
}
G_BASE_VISUAL = ((0, 0.017, 0), (-PI / 2, 0, 0))
# closing direction per finger joint (sign of the joint value that closes) and the distal/proximal ratio used by
# the reference single-scalar mapping (-0.45, -0.35, +0.45, -0.35): J2 = 0.35/0.45 * J1 in magnitude
CLOSE_SIGN = {"Joint1A": -1.0, "Joint2A": -1.0, "Joint1B": +1.0, "Joint2B": -1.0}
DISTAL_RATIO = 0.35 / 0.45
OPEN_MARGIN = 0.05            # rad allowed beyond the CAD (fully open) pose
TOKEN_MASS = {"gripper_base_link": 0.005, "Link1A": 0.005, "Link1B": 0.005, "Link2A": 0.003, "Link2B": 0.003}
MATERIALS = {"qarm_light_grey": "0.80 0.80 0.80 1", "qarm_dark_grey": "0.25 0.25 0.25 1"}


def H(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    return B.homog(B.rpy_to_R(*rpy), xyz)


def R_to_rpy(R):
    """URDF rpy (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    pitch = np.arcsin(-np.clip(R[2, 0], -1, 1))
    if abs(np.cos(pitch)) > 1e-9:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll, yaw = np.arctan2(-R[1, 2], R[1, 1]), 0.0
    return np.array([roll, pitch, yaw])


def fmt(v):
    return " ".join(f"{x:.9g}" for x in v)


def origin_el(parent, T):
    ET.SubElement(parent, "origin", {"xyz": fmt(T[:3, 3]), "rpy": fmt(R_to_rpy(T[:3, :3]))})


def add_visual(link_el, mesh_name, T_vis, material):
    v = ET.SubElement(link_el, "visual")
    origin_el(v, T_vis)
    g = ET.SubElement(v, "geometry")
    ET.SubElement(g, "mesh", {"filename": f"meshes/visual/{mesh_name}.stl"})
    ET.SubElement(v, "material", {"name": material})


def add_collisions(link_el, prims, T_vis):
    for p in prims:
        q = p.transformed(T_vis)
        c = ET.SubElement(link_el, "collision")
        origin_el(c, q.T)
        g = ET.SubElement(c, "geometry")
        if q.kind == "box":
            ET.SubElement(g, "box", {"size": fmt(q.size)})
        elif q.kind == "cylinder":
            ET.SubElement(g, "cylinder", {"radius": f"{q.size[0]:.9g}", "length": f"{q.size[1]:.9g}"})
        else:
            ET.SubElement(g, "sphere", {"radius": f"{q.size[0]:.9g}"})


def finger_parts(q, pts):
    """World (gripper-frame) points of each finger part for finger joint values q (dict)."""
    out = {}
    for side in "AB":
        F = np.eye(4)
        for seg in ("1", "2"):
            name = f"Link{seg}{side}"
            _, joint, (jxyz, jrpy), axis, mesh, (vxyz, vrpy) = G_LINKS[name]
            F = F @ H(jxyz, jrpy) @ B.homog(B.axis_angle_R(axis, q[joint]))
            M = F @ H(vxyz, vrpy)
            out[name] = pts[mesh] @ M[:3, :3].T + M[:3, 3]
    return out


def closing_q(s):
    return {j: CLOSE_SIGN[j] * s * (1.0 if j.startswith("Joint1") else DISTAL_RATIO) for j in CLOSE_SIGN}


def finger_contact(pts, touch=0.0015):
    """Closing amount s (proximal joint angle) at which the two fingers' visual meshes first come within `touch`."""
    def gap(s):
        p = finger_parts(closing_q(s), pts)
        A = np.vstack([p["Link1A"], p["Link2A"]])
        Bp = np.vstack([p["Link1B"], p["Link2B"]])
        return float(cKDTree(Bp).query(A, k=1)[0].min())
    lo, hi = 0.0, 0.6
    assert gap(lo) > touch and gap(hi) < touch, (gap(lo), gap(hi))
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if gap(mid) > touch else (lo, mid)
    s = lo
    p = finger_parts(closing_q(s), pts)
    tipA = p["Link2A"][np.argmax(p["Link2A"][:, 1])]
    tipB = p["Link2B"][np.argmax(p["Link2B"][:, 1])]
    return s, {"gap_open_mm": round(gap(0.0) * 1e3, 2), "tipA": tipA.round(4).tolist(),
               "tipB": tipB.round(4).tolist(), "tip_mid_y": round(float((tipA[1] + tipB[1]) / 2), 4)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "models" / "qarm_v2"))
    a = ap.parse_args()
    out = Path(a.out)
    (out / "meshes" / "visual").mkdir(parents=True, exist_ok=True)

    meshes_full = {n: trimesh.load_mesh(OFFICIAL / src) for n, (src, _, _) in MESHES.items()}
    for n, (_, vis, _) in MESHES.items():
        shutil.copyfile(VISUAL_SRC / vis, out / "meshes" / "visual" / f"{n}.stl")

    # ---------------- primitive collision geometry (mesh frame), coverage on full-res vertices
    col_report, prims = {}, {}
    for n, (src, _, budget) in MESHES.items():
        fit = CP.fit_mesh(meshes_full[n], max_prims=budget)
        cov, n_out = CP.check_coverage(fit.primitives, meshes_full[n].vertices)
        assert n_out == 0, f"{n}: {n_out} vertices outside the collision primitives"
        prims[n] = fit.primitives
        uv = CP.union_volume_mc(fit.primitives)
        col_report[n] = {"n_primitives": fit.k, "kinds": [p.kind for p in fit.primitives],
                         "union_volume_cm3": round(uv * 1e6, 1), "convex_hull_volume_cm3": round(fit.hull_volume * 1e6, 1),
                         "union_over_hull": round(uv / fit.hull_volume, 3), "vertex_coverage": cov,
                         "official_vertices_checked": int(len(meshes_full[n].vertices))}
        print(f"{n:18s} {fit.k} prims {[p.kind[0] for p in fit.primitives]}  union/hull {uv / fit.hull_volume:.2f}  coverage {cov}")

    # ---------------- arm (real inertias, limits, tcp) from the course_project builder
    robot = B.build_arm({})
    robot.set("name", "QARM")
    for l in robot.findall("link"):
        for tag in ("visual", "collision"):
            for el in l.findall(tag):
                l.remove(el)
        if l.get("name") in ARM_LINK_MESH:
            m = ARM_LINK_MESH[l.get("name")]
            add_visual(l, m, np.eye(4), "qarm_light_grey")
            add_collisions(l, prims[m], np.eye(4))
    for name, rgba in MATERIALS.items():          # robot-level material definitions
        mat = ET.Element("material", {"name": name})
        ET.SubElement(mat, "color", {"rgba": rgba})
        robot.insert(0, mat)

    # ---------------- gripper (mmic frames)
    pts = {n: m.vertices[np.random.default_rng(0).choice(len(m.vertices), min(len(m.vertices), 20000), replace=False)]
           for n, m in meshes_full.items() if n.startswith("LINK")}
    s_contact, grip_info = finger_contact(pts)
    gl = ET.SubElement(robot, "link", {"name": "gripper_base_link"})
    T_gv = H(*G_BASE_VISUAL)
    c_g = T_gv[:3, :3] @ meshes_full["gripper_base_link"].bounding_box.centroid + T_gv[:3, 3]
    B.set_inertial(gl, TOKEN_MASS["gripper_base_link"], c_g, np.eye(3) * 1e-7)
    add_visual(gl, "gripper_base_link", T_gv, "qarm_dark_grey")
    add_collisions(gl, prims["gripper_base_link"], T_gv)
    j = ET.SubElement(robot, "joint", {"name": "gripper_mount", "type": "fixed"})
    origin_el(j, H(*G_MOUNT))
    ET.SubElement(j, "parent", {"link": "END-EFFECTOR"})
    ET.SubElement(j, "child", {"link": "gripper_base_link"})
    limits = {}
    for name, (parent, joint, (jxyz, jrpy), axis, mesh, (vxyz, vrpy)) in G_LINKS.items():
        l = ET.SubElement(robot, "link", {"name": name})
        T_v = H(vxyz, vrpy)
        c = T_v[:3, :3] @ meshes_full[mesh].bounding_box.centroid + T_v[:3, 3]
        B.set_inertial(l, TOKEN_MASS[name], c, np.eye(3) * 1e-7)
        add_visual(l, mesh, T_v, "qarm_dark_grey")
        add_collisions(l, prims[mesh], T_v)
        jj = ET.SubElement(robot, "joint", {"name": joint, "type": "revolute"})
        origin_el(jj, H(jxyz, jrpy))
        ET.SubElement(jj, "parent", {"link": parent})
        ET.SubElement(jj, "child", {"link": name})
        ET.SubElement(jj, "axis", {"xyz": fmt(axis)})
        q_close = closing_q(s_contact)[joint]
        lo, hi = sorted((q_close, -np.sign(q_close) * OPEN_MARGIN))
        limits[joint] = (round(float(lo), 4), round(float(hi), 4))
        ET.SubElement(jj, "limit", {"lower": f"{lo:.4f}", "upper": f"{hi:.4f}",
                                    "effort": f"{spec.GRIP_TAU_INST}", "velocity": "2.0"})

    header = ("GENERATED by course_project scripts/model/build_qarm_v2.py - do not edit by hand.\n"
              "  Meshes and arm kinematics: Quanser QArm URDF representations (BSD-3, see LICENSE).\n"
              "  Arm inertias: Quanser QArm 'Dynamic Parameters'; limits: QArm User Manual Table 7.\n"
              "  Gripper frames: LuoShaoyinn/mmic-project 2b8ab95. Collision: boxes/cylinders/spheres that fully\n"
              "  enclose the full-resolution official meshes (collision is never smaller than the visual mesh).")
    ET.indent(robot, space="  ")
    (out / "qarm_with_gripper.urdf").write_text(
        f'<?xml version="1.0" encoding="utf-8"?>\n<!--\n  {header}\n-->\n' + ET.tostring(robot, encoding="unicode") + "\n")

    grip_report = {"closing_scalar_rad_at_contact": round(s_contact, 4), "finger_limits": limits,
                   "recommended_mapping": {"joints": ["Joint1A", "Joint2A", "Joint1B", "Joint2B"],
                                           "coefficients_at_grip_1": [round(closing_q(s_contact)[k], 4) for k in
                                                                      ("Joint1A", "Joint2A", "Joint1B", "Joint2B")]},
                   **grip_info}
    (out / "collision_report.json").write_text(json.dumps(col_report, indent=2))
    (out / "gripper_report.json").write_text(json.dumps(grip_report, indent=2))
    sha = {str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest()
           for p in sorted(out.rglob("*")) if p.is_file() and p.suffix in (".stl", ".urdf")}
    (out / "files_sha256.json").write_text(json.dumps(sha, indent=2))
    print(json.dumps(grip_report, indent=1))
    print("->", out)


if __name__ == "__main__":
    main()

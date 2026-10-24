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
    official meshes so that every vertex is inside the collision geometry (larger, never smaller). Budgets from
    qarm_lab.collision_simplify.PROFILES (default "coarse-arm": coarse arm, fine gripper), then primitives buried
    in their own body or in an adjacent body over the whole joint range are pruned (Genesis ignores parent-child
    collisions). The written URDF is re-checked: every full-resolution vertex of each link lies inside its own
    body's primitives or, over the whole joint range, inside an adjacent body's; the same check on the decimated
    visual meshes is reported in collision_report.json
  * visual: official meshes decimated to <= 50k faces (models/qarm/meshes/visual) -> Genesis' primary parser
    accepts them (MuJoCo's STL limit is 200k faces) and loading is fast
  * named materials (Genesis' URDF parser rejects <material> without a name)

Usage:  python scripts/model/build_qarm_v2.py [--out models/qarm_v2] [--profile coarse-arm|uniform]
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
from qarm_lab.collision_simplify import (DEFAULT_BUDGET, PROFILES, add_collision_element,  # noqa: E402
                                         parse_structure, prune, verify_written)

PI = np.pi
OFFICIAL = ROOT / "models" / "qarm_official"
VISUAL_SRC = ROOT / "models" / "qarm" / "meshes" / "visual"

# mesh name in the output -> (official STL, decimated visual STL). Primitive budgets: PROFILES[--profile][name.lower()]
MESHES = {
    "base_link": ("qarm/meshes/base_link.STL", "qarm__base_link__vis.stl"),
    "YAW": ("qarm/meshes/YAW.STL", "qarm__YAW__vis.stl"),
    "BICEP": ("qarm/meshes/BICEP.STL", "qarm__BICEP__vis.stl"),
    "FOREARM": ("qarm/meshes/FOREARM.STL", "qarm__FOREARM__vis.stl"),
    "END-EFFECTOR": ("qarm/meshes/END-EFFECTOR.STL", "qarm__END-EFFECTOR__vis.stl"),
    "gripper_base_link": ("qarm_gripper/meshes/base_link.STL", "qarm_gripper__base_link__vis.stl"),
    "LINK1A": ("qarm_gripper/meshes/Link1A.STL", "qarm_gripper__Link1A__vis.stl"),
    "LINK1B": ("qarm_gripper/meshes/Link1B.STL", "qarm_gripper__Link1B__vis.stl"),
    "LINK2A": ("qarm_gripper/meshes/Link2A.STL", "qarm_gripper__Link2A__vis.stl"),
    "LINK2B": ("qarm_gripper/meshes/Link2B.STL", "qarm_gripper__Link2B__vis.stl"),
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
    ap.add_argument("--profile", default="coarse-arm", choices=sorted(PROFILES))
    a = ap.parse_args()
    out = Path(a.out)
    (out / "meshes" / "visual").mkdir(parents=True, exist_ok=True)

    meshes_full = {n: trimesh.load_mesh(OFFICIAL / src) for n, (src, _) in MESHES.items()}
    for n, (_, vis) in MESHES.items():
        shutil.copyfile(VISUAL_SRC / vis, out / "meshes" / "visual" / f"{n}.stl")

    # ---------------- primitive collision geometry (mesh frame), coverage on full-res vertices
    fit_report, prims = {}, {}
    for n in MESHES:
        k, gain = PROFILES[a.profile].get(n.lower(), DEFAULT_BUDGET)
        fit = CP.fit_mesh(meshes_full[n], max_prims=k, min_gain=gain)
        cov, n_out = CP.check_coverage(fit.primitives, meshes_full[n].vertices)
        assert n_out == 0, f"{n}: {n_out} vertices outside the collision primitives"
        prims[n] = fit.primitives
        uv = CP.union_volume_mc(fit.primitives)
        fit_report[n] = {"budget": [k, gain], "fitted": fit.k, "kinds": [p.kind for p in fit.primitives],
                         "union_volume_cm3": round(uv * 1e6, 1), "convex_hull_volume_cm3": round(fit.hull_volume * 1e6, 1),
                         "union_over_hull": round(uv / fit.hull_volume, 3), "vertex_coverage": cov,
                         "official_vertices_checked": int(len(meshes_full[n].vertices))}
        print(f"{n:18s} budget {(k, gain)} -> {fit.k} {''.join(p.kind[0] for p in fit.primitives)}  "
              f"union/hull {uv / fit.hull_volume:.2f}  coverage {cov}")
    link_prims = {}          # link -> primitives in the link frame (collision elements are written after pruning)

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
            link_prims[l.get("name")] = list(prims[m])
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
    link_prims["gripper_base_link"] = [p.transformed(T_gv) for p in prims["gripper_base_link"]]
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
        link_prims[name] = [p.transformed(T_v) for p in prims[mesh]]
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

    # ---------------- prune buried primitives (whole kinematic tree known now), then write the collisions
    body_of, T_body, movable = parse_structure(robot)
    link_prims, pruned = prune(link_prims, body_of, T_body, movable)
    links = {l.get("name"): l for l in robot.findall("link")}
    for name, ps in link_prims.items():
        for p in ps:
            add_collision_element(links[name], p)

    header = ("GENERATED by course_project scripts/model/build_qarm_v2.py - do not edit by hand.\n"
              "  Meshes and arm kinematics: Quanser QArm URDF representations (BSD-3, see LICENSE).\n"
              "  Arm inertias: Quanser QArm 'Dynamic Parameters'; limits: QArm User Manual Table 7.\n"
              "  Gripper frames: LuoShaoyinn/mmic-project 2b8ab95. Collision: boxes/cylinders/spheres enclosing the\n"
              f"  full-resolution official meshes, profile {a.profile}, buried primitives pruned (collision is never\n"
              "  smaller than the visual mesh: every vertex is inside its own or, over the whole joint range, an\n"
              "  adjacent link's primitives).")
    ET.indent(robot, space="  ")
    urdf_text = (f'<?xml version="1.0" encoding="utf-8"?>\n<!--\n  {header}\n-->\n'
                 + ET.tostring(robot, encoding="unicode") + "\n")

    # ---------------- round-trip check of the written collisions (full-res originals must be 100 % covered)
    full_res = {f"meshes/visual/{n}.stl": OFFICIAL / src for n, (src, _) in MESHES.items()}
    check_full = verify_written(urdf_text, out, mesh_override=full_res)
    check_vis = verify_written(urdf_text, out)
    for l, s in check_full.items():
        print(f"check {l:18s} own prims {s['own_primitives']:2d}  full-res coverage incl. adjacent "
              f"{s['coverage_incl_adjacent']:.6f}  | decimated visual {check_vis[l]['coverage_incl_adjacent']:.6f}")
    bad = {l: s["outside"] for l, s in check_full.items() if s["outside"]}
    assert not bad, f"full-resolution vertices outside the written collision primitives: {bad}"
    (out / "qarm_with_gripper.urdf").write_text(urdf_text)
    kinds = {}
    for ps in link_prims.values():
        for p in ps:
            kinds[p.kind] = kinds.get(p.kind, 0) + 1
    col_report = {"profile": a.profile, "total_primitives": sum(kinds.values()), "kinds": kinds,
                  "per_link": {l: [p.kind for p in ps] for l, ps in link_prims.items()}, "pruned": pruned,
                  "fit_per_mesh": fit_report, "written_check_full_resolution": check_full,
                  "written_check_decimated_visual": check_vis}

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

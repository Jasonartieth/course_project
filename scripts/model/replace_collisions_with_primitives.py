"""Replace every mesh <collision> of a URDF with boxes / cylinders / spheres written directly in the URDF.

Only the <collision> blocks change; the edit is done on the text, so visuals, inertials, joints, materials,
comments and formatting stay byte-identical (checked at the end). No mesh file is created or modified.

For each <collision> block referencing a mesh:
  * primitives are fitted to the full-resolution mesh with qarm_lab.collision_primitives (they enclose every
    vertex, so the collision is never smaller than the mesh), then placed with the block's <origin>
  * if the link's <visual> uses the same mesh and origin, enclosing the collision mesh = enclosing the visual;
    links where they differ are reported (pass --require-same-visual to fail on them)
  * the block is replaced by one <collision> per primitive, preceded by a one-line comment

Usage:  python scripts/model/replace_collisions_with_primitives.py <in.urdf> <out.urdf>
            [--mesh-root DIR]  (default: the input URDF's directory)  [--require-same-visual]
Writes <out.urdf>, <out>.report.json and <out>.diff next to it.
"""

import argparse
import difflib
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts" / "model"))
from build_qarm_v2 import R_to_rpy  # noqa: E402
from build_qarm_urdf import homog, rpy_to_R  # noqa: E402
from qarm_lab import collision_primitives as CP  # noqa: E402

# primitive budget per mesh (case-insensitive file stem); same budgets as build_qarm_v2
BUDGET = {"base_link": 6, "yaw": 4, "bicep": 6, "forearm": 8, "end-effector": 2, "gripper_base_link": 8,
          "link1a": 4, "link1b": 4, "link2a": 3, "link2b": 3}
DEFAULT_BUDGET = 6
COLLISION_RE = re.compile(r"(?P<indent>[ \t]*)<collision\b[^>]*>.*?</collision>[ \t]*\n", re.S)
LINK_RE = re.compile(r'<link\s+name="([^"]+)"')
GENERATED_TAG = "collision primitive"


def g(v):
    return " ".join(f"{x:.7g}" for x in v)


def origin_T(el):
    o = el.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    return homog(rpy_to_R(*rpy), xyz), (tuple(xyz), tuple(rpy))


def primitive_block(p, indent):
    rpy = R_to_rpy(p.T[:3, :3])
    if p.kind == "box":
        geo = f'<box size="{g(p.size)}"/>'
    elif p.kind == "cylinder":
        geo = f'<cylinder radius="{p.size[0]:.7g}" length="{p.size[1]:.7g}"/>'
    else:
        geo = f'<sphere radius="{p.size[0]:.7g}"/>'
    i2, i4 = indent + "  ", indent + "    "
    return (f"{indent}<collision>\n{i2}<origin xyz=\"{g(p.T[:3, 3])}\" rpy=\"{g(rpy)}\"/>\n"
            f"{i2}<geometry>\n{i4}{geo}\n{i2}</geometry>\n{indent}</collision>\n")


def strip_collisions(text):
    """Text with every <collision> block and the generated comment lines removed (for the 'nothing else
    changed' check)."""
    text = COLLISION_RE.sub("", text)
    return "".join(l for l in text.splitlines(keepends=True) if GENERATED_TAG not in l)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf_in")
    ap.add_argument("urdf_out")
    ap.add_argument("--mesh-root")
    ap.add_argument("--require-same-visual", action="store_true")
    a = ap.parse_args()
    src = Path(a.urdf_in)
    mesh_root = Path(a.mesh_root) if a.mesh_root else src.parent
    text = src.read_text()
    robot = ET.fromstring(text)
    visuals = {l.get("name"): [(origin_T(v)[1], v.find("geometry/mesh").get("filename"))
                               for v in l.findall("visual") if v.find("geometry/mesh") is not None]
               for l in robot.findall("link")}
    report, fits = {}, {}
    out, pos = [], 0
    for m in COLLISION_RE.finditer(text):
        block = ET.fromstring(m.group(0).strip())
        mesh_el = block.find("geometry/mesh")
        link = LINK_RE.findall(text[:m.start()])[-1]
        out.append(text[pos:m.start()])
        pos = m.end()
        if mesh_el is None:                       # already a primitive: keep as is
            out.append(m.group(0))
            continue
        fn = mesh_el.get("filename")
        if mesh_el.get("scale") not in (None, "1 1 1"):
            raise SystemExit(f"{link}: scaled collision meshes are not supported ({mesh_el.get('scale')})")
        T, key = origin_T(block)
        same_visual = (key, fn) in visuals.get(link, [])
        if a.require_same_visual and not same_visual:
            raise SystemExit(f"{link}: collision mesh/origin differ from the visual; refusing (--require-same-visual)")
        if fn not in fits:
            mesh = trimesh.load_mesh(mesh_root / fn)
            fit = CP.fit_mesh(mesh, max_prims=BUDGET.get(Path(fn).stem.lower(), DEFAULT_BUDGET))
            cov, n_out = CP.check_coverage(fit.primitives, mesh.vertices)
            assert n_out == 0, f"{fn}: {n_out} vertices outside the primitives"
            fits[fn] = (fit, cov, len(mesh.vertices), CP.union_volume_mc(fit.primitives))
        fit, cov, nv, uv = fits[fn]
        prims = [p.transformed(T) for p in fit.primitives]
        indent = m.group("indent")
        out.append(f"{indent}<!-- {GENERATED_TAG}s ({len(prims)}) enclosing every vertex of {fn}; "
                   f"generated by course_project scripts/model/replace_collisions_with_primitives.py -->\n")
        out += [primitive_block(p, indent) for p in prims]
        report[link] = {"mesh": fn, "n_primitives": len(prims), "kinds": [p.kind for p in prims],
                        "vertex_coverage": cov, "vertices_checked": nv, "same_as_visual": same_visual,
                        "union_volume_cm3": round(uv * 1e6, 1),
                        "convex_hull_volume_cm3": round(fit.hull_volume * 1e6, 1),
                        "union_over_hull": round(uv / fit.hull_volume, 3)}
        print(f"{link:18s} {fn:28s} -> {len(prims)} {''.join(p.kind[0] for p in prims)}  coverage {cov}  "
              f"same-as-visual {same_visual}  union/hull {uv / fit.hull_volume:.2f}")
    out.append(text[pos:])
    new = "".join(out)
    ET.fromstring(new)                                         # still well-formed
    assert strip_collisions(new) == strip_collisions(text), "something outside <collision> changed"
    dst = Path(a.urdf_out)
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(new)
    Path(str(dst) + ".report.json").write_text(json.dumps(report, indent=2))
    diff = difflib.unified_diff(text.splitlines(keepends=True), new.splitlines(keepends=True),
                                fromfile=str(src), tofile=str(dst))
    Path(str(dst) + ".diff").write_text("".join(diff))
    n = sum(r["n_primitives"] for r in report.values())
    print(f"{len(report)} collision meshes -> {n} primitives; only <collision> blocks changed -> {dst}")


if __name__ == "__main__":
    main()

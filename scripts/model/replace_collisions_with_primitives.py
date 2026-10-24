"""Replace every mesh <collision> of a URDF with boxes / cylinders / spheres written directly in the URDF.

Only the <collision> blocks change; the edit is done on the text, so visuals, inertials, joints, materials,
comments and formatting stay byte-identical (checked at the end). No mesh file is created or modified.

Steps
  1. Fit: each <collision> mesh gets primitives that enclose every vertex (qarm_lab.collision_primitives),
     placed with the block's <origin>. Budgets come from a profile:
       coarse-arm (default)  arm links coarse (base_link, YAW, END-EFFECTOR: <= 2 primitives, split only if it
                             saves >= 25 % volume; BICEP <= 5 and FOREARM <= 3, the fewest that avoid false
                             self-contacts); gripper (gripper_base_link, LINK1A/1B/2A/2B): fine
       uniform               the earlier per-mesh budgets (8 for the forearm, ...)
  2. Prune buried primitives (unless --no-prune). Bodies follow Genesis: links joined by fixed joints form one
     body, and collisions between parent-child bodies are ignored (enable_adjacent_collision=False). A primitive
     is removed if every point of its surface (sampled every 3 mm) is covered, point by point, by
       - another primitive of its own body, or
       - an adjacent body's primitives at every joint angle in the joint's limits (sampled every 2 deg).
     If that holds only after growing the covering primitives by --bury-tol (default 1 mm), exactly the covering
     primitives that touch the uncovered points are grown by that much and the primitive is removed (collision
     only ever grows, so it still encloses every visual). One primitive at a time (smallest first),
     re-evaluated after each removal.
  3. Write the replacement <collision> blocks (one per primitive, after a one-line comment).
  4. Verify the WRITTEN file: primitives are rebuilt from <origin rpy>/<geometry> as a URDF parser would; every
     vertex of each link's visual mesh must lie inside its own body's primitives or, for every sampled joint
     angle, inside an adjacent body's primitives. Also re-checks that nothing outside <collision> changed.

Usage:  python scripts/model/replace_collisions_with_primitives.py <in.urdf> <out.urdf>
            [--mesh-root DIR] [--profile coarse-arm|uniform] [--no-prune] [--bury-tol 0.001] [--require-same-visual]
Writes <out.urdf>, <out>.report.json and <out>.diff.
The fitting / pruning / verification code lives in src/qarm_lab/collision_simplify.py (shared with
scripts/model/build_qarm_v2.py).
"""

import argparse
import difflib
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import trimesh

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from qarm_lab import collision_primitives as CP  # noqa: E402
from qarm_lab.collision_simplify import (DEFAULT_BUDGET, PROFILES, burial_table, g, parse_structure,  # noqa: E402
                                         prune, verify_written)
from qarm_lab.transforms import R_to_rpy, origin_T  # noqa: E402

COLLISION_RE = re.compile(r"(?P<indent>[ \t]*)<collision\b[^>]*>.*?</collision>[ \t]*\n", re.S)
LINK_RE = re.compile(r'<link\s+name="([^"]+)"')
GENERATED_TAG = "collision primitive"


# ------------------------------------------------------------------------------------------- writing
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
    """Text with every <collision> block and the generated comment lines removed."""
    text = COLLISION_RE.sub("", text)
    return "".join(l for l in text.splitlines(keepends=True) if GENERATED_TAG not in l)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf_in")
    ap.add_argument("urdf_out")
    ap.add_argument("--mesh-root")
    ap.add_argument("--profile", default="coarse-arm", choices=sorted(PROFILES))
    ap.add_argument("--no-prune", action="store_true")
    ap.add_argument("--bury-tol", type=float, default=0.001, help="m; see step 2 (0 = exact burial only)")
    ap.add_argument("--budget", action="append", default=[], metavar="STEM=K,GAIN",
                    help="override one mesh's budget, e.g. --budget forearm=4,0.1 (repeatable)")
    ap.add_argument("--require-same-visual", action="store_true")
    a = ap.parse_args()
    src = Path(a.urdf_in)
    mesh_root = Path(a.mesh_root) if a.mesh_root else src.parent
    text = src.read_text()
    robot = ET.fromstring(text)
    body_of, T_body, movable = parse_structure(robot)
    visuals = {l.get("name"): [(origin_T(v)[1], v.find("geometry/mesh").get("filename"))
                               for v in l.findall("visual") if v.find("geometry/mesh") is not None]
               for l in robot.findall("link")}
    budgets = dict(PROFILES[a.profile])
    for spec_ in a.budget:
        stem, kg = spec_.split("=")
        k, gain = kg.split(",")
        budgets[stem.lower()] = (int(k), float(gain))

    # 1) fit
    matches = list(COLLISION_RE.finditer(text))
    link_prims, fit_info, cache = {}, {}, {}
    for m in matches:
        block = ET.fromstring(m.group(0).strip())
        mesh_el = block.find("geometry/mesh")
        if mesh_el is None:
            continue
        link = LINK_RE.findall(text[:m.start()])[-1]
        fn = mesh_el.get("filename")
        if mesh_el.get("scale") not in (None, "1 1 1"):
            raise SystemExit(f"{link}: scaled collision meshes are not supported")
        T, key = origin_T(block)
        if a.require_same_visual and (key, fn) not in visuals.get(link, []):
            raise SystemExit(f"{link}: collision mesh/origin differ from the visual (--require-same-visual)")
        if fn not in cache:
            mesh = trimesh.load_mesh(mesh_root / fn)
            k, gain = budgets.get(Path(fn).stem.lower(), DEFAULT_BUDGET)
            fit = CP.fit_mesh(mesh, max_prims=k, min_gain=gain)
            cov, n_out = CP.check_coverage(fit.primitives, mesh.vertices)
            assert n_out == 0, f"{fn}: {n_out} vertices outside the primitives"
            cache[fn] = (fit, len(mesh.vertices), CP.union_volume_mc(fit.primitives), (k, gain))
        fit, nv, uv, budget = cache[fn]
        link_prims.setdefault(link, []).extend(p.transformed(T) for p in fit.primitives)
        fit_info[link] = {"mesh": fn, "budget": budget, "fitted": fit.k, "vertices": nv,
                          "union_over_hull": round(uv / fit.hull_volume, 3)}
        print(f"fit   {link:18s} {fn:28s} budget {budget} -> {fit.k} {''.join(p.kind[0] for p in fit.primitives)}"
              f"  union/hull {uv / fit.hull_volume:.2f}")

    # 2) prune (burial diagnostics first: fraction of surface inside same body / adjacent bodies)
    burial = burial_table(link_prims, body_of, T_body, movable)
    for r in burial:
        print(f"  burial {r['link']:18s} {r['kind']:8s} {r['volume_cm3']:8.2f} cm3  same-body {r['same_body']:.3f}  "
              f"pointwise {r['pointwise']}  adjacent {r['adjacent_min_over_q']}")
    removed = []
    if not a.no_prune:
        link_prims, removed = prune(link_prims, body_of, T_body, movable, tol=a.bury_tol)

    # 3) write
    out, pos, done = [], 0, set()
    for m in matches:
        block = ET.fromstring(m.group(0).strip())
        link = LINK_RE.findall(text[:m.start()])[-1]
        out.append(text[pos:m.start()])
        pos = m.end()
        if block.find("geometry/mesh") is None:
            out.append(m.group(0))
            continue
        if link in done:                        # several mesh collisions in one link: write once
            continue
        done.add(link)
        prims, indent = link_prims.get(link, []), m.group("indent")
        fn = fit_info[link]["mesh"]
        note = (f"{len(prims)} {GENERATED_TAG}(s) for {fn}" if prims else
                f"no {GENERATED_TAG}: {fn} is buried in adjacent / same-body primitives")
        # no "--" inside XML comments
        out.append(f"{indent}<!-- {note}; generated by course_project scripts/model/"
                   f"replace_collisions_with_primitives.py, profile={a.profile} -->\n")
        out += [primitive_block(p, indent) for p in prims]
    out.append(text[pos:])
    new = "".join(out)
    ET.fromstring(new)
    assert strip_collisions(new) == strip_collisions(text), "something outside <collision> changed"

    # 4) verify the written file
    written = verify_written(new, mesh_root)
    for l, s in written.items():
        print(f"check {l:18s} own prims {s['own_primitives']:2d}  own coverage {s['own_coverage']:.4f}  "
              f"incl. adjacent {s['coverage_incl_adjacent']:.6f}  ({s['outside']} of {s['vertices_checked']} outside)")
    bad = {l: s["outside"] for l, s in written.items() if s["outside"]}
    assert not bad, f"visual vertices not covered: {bad}"

    dst = Path(a.urdf_out)
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(new)
    total = sum(len(v) for v in link_prims.values())
    kinds = {}
    for v in link_prims.values():
        for p in v:
            kinds[p.kind] = kinds.get(p.kind, 0) + 1
    report = {"profile": a.profile, "pruned": removed, "burial_before_pruning": burial, "fit": fit_info,
              "written_check": written,
              "total_primitives": total, "kinds": kinds,
              "per_link": {l: [p.kind for p in v] for l, v in link_prims.items()}}
    Path(str(dst) + ".report.json").write_text(json.dumps(report, indent=2))
    diff = difflib.unified_diff(text.splitlines(keepends=True), new.splitlines(keepends=True),
                                fromfile=str(src), tofile=str(dst))
    Path(str(dst) + ".diff").write_text("".join(diff))
    print(f"{total} primitives {kinds} after pruning {len(removed)}; only <collision> blocks changed -> {dst}")


if __name__ == "__main__":
    main()

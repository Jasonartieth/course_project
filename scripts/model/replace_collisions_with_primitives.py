"""Replace every mesh <collision> of a URDF with boxes / cylinders / spheres written directly in the URDF.

Only the <collision> blocks change; the edit is done on the text, so visuals, inertials, joints, materials,
comments and formatting stay byte-identical (checked at the end). No mesh file is created or modified.

Steps
  1. Fit: each <collision> mesh gets primitives that enclose every vertex (qarm_lab.collision_primitives),
     placed with the block's <origin>. Budgets come from a profile:
       coarse-arm (default)  arm links (base_link, YAW, BICEP, FOREARM, END-EFFECTOR): <= 2 primitives, split only
                             if it saves >= 25 % volume; gripper (gripper_base_link, LINK1A/1B/2A/2B): fine
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
from build_qarm_urdf import axis_angle_R, homog, rpy_to_R  # noqa: E402
from qarm_lab import collision_primitives as CP  # noqa: E402

ARM = {"base_link", "yaw", "bicep", "forearm", "end-effector"}
PROFILES = {   # mesh stem (lower case) -> (max_prims, min_gain)
    "coarse-arm": {**{s: (2, 0.25) for s in ARM}, "gripper_base_link": (10, 0.04),
                   "link1a": (5, 0.04), "link1b": (5, 0.04), "link2a": (4, 0.04), "link2b": (4, 0.04)},
    "uniform": {"base_link": (6, 0.08), "yaw": (4, 0.08), "bicep": (6, 0.08), "forearm": (8, 0.08),
                "end-effector": (2, 0.08), "gripper_base_link": (8, 0.08), "link1a": (4, 0.08),
                "link1b": (4, 0.08), "link2a": (3, 0.08), "link2b": (3, 0.08)},
}
DEFAULT_BUDGET = (6, 0.08)
Q_STEP = np.deg2rad(2.0)
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


# ------------------------------------------------------------------------------------------- kinematics
def parse_structure(robot):
    """Genesis-like bodies: body_of[link], T_body[link] (body root -> link, fixed), and the list of movable
    joints between bodies as (parent_body, child_body, T_parentroot_jointframe, axis, q_samples)."""
    joints = []
    for j in robot.findall("joint"):
        T, _ = origin_T(j)
        ax = j.find("axis")
        lim = j.find("limit")
        joints.append({"name": j.get("name"), "type": j.get("type"), "parent": j.find("parent").get("link"),
                       "child": j.find("child").get("link"), "T": T,
                       "axis": [float(v) for v in ax.get("xyz").split()] if ax is not None else [1, 0, 0],
                       "lo": float(lim.get("lower", -np.pi)) if lim is not None else -np.pi,
                       "hi": float(lim.get("upper", np.pi)) if lim is not None else np.pi})
    children = {j["child"] for j in joints}
    links = [l.get("name") for l in robot.findall("link")]
    body_of, T_body = {}, {}
    for r in (l for l in links if l not in children):
        body_of[r], T_body[r] = r, np.eye(4)
    pending = list(joints)
    while pending:
        rest = []
        for j in pending:
            if j["parent"] not in body_of:
                rest.append(j)
                continue
            if j["type"] == "fixed":
                body_of[j["child"]] = body_of[j["parent"]]
                T_body[j["child"]] = T_body[j["parent"]] @ j["T"]
            else:
                body_of[j["child"]], T_body[j["child"]] = j["child"], np.eye(4)
        if len(rest) == len(pending):
            raise SystemExit("joint tree is not connected")
        pending = rest
    movable = []
    for j in joints:
        if j["type"] == "fixed":
            continue
        if j["type"] == "continuous":
            qs = np.arange(-np.pi, np.pi, Q_STEP)
        else:
            qs = np.unique(np.r_[np.arange(j["lo"], j["hi"], Q_STEP), j["hi"]])
        movable.append((body_of[j["parent"]], body_of[j["child"]], T_body[j["parent"]] @ j["T"], j["axis"], qs))
    return body_of, T_body, movable


def adjacent_transforms(body, movable):
    """For `body`, yield (other_body, [T_other_body for each joint sample]) for every adjacent body."""
    for pb, cb, T_pj, axis, qs in movable:
        if body == cb:          # body is the child: child-root -> parent-root = T_pj @ R(q)
            yield pb, [T_pj @ homog(axis_angle_R(axis, q)) for q in qs]
        elif body == pb:        # body is the parent: parent-root -> child-root = (T_pj @ R(q))^-1
            yield cb, [np.linalg.inv(T_pj @ homog(axis_angle_R(axis, q))) for q in qs]


def inside_union(prims, pts):
    ok = np.zeros(len(pts), bool)
    for p in prims:
        ok |= p.contains(pts, tol=1e-7)
    return ok


def inflated(p, d):
    """Primitive grown by d on every side (same pose)."""
    if p.kind == "box":
        return CP.Primitive("box", p.T, p.size + 2 * d)
    if p.kind == "cylinder":
        return CP.Primitive("cylinder", p.T, p.size + np.array([d, 2 * d]))
    return CP.Primitive("sphere", p.T, p.size + d)


def pointwise_cover(pts, others, adj_covers):
    """Boolean per point: inside `others` (same body) or, for some adjacent body, inside it at every sample."""
    covered = inside_union(others, pts) if others else np.zeros(len(pts), bool)
    for cover, Ts in adj_covers:
        rest = ~covered
        if not rest.any() or not cover:
            continue
        ok = np.ones(rest.sum(), bool)
        for T in Ts:
            ok &= inside_union(cover, pts[rest] @ T[:3, :3].T + T[:3, 3])
            if not ok.any():
                break
        covered[np.flatnonzero(rest)[ok]] = True
    return covered


def burial_table(link_prims, body_of, T_body, movable):
    """Diagnostics per primitive: fraction of its surface samples covered by (a) the other primitives of its body,
    (b) each adjacent body (minimum over the joint samples), (c) point-wise (a)+(b), and (c) with every covering
    primitive inflated by 1 mm / 2 mm. 1.0 = buried."""
    body_prims = {}
    for l, ps in link_prims.items():
        for p in ps:
            body_prims.setdefault(body_of[l], []).append((l, p, p.transformed(T_body[l])))
    rows = []
    for l, ps in link_prims.items():
        b = body_of[l]
        adjs = list(adjacent_transforms(b, movable))
        for p in ps:
            pts = CP.surface_points(p.transformed(T_body[l]))
            others = [q for (_, p2, q) in body_prims[b] if p2 is not p]
            same = float(inside_union(others, pts).mean()) if others else 0.0
            adj = {}
            for ob, Ts in adjs:
                cover = [q for (_, _, q) in body_prims.get(ob, [])]
                adj[ob] = round(min(float(inside_union(cover, pts @ T[:3, :3].T + T[:3, 3]).mean()) for T in Ts), 3) if cover else 0.0
            comb = {}
            for d in (0.0, 0.001, 0.002):
                oth = [inflated(q, d) for q in others]
                ac = [([inflated(q, d) for (_, _, q) in body_prims.get(ob, [])], Ts) for ob, Ts in adjs]
                comb[f"{d * 1e3:.0f}mm"] = round(float(pointwise_cover(pts, oth, ac).mean()), 4)
            rows.append({"link": l, "kind": p.kind, "volume_cm3": round(p.volume * 1e6, 2), "same_body": round(same, 3),
                         "adjacent_min_over_q": adj, "pointwise": comb})
    return rows


def prune(link_prims, body_of, T_body, movable, tol=0.001):
    """Remove buried primitives (see module docstring). link_prims: {link: [Primitive in link frame]}.

    A primitive P is buried if its surface is covered point-wise (same-body primitives, or an adjacent body at
    every joint sample). If that only holds after growing the covering primitives by `tol`, exactly the covering
    primitives that touch P's uncovered points are grown by `tol` and P is removed: collision geometry only ever
    grows, so it still encloses every visual. Returns (new link_prims, removal records)."""
    items = [{"link": l, "p": p} for l, ps in link_prims.items() for p in ps]
    removed = []

    def body_view():
        view = {}
        for i, it in enumerate(items):
            view.setdefault(body_of[it["link"]], []).append((i, it["p"].transformed(T_body[it["link"]])))
        return view

    def coverage(i_p, view, d):
        """(covered mask, pts, list of (item index, T-list or None) usable as covers) with covers grown by d."""
        it = items[i_p]
        b = body_of[it["link"]]
        pts = CP.surface_points(it["p"].transformed(T_body[it["link"]]))
        same = [(j, q) for (j, q) in view[b] if j != i_p]
        adj = [([(j, q) for (j, q) in view.get(ob, [])], Ts) for ob, Ts in adjacent_transforms(b, movable)]
        covered = pointwise_cover(pts, [inflated(q, d) for _, q in same],
                                  [([inflated(q, d) for _, q in c], Ts) for c, Ts in adj])
        return covered, pts, same, adj

    while True:
        view = body_view()
        victim = None
        for i_p in sorted(range(len(items)), key=lambda i: items[i]["p"].volume):
            covered, pts, same, adj = coverage(i_p, view, 0.0)
            if covered.all():
                victim = (i_p, [], "buried")
                break
            if tol > 0 and coverage(i_p, view, tol)[0].all():
                miss = pts[~covered]
                grow = {j for j, q in same if inflated(q, tol).contains(miss).any()}
                for c, Ts in adj:
                    for j, q in c:
                        qi = inflated(q, tol)
                        if any(qi.contains(miss @ T[:3, :3].T + T[:3, 3]).any() for T in Ts):
                            grow.add(j)
                victim = (i_p, sorted(grow), f"buried after growing {len(grow)} covering primitive(s) by {tol * 1e3:.0f} mm")
                break
        if victim is None:
            break
        i_p, grow, why = victim
        for j in grow:
            items[j]["p"] = inflated(items[j]["p"], tol)
        if grow:                                      # confirm with the grown covers before removing
            assert coverage(i_p, body_view(), 0.0)[0].all(), "tolerance pruning failed to cover the primitive"
        it = items.pop(i_p)
        removed.append({"link": it["link"], "kind": it["p"].kind, "volume_cm3": round(it["p"].volume * 1e6, 2),
                        "reason": why, "grown": [items[j - (j > i_p)]["link"] for j in grow]})
        print(f"  pruned {it['link']:18s} {it['p'].kind:8s} {it['p'].volume * 1e6:8.2f} cm3  ({why}; grown: "
              f"{removed[-1]['grown']})")
    out = {l: [] for l in link_prims}
    for it in items:
        out[it["link"]].append(it["p"])
    return out, removed


# ------------------------------------------------------------------------------------------- writing / checks
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


def prims_from_urdf(robot):
    out = {}
    for l in robot.findall("link"):
        ps = []
        for c in l.findall("collision"):
            geo = c.find("geometry")[0]
            T, _ = origin_T(c)
            if geo.tag == "box":
                ps.append(CP.Primitive("box", T, np.array([float(v) for v in geo.get("size").split()])))
            elif geo.tag == "cylinder":
                ps.append(CP.Primitive("cylinder", T, np.array([float(geo.get("radius")), float(geo.get("length"))])))
            elif geo.tag == "sphere":
                ps.append(CP.Primitive("sphere", T, np.array([float(geo.get("radius"))])))
        out[l.get("name")] = ps
    return out


def verify_written(urdf_text, mesh_root, n_max=30000):
    """Round-trip check of the written file (see module docstring, step 4). Returns {link: stats}."""
    robot = ET.fromstring(urdf_text)
    body_of, T_body, movable = parse_structure(robot)
    lp = prims_from_urdf(robot)
    body_prims = {}
    for l, ps in lp.items():
        body_prims.setdefault(body_of[l], []).extend(p.transformed(T_body[l]) for p in ps)
    res = {}
    rng = np.random.default_rng(0)
    for l in robot.findall("link"):
        pts = []
        for v in l.findall("visual"):
            mesh_el = v.find("geometry/mesh")
            if mesh_el is not None:
                Tv, _ = origin_T(v)
                V = trimesh.load_mesh(mesh_root / mesh_el.get("filename")).vertices
                pts.append(V @ Tv[:3, :3].T + Tv[:3, 3])
        if not pts:
            continue
        P = np.vstack(pts)
        if len(P) > n_max:
            P = P[rng.choice(len(P), n_max, replace=False)]
        name, b = l.get("name"), body_of[l.get("name")]
        Pb = P @ T_body[name][:3, :3].T + T_body[name][:3, 3]          # into the body root frame
        own = inside_union(body_prims.get(b, []), Pb)
        covered = own.copy()
        for ob, Ts in adjacent_transforms(b, movable):
            rest = ~covered
            if not rest.any():
                break
            ok = np.ones(rest.sum(), bool)
            for T in Ts:
                ok &= inside_union(body_prims.get(ob, []), Pb[rest] @ T[:3, :3].T + T[:3, 3])
            covered[np.flatnonzero(rest)[ok]] = True
        res[name] = {"own_primitives": len(lp[name]), "own_coverage": round(float(own.mean()), 6),
                     "coverage_incl_adjacent": round(float(covered.mean()), 6),
                     "vertices_checked": int(len(P)), "outside": int((~covered).sum())}
    return res


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

"""Collision simplification shared by scripts/model/replace_collisions_with_primitives.py and build_qarm_v2.py.

  PROFILES        per-mesh primitive budgets (coarse arm / fine gripper, or the earlier uniform budgets)
  parse_structure Genesis-like bodies (links joined by fixed joints form one body) and the movable joints
                  between bodies; parent-child bodies do not collide in Genesis (enable_adjacent_collision=False)
  prune           remove primitives buried in their own body or in an adjacent body over the whole joint range
  verify_written  round-trip coverage check of a written URDF (visual vertices inside own or adjacent primitives)

Primitives are qarm_lab.collision_primitives.Primitive objects; "link frame" means the URDF link frame.
"""

import xml.etree.ElementTree as ET

import numpy as np
import trimesh

from . import collision_primitives as CP
from .transforms import R_to_rpy, axis_angle_R, homog, origin_T

ARM = {"base_link", "yaw", "bicep", "forearm", "end-effector"}
PROFILES = {   # mesh stem (lower case) -> (max_prims, min_gain)
    # Arm links coarse, gripper fine. BICEP and FOREARM need a few more primitives than the other arm links:
    # with 2-4 BICEP primitives the shoulder-end bulge produced false BICEP-base contacts (89/400 random poses vs
    # 14 with the fine fit), and with 2 FOREARM primitives the forearm end touched the finger roots when the
    # wrist rotates (54/400) - measured with scripts/model/collision_audit.py, see results/collision_audit_*try*.
    "coarse-arm": {**{s: (2, 0.25) for s in ARM}, "bicep": (5, 0.05), "forearm": (3, 0.12),
                   "gripper_base_link": (10, 0.04), "link1a": (5, 0.04), "link1b": (5, 0.04),
                   "link2a": (4, 0.04), "link2b": (4, 0.04)},
    "uniform": {"base_link": (6, 0.08), "yaw": (4, 0.08), "bicep": (6, 0.08), "forearm": (8, 0.08),
                "end-effector": (2, 0.08), "gripper_base_link": (8, 0.08), "link1a": (4, 0.08),
                "link1b": (4, 0.08), "link2a": (3, 0.08), "link2b": (3, 0.08)},
}
DEFAULT_BUDGET = (6, 0.08)
Q_STEP = np.deg2rad(2.0)


def g(v):
    return " ".join(f"{x:.7g}" for x in v)


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


def prune(link_prims, body_of, T_body, movable, tol=0.001, verbose=True):
    """Remove buried primitives. link_prims: {link: [Primitive in link frame]}.

    A primitive P is buried if its surface (sampled every 3 mm) is covered point-wise by same-body primitives or by
    an adjacent body at every joint sample (2 deg steps). If that only holds after growing the covering primitives
    by `tol`, exactly the covering primitives that touch P's uncovered points are grown by `tol` and P is removed:
    collision geometry only ever grows, so it still encloses every visual. One primitive at a time, smallest
    first, re-evaluated after each removal. Returns (new link_prims, removal records)."""
    items = [{"link": l, "p": p} for l, ps in link_prims.items() for p in ps]
    removed = []

    def body_view():
        view = {}
        for i, it in enumerate(items):
            view.setdefault(body_of[it["link"]], []).append((i, it["p"].transformed(T_body[it["link"]])))
        return view

    def coverage(i_p, view, d):
        """(covered mask, pts, same-body covers, adjacent covers) with every cover grown by d."""
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
        if verbose:
            print(f"  pruned {it['link']:18s} {it['p'].kind:8s} {it['p'].volume * 1e6:8.2f} cm3  ({why}; grown: "
                  f"{removed[-1]['grown']})")
    out = {l: [] for l in link_prims}
    for it in items:
        out[it["link"]].append(it["p"])
    return out, removed


# ------------------------------------------------------------------------------------------- URDF I/O
def add_collision_element(link_el, p):
    """Append one <collision> for primitive p (pose in the link frame) to an ElementTree <link>."""
    c = ET.SubElement(link_el, "collision")
    ET.SubElement(c, "origin", {"xyz": g(p.T[:3, 3]), "rpy": g(R_to_rpy(p.T[:3, :3]))})
    geo = ET.SubElement(c, "geometry")
    if p.kind == "box":
        ET.SubElement(geo, "box", {"size": g(p.size)})
    elif p.kind == "cylinder":
        ET.SubElement(geo, "cylinder", {"radius": f"{p.size[0]:.7g}", "length": f"{p.size[1]:.7g}"})
    else:
        ET.SubElement(geo, "sphere", {"radius": f"{p.size[0]:.7g}"})


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


def verify_written(urdf_text, mesh_root, n_max=30000, mesh_override=None):
    """Round-trip check of a written URDF: primitives are rebuilt from <origin rpy>/<geometry> as a URDF parser
    would; each visual vertex must lie inside its own body's primitives or, at every sampled joint angle, inside an
    adjacent body's primitives. mesh_override maps a visual mesh filename to the file to test instead (e.g. the
    full-resolution original of a decimated visual). Returns {link: stats}."""
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
                fn = mesh_el.get("filename")
                path = (mesh_override or {}).get(fn, mesh_root / fn)
                V = trimesh.load_mesh(path).vertices
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

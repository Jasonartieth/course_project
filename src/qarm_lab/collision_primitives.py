"""Conservative primitive collision geometry: boxes, cylinders and spheres that fully enclose a mesh.

Method (per mesh):
  1. Long CAD sliver triangles are subdivided (edges <= max_edge) so faces are local.
  2. Greedy top-down splitting (like a BVH with a volume heuristic): starting from one cluster holding all faces,
     repeatedly take the cluster whose best split saves the most volume and split it. A split is a plane
     perpendicular to one of the cluster's box axes, at one of 9 quantiles of the face centroids; it is accepted
     only if the two children's primitive volumes sum to < (1 - min_gain) of the parent's. Stops at max_prims
     or when no split gains enough.
  3. Every cluster gets the smallest of three enclosing primitives:
       box      - minimum-volume oriented bounding box of the cluster's vertices
       cylinder - axis = one of the box axes, radius = minimal enclosing circle of the projected vertices
       sphere   - minimal enclosing sphere
Containment guarantee: every face's three vertices lie inside its cluster's (convex) primitive, so every face,
hence the whole visual mesh, lies inside the union of primitives. `check_coverage` re-verifies this on all
original vertices. Primitives are never smaller than the mesh: where they cannot fit exactly they are larger.

Primitive poses are 4x4 transforms in the mesh frame; cylinders are along their local z (URDF convention).
"""

from dataclasses import dataclass, field

import numpy as np
import trimesh

MIN_EXTENT = 1e-3          # m: no primitive thinner than 1 mm (flat clusters would give zero-thickness boxes)
MARGIN = 2e-4              # m: 0.2 mm padding on every primitive, absorbs floating-point error in the fits


@dataclass
class Primitive:
    kind: str                      # "box" | "cylinder" | "sphere"
    T: np.ndarray                  # 4x4 pose in the mesh frame (cylinder axis = local z)
    size: np.ndarray               # box: (x, y, z) full extents; cylinder: (radius, length); sphere: (radius,)

    @property
    def volume(self):
        if self.kind == "box":
            return float(np.prod(self.size))
        if self.kind == "cylinder":
            return float(np.pi * self.size[0] ** 2 * self.size[1])
        return float(4.0 / 3.0 * np.pi * self.size[0] ** 3)

    def contains(self, pts, tol=1e-7):
        local = (pts - self.T[:3, 3]) @ self.T[:3, :3]
        if self.kind == "box":
            return np.all(np.abs(local) <= self.size / 2 + tol, axis=1)
        if self.kind == "cylinder":
            return (np.hypot(local[:, 0], local[:, 1]) <= self.size[0] + tol) & (np.abs(local[:, 2]) <= self.size[1] / 2 + tol)
        return np.linalg.norm(local, axis=1) <= self.size[0] + tol

    def transformed(self, M):
        return Primitive(self.kind, M @ self.T, self.size.copy())


@dataclass
class FitResult:
    primitives: list
    k: int
    volume: float                                        # sum of primitive volumes (overlaps counted twice)
    hull_volume: float                                   # convex hull volume of the mesh, for comparison
    history: list = field(default_factory=list)          # total volume after each accepted split


# ------------------------------------------------------------------------------------------- 2-D / 3-D enclosing
def _min_circle(p, rng):
    """Welzl-style randomised incremental minimal enclosing circle of 2-D points. Returns (center, radius)."""
    p = p[rng.permutation(len(p))]

    def circ2(a, b):
        c = (a + b) / 2
        return c, np.linalg.norm(a - c)

    def circ3(a, b, c):
        d = 2 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
        if abs(d) < 1e-14:
            pts = np.array([a, b, c])
            i, j = max(((0, 1), (0, 2), (1, 2)), key=lambda ij: np.linalg.norm(pts[ij[0]] - pts[ij[1]]))
            return circ2(pts[i], pts[j])
        ux = ((a @ a) * (b[1] - c[1]) + (b @ b) * (c[1] - a[1]) + (c @ c) * (a[1] - b[1])) / d
        uy = ((a @ a) * (c[0] - b[0]) + (b @ b) * (a[0] - c[0]) + (c @ c) * (b[0] - a[0])) / d
        o = np.array([ux, uy])
        return o, np.linalg.norm(a - o)

    eps = 1e-12
    c, r = p[0], 0.0
    for i in range(1, len(p)):
        if np.linalg.norm(p[i] - c) <= r + eps:
            continue
        c, r = p[i], 0.0
        for j in range(i):
            if np.linalg.norm(p[j] - c) <= r + eps:
                continue
            c, r = circ2(p[i], p[j])
            for k in range(j):
                if np.linalg.norm(p[k] - c) > r + eps:
                    c, r = circ3(p[i], p[j], p[k])
    return c, r


def _hull_points(pts):
    try:
        return trimesh.convex.convex_hull(pts).vertices
    except Exception:          # degenerate (coplanar / collinear) clusters
        return pts


def _frame_with_z(axis):
    z = axis / np.linalg.norm(axis)
    x = np.cross(z, [1.0, 0, 0]) if abs(z[0]) < 0.9 else np.cross(z, [0, 1.0, 0])
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def fit_cluster(pts, rng):
    """Smallest enclosing primitive (box / cylinder / sphere) of a point set."""
    h = _hull_points(pts)
    cands = []
    try:
        to_origin, ext = trimesh.bounds.oriented_bounds(h)
        Tb = np.linalg.inv(to_origin)
    except Exception:          # degenerate cluster: PCA-aligned box (still encloses every point exactly)
        c = h.mean(0)
        _, _, Vt = np.linalg.svd(h - c, full_matrices=False)
        R = Vt.T * np.array([1, 1, np.linalg.det(Vt.T)])
        loc = (h - c) @ R
        lo, hi = loc.min(0), loc.max(0)
        Tb = np.eye(4)
        Tb[:3, :3] = R
        Tb[:3, 3] = c + R @ ((lo + hi) / 2)
        ext = hi - lo
    cands.append(Primitive("box", Tb, np.maximum(ext + 2 * MARGIN, MIN_EXTENT)))
    for a in range(3):
        R = _frame_with_z(Tb[:3, a])
        loc = h @ R
        c2, r = _min_circle(loc[:, :2], rng)
        zmin, zmax = loc[:, 2].min(), loc[:, 2].max()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = R @ np.array([c2[0], c2[1], (zmin + zmax) / 2])
        cands.append(Primitive("cylinder", T, np.array([max(r + MARGIN, MIN_EXTENT / 2),
                                                        max(zmax - zmin + 2 * MARGIN, MIN_EXTENT)])))
    try:
        center, radius = trimesh.nsphere.minimum_nsphere(h)
    except Exception:          # qhull fails on (near-)coplanar clusters: enclosing, not minimal, sphere
        center = (h.min(0) + h.max(0)) / 2
        radius = float(np.linalg.norm(h - center, axis=1).max())
    T = np.eye(4)
    T[:3, 3] = center
    cands.append(Primitive("sphere", T, np.array([max(radius + MARGIN, MIN_EXTENT / 2)])))
    best = min(cands, key=lambda p: p.volume)
    # robustness: the chosen primitive must contain every point; otherwise fall back to the box (exact by OBB)
    if not best.contains(pts).all():
        best = cands[0]
    return best


# ------------------------------------------------------------------------------------------- splitting
_EVAL_PTS = 4000           # vertices used to *evaluate* candidate splits (final fits use all vertices)


def _cluster_points(V, faces, idx):
    return V[np.unique(faces[idx].ravel())]


def _best_split(V, faces, cent, idx, prim, rng):
    """Best (gain, left_idx, right_idx) for one cluster, or None."""
    if len(idx) < 8:
        return None
    best = None
    # candidate split planes are perpendicular to the cluster's oriented-bounding-box axes
    if prim.kind == "box":
        axes = prim.T[:3, :3]
    else:
        to_origin, _ = trimesh.bounds.oriented_bounds(_hull_points(_cluster_points(V, faces, idx)))
        axes = np.linalg.inv(to_origin)[:3, :3]
    for a in range(3):
        proj = cent[idx] @ axes[:, a]
        for qtl in np.linspace(0.1, 0.9, 9):
            cut = np.quantile(proj, qtl)
            left, right = idx[proj <= cut], idx[proj > cut]
            if len(left) < 4 or len(right) < 4:
                continue
            vols = []
            for part in (left, right):
                pts = _cluster_points(V, faces, part)
                if len(pts) > _EVAL_PTS:
                    pts = pts[rng.choice(len(pts), _EVAL_PTS, replace=False)]
                vols.append(fit_cluster(pts, rng).volume)
            gain = prim.volume - sum(vols)
            if best is None or gain > best[0]:
                best = (gain, left, right)
    return best


def fit_mesh(mesh, max_prims=6, min_gain=0.08, seed=0, max_edge=0.005):
    """Greedy top-down primitive fit of `mesh` (trimesh.Trimesh); see the module docstring."""
    rng = np.random.default_rng(seed)
    V, faces = trimesh.remesh.subdivide_to_size(mesh.vertices, mesh.faces, max_edge=max_edge, max_iter=20)
    cent = V[faces].mean(axis=1)
    all_idx = np.arange(len(faces))
    clusters = [(all_idx, fit_cluster(_cluster_points(V, faces, all_idx), rng))]
    cache = {}
    history = [clusters[0][1].volume]
    while len(clusters) < max_prims:
        best_i, best = None, None
        for i, (idx, prim) in enumerate(clusters):
            key = (len(idx), int(idx[0]), int(idx[-1]))
            if key not in cache:
                cache[key] = _best_split(V, faces, cent, idx, prim, rng)
            s = cache[key]
            if s is not None and s[0] > min_gain * prim.volume and (best is None or s[0] > best[0]):
                best_i, best = i, s
        if best is None:
            break
        _, left, right = best
        clusters.pop(best_i)
        clusters += [(part, fit_cluster(_cluster_points(V, faces, part), rng)) for part in (left, right)]
        history.append(sum(p.volume for _, p in clusters))
    prims = [p for _, p in clusters]
    return FitResult(prims, len(prims), sum(p.volume for p in prims), float(mesh.convex_hull.volume), history)


def surface_points(p, spacing=0.003):
    """Points on the surface of primitive p (in p's parent frame) with roughly `spacing` between them; used to
    decide whether p is buried inside other primitives (all surface points inside -> buried)."""
    if p.kind == "box":
        hx, hy, hz = p.size / 2
        pts = []
        for ax, (a, b, c) in enumerate(((hy, hz, hx), (hx, hz, hy), (hx, hy, hz))):
            u = np.linspace(-a, a, max(2, int(np.ceil(2 * a / spacing)) + 1))
            v = np.linspace(-b, b, max(2, int(np.ceil(2 * b / spacing)) + 1))
            U, V = np.meshgrid(u, v)
            for s in (-c, c):
                W = np.full_like(U, s)
                if ax == 0:
                    pts.append(np.column_stack([W.ravel(), U.ravel(), V.ravel()]))
                elif ax == 1:
                    pts.append(np.column_stack([U.ravel(), W.ravel(), V.ravel()]))
                else:
                    pts.append(np.column_stack([U.ravel(), V.ravel(), W.ravel()]))
        local = np.vstack(pts)
    elif p.kind == "cylinder":
        r, L = p.size
        nth = max(8, int(np.ceil(2 * np.pi * r / spacing)))
        th = np.linspace(0, 2 * np.pi, nth, endpoint=False)
        z = np.linspace(-L / 2, L / 2, max(2, int(np.ceil(L / spacing)) + 1))
        Th, Z = np.meshgrid(th, z)
        side = np.column_stack([r * np.cos(Th).ravel(), r * np.sin(Th).ravel(), Z.ravel()])
        rr = np.linspace(0, r, max(2, int(np.ceil(r / spacing)) + 1))
        Rr, Tc = np.meshgrid(rr, th)
        disk = np.column_stack([(Rr * np.cos(Tc)).ravel(), (Rr * np.sin(Tc)).ravel()])
        caps = [np.column_stack([disk, np.full(len(disk), s)]) for s in (-L / 2, L / 2)]
        local = np.vstack([side] + caps)
    else:
        r = p.size[0]
        n = max(50, int(4 * np.pi * r * r / spacing ** 2))
        i = np.arange(n) + 0.5
        phi = np.arccos(1 - 2 * i / n)
        th = np.pi * (1 + 5 ** 0.5) * i
        local = r * np.column_stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)])
    return local @ p.T[:3, :3].T + p.T[:3, 3]


def check_coverage(prims, pts, tol=1e-7):
    """Fraction of points inside the union of primitives (must be 1.0) and the worst outside distance proxy."""
    inside = np.zeros(len(pts), bool)
    for p in prims:
        inside |= p.contains(pts, tol)
    return float(inside.mean()), int((~inside).sum())


def union_volume_mc(prims, n=400_000, seed=0):
    """Monte-Carlo volume of the union of primitives (overlaps counted once)."""
    rng = np.random.default_rng(seed)
    lo = np.full(3, np.inf)
    hi = -lo
    for p in prims:
        r = np.linalg.norm(p.size) if p.kind != "sphere" else p.size[0]
        lo = np.minimum(lo, p.T[:3, 3] - r)
        hi = np.maximum(hi, p.T[:3, 3] + r)
    x = rng.uniform(lo, hi, size=(n, 3))
    inside = np.zeros(n, bool)
    for p in prims:
        inside |= p.contains(x)
    return float(inside.mean() * np.prod(hi - lo))

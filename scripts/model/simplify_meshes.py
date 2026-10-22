"""Derive simulation meshes from the official (untouched) QArm STL files.

Why: the official meshes are CAD exports with up to 461k triangles. MuJoCo refuses STL files with more than
200k faces, and every simulator pays for them in collision checking. The official files stay unchanged in
models/qarm_official/; derived meshes go to models/qarm/meshes/:
  visual/<name>__vis.stl     decimated to at most MAX_VISUAL_FACES faces (shape preserved for rendering)
  collision/<name>__col.stl  convex hull of the original mesh (cheap, conservative contact geometry)
Visual and collision files get different stems because importers such as MuJoCo's name mesh assets by file
stem, so equal stems would clash.
  mesh_report.json      face counts, file sizes and the decimation error (max/mean surface distance)

Usage:  python scripts/model/simplify_meshes.py
"""

import json
from pathlib import Path

import fast_simplification
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "models" / "qarm_official"
OUT = ROOT / "models" / "qarm" / "meshes"
MAX_VISUAL_FACES = 50_000
N_SAMPLES = 20_000


def surface_error(ref, approx, rng):
    """Distance from points sampled on `ref` to the surface of `approx` (m)."""
    pts, _ = trimesh.sample.sample_surface(ref, N_SAMPLES, seed=int(rng.integers(1 << 31)))
    _, dist, _ = trimesh.proximity.closest_point(approx, pts)
    return float(dist.max()), float(dist.mean())


def main():
    rng = np.random.default_rng(0)
    (OUT / "visual").mkdir(parents=True, exist_ok=True)
    (OUT / "collision").mkdir(parents=True, exist_ok=True)
    report = {}
    for src in sorted(SRC.glob("*/meshes/*.STL")):
        name = f"{src.parent.parent.name}__{src.stem}"          # e.g. qarm__BICEP, qarm_gripper__base_link
        mesh = trimesh.load_mesh(src, process=True)
        n0 = len(mesh.faces)
        if n0 > MAX_VISUAL_FACES:
            v, f = fast_simplification.simplify(mesh.vertices, mesh.faces, target_reduction=1 - MAX_VISUAL_FACES / n0)
            vis = trimesh.Trimesh(v, f, process=True)
        else:
            vis = mesh
        hull = mesh.convex_hull
        vis_path, col_path = OUT / "visual" / f"{name}__vis.stl", OUT / "collision" / f"{name}__col.stl"
        vis.export(vis_path)
        hull.export(col_path)
        emax, emean = surface_error(mesh, vis, rng) if vis is not mesh else (0.0, 0.0)
        report[name] = {
            "source": str(src.relative_to(ROOT)), "faces_official": n0, "faces_visual": len(vis.faces),
            "faces_collision": len(hull.faces), "visual_err_max_mm": round(emax * 1e3, 3),
            "visual_err_mean_mm": round(emean * 1e3, 4), "kb_official": src.stat().st_size // 1024,
            "kb_visual": vis_path.stat().st_size // 1024, "kb_collision": col_path.stat().st_size // 1024,
        }
        print(f"{name:28s} faces {n0:7d} -> visual {len(vis.faces):6d} (err max {emax*1e3:.2f} mm) | hull {len(hull.faces):4d}")
    (OUT / "mesh_report.json").write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()

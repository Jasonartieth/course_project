"""Analysis (2026-10-24): where do the mmic-project 2b8ab95 gripper parts sit, and how do the fingers move?
Result: the finger frames are mirrored correctly; the closing scalar (-0.45, -0.35, 0.45, -0.35) overshoots
(fingertips meet near grip 0.45 at y=0.165, then pass through each other). Plot: results/research_2026-10-24/.

Plots projections of the gripper meshes in the gripper_base_link frame (x-y = finger plane, joint axes z)
for several grip values, using a configurable finger-frame description. Prints fingertip positions.
Usage: python gripper_geom.py <meshes_dir> <out_png> [variant]
"""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def T(xyz=(0, 0, 0), r=(0, 0, 0)):
    M = np.eye(4)
    M[:3, :3] = rpy(*r)
    M[:3, 3] = xyz
    return M


def Rz(q, axis_sign=1.0):
    return T(r=(0, 0, axis_sign * q))


PI = np.pi
MMIC = {
    "base_vis": T((0, 0.017, 0), (-PI / 2, 0, 0)),
    "J1A": (T((-0.0305, 0.0515, 0), (0, 0, PI / 2)), +1), "L1A_vis": T((0.0353, 0, 0), (-PI / 2, PI, 0)),
    "J2A": (T((0.07, 0, 0)), +1), "L2A_vis": T((0.02, 0.01, -0.015), (-PI / 2, PI, 0)),
    "J1B": (T((0.0305, 0.0515, 0), (0, 0, PI / 2)), +1), "L1B_vis": T((0.0353, 0, 0.017), (-PI / 2, 0, PI)),
    "J2B": (T((0.07, 0, 0)), -1), "L2B_vis": T((0.02, -0.01, 0.015), (-PI / 2, 0, PI)),
}
CLOSE = {"J1A": -0.45, "J2A": -0.35, "J1B": 0.45, "J2B": -0.35}
R_MOUNT = rpy(PI / 2, 0, PI / 2)                           # END-EFFECTOR -> gripper_base_link
TCP_G = R_MOUNT.T @ np.array([0, 0, 0.162])                 # TCP expressed in the gripper frame


def parts(desc, q, meshes):
    out = {"base": (desc["base_vis"], meshes["gripper_base_link"])}
    for side in "AB":
        J1, s1 = desc[f"J1{side}"]
        J2, s2 = desc[f"J2{side}"]
        F1 = J1 @ Rz(q[f"J1{side}"], s1)
        F2 = F1 @ J2 @ Rz(q[f"J2{side}"], s2)
        out[f"L1{side}"] = (F1 @ desc[f"L1{side}_vis"], meshes[f"LINK1{side}"])
        out[f"L2{side}"] = (F2 @ desc[f"L2{side}_vis"], meshes[f"LINK2{side}"])
    return out


def main():
    mdir, out_png = sys.argv[1], sys.argv[2]
    meshes = {}
    for n in ("gripper_base_link", "LINK1A", "LINK2A", "LINK1B", "LINK2B"):
        m = trimesh.load_mesh(f"{mdir}/{n}.STL")
        meshes[n] = m.vertices[np.random.default_rng(0).choice(len(m.vertices), 6000)]
    grips = (0.0, 0.5, 1.0)
    fig, axs = plt.subplots(2, len(grips), figsize=(15, 9))
    colors = {"base": "0.6", "L1A": "tab:red", "L2A": "tab:orange", "L1B": "tab:blue", "L2B": "tab:cyan"}
    for c, g in enumerate(grips):
        q = {k: g * v for k, v in CLOSE.items()}
        ps = parts(MMIC, q, meshes)
        for r, (i, j, lab) in enumerate(((0, 1, "x-y (finger plane)"), (0, 2, "x-z"))):
            ax = axs[r, c]
            for name, (M, V) in ps.items():
                W = V @ M[:3, :3].T + M[:3, 3]
                ax.scatter(W[:, i], W[:, j], s=0.3, c=colors[name], label=name)
            ax.plot(TCP_G[i], TCP_G[j], "k*", ms=12, label="TCP")
            ax.set_aspect("equal")
            ax.set_title(f"grip={g} {lab}")
        tips = {}
        for side in "AB":
            M, V = ps[f"L2{side}"]
            W = V @ M[:3, :3].T + M[:3, 3]
            tips[side] = W[np.argmax(W[:, 1])]
        print(f"grip={g}: tipA={tips['A'].round(4)} tipB={tips['B'].round(4)} gap_x={tips['B'][0]-tips['A'][0]:.4f} TCP={TCP_G.round(4)}")
    axs[0, 0].legend(markerscale=20, fontsize=7)
    fig.tight_layout()
    fig.savefig(out_png, dpi=80)


if __name__ == "__main__":
    main()

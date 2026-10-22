"""Build the fixed QArm URDFs from the untouched official files in models/qarm_official/.

Outputs (models/qarm/urdf/):
  qarm.urdf           arm only + `tcp` link
  qarm_gripper.urdf   arm + gripper (fixed mount, 4 finger joints) + `tcp` link
  build_report.json   frame-alignment residuals and every value that was changed

Fixes relative to the official URDF (docs/research/qarm_survey_2026-10-22.md, section 6):
  1. mesh paths   package://QARM/... -> relative paths to the derived meshes in models/qarm/meshes/
                  (visual = decimated, collision = convex hull; run scripts/model/simplify_meshes.py first)
  2. joint limits manual values (wrist +-160 deg), effort = instantaneous torque, velocity = pi/2 rad/s
  3. inertials    official "7. Dynamic Parameters.pdf" bodies B1..B4 transformed from DH frame i into the
                  URDF link frame (B4 = rotating wrist + gripper -> END-EFFECTOR); gripper links get token
                  masses so the gripper is not counted twice. Base link keeps the official (placeholder) value.
  4. tcp link     fixed joint at xyz = spec.URDF_EE_TO_TCP from END-EFFECTOR (matches official DH FK, <1 mm)
  5. gripper      attached to END-EFFECTOR by a fixed joint (GRIPPER_MOUNT_*); finger joints get limits.
                  The finger kinematics are approximate (the real gripper is a tendon-driven underactuated
                  mechanism); they are good for visuals/contacts, not for gripper dynamics.

Usage:  python scripts/model/build_qarm_urdf.py
"""

import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from qarm_lab import kinematics, spec  # noqa: E402

OFFICIAL = ROOT / "models" / "qarm_official"
OUT = ROOT / "models" / "qarm" / "urdf"

ARM_JOINTS = ("YAW", "SHOULDER", "ELBOW", "WRIST")          # official joint names, same order as spec
ARM_LINKS = ("YAW", "BICEP", "FOREARM", "END-EFFECTOR")      # child link of each joint = DH body 1..4

# Gripper mount (END-EFFECTOR -> gripper base). Rotation maps the gripper's +y (finger direction) onto the
# wrist axis (+z of END-EFFECTOR). Translation verified visually: see scripts/model/render_model_check.py.
GRIPPER_MOUNT_XYZ = (0.0, 0.0, 0.0)
GRIPPER_MOUNT_RPY = (np.pi / 2, 0.0, np.pi / 2)

# Finger joints: (lower, upper) in rad. q = 0 is the CAD pose (fully open). Closing direction from geometry:
# Joint1A closes negative, Joint1B positive, Joint2A/Joint2B (mirrored axis) both negative.
FINGER_LIMITS = {"Joint1A": (-0.9, 0.05), "Joint2A": (-1.2, 0.05), "Joint1B": (-0.05, 0.9), "Joint2B": (-1.2, 0.05)}
FINGER_EFFORT, FINGER_VELOCITY = spec.GRIP_TAU_INST, 2.0
TOKEN_MASS = {"gripper_base": 0.005, "Link1A": 0.005, "Link1B": 0.005, "Link2A": 0.003, "Link2B": 0.003}


# ------------------------------------------------------------------------------------------- math helpers
def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle_R(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


def homog(R=np.eye(3), t=(0, 0, 0)):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def fvec(s):
    return [float(v) for v in s.split()]


def fmt(v):
    return " ".join(f"{x:.9g}" for x in v)


# ------------------------------------------------------------------------------------------- URDF FK
def urdf_link_frames(robot, q):
    """Pose of each arm link (in base_link frame) for joint angles q, using the official joint origins."""
    joints = {j.get("name"): j for j in robot.findall("joint")}
    T = np.eye(4)
    out = {}
    for name, link, qi in zip(ARM_JOINTS, ARM_LINKS, q):
        j = joints[name]
        o = j.find("origin")
        T = T @ homog(rpy_to_R(*fvec(o.get("rpy"))), fvec(o.get("xyz")))
        T = T @ homog(axis_angle_R(fvec(j.find("axis").get("xyz")), qi))
        out[link] = T.copy()
    return out


def dh_to_link_transforms(robot):
    """X_i with T_dh_i(q) = T_urdf_link_i(q) @ X_i; checked to be constant over random q."""
    rng = np.random.default_rng(0)
    qs = [np.zeros(4)] + [rng.uniform(spec.Q_MIN, spec.Q_MAX) for _ in range(20)]
    Xs = {link: [] for link in ARM_LINKS}
    for q in qs:
        L = urdf_link_frames(robot, q)
        for i, link in enumerate(ARM_LINKS):
            Xs[link].append(np.linalg.inv(L[link]) @ kinematics.fk_frames(q)[i])
    X, resid = {}, {}
    for link in ARM_LINKS:
        stack = np.array(Xs[link])
        X[link] = stack[0]
        resid[link] = {
            "max_translation_m": float(np.abs(stack[:, :3, 3] - stack[0, :3, 3]).max()),
            "max_rotation_elem": float(np.abs(stack[:, :3, :3] - stack[0, :3, :3]).max()),
        }
    return X, resid


# ------------------------------------------------------------------------------------------- edits
def set_inertial(link_el, mass, com, inertia):
    inr = link_el.find("inertial")
    if inr is None:
        inr = ET.SubElement(link_el, "inertial")
        ET.SubElement(inr, "origin")
        ET.SubElement(inr, "mass")
        ET.SubElement(inr, "inertia")
    inr.find("origin").attrib.update({"xyz": fmt(com), "rpy": "0 0 0"})
    inr.find("mass").set("value", f"{mass:.9g}")
    I = inertia
    inr.find("inertia").attrib.update({
        "ixx": f"{I[0, 0]:.9g}", "ixy": f"{I[0, 1]:.9g}", "ixz": f"{I[0, 2]:.9g}",
        "iyy": f"{I[1, 1]:.9g}", "iyz": f"{I[1, 2]:.9g}", "izz": f"{I[2, 2]:.9g}"})


def fix_mesh_paths(robot, pkg_prefix, pkg_name):
    """Point <visual> meshes at the decimated copies and <collision> meshes at the convex hulls
    (both generated by scripts/model/simplify_meshes.py into models/qarm/meshes/)."""
    for kind in ("visual", "collision"):
        for m in robot.findall(f"link/{kind}/geometry/mesh"):
            fn = m.get("filename")
            assert fn.startswith(pkg_prefix), fn
            stem = Path(fn[len(pkg_prefix):]).stem
            m.set("filename", f"../meshes/{kind}/{pkg_name}__{stem}__{kind[:3] if kind == 'visual' else 'col'}.stl")


def build_arm(report):
    robot = ET.parse(OFFICIAL / "qarm" / "urdf" / "QARM.urdf").getroot()
    robot.set("name", "qarm")
    fix_mesh_paths(robot, "package://QARM/meshes/", "qarm")

    joints = {j.get("name"): j for j in robot.findall("joint")}
    report["limits"] = {}
    for k, name in enumerate(ARM_JOINTS):
        lim = joints[name].find("limit")
        before = dict(lim.attrib)
        lim.attrib.update({"lower": f"{spec.Q_MIN[k]:.6f}", "upper": f"{spec.Q_MAX[k]:.6f}",
                           "effort": f"{spec.TAU_INST[k]:.6g}", "velocity": f"{spec.DQ_MAX[k]:.6f}"})
        report["limits"][name] = {"before": before, "after": dict(lim.attrib)}

    X, resid = dh_to_link_transforms(robot)
    report["dh_to_link_residual"] = resid
    links = {l.get("name"): l for l in robot.findall("link")}
    report["inertials"] = {}
    for i, link in enumerate(ARM_LINKS):
        R, t = X[link][:3, :3], X[link][:3, 3]
        com = R @ spec.DH_BODY_COM[i] + t
        inertia = R @ np.diag(spec.DH_BODY_INERTIA_DIAG[i]) @ R.T
        set_inertial(links[link], spec.DH_BODY_MASS[i], com, inertia)
        report["inertials"][link] = {"mass": float(spec.DH_BODY_MASS[i]), "com_link": com.round(6).tolist()}

    tcp = ET.SubElement(robot, "link", {"name": "tcp"})
    set_inertial(tcp, 1e-4, np.zeros(3), np.eye(3) * 1e-9)     # tiny inertial so every simulator keeps the link
    j = ET.SubElement(robot, "joint", {"name": "tcp_joint", "type": "fixed"})
    ET.SubElement(j, "origin", {"xyz": fmt(spec.URDF_EE_TO_TCP), "rpy": "0 0 0"})
    ET.SubElement(j, "parent", {"link": "END-EFFECTOR"})
    ET.SubElement(j, "child", {"link": "tcp"})
    return robot


def add_gripper(robot, report):
    grip = ET.parse(OFFICIAL / "qarm_gripper" / "urdf" / "qarm_gripper.urdf").getroot()
    fix_mesh_paths(grip, "package://qarm_gripper/meshes/", "qarm_gripper")
    # rename the gripper's base link so it does not clash with the arm's base_link
    for el in grip.iter():
        if el.tag in ("parent", "child") and el.get("link") == "base_link":
            el.set("link", "gripper_base")
    for l in grip.findall("link"):
        if l.get("name") == "base_link":
            l.set("name", "gripper_base")
        name = l.get("name")
        m = TOKEN_MASS[name]
        set_inertial(l, m, fvec(l.find("inertial/origin").get("xyz")), np.eye(3) * m * 0.02 ** 2 * 0.4)
        robot.append(l)
    for j in grip.findall("joint"):
        lo, hi = FINGER_LIMITS[j.get("name")]
        j.find("limit").attrib.update({"lower": f"{lo}", "upper": f"{hi}",
                                       "effort": f"{FINGER_EFFORT}", "velocity": f"{FINGER_VELOCITY}"})
        robot.append(j)
    mount = ET.SubElement(robot, "joint", {"name": "gripper_mount", "type": "fixed"})
    ET.SubElement(mount, "origin", {"xyz": fmt(GRIPPER_MOUNT_XYZ), "rpy": fmt(GRIPPER_MOUNT_RPY)})
    ET.SubElement(mount, "parent", {"link": "END-EFFECTOR"})
    ET.SubElement(mount, "child", {"link": "gripper_base"})
    report["gripper"] = {"mount_xyz": list(GRIPPER_MOUNT_XYZ), "mount_rpy": list(GRIPPER_MOUNT_RPY),
                         "finger_limits": FINGER_LIMITS, "token_masses": TOKEN_MASS}
    return robot


def write(robot, path, header):
    ET.indent(robot, space="  ")
    xml = ET.tostring(robot, encoding="unicode")
    path.write_text(f'<?xml version="1.0" encoding="utf-8"?>\n<!-- {header} -->\n{xml}\n')


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    report = {"source": "models/qarm_official (quanser/urdf_representations @ 071f9a6)"}
    header = ("GENERATED by scripts/model/build_qarm_urdf.py from the official Quanser URDF (BSD-3). "
              "Do not edit by hand; edit the script and re-run.")
    arm = build_arm(report)
    write(arm, OUT / "qarm.urdf", header)
    arm_g = add_gripper(build_arm({}), report)
    write(arm_g, OUT / "qarm_gripper.urdf", header)
    (OUT / "build_report.json").write_text(json.dumps(report, indent=2))
    worst = max(max(v.values()) for v in report["dh_to_link_residual"].values())
    print(f"wrote {OUT/'qarm.urdf'}\nwrote {OUT/'qarm_gripper.urdf'}\nDH->link transform max residual: {worst:.2e}")


if __name__ == "__main__":
    main()

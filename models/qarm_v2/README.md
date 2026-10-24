# QArm with gripper (v2)

Quanser QArm (4-DOF arm + tendon gripper) for Genesis, generated from the **official Quanser files**. Collision
geometry is made only of **boxes, cylinders and spheres** that enclose every vertex of the full-resolution
official meshes, so the collision geometry is never smaller than the visual mesh. It is coarse on the arm and
fine on the gripper. Primitives buried in their own link, or in an adjacent link over the whole joint range, are
removed: Genesis does not check parent-child collisions.

| File | Content |
|---|---|
| `qarm_with_gripper.urdf` | the model (same link/joint names, same path as before) |
| `meshes/visual/*.stl` | official meshes decimated to at most 50k faces (largest deviation 6.8 mm) |
| `collision_report.json` | primitives per link, pruned primitives, per-mesh fit (budget, union volume vs convex hull), coverage of the written URDF (full-resolution: 100 %; decimated visuals: reported) |
| `gripper_report.json` | finger joint limits, recommended single-scalar mapping, fingertip positions |
| `files_sha256.json` | hashes of the URDF and meshes |
| `LICENSE` | Quanser BSD-3 licence of the URDF representations and meshes |

## What changed compared with 2b8ab95

| Problem in 2b8ab95 | Fix |
|---|---|
| **Did not load** in Genesis 1.4.4: 10 `<material>` tags had no `name` (`ValueError: Missing required attribute name ... Material`), so `gripper_demo.py` crashed | materials are named (`qarm_light_grey`, `qarm_dark_grey`) |
| Collision used the full CAD meshes. Genesis reduces each robot link to **one convex hull**, which filled the palm between the fingers | **32 boxes/cylinders/spheres** (25 / 6 / 1) fitted to the official full-resolution meshes. Arm: base_link 1, YAW 2, BICEP 4, FOREARM 3, END-EFFECTOR 1. Gripper: gripper_base_link 10, Link1A 4, Link1B 5, Link2A 1, Link2B 1. Every official vertex is inside its link's primitives or, over the whole joint range, inside the adjacent link's (checked on the written URDF) |
| Meshes up to 461k faces → Genesis fell back to its legacy URDF parser ("physics defaults may be off"); 36 s to load | visual meshes ≤ 50k faces: primary parser, 5.2 s to load |
| Fingers passed through each other when closing | The finger frames from 2b8ab95 were already correct and mirrored. The cause was the closing scalar (−0.45, −0.35, 0.45, −0.35), which goes too far: the fingertips already meet at grip ≈ 0.36. **Finger joint limits now stop where the two fingers' visual meshes first touch** (proximal ±0.1616 rad, distal −0.1257 rad) |
| Placeholder inertias ("example values only"), `Link1B` without mass, arm effort/velocity = 0 | arm inertias from Quanser's QArm "Dynamic Parameters" (rotating wrist + gripper mass in `END-EFFECTOR`); joint limits from the QArm User Manual (wrist ±160°); effort = instantaneous torque (10.6 / 21.2 / 10.6 / 4.1 N·m); velocity π/2 rad/s; small finger masses with each centre of mass at its part's geometric centre |
| no tool frame | `tcp` link 0.162 m beyond `END-EFFECTOR`. It matches Quanser's DH forward kinematics (λ3 = 0.40 m) and the point where the fingertips meet (0.168 m) |

Unchanged: the gripper mount, finger joint origins/axes and visual origins from 2b8ab95 (the official gripper URDF
leaves the finger parts floating; the 2b8ab95 alignment is the one that looks right), and all link/joint names.
`src/scripts/qarm/gripper_demo.py` runs **unchanged** with this model.

## Single-scalar gripper mapping

Fingertips touch at grip = 1 with

```
(Joint1A, Joint2A, Joint1B, Joint2B) = grip * (-0.1616, -0.1257, +0.1616, -0.1257)   # rad
```

This mapping is the same direction and ratio as the current `FINGER_CLOSE_RAD` in
`src/mmic_project/qarm_gripper.py`, scaled down to the geometry. With the current coefficients (−0.45, …) the
fingers stop at the joint limits from grip ≈ 0.36 onward.

## Validation (Genesis 1.4.4, CPU)

| Check | 2b8ab95 (materials patched so it loads) | v2 |
|---|---|---|
| load / parser | 36 s, legacy-parser fallback | 5.2 s, primary parser |
| "dubious centre of mass" warnings | Link1A, Link2A, Link2B | none |
| self-contacts at the zero pose | none | none, and no pairs auto-filtered |
| finger contacts in a 21-step open→close sweep | 12 steps (interpenetration) | 4 steps, at the very end of closing. The collision geometry is slightly larger than the visual mesh, so the fingers touch a little early, as intended |
| collisions + gravity on, PD hold at HOME while the gripper cycles | — | finite; arm error ≤ 0.23° |
| 3 cm cubes dropped on the palm / finger A / finger B | — | each lands on the link it was dropped on and stays on the gripper (z ≈ 0.52 m) |
| `gripper_demo.py --headless --cycles 2` (unchanged) | crashed (materials) | runs; fingers stop at the contact limits |
| self-contacts over 400 random poses (arm pairs) | — | BICEP–base 14 and FOREARM–finger 0, the same as a finer 43-primitive fit; coarser arm fits created false contacts (BICEP–base 89, FOREARM–finger 54) |

## Notes and open items

- The gripper frames and finger limits have **not been verified on the physical arm**. The real gripper is
  tendon-driven and underactuated; the 4 independent finger joints are an approximation.
- With this model `recompute_inertia=True` is no longer needed (it discards the inertias above), and
  collisions can be enabled.
- Regenerate: [`scripts/model/build_qarm_v2.py`](https://github.com/Jasonartieth/course_project/blob/167b649/scripts/model/build_qarm_v2.py)
  (profile `coarse-arm`) with [`src/qarm_lab/collision_primitives.py`](https://github.com/Jasonartieth/course_project/blob/167b649/src/qarm_lab/collision_primitives.py)
  and [`src/qarm_lab/collision_simplify.py`](https://github.com/Jasonartieth/course_project/blob/167b649/src/qarm_lab/collision_simplify.py)
  in Jasonartieth/course_project. Checks: `scripts/model/collision_audit.py` and `scripts/model/verify_qarm_genesis.py`.
- The previous 43-primitive collision set (no pruning) is commit 7701618 of `models/qarm_v2`.

## Sources

- Meshes and arm kinematics: [quanser/urdf_representations](https://github.com/quanser/urdf_representations) @ 071f9a6
  (BSD-3, see `LICENSE`). The previously shipped community STLs are byte-identical to these official files.
- Arm inertias: Quanser QArm ancillary material "7. Dynamic Parameters"; joint limits and actuator data: QArm User Manual, Table 7.
- Gripper frames: this repository, commit 2b8ab95.

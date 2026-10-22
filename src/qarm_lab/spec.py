"""Quanser QArm specifications (single source of truth).

Sources (see docs/research/qarm_survey_2026-10-22.md):
  * QArm User Manual, Table 2 (geometry) and Table 7 (joint specifications)
  * QArm Data Sheet (joint ranges, speeds)
  * Quanser `pal.products.qarm.QArm` (HOME/SLEEP poses, default profile & PID)
  * QArm ancillary "7. Dynamic Parameters.pdf" (raw centre-of-mass / inertia per DH body)
  * ROBOTIS e-Manual (Dynamixel XM540-W270-R, XM430-W350-R, XC430-W240-T)

Joint order everywhere in this package: [base(yaw), shoulder, elbow, wrist].
All angles in rad, lengths in m, torques in N*m, currents in A.
"""

import numpy as np

JOINT_NAMES = ("base", "shoulder", "elbow", "wrist")
N_JOINTS = 4

# ---------------------------------------------------------------- geometry (User Manual Table 2, standard DH)
L1, L2, L3, L4, L5 = 0.14, 0.35, 0.05, 0.25, 0.15
BETA = float(np.arctan2(L3, L2))          # 8.13 deg
LAMBDA1 = L1
LAMBDA2 = float(np.hypot(L2, L3))         # 0.3536
LAMBDA3 = L4 + L5                         # 0.40 : elbow axis -> TCP along the forearm

# Official URDF END-EFFECTOR frame -> DH TCP, expressed in END-EFFECTOR frame.
# Measured by scripts/research/fk_crosscheck_urdf_vs_dh.py (residual < 1 mm over many poses).
URDF_EE_TO_TCP = np.array([0.0, 0.0, 0.162])

# ---------------------------------------------------------------- joint limits (User Manual Table 7 / pal LIMITS)
Q_MIN = np.deg2rad([-170.0, -85.0, -95.0, -160.0])
Q_MAX = np.deg2rad([170.0, 85.0, 75.0, 160.0])
DQ_MAX = np.full(4, np.pi / 2)            # rad/s  (Table 7, also pal default profile_velocity 1.5708)
DDQ_MAX = np.full(4, np.pi / 3)           # rad/s^2 (Table 7, also pal default profile_acceleration 1.0472)

# Gripper command range accepted by pal (it clips to [0.1, 0.9]; 0 = open, 1 = closed)
GRIP_MIN, GRIP_MAX = 0.1, 0.9

# ---------------------------------------------------------------- poses (pal.products.qarm.QArm)
HOME_POSE = np.zeros(4)                                         # TCP at (0.45, 0, 0.49)
SLEEP_POSE = np.array([0.0, -17 * np.pi / 36, 15 * np.pi / 36, 0.0])   # (0, -85, +75, 0) deg

# ---------------------------------------------------------------- actuators (User Manual Table 7 + ROBOTIS)
ACTUATOR_MODEL = ("XM540-W270-R", "2x XM540-W270-R", "XM540-W270-R", "XM430-W350-R")
GRIPPER_ACTUATOR_MODEL = "XC430-W240-T"
GEAR_RATIO = np.array([272.5, 272.5, 272.5, 353.5])
TAU_INST = np.array([10.6, 21.2, 10.6, 4.1])     # do not exceed for > 0.1 s
TAU_CONT = np.array([2.65, 5.3, 2.65, 1.0])
I_INST = np.array([4.4, 8.8, 4.4, 2.3])
I_CONT = np.array([1.1, 2.2, 1.1, 0.6])
GRIP_TAU_INST, GRIP_TAU_CONT = 1.9, 0.5
GRIP_I_INST, GRIP_I_CONT = 1.4, 0.35
# Nominal output torque constant from stall data (tau_stall / i_stall @ 12 V). Real2sim will re-calibrate this.
KT_NOMINAL = TAU_INST / I_INST                    # ~2.41, 2.41, 2.41, 1.78 N*m/A
TEMP_MAX_C = 50.0                                 # manual: never exceed 50 degC

# pal's position-loop gains written on open (QUARC double properties 128..141)
PAL_PID = {"P": 8.89, "I": 0.012, "D": 10.23}

# ---------------------------------------------------------------- dynamics ("7. Dynamic Parameters.pdf", raw values)
# Body i is rigidly attached to DH frame i (1 = yaw body, 2 = upper arm, 3 = forearm, 4 = wrist + gripper).
# COM vectors and principal inertias are expressed in DH frame i; inertia is about the COM, axes = frame i.
DH_BODY_MASS = np.array([0.7906, 0.4591, 0.269, 0.257])
DH_BODY_COM = np.array([
    [0.0000, 0.0399, 0.0003],
    [-0.1071, -0.0154, -0.0001],
    [0.0094, 0.0020, 0.1561],
    [-0.0116, 0.0000, -0.0998],
])
DH_BODY_INERTIA_DIAG = np.array([
    [1.556e-3, 1.489e-3, 9.624e-4],
    [1.922e-4, 9.541e-3, 9.610e-3],
    [2.069e-3, 1.976e-3, 2.679e-4],
    [1.120e-3, 9.994e-4, 6.528e-4],
])

# ---------------------------------------------------------------- workspace safety (our own conservative choices)
TABLE_Z = 0.0               # work surface height in base frame (base sits on the table)
TCP_MIN_Z = 0.05            # keep TCP at least 5 cm above the table during experiments
LIMIT_MARGIN = np.deg2rad(5.0)   # trajectories must stay this far inside Q_MIN/Q_MAX

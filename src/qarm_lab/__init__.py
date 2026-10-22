"""qarm_lab: simulation, hardware I/O and real2sim tooling for the Quanser QArm (4-DOF).

Modules
-------
spec        Manufacturer specifications (single source of truth for limits, DH lengths, actuator data).
kinematics  Official standard-DH forward kinematics, analytic inverse kinematics, Jacobian.
profile     Emulation of the Dynamixel velocity-based trapezoidal profile used in QArm position mode.
trajectory  Trajectory container, generators, and offline safety checks.
safety      Runtime safety monitor (current / temperature / tracking error).
logio       Run-log container shared by the real and simulated backends.
runner      Backend-agnostic trajectory executor (approach -> stream -> hold).
backends    `sim_mujoco` (MuJoCo) and `pal_hw` (real arm through Quanser `pal`).
"""

__version__ = "0.1.0"

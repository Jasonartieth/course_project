"""Pre-flight check on the machine wired to the QArm (no motion, does not open the arm unless --open).

Checks: python / numpy, qarm_lab import, Quanser SDK (`quanser.hardware`), Quanser `pal`, USB devices,
trajectory set present and passing the offline checks. With --open it also opens the arm and reads it once.

Usage:  python scripts/hw/check_env.py [--open]
"""

import argparse
import platform
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))


def item(name, fn):
    try:
        msg = fn()
        print(f"  [OK]   {name}: {msg}")
        return True
    except Exception as e:  # noqa: BLE001 - report every failure, keep going
        print(f"  [FAIL] {name}: {type(e).__name__}: {e}")
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--open", action="store_true", help="also open the arm and read one sample (no motion)")
    ap.add_argument("--set", default=str(ROOT / "data/trajectories/hwday_20261023"))
    a = ap.parse_args()
    print(f"python {sys.version.split()[0]} on {platform.platform()}")
    ok = True
    ok &= item("numpy", lambda: __import__("numpy").__version__)
    ok &= item("qarm_lab", lambda: __import__("qarm_lab").__version__)
    ok &= item("quanser.hardware (Quanser SDK)", lambda: __import__("quanser.hardware", fromlist=["HIL"]).HIL.__name__)
    ok &= item("pal.products.qarm (Quanser_Academic_Resources)",
               lambda: __import__("pal.products.qarm", fromlist=["QArm"]).QArm.__name__)
    item("USB devices (lsusb)", lambda: "\n           " + "\n           ".join(
        subprocess.run(["lsusb"], capture_output=True, text=True).stdout.strip().splitlines()))

    def traj_check():
        from qarm_lab import trajectory as T
        files = sorted(Path(a.set).glob("[0-9][0-9]_*.npz"))
        bad = [f.name for f in files if not T.check_trajectory(T.Trajectory.load(f)).ok]
        if bad:
            raise RuntimeError(f"failing: {bad}")
        return f"{len(files)} files in {a.set} pass"
    ok &= item("trajectory set", traj_check)

    if a.open:
        def open_read():
            import numpy as np
            from qarm_lab.backends import make_backend
            be = make_backend("pal")
            be.open()
            try:
                s = be.read()
                return f"q[deg]={np.rad2deg(s.q[:4]).round(1)} grip={s.q[4]:.2f} I={s.current.round(2)} T={s.temp.round(0)}"
            finally:
                be.close()
        ok &= item("open arm + read once", open_read)
    print("ALL OK" if ok else "SOME CHECKS FAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

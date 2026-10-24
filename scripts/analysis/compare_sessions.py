"""Compare two run sessions trajectory by trajectory (e.g. v1 vs v2 model dry runs, or sim vs real).

Reads <session>/session.json from both (written by scripts/run.py) and reports, per trajectory: abort status,
max tracking error per joint and max |current| per joint, plus the per-joint differences.

Usage:  python scripts/analysis/compare_sessions.py <session_a> <session_b> [--labels v1 v2]
Output: results/compare_<timestamp>_<la>_vs_<lb>/{compare.md, compare.json}
"""

import argparse
import datetime as dt
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def load(session):
    runs = json.loads((Path(session) / "session.json").read_text())["runs"]
    return {r["trajectory"]: r for r in runs}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--labels", nargs=2, default=("A", "B"))
    a = ap.parse_args()
    la, lb = a.labels
    A, Bs = load(a.a), load(a.b)
    rows, out = [], {"a": a.a, "b": a.b, "labels": [la, lb], "trajectories": {}}
    lines = [f"# {la} vs {lb}\n", f"- {la}: `{a.a}`", f"- {lb}: `{a.b}`\n",
             f"| trajectory | abort {la} / {lb} | max track err [deg] {la} | {lb} | max abs current [A] {la} | {lb} |",
             "|---|---|---|---|---|---|"]
    worst_err, worst_cur = 0.0, 0.0
    for name in A:
        ra, rb = A[name], Bs.get(name)
        if rb is None:
            lines.append(f"| {name} | {ra.get('aborted')} / missing | | | | |")
            continue
        de = [round(y - x, 3) for x, y in zip(ra["max_track_err_deg"], rb["max_track_err_deg"])]
        dc = [round(y - x, 3) for x, y in zip(ra["max_abs_current_A"], rb["max_abs_current_A"])]
        worst_err = max(worst_err, max(map(abs, de)))
        worst_cur = max(worst_cur, max(map(abs, dc)))
        out["trajectories"][name] = {"abort": [ra.get("aborted"), rb.get("aborted")], "d_track_err_deg": de,
                                     "d_current_A": dc}
        lines.append(f"| {name} | {ra.get('aborted')} / {rb.get('aborted')} | {ra['max_track_err_deg']} | "
                     f"{rb['max_track_err_deg']} | {ra['max_abs_current_A']} | {rb['max_abs_current_A']} |")
    out["max_abs_diff"] = {"track_err_deg": worst_err, "current_A": worst_cur}
    lines.append(f"\nLargest per-joint difference: tracking error {worst_err:.3f} deg, current {worst_cur:.3f} A.")
    res = ROOT / "results" / f"compare_{dt.datetime.now():%Y%m%d_%H%M%S}_{la}_vs_{lb}"
    res.mkdir(parents=True)
    (res / "compare.md").write_text("\n".join(lines) + "\n")
    (res / "compare.json").write_text(json.dumps(out, indent=2))
    print("\n".join(lines))
    print("->", res)


if __name__ == "__main__":
    main()

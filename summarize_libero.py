"""
Summarize eval_libero_xe.py results: success rate per suite x robot/gripper, relative to the Panda reference.
Works on finished runs (summary.json) and on runs still in progress (episodes.jsonl).
Episodes repeated by a restarted run are counted once (the last record wins).

    python summarize_libero.py data/libero_xe
"""

import glob
import json
import os
import sys
from collections import defaultdict

ARMS = ["Panda", "UR5e", "IIWA", "Kinova3", "Jaco", "Sawyer"]
COLS = ["PG", "Rethink", "R85", "R140", "R3F", "Jaco3F"]
OWN = {"Panda": "PG", "UR5e": "R85", "IIWA": "R140", "Kinova3": "R85", "Jaco": "Jaco3F", "Sawyer": "Rethink"}


def grid_key(tag: str) -> tuple:
    """Folder tag -> (arm, gripper column); "<arm>_own" means the arm's own gripper (Panda_own = reference)."""
    arm, g = tag.split("_", 1)
    return arm, OWN.get(arm, g) if g == "own" else g


def main() -> None:
    root = sys.argv[1] if len(sys.argv) > 1 else "."
    for suite_dir in sorted(glob.glob(os.path.join(root, "*"))):
        if not os.path.isdir(suite_dir):
            continue
        suite = os.path.basename(suite_dir)
        rows = {}
        for run_dir in sorted(glob.glob(os.path.join(suite_dir, "*"))):
            path = os.path.join(run_dir, "episodes.jsonl")
            if not os.path.exists(path):
                continue
            # A restarted run appends to the same file; keep the last row per (task, episode), i.e. the finished run.
            last = {}
            for line in open(path):
                if line.strip():
                    e = json.loads(line)
                    last[(e["task_id"], e["episode"])] = e
            eps = list(last.values())
            per_task = defaultdict(list)
            for e in eps:
                per_task[e["task_id"]].append(e["success"])
            done = os.path.exists(os.path.join(run_dir, "summary.json"))
            rows[os.path.basename(run_dir)] = (eps, per_task, done)
        if not rows:
            continue
        ref = rows.get("Panda_own")
        ref_sr = sum(e["success"] for e in ref[0]) / len(ref[0]) if ref and ref[0] else None
        print(f"\n== {suite}")
        print(f"{'robot/gripper':16s} {'SR':>6s} {'n':>5s} {'vs Panda':>9s}  {'status':8s}  per task")
        for tag, (eps, per_task, done) in sorted(rows.items(), key=lambda kv: (kv[0] != "Panda_own", kv[0])):
            sr = sum(e["success"] for e in eps) / len(eps) if eps else 0.0
            rel = f"{100 * sr / ref_sr:7.0f}%" if ref_sr else "      -"
            tasks = " ".join(f"{100 * sum(v) / len(v):3.0f}" for _, v in sorted(per_task.items()))
            print(f"{tag:16s} {sr:6.2f} {len(eps):5d} {rel:>9s}  {'done' if done else 'running':8s}  {tasks}")
        cells = {}
        for tag, (eps, _, done) in rows.items():
            if eps:
                cells[grid_key(tag)] = (sum(e["success"] for e in eps) / len(eps), done)
        if len(cells) > 2:
            print(f"\n   grid ({suite}): zero-shot SR, * = arm's own gripper, ~ = still running")
            print(f"   {'arm':8s}" + "".join(f"{c:>9s}" for c in COLS))
            for arm in ARMS:
                line = f"   {arm:8s}"
                for c in COLS:
                    v = cells.get((arm, c))
                    line += f"{'-':>9s}" if v is None else f"{f'{v[0]:.2f}' + ('*' if OWN[arm] == c else ' ') + ('' if v[1] else '~'):>9s}"
                print(line)
    print("\nown = robot's own gripper, PG = Panda gripper, Rethink, R85/R140 = Robotiq 85/140, R3F = Robotiq 3-finger, "
          "Jaco3F = Jaco 3-finger (e.g. Panda_R85 = gripper-only swap); per task in %")


if __name__ == "__main__":
    main()

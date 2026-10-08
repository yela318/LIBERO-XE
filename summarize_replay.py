"""
Replay-ceiling grid per suite (arm rows x gripper columns) from replay_demos.py rows.jsonl files.

    python summarize_replay.py data/libero_xe_replay
Rows for the same (task, demo, arm, gripper) keep the last value, so re-runs do not double count.
"""

import glob
import json
import os
import sys
from collections import defaultdict

ARMS = ["Panda", "UR5e", "IIWA", "Kinova3", "Jaco", "Sawyer"]
GRIPPERS = ["PandaGripper", "RethinkGripper", "Robotiq85Gripper", "Robotiq140Gripper", "RobotiqThreeFingerGripper", "JacoThreeFingerGripper"]
SHORT = {"PandaGripper": "PG", "RethinkGripper": "Rethink", "Robotiq85Gripper": "R85", "Robotiq140Gripper": "R140",
         "RobotiqThreeFingerGripper": "R3F", "JacoThreeFingerGripper": "Jaco3F"}
DEFAULT = {"Panda": "PandaGripper", "UR5e": "Robotiq85Gripper", "IIWA": "Robotiq140Gripper",
           "Kinova3": "Robotiq85Gripper", "Jaco": "JacoThreeFingerGripper", "Sawyer": "RethinkGripper"}


def main() -> None:
    root = sys.argv[1] if len(sys.argv) > 1 else "."
    for path in sorted(glob.glob(os.path.join(root, "*", "rows.jsonl"))):
        suite = os.path.basename(os.path.dirname(path))
        last = {}
        for line in open(path):
            if line.strip():
                r = json.loads(line)
                g = DEFAULT[r["robot"]] if r["gripper"] == "default" else r["gripper"]
                last[(r["robot"], g, r["task_id"], r["demo"])] = bool(r["success"])
        cell = defaultdict(list)
        for (arm, g, _, _), s in last.items():
            cell[(arm, g)].append(s)
        print(f"\n== {suite}  (replay success rate; * = robot's own gripper; n per cell in brackets if < 100)")
        print(f"{'arm':8s}" + "".join(f"{SHORT[g]:>11s}" for g in GRIPPERS))
        for arm in ARMS:
            line = f"{arm:8s}"
            for g in GRIPPERS:
                v = cell.get((arm, g))
                if not v:
                    line += f"{'-':>11s}"
                    continue
                mark = "*" if DEFAULT[arm] == g else " "
                n = f"[{len(v)}]" if len(v) < 100 else ""
                line += f"{f'{sum(v) / len(v):.2f}{mark}{n}':>11s}"
            print(line)


if __name__ == "__main__":
    main()

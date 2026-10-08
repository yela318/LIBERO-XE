"""
Replay LIBERO Panda demonstrations on other robots: can this body reproduce the Panda's motion?
(Not an upper bound on zero-shot transfer: a closed-loop policy can beat a replay, and does on Panda itself.)

For each demo: the Panda env loads the recorded start state (demo states[0]); the target robot gets the same
object/fixture state (transfer_objects) and its gripper site is moved to the Panda's start pose (align_ee);
then the recorded Panda actions (world-frame OSC deltas + gripper) are executed. In the default --mode track
the Panda replays alongside and each action gets a correction toward the Panda's current gripper-site pose;
--mode open executes the recorded actions as they are. No cameras are rendered, so this runs on CPU.

Run in openpi's LIBERO env with the LIBERO demos downloaded (see README.md):
    python replay_demos.py --suite libero_object --robots UR5e --grippers default --demos 10
Results: <out>/<suite>_<mode>/rows.jsonl (one row per demo) and a printed table (task x robot/gripper).
"""

import argparse
import json
import os
import time

import h5py
import numpy as np
import robosuite.utils.transform_utils as T
from libero.libero import benchmark
from libero.libero import get_libero_path

import xe_libero as X


OSC_MAX_POS, OSC_MAX_ROT = 0.05, 0.5  # robosuite OSC_POSE output_max: an action of 1 moves 5 cm / 0.5 rad


def corrected_action(action, ref_pos, ref_rot, cur_pos, cur_rot) -> np.ndarray:
    """Recorded action plus the OSC delta that removes the current offset from the Panda's pose (both taken
    before the step). With zero offset this is the recorded action itself, so tracking adds no lag."""
    out = np.array(action, dtype=np.float64)
    out[0:3] += (ref_pos - cur_pos) / OSC_MAX_POS
    out[3:6] += T.quat2axisangle(T.mat2quat(ref_rot @ cur_rot.T)) / OSC_MAX_ROT
    out[0:6] = np.clip(out[0:6], -1, 1)
    return out


def run_demo(panda, env, robot, gripper, demo, mode) -> dict:
    """mode "open": the recorded Panda actions open-loop (dynamics differences accumulate as drift).
    mode "track": the Panda replays the demo alongside; each step the robot gets the recorded action plus a
    correction toward the Panda's current gripper-site pose (same gripper command), i.e. whether the arm and
    gripper can reproduce the Panda's motion once dynamics drift is removed."""
    states, actions = demo["states"][()], demo["actions"][()]
    panda.reset()
    panda.set_init_state(states[0])
    pos, rot = X.ee_pose(panda)
    env.reset()
    if X.is_reference(robot, gripper):
        env.set_init_state(states[0])
        align = {"pos_err_mm": 0.0, "rot_err_deg": 0.0}
    else:
        X.transfer_objects(panda, env)
        align = X.align_ee(env, pos, rot)
    success, err = False, []
    for t, a in enumerate(actions):
        if mode == "track":
            env.step(corrected_action(a, *X.ee_pose(panda), *X.ee_pose(env)))
        else:
            env.step(a)
        panda.step(a)
        err.append(np.linalg.norm(X.ee_pose(env)[0] - X.ee_pose(panda)[0]))
        if env.check_success():
            success = True
            break
    return {"success": success, "steps": t + 1, "demo_len": len(actions), **align,
            "track_err_mm": round(1000 * float(np.mean(err)), 1), "mode": mode}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", default="libero_10")
    ap.add_argument("--task-ids", type=int, nargs="*", default=None, help="default: all tasks of the suite")
    ap.add_argument("--demos", type=int, default=10, help="demos per task (demo_0 ...)")
    ap.add_argument("--robots", nargs="*", default=X.ROBOTS)
    ap.add_argument("--grippers", nargs="*", default=["default", "PandaGripper"],
                    help="default = each robot's own gripper; PandaGripper = arm-only swap")
    ap.add_argument("--mode", choices=["track", "open"], default="track",
                    help="track: follow the Panda gripper pose each step; open: recorded actions open-loop")
    ap.add_argument("--out", default="data/libero_xe_replay")
    a = ap.parse_args()

    suite = benchmark.get_benchmark_dict()[a.suite]()
    task_ids = a.task_ids if a.task_ids is not None else list(range(suite.n_tasks))
    out_dir = os.path.join(a.out, f"{a.suite}_{a.mode}")
    os.makedirs(out_dir, exist_ok=True)
    log = open(os.path.join(out_dir, "rows.jsonl"), "a")
    combos = list(dict.fromkeys(X.canonical(r, g) for r in a.robots for g in a.grippers))  # dedupe e.g. UR5e default = R85
    table = {}
    for tid in task_ids:
        task = suite.get_task(tid)
        bddl = os.path.join(get_libero_path("bddl_files"), task.problem_folder, task.bddl_file)
        demo_file = os.path.join(get_libero_path("datasets"), a.suite, f"{task.name}_demo.hdf5")
        panda = X.make_env(bddl, "Panda", camera_obs=False)
        with h5py.File(demo_file, "r") as f:
            names = sorted(f["data"].keys(), key=lambda n: int(n.split("_")[-1]))[: a.demos]
            for robot, gripper in combos:
                t0 = time.time()
                try:
                    env = X.make_env(bddl, robot, gripper, camera_obs=False)
                except Exception as e:  # noqa: BLE001
                    print(f"[{tid}] {robot}/{gripper}: env error {type(e).__name__}: {e}", flush=True)
                    continue
                rows = []
                for n in names:
                    try:
                        row = run_demo(panda, env, robot, gripper, f["data"][n], a.mode)
                    except Exception as e:  # noqa: BLE001
                        row = {"success": False, "error": f"{type(e).__name__}: {e}"}
                    row.update({"suite": a.suite, "task_id": tid, "task": task.language, "robot": robot,
                                "gripper": gripper, "demo": n})
                    rows.append(row)
                    log.write(json.dumps(row) + "\n")
                    log.flush()
                env.close()
                sr = np.mean([r["success"] for r in rows])
                table[(tid, robot, gripper)] = sr
                ok = [r for r in rows if "error" not in r]
                print(f"[{tid}] {robot:8s} {gripper:12s} SR {sr:.2f} | align "
                      f"{np.mean([r['pos_err_mm'] for r in ok]) if ok else float('nan'):.1f}mm "
                      f"{np.mean([r['rot_err_deg'] for r in ok]) if ok else float('nan'):.1f}deg | track "
                      f"{np.mean([r['track_err_mm'] for r in ok]) if ok else float('nan'):.1f}mm | errors "
                      f"{len(rows) - len(ok)} | {time.time() - t0:.0f}s | {task.language[:50]}", flush=True)
        panda.close()

    cols = [c for c in combos if any((t, *c) in table for t in task_ids)]
    print("\n" + f"{'task':>4s} " + " ".join(f"{X.tag(r, g).replace('_', '/')[:14]:>14s}" for r, g in cols))
    for tid in task_ids:
        print(f"{tid:>4d} " + " ".join(f"{table.get((tid, r, g), float('nan')):>14.2f}" for r, g in cols))
    print(f"{'avg':>4s} " + " ".join(f"{np.nanmean([table.get((t, r, g), np.nan) for t in task_ids]):>14.2f}" for r, g in cols))
    print("Panda/own = unmodified LIBERO robot; PG = Panda gripper, Rethink, R85/R140 = Robotiq 85/140, R3F = Robotiq 3-finger, Jaco3F")


if __name__ == "__main__":
    main()

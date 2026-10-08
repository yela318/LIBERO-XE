"""
Zero-shot cross-embodiment eval of a LIBERO policy (e.g. the official pi05_libero, trained on Panda) on other arms.

Same protocol as openpi's examples/libero/main.py (official init states, 10 settle steps with the dummy action,
180-degree image rotation, resize_with_pad 224, replan every 5 actions, per-suite max steps, env seed 7). Added:
  --robot / --gripper   robot from xe_libero.py; gripper "default" = its own, "PandaGripper" = arm-only swap
  init state            Panda gets the official init state directly. Other robots get the object/fixture state
                        of that init state (transfer_objects) and their gripper site aligned to the Panda's
                        start pose (align_ee), so the scene and the start pose match the Panda episode.
  state input           pi05_libero does not feed the state to the model (discrete_state_input=False), but the
                        input transform still normalizes an 8-D state. Non-Panda grippers report a different
                        number of joints, so they get a Panda-format placeholder for the 2 finger values
                        (open = [0.04, -0.04], closed = [0, 0], from the last gripper command).

Run in openpi's LIBERO env with a policy server running (see README.md; run_benchmark.sh runs all settings).
Results: <out>/<suite>/<robot>_<gripper tag>/episodes.jsonl, summary.json, videos/ (optional)
"""

import collections
import dataclasses
import json
import logging
import math
import os
import pathlib
import time
from typing import List
from typing import Optional

import imageio
import numpy as np
import tyro
from libero.libero import benchmark
from libero.libero import get_libero_path
from openpi_client import image_tools
from openpi_client import websocket_client_policy as _websocket_client_policy

import xe_libero as X

LIBERO_DUMMY_ACTION = [0.0] * 6 + [-1.0]
LIBERO_ENV_RESOLUTION = 256
MAX_STEPS = {"libero_spatial": 220, "libero_object": 280, "libero_goal": 300, "libero_10": 520, "libero_90": 400}


@dataclasses.dataclass
class Args:
    host: str = "127.0.0.1"
    port: int = 8000
    resize_size: int = 224
    replan_steps: int = 5
    task_suite_name: str = "libero_spatial"
    task_ids: Optional[List[int]] = None  # default: all tasks of the suite (Python 3.8: no list[int] | None)
    num_steps_wait: int = 10
    num_trials_per_task: int = 50  # official init states 0..N-1
    robot: str = "Panda"
    gripper: str = "default"  # "default" (own gripper) or a name from xe_libero.GRIPPERS, e.g. "PandaGripper"
    video_every: int = 0  # save a video every N episodes per task (0 = never)
    out: str = "data/libero_xe"
    seed: int = 7


def _quat2axisangle(quat):
    """Same as openpi's examples/libero/main.py (copied from robosuite)."""
    quat = np.array(quat, dtype=np.float64)
    quat[3] = np.clip(quat[3], -1.0, 1.0)
    den = np.sqrt(1.0 - quat[3] * quat[3])
    if math.isclose(den, 0.0):
        return np.zeros(3)
    return (quat[:3] * 2.0 * math.acos(quat[3])) / den


def _state(obs, last_gripper_cmd: float) -> np.ndarray:
    fingers = np.asarray(obs["robot0_gripper_qpos"], dtype=np.float64)
    if fingers.shape != (2,):  # non-Panda gripper: Panda-format placeholder (state is not a model input)
        fingers = np.array([0.04, -0.04]) if last_gripper_cmd < 0 else np.zeros(2)
    return np.concatenate((obs["robot0_eef_pos"], _quat2axisangle(obs["robot0_eef_quat"]), fingers))


def eval_libero(args: Args) -> None:
    np.random.seed(args.seed)
    suite = benchmark.get_benchmark_dict()[args.task_suite_name]()
    task_ids = args.task_ids if args.task_ids is not None else list(range(suite.n_tasks))
    max_steps = MAX_STEPS[args.task_suite_name]
    args.robot, args.gripper = X.canonical(args.robot, args.gripper)
    tag = X.tag(args.robot, args.gripper)
    out_dir = pathlib.Path(args.out) / args.task_suite_name / tag
    (out_dir / "videos").mkdir(parents=True, exist_ok=True)
    log = open(out_dir / "episodes.jsonl", "a")
    client = _websocket_client_policy.WebsocketClientPolicy(args.host, args.port)
    is_panda = X.is_reference(args.robot, args.gripper)

    per_task = {}
    for task_id in task_ids:
        task = suite.get_task(task_id)
        initial_states = suite.get_task_init_states(task_id)
        bddl = str(pathlib.Path(get_libero_path("bddl_files")) / task.problem_folder / task.bddl_file)
        env = X.make_env(bddl, args.robot, args.gripper, camera_obs=True, resolution=LIBERO_ENV_RESOLUTION)
        env.seed(args.seed)
        ref = None if is_panda else X.make_env(bddl, "Panda", camera_obs=False)
        successes = 0
        for ep in range(args.num_trials_per_task):
            t0 = time.time()
            env.reset()
            if is_panda:
                obs, align = env.set_init_state(initial_states[ep]), {}
            else:
                ref.reset()
                ref.set_init_state(initial_states[ep])
                X.transfer_objects(ref, env)
                align = X.align_ee(env, *X.ee_pose(ref))
                obs = X.observe(env)
            action_plan, frames, done, t, last_grip, error = collections.deque(), [], False, 0, -1.0, None
            while t < max_steps + args.num_steps_wait:
                try:
                    if t < args.num_steps_wait:
                        obs, _, done, _ = env.step(LIBERO_DUMMY_ACTION)
                        t += 1
                        continue
                    img = np.ascontiguousarray(obs["agentview_image"][::-1, ::-1])
                    wrist = np.ascontiguousarray(obs["robot0_eye_in_hand_image"][::-1, ::-1])
                    img = image_tools.convert_to_uint8(image_tools.resize_with_pad(img, args.resize_size, args.resize_size))
                    wrist = image_tools.convert_to_uint8(image_tools.resize_with_pad(wrist, args.resize_size, args.resize_size))
                    if args.video_every and ep % args.video_every == 0:
                        frames.append(np.concatenate([img, wrist], axis=1))
                    if not action_plan:
                        element = {"observation/image": img, "observation/wrist_image": wrist,
                                   "observation/state": _state(obs, last_grip), "prompt": str(task.language)}
                        chunk = client.infer(element)["actions"]
                        action_plan.extend(chunk[: args.replan_steps])
                    action = np.asarray(action_plan.popleft(), dtype=np.float64)
                    last_grip = float(action[-1])
                    obs, _, done, _ = env.step(action.tolist())
                    if done:
                        break
                    t += 1
                except Exception as e:  # noqa: BLE001
                    error = f"{type(e).__name__}: {e}"
                    logging.error(f"episode error: {error}")
                    break
            successes += int(done)
            row = {"suite": args.task_suite_name, "task_id": task_id, "task": task.language, "episode": ep,
                   "robot": args.robot, "gripper": args.gripper, "success": bool(done), "steps": t, **align,
                   "sec": round(time.time() - t0, 1), "error": error}
            log.write(json.dumps(row) + "\n")
            log.flush()
            if frames:
                imageio.mimwrite(out_dir / "videos" / f"t{task_id}_ep{ep}_{'success' if done else 'fail'}.mp4", frames, fps=10)
            print(f"[{tag}] task {task_id} ep {ep + 1}/{args.num_trials_per_task} success={done} "
                  f"steps={t} running {successes}/{ep + 1} ({time.time() - t0:.0f}s)", flush=True)
        per_task[task_id] = {"task": task.language, "successes": successes, "episodes": args.num_trials_per_task,
                             "success_rate": successes / args.num_trials_per_task}
        env.close()
        if ref is not None:
            ref.close()

    total = sum(v["successes"] for v in per_task.values())
    n = sum(v["episodes"] for v in per_task.values())
    summary = {"suite": args.task_suite_name, "robot": args.robot, "gripper": args.gripper,
               "success_rate": total / max(n, 1), "successes": total, "episodes": n, "per_task": per_task}
    with open(out_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps({k: v for k, v in summary.items() if k != "per_task"}))
    for tid, v in per_task.items():
        print(f"  task {tid}: {v['success_rate']:.2f}  {v['task']}")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    eval_libero(tyro.cli(Args))

"""
LIBERO with other robosuite arms, for cross-embodiment evaluation of Panda-trained policies.

LIBERO builds robots by name: tabletop/kitchen/study scenes use "Mounted<robot>", coffee-table/living-room/floor
scenes use "OnTheGround<robot>" (libero/libero/envs/problems/*). It only ships Panda versions. This module
registers the same two variants for the other single-arm robots of robosuite 1.4, placed exactly like LIBERO's
Panda (same base offsets and mounts), so `robots=["UR5e"]` etc. work in any LIBERO task.

LIBERO actions are OSC_POSE deltas of the gripper site in the world frame plus a gripper command, so the same
action means the same end-effector motion on every arm. Two pieces make an episode comparable across robots:
  transfer_objects  LIBERO init states / demo states are full MuJoCo states of the Panda model. Only the
                    object and fixture joints (everything not robot0_/gripper0_/mount0_) are copied by name.
  align_ee          the target arm's gripper site is moved kinematically (damped least-squares IK, no physics)
                    to the Panda's start pose, so start pose and action deltas match.

Gripper: gripper_types="default" uses each robot's own gripper (full embodiment swap); "PandaGripper" keeps the
Panda hand on the new arm (arm-only swap).

Import this module after `libero.libero.envs` and before creating environments.
"""

import numpy as np
import robosuite.utils.transform_utils as T
from robosuite.models.robots import IIWA
from robosuite.models.robots import UR5e
from robosuite.models.robots import Jaco
from robosuite.models.robots import Kinova3
from robosuite.models.robots import Sawyer
from robosuite.robots import ROBOT_CLASS_MAPPING
from robosuite.robots.single_arm import SingleArm

import libero.libero.envs  # noqa: F401  (registers MountedPanda / OnTheGroundPanda)

ARMS = {"UR5e": UR5e, "IIWA": IIWA, "Kinova3": Kinova3, "Sawyer": Sawyer, "Jaco": Jaco}
ROBOTS = ["Panda", *ARMS]

# Same placement as LIBERO's MountedPanda / OnTheGroundPanda (libero/libero/envs/robots/*.py)
_MOUNTED_OFFSETS = {
    "bins": (-0.5, -0.1, 0),
    "empty": (-0.6, 0, 0),
    "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
    "study_table": lambda table_length: (-0.25 - table_length / 2, 0, 0),
    "kitchen_table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
}
_GROUND_OFFSETS = {
    "bins": (-0.5, -0.1, 0),
    "empty": (-0.6, 0, 0),
    "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
    "coffee_table": lambda table_length: (-0.16 - table_length / 2, 0, 0.41),
    "living_room_table": lambda table_length: (-0.16 - table_length / 2, 0, 0.42),
}


def _register(name: str, base, offsets: dict, mount):
    cls = type(name, (base,), {
        "base_xpos_offset": property(lambda self: offsets),
        "default_mount": property(lambda self: mount),
        "top_offset": property(lambda self: np.array((0, 0, 1.0))),
        "_horizontal_radius": property(lambda self: 0.5),
    })  # robosuite's robot-model metaclass registers the class under its name
    ROBOT_CLASS_MAPPING[name] = SingleArm
    return cls


for _arm, _base in ARMS.items():
    _register(f"Mounted{_arm}", _base, _MOUNTED_OFFSETS, "RethinkMount")
    _register(f"OnTheGround{_arm}", _base, _GROUND_OFFSETS, None)

ROBOT_PREFIXES = ("robot0_", "gripper0_", "mount0_")

# Grippers driven by one open/close command, like the Panda gripper the LIBERO policies were trained with
# (robosuite's *Dexterous* grippers take several finger commands, the wiping gripper none).
GRIPPERS = ["PandaGripper", "RethinkGripper", "Robotiq85Gripper", "Robotiq140Gripper",
            "RobotiqThreeFingerGripper", "JacoThreeFingerGripper"]
DEFAULT_GRIPPER = {"Panda": "PandaGripper", "UR5e": "Robotiq85Gripper", "IIWA": "Robotiq140Gripper",
                   "Kinova3": "Robotiq85Gripper", "Jaco": "JacoThreeFingerGripper", "Sawyer": "RethinkGripper"}
GRIPPER_TAGS = {"PandaGripper": "PG", "RethinkGripper": "Rethink", "Robotiq85Gripper": "R85",
                "Robotiq140Gripper": "R140", "RobotiqThreeFingerGripper": "R3F", "JacoThreeFingerGripper": "Jaco3F"}


def canonical(robot: str, gripper: str) -> tuple:
    """One name per physical setup: "default" becomes the robot's own gripper name, except that Panda with the
    Panda gripper is the unmodified LIBERO robot, written (Panda, "default")."""
    g = DEFAULT_GRIPPER[robot] if gripper == "default" else gripper
    return (robot, "default") if (robot == "Panda" and g == "PandaGripper") else (robot, g)


def tag(robot: str, gripper: str) -> str:
    """Result folder / table name, e.g. Panda_own (reference), UR5e_PG (arm-only swap), Panda_R85, UR5e_R85."""
    robot, g = canonical(robot, gripper)
    return f"{robot}_{'own' if g == 'default' else GRIPPER_TAGS.get(g, g)}"


def is_reference(robot: str, gripper: str) -> bool:
    """The unmodified LIBERO robot: the official init states can be loaded directly."""
    return canonical(robot, gripper) == ("Panda", "default")


def make_env(bddl_file: str, robot: str = "Panda", gripper: str = "default", camera_obs: bool = True, resolution: int = 256):
    """camera_obs=False builds the env without any renderer (pure CPU, e.g. for replays)."""
    from libero.libero.envs import OffScreenRenderEnv
    from libero.libero.envs.env_wrapper import ControlEnv

    if camera_obs:
        return OffScreenRenderEnv(bddl_file_name=bddl_file, robots=[robot], gripper_types=gripper,
                                  camera_heights=resolution, camera_widths=resolution)
    return ControlEnv(bddl_file_name=bddl_file, robots=[robot], gripper_types=gripper, use_camera_obs=False,
                      has_renderer=False, has_offscreen_renderer=False)


def object_joints(env) -> list:
    m = env.sim.model
    names = [m.joint_id2name(i) for i in range(m.njnt)]
    return [n for n in names if n and not n.startswith(ROBOT_PREFIXES)]


def transfer_objects(src_env, dst_env) -> int:
    """Copy object/fixture joint positions from src_env (e.g. Panda with a LIBERO init state) to dst_env."""
    names = object_joints(src_env)
    for n in names:
        dst_env.sim.data.set_joint_qpos(n, np.array(src_env.sim.data.get_joint_qpos(n)))
        dst_env.sim.data.set_joint_qvel(n, np.zeros_like(np.atleast_1d(src_env.sim.data.get_joint_qvel(n))))
    dst_env.sim.forward()
    return len(names)


def ee_pose(env) -> tuple:
    """Gripper-site position and rotation matrix (the frame OSC_POSE controls)."""
    sim, robot = env.sim, env.robots[0]
    sid = sim.model.site_name2id(robot.gripper.important_sites["grip_site"])
    return sim.data.site_xpos[sid].copy(), sim.data.site_xmat[sid].reshape(3, 3).copy()


def align_ee(env, pos, rot, iters: int = 500, damping: float = 0.02) -> dict:
    """Kinematically move the arm so its gripper site reaches (pos, rot); returns the remaining errors."""
    sim, robot = env.sim, env.robots[0]
    site = robot.gripper.important_sites["grip_site"]
    sid = sim.model.site_name2id(site)
    qi, vi = robot._ref_joint_pos_indexes, robot._ref_joint_vel_indexes
    jids = [sim.model.joint_name2id(n) for n in robot.robot_model.joints]
    lo, hi = sim.model.jnt_range[jids, 0], sim.model.jnt_range[jids, 1]
    limited = sim.model.jnt_limited[jids].astype(bool)
    for _ in range(iters):
        ep = pos - sim.data.site_xpos[sid]
        er = T.quat2axisangle(T.mat2quat(rot @ sim.data.site_xmat[sid].reshape(3, 3).T))
        if np.linalg.norm(ep) < 1e-4 and np.linalg.norm(er) < 1e-3:
            break
        J = np.vstack([sim.data.get_site_jacp(site).reshape(3, -1)[:, vi], sim.data.get_site_jacr(site).reshape(3, -1)[:, vi]])
        dq = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(6), np.concatenate([ep, er]))
        q = sim.data.qpos[qi] + np.clip(dq, -0.1, 0.1)
        q = np.where(limited, np.clip(q, lo, hi), q)
        sim.data.qpos[qi] = q
        sim.forward()
    sim.data.qvel[vi] = 0.0
    sim.forward()
    robot.controller.update_initial_joints(sim.data.qpos[qi].copy())  # null-space target of OSC
    robot.controller.update(force=True)  # refresh the controller's cached ee pose before resetting its goal
    robot.controller.reset_goal()
    ep = pos - sim.data.site_xpos[sid]
    er = T.quat2axisangle(T.mat2quat(rot @ sim.data.site_xmat[sid].reshape(3, 3).T))
    return {"pos_err_mm": round(1000 * float(np.linalg.norm(ep)), 2), "rot_err_deg": round(float(np.degrees(np.linalg.norm(er))), 2)}


def observe(env) -> dict:
    """Fresh observations after setting the state by hand."""
    return env.env._get_observations(force_update=True)

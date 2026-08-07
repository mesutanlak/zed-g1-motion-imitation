"""Name-based G1 29-DOF to physical G1 EDU 23-DOF projection."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from pathlib import Path

import numpy as np
import mujoco
import mink


G1_23DOF_ORDER = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
)

LOCKED_G1_29DOF_JOINTS = (
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)


class AnatomicalElbowRegularizer:
    """Low-priority closed-form elbow-flexion target for Mink/GMR.

    Cartesian elbow and wrist targets determine an arm pose, but close to a
    straight arm their Jacobian is ill-conditioned and the shoulder/elbow
    decomposition may jump between mirrored branches.  This task contributes
    only the two hinge DoFs.  Frame tasks still determine the actual endpoint
    pose, while the law-of-cosines target selects the human-like branch.
    """

    def __init__(
        self,
        retargeter,
        *,
        cost: float = 0.05,
        nominal_fps: float = 15.0,
        max_speed_rad_s: float = 7.0,
    ) -> None:
        self.retargeter = retargeter
        self.model = retargeter.model
        self.nominal_fps = float(max(1.0, nominal_fps))
        self.max_step = float(max_speed_rad_s) / self.nominal_fps
        costs = np.zeros(self.model.nv, dtype=np.float64)
        self.joints: dict[str, tuple[int, int]] = {}
        for side in ("left", "right"):
            name = f"{side}_elbow_joint"
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise KeyError(name)
            qpos_id = int(self.model.jnt_qposadr[joint_id])
            dof_id = int(self.model.jnt_dofadr[joint_id])
            costs[dof_id] = float(max(0.0, cost))
            self.joints[side] = (joint_id, qpos_id)
        self.task = mink.PostureTask(
            self.model, cost=costs, gain=0.8, lm_damping=1.0e-3
        )
        self.previous: dict[str, float] = {}
        self.last_target_rad: dict[str, float] = {}

    def reset(self, qpos: np.ndarray | None = None) -> None:
        """Clear elbow branch history and restore a neutral posture target."""
        self.previous.clear()
        self.last_target_rad.clear()
        target = (
            self.retargeter.configuration.data.qpos.copy()
            if qpos is None
            else np.asarray(qpos, dtype=np.float64).copy()
        )
        self.task.set_target(target)

    @staticmethod
    def _flexion(
        shoulder: np.ndarray, elbow: np.ndarray, wrist: np.ndarray
    ) -> float | None:
        upper = shoulder - elbow
        fore = wrist - elbow
        denominator = float(np.linalg.norm(upper) * np.linalg.norm(fore))
        if denominator < 1.0e-9:
            return None
        interior = float(
            np.arccos(np.clip(np.dot(upper, fore) / denominator, -1.0, 1.0))
        )
        return float(np.pi - interior)

    def update(self, human_data: Mapping[str, tuple[np.ndarray, np.ndarray]]) -> None:
        target = self.retargeter.configuration.data.qpos.copy()
        for side, (joint_id, qpos_id) in self.joints.items():
            names = (
                f"{side}_shoulder", f"{side}_elbow", f"{side}_wrist"
            )
            if not all(name in human_data for name in names):
                continue
            flexion = self._flexion(
                *(np.asarray(human_data[name][0], dtype=np.float64) for name in names)
            )
            if flexion is None or not np.isfinite(flexion):
                continue
            low, high = map(float, self.model.jnt_range[joint_id])
            desired = float(np.clip(flexion, low, high))
            previous = self.previous.get(side, desired)
            desired = previous + float(
                np.clip(desired - previous, -self.max_step, self.max_step)
            )
            self.previous[side] = desired
            self.last_target_rad[side] = desired
            target[qpos_id] = desired
        self.task.set_target(target)


def official_g1_23dof_xml() -> Path:
    """Locate Unitree's unmodified official 23-DOF MuJoCo model.

    The first path is the dedicated official ``unitree_ros`` checkout used by
    the Isaac project.  The second is the official ``unitree_mujoco`` model in
    the ROS workspace.  Keeping this lookup here prevents GMR from silently
    falling back to its 29-DOF mocap model, whose extra wrist/waist links do
    not exist in the Isaac 23-DOF articulation.
    """
    candidates = (
        Path.home()
        / "g1_isaaclab_project/repos/unitree_ros/robots/g1_description/g1_23dof.xml",
        Path.home()
        / "ros2_ws/src/unitree_mujoco/unitree_robots/g1/g1_23dof.xml",
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(
        "Official Unitree G1 23-DOF XML was not found; expected one of: "
        + ", ".join(str(path) for path in candidates)
    )


def constrain_gmr_to_23dof(
    retargeter,
    epsilon: float = 1.0e-6,
    *,
    anatomical_elbows: bool = True,
    lock_waist_yaw: bool = False,
    restrict_backward_arms: bool = False,
    command_margin_rad: float = 0.041,
) -> None:
    """Lock joints absent from the official G1 23-DOF model during IK.

    Projecting a free 29-DOF solution after IK changes the achieved wrist and
    torso poses.  Constraining those six coordinates before solving makes GMR
    optimize the kinematics that Isaac and the physical 23-DOF G1 actually
    possess.
    """
    for name in LOCKED_G1_29DOF_JOINTS:
        joint_id = mujoco.mj_name2id(
            retargeter.model, mujoco.mjtObj.mjOBJ_JOINT, name
        )
        # The exact official 23-DOF model has no such joint, which is already
        # the strongest possible constraint.  This branch is retained only
        # for compatibility with old 29-DOF offline artifacts.
        if joint_id < 0:
            continue
        qpos_address = int(retargeter.model.jnt_qposadr[joint_id])
        retargeter.configuration.data.qpos[qpos_address] = 0.0
        retargeter.model.jnt_limited[joint_id] = True
        retargeter.model.jnt_range[joint_id] = (-float(epsilon), float(epsilon))
    if anatomical_elbows:
        # A Cartesian wrist/elbow target admits a mirrored IK branch. The G1
        # hardware permits limited negative elbow motion, but a human elbow
        # cannot flex through that branch. Constrain only the retarget solver;
        # the official hardware limits remain unchanged at the command safety
        # boundary.
        for name in ("left_elbow_joint", "right_elbow_joint"):
            joint_id = mujoco.mj_name2id(
                retargeter.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise KeyError(f"GMR model is missing elbow joint: {name}")
            retargeter.model.jnt_limited[joint_id] = True
            retargeter.model.jnt_range[joint_id, 0] = 0.0
    if lock_waist_yaw:
        joint_id = mujoco.mj_name2id(
            retargeter.model, mujoco.mjtObj.mjOBJ_JOINT, "waist_yaw_joint"
        )
        if joint_id < 0:
            raise KeyError("GMR model is missing waist_yaw_joint")
        qpos_address = int(retargeter.model.jnt_qposadr[joint_id])
        retargeter.configuration.data.qpos[qpos_address] = 0.0
        retargeter.model.jnt_limited[joint_id] = True
        retargeter.model.jnt_range[joint_id] = (-float(epsilon), float(epsilon))
    if restrict_backward_arms:
        # Positive shoulder pitch sends the hand behind a frontal operator.
        # Keep a small natural rear reach while excluding the ambiguous
        # arm-behind-torso IK branch produced by single-camera depth noise.
        for name in ("left_shoulder_pitch_joint", "right_shoulder_pitch_joint"):
            joint_id = mujoco.mj_name2id(
                retargeter.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise KeyError(f"GMR model is missing shoulder joint: {name}")
            retargeter.model.jnt_limited[joint_id] = True
            retargeter.model.jnt_range[joint_id, 1] = min(
                float(retargeter.model.jnt_range[joint_id, 1]), 0.75
            )
    # Put the final command boundary's soft margin into IK itself. Otherwise
    # GMR repeatedly selects an exact mechanical limit and the safety layer
    # has to clip/blend every otherwise-valid frame, causing visible lag.
    margin = float(max(0.0, command_margin_rad))
    for name in G1_23DOF_ORDER:
        joint_id = mujoco.mj_name2id(
            retargeter.model, mujoco.mjtObj.mjOBJ_JOINT, name
        )
        if joint_id < 0 or not bool(retargeter.model.jnt_limited[joint_id]):
            continue
        low, high = map(float, retargeter.model.jnt_range[joint_id])
        if high - low > 2.0 * margin:
            retargeter.model.jnt_range[joint_id] = (
                low + margin, high - margin
            )
    # ConfigurationLimit copies the ranges at construction, so rebuild it
    # after narrowing the six joints. Preserve any optional velocity limits.
    retargeter.ik_limits[0] = mink.ConfigurationLimit(retargeter.model, gain=1.0)
    mujoco.mj_forward(retargeter.model, retargeter.configuration.data)


def named_joint_values(
    joint_names: Sequence[str], joint_positions: Sequence[float]
) -> dict[str, float]:
    if len(joint_names) != len(joint_positions):
        raise ValueError("joint name/value lengths differ")
    return {str(name): float(value) for name, value in zip(joint_names, joint_positions)}


def project_29_to_23(values: Mapping[str, float]) -> np.ndarray:
    missing = [name for name in G1_23DOF_ORDER if name not in values]
    if missing:
        raise KeyError(f"GMR output is missing G1 joints: {missing}")
    return np.asarray([values[name] for name in G1_23DOF_ORDER], dtype=np.float64)


def expand_23_to_29(
    values_23: Sequence[float], target_29_order: Sequence[str]
) -> np.ndarray:
    if len(values_23) != len(G1_23DOF_ORDER):
        raise ValueError(f"expected 23 values, got {len(values_23)}")
    source = dict(zip(G1_23DOF_ORDER, map(float, values_23)))
    return np.asarray([source.get(name, 0.0) for name in target_29_order], dtype=np.float64)

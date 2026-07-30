"""Name-based G1 29-DOF to physical G1 EDU 23-DOF projection."""

from __future__ import annotations

from collections.abc import Mapping, Sequence

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


def constrain_gmr_to_23dof(retargeter, epsilon: float = 1.0e-6) -> None:
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
        if joint_id < 0:
            raise KeyError(f"GMR model is missing lock joint: {name}")
        qpos_address = int(retargeter.model.jnt_qposadr[joint_id])
        retargeter.configuration.data.qpos[qpos_address] = 0.0
        retargeter.model.jnt_limited[joint_id] = True
        retargeter.model.jnt_range[joint_id] = (-float(epsilon), float(epsilon))
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

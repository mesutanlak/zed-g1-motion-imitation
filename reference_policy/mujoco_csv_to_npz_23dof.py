"""Build a reference-motion NPZ with official Unitree G1 23-DOF MuJoCo FK.

This converter is deliberately independent of Isaac Sim startup.  Body and
joint arrays carry names and are mapped by name in the Isaac training task.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np


JOINT_ORDER = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint",
]


def normalized_quaternions(values: np.ndarray) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64).copy()
    for index in range(1, len(result)):
        if np.dot(result[index - 1], result[index]) < 0:
            result[index] *= -1.0
    result /= np.maximum(np.linalg.norm(result, axis=-1, keepdims=True), 1.0e-12)
    return result


def quat_mul(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.moveaxis(first, -1, 0)
    bw, bx, by, bz = np.moveaxis(second, -1, 0)
    return np.stack(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ),
        axis=-1,
    )


def angular_velocity(quaternions: np.ndarray, dt: float) -> np.ndarray:
    quaternions = normalized_quaternions(quaternions)
    previous = quaternions[:-2]
    following = quaternions[2:]
    conjugate = previous.copy()
    conjugate[:, 1:] *= -1.0
    relative = normalized_quaternions(quat_mul(following, conjugate))
    xyz_norm = np.linalg.norm(relative[:, 1:], axis=-1)
    angle = 2.0 * np.arctan2(xyz_norm, np.clip(relative[:, 0], -1.0, 1.0))
    axis = relative[:, 1:] / np.maximum(xyz_norm[:, None], 1.0e-12)
    omega = axis * (angle / (2.0 * dt))[:, None]
    return np.concatenate((omega[:1], omega, omega[-1:]), axis=0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-file", type=Path, required=True)
    parser.add_argument("--output-file", type=Path, required=True)
    parser.add_argument("--metadata-file", type=Path)
    parser.add_argument("--robot-xml", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=50.0)
    args = parser.parse_args()

    motion = np.loadtxt(args.input_file, delimiter=",", ndmin=2)
    if motion.shape[1] != 30 or len(motion) < 3 or not np.isfinite(motion).all():
        raise ValueError("input CSV must be finite Nx30")
    model = mujoco.MjModel.from_xml_path(str(args.robot_xml.resolve()))
    data = mujoco.MjData(model)
    body_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, index) for index in range(1, model.nbody)]
    if any(name is None for name in body_names):
        raise RuntimeError("official robot XML contains an unnamed body")
    joint_addresses = []
    for name in JOINT_ORDER:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"joint missing in robot XML: {name}")
        joint_addresses.append(int(model.jnt_qposadr[joint_id]))

    root_pos = motion[:, :3]
    root_quat = normalized_quaternions(motion[:, 3:7][:, [3, 0, 1, 2]])
    joints = motion[:, 7:]
    body_pos, body_quat = [], []
    for frame in range(len(motion)):
        data.qpos[:] = 0.0
        data.qpos[:3] = root_pos[frame]
        data.qpos[3:7] = root_quat[frame]
        data.qpos[joint_addresses] = joints[frame]
        mujoco.mj_forward(model, data)
        body_pos.append(data.xpos[1:].copy())
        body_quat.append(data.xquat[1:].copy())
    body_pos = np.stack(body_pos)
    body_quat = normalized_quaternions(np.stack(body_quat).reshape(-1, 4)).reshape(len(motion), -1, 4)

    dt = 1.0 / args.fps
    joint_vel = np.gradient(joints, dt, axis=0)
    body_lin_vel = np.gradient(body_pos, dt, axis=0)
    body_ang_vel = np.stack([angular_velocity(body_quat[:, index], dt) for index in range(body_quat.shape[1])], axis=1)

    metadata = {
        "reference_confidence": np.ones(len(motion), dtype=np.float32),
        "phase": np.linspace(0.0, 1.0, len(motion), dtype=np.float32),
        "foot_contact": np.ones((len(motion), 2), dtype=np.float32),
        "source_timestamp_ns": np.arange(len(motion), dtype=np.int64) * int(round(1.0e9 / args.fps)),
    }
    if args.metadata_file:
        source = np.load(args.metadata_file, allow_pickle=False)
        for key in tuple(metadata):
            if key in source.files:
                value = np.asarray(source[key])
                if len(value) != len(motion):
                    raise ValueError(f"metadata {key} has wrong frame count")
                metadata[key] = value

    args.output_file.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output_file,
        schema=np.asarray("g1_23dof_reference_motion/v1"),
        fps=np.asarray([args.fps], dtype=np.float32),
        joint_names=np.asarray(JOINT_ORDER),
        body_names=np.asarray(body_names),
        joint_pos=joints.astype(np.float32),
        joint_vel=joint_vel.astype(np.float32),
        body_pos_w=body_pos.astype(np.float32),
        body_quat_w=body_quat.astype(np.float32),
        body_lin_vel_w=body_lin_vel.astype(np.float32),
        body_ang_vel_w=body_ang_vel.astype(np.float32),
        source_csv=np.asarray(str(args.input_file.resolve())),
        **metadata,
    )
    print(
        f"MUJOCO_REFERENCE_NPZ_OK frames={len(motion)} joints={len(JOINT_ORDER)} "
        f"bodies={len(body_names)} output={args.output_file.resolve()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

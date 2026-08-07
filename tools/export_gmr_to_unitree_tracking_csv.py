"""Export deterministic GMR output to Unitree's official 23-DOF motion CSV.

The Unitree converter expects one row per fixed-rate sample:
  root xyz, root quaternion xyzw, 23 joint positions.
This tool performs timestamp-aware interpolation and quaternion nlerp, so
camera packet jitter is not baked into the physics-policy reference motion.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def normalized_quaternions_wxyz(values: np.ndarray) -> np.ndarray:
    output = np.asarray(values, dtype=np.float64).copy()
    for index in range(1, len(output)):
        if float(np.dot(output[index - 1], output[index])) < 0.0:
            output[index] *= -1.0
    norm = np.linalg.norm(output, axis=1, keepdims=True)
    return output / np.maximum(norm, 1.0e-12)


def interpolate_columns(t: np.ndarray, values: np.ndarray, target: np.ndarray) -> np.ndarray:
    return np.column_stack(
        [np.interp(target, t, values[:, column]) for column in range(values.shape[1])]
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_npz", type=Path)
    parser.add_argument("output_csv", type=Path)
    parser.add_argument("--output-fps", type=float, default=30.0)
    args = parser.parse_args()

    source = np.load(args.input_npz)
    timestamps = np.asarray(source["timestamp_ns"], dtype=np.int64)
    qpos = np.asarray(source["q_g1_full_qpos"], dtype=np.float64)
    joints = np.asarray(source["q_g1_23dof"], dtype=np.float64)
    if len(timestamps) < 2 or qpos.shape[1] < 7 or joints.shape[1] != 23:
        raise ValueError("expected at least two frames, floating root and 23 joints")
    valid = np.concatenate(([True], np.diff(timestamps) > 0))
    timestamps, qpos, joints = timestamps[valid], qpos[valid], joints[valid]
    time_s = (timestamps - timestamps[0]).astype(np.float64) * 1.0e-9
    target_s = np.arange(0.0, time_s[-1] + 0.5 / args.output_fps, 1.0 / args.output_fps)

    root_xyz = interpolate_columns(time_s, qpos[:, :3], target_s)
    quat_wxyz = normalized_quaternions_wxyz(qpos[:, 3:7])
    quat_interp = normalized_quaternions_wxyz(
        interpolate_columns(time_s, quat_wxyz, target_s)
    )
    quat_xyzw = quat_interp[:, [1, 2, 3, 0]]
    joint_interp = interpolate_columns(time_s, joints, target_s)
    output = np.column_stack((root_xyz, quat_xyzw, joint_interp))
    if not np.isfinite(output).all() or output.shape[1] != 30:
        raise RuntimeError("non-finite or malformed Unitree motion CSV")
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.output_csv, output, delimiter=",", fmt="%.9f")
    print(
        f"UNITREE_TRACKING_CSV_OK input_frames={len(timestamps)} "
        f"output_frames={len(output)} fps={args.output_fps:g} "
        f"duration_s={target_s[-1]:.3f} output={args.output_csv.resolve()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

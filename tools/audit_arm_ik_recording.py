"""Audit human-down arm frames against offline G1 23-DOF FK output."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from motion_pipeline.collision_geometry import upper_body_capsule_report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("jsonl", type=Path)
    parser.add_argument("npz", type=Path)
    parser.add_argument("--model", type=Path, required=True)
    return parser.parse_args()


def direction_error_deg(first: np.ndarray, second: np.ndarray) -> float:
    first = first / max(float(np.linalg.norm(first)), 1.0e-9)
    second = second / max(float(np.linalg.norm(second)), 1.0e-9)
    return float(np.degrees(np.arctan2(
        np.linalg.norm(np.cross(first, second)),
        np.clip(np.dot(first, second), -1.0, 1.0),
    )))


def main() -> int:
    args = parse_args()
    result = np.load(args.npz)
    source_ids = {int(value) for value in result["source_frame_index"]}
    human_down: dict[int, dict[str, bool]] = {}
    human_directions: dict[int, dict[str, tuple[np.ndarray, np.ndarray]]] = {}
    human_chains: dict[int, dict[str, np.ndarray]] = {}
    with args.jsonl.open("r", encoding="utf-8") as stream:
        for line in stream:
            frame = json.loads(line)
            frame_id = int(frame.get("frame_index", -1))
            if frame_id not in source_ids:
                continue
            names = frame.get("keypoint_names") or []
            index = {name: idx for idx, name in enumerate(names)}
            points = (frame.get("pelvis_frame") or {}).get("keypoints_m") or []
            values = {}
            directions = {}
            chains = {}
            for side in ("left", "right"):
                prefix = side.upper()
                try:
                    shoulder = np.asarray(points[index[f"{prefix}_SHOULDER"]], float)
                    elbow = np.asarray(points[index[f"{prefix}_ELBOW"]], float)
                    wrist = np.asarray(points[index[f"{prefix}_WRIST"]], float)
                    finite = np.isfinite(np.concatenate((shoulder, elbow, wrist))).all()
                    values[side] = bool(
                        finite
                        and elbow[2] < shoulder[2] - 0.10
                        and wrist[2] < elbow[2] - 0.07
                    )
                    if finite:
                        directions[side] = (elbow - shoulder, wrist - elbow)
                        chains[side] = np.stack((shoulder, elbow, wrist))
                except (KeyError, IndexError, TypeError, ValueError):
                    values[side] = False
            human_down[frame_id] = values
            human_directions[frame_id] = directions
            human_chains[frame_id] = chains

    model = mujoco.MjModel.from_xml_path(str(args.model))
    data = mujoco.MjData(model)
    fixed_base_qpos = data.qpos.copy()
    body_ids = {
        side: [
            mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_{suffix}"
            )
            for suffix in (
                "shoulder_pitch_link", "elbow_link", "wrist_roll_rubber_hand"
            )
        ]
        for side in ("left", "right")
    }
    counts = {side: [0, 0] for side in body_ids}
    vertical = {side: [] for side in body_ids}
    direction_errors = {
        side: {"upper": [], "forearm": []} for side in body_ids
    }
    static_robot_steps = {side: [] for side in body_ids}
    q23_names = [str(name) for name in result["joint_names_23"]]
    arm_q23_indices = {
        side: [
            q23_names.index(f"{side}_{name}_joint")
            for name in (
                "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow"
            )
        ]
        for side in body_ids
    }
    previous_frame_id: int | None = None
    previous_q23: np.ndarray | None = None
    collision_margins: list[float] = []
    collision_body_names = (
        "pelvis", "torso_link",
        "left_shoulder_pitch_link", "left_elbow_link",
        "left_wrist_roll_rubber_hand",
        "right_shoulder_pitch_link", "right_elbow_link",
        "right_wrist_roll_rubber_hand",
    )
    collision_body_ids = {
        name: mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, name
        )
        for name in collision_body_names
    }
    for frame_id, qpos, q23 in zip(
        result["source_frame_index"], result["q_g1_full_qpos"],
        result["q_g1_23dof"],
    ):
        # Isaac upper-body mode has a fixed identity base.  GMR's floating
        # root is intentionally not transmitted, so auditing full_qpos would
        # hide precisely the frame mismatch that caused the 164136 failure.
        data.qpos[:] = fixed_base_qpos
        for name, value in zip(q23_names, q23):
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            data.qpos[int(model.jnt_qposadr[joint_id])] = value
        mujoco.mj_forward(model, data)
        collision_report = upper_body_capsule_report({
            name: data.xpos[body_id]
            for name, body_id in collision_body_ids.items()
        })
        collision_margins.append(collision_report.minimum_margin_m)
        for side, ids in body_ids.items():
            shoulder, elbow, wrist = (data.xpos[body_id] for body_id in ids)
            upper = (elbow - shoulder) / np.linalg.norm(elbow - shoulder)
            fore = (wrist - elbow) / np.linalg.norm(wrist - elbow)
            human = human_directions.get(int(frame_id), {}).get(side)
            if human is not None:
                direction_errors[side]["upper"].append(
                    direction_error_deg(human[0], upper)
                )
                direction_errors[side]["forearm"].append(
                    direction_error_deg(human[1], fore)
                )
            if not human_down.get(int(frame_id), {}).get(side, False):
                continue
            good = bool(upper[2] < -0.70 and fore[2] < -0.50)
            counts[side][0] += 1
            counts[side][1] += int(good)
            vertical[side].append((float(upper[2]), float(fore[2])))
        current_frame_id = int(frame_id)
        if previous_frame_id is not None and previous_q23 is not None:
            for side in body_ids:
                previous_chain = human_chains.get(previous_frame_id, {}).get(side)
                current_chain = human_chains.get(current_frame_id, {}).get(side)
                if previous_chain is None or current_chain is None:
                    continue
                direction_change = max(
                    direction_error_deg(
                        previous_chain[1] - previous_chain[0],
                        current_chain[1] - current_chain[0],
                    ),
                    direction_error_deg(
                        previous_chain[2] - previous_chain[1],
                        current_chain[2] - current_chain[1],
                    ),
                )
                endpoint_change = float(np.linalg.norm(
                    (current_chain[2] - current_chain[0])
                    - (previous_chain[2] - previous_chain[0])
                ))
                if direction_change <= 2.0 and endpoint_change <= 0.015:
                    indices = arm_q23_indices[side]
                    static_robot_steps[side].append(float(np.linalg.norm(
                        q23[indices] - previous_q23[indices]
                    )))
        previous_frame_id = current_frame_id
        previous_q23 = np.asarray(q23, dtype=float)

    for side in ("left", "right"):
        total, good = counts[side]
        values = np.asarray(vertical[side], dtype=float)
        ratio = good / total if total else 0.0
        median = np.median(values, axis=0) if len(values) else [np.nan, np.nan]
        print(
            f"{side}: human_down={total} g1_down={good} ratio={ratio:.3f} "
            f"median_upper_z={median[0]:.3f} median_fore_z={median[1]:.3f}"
        )
        for segment in ("upper", "forearm"):
            errors = np.asarray(direction_errors[side][segment], dtype=float)
            print(
                f"  {segment}_direction_error_deg "
                f"p50={np.percentile(errors, 50):.3f} "
                f"p90={np.percentile(errors, 90):.3f} "
                f"p99={np.percentile(errors, 99):.3f} "
                f"max={np.max(errors):.3f}"
            )
        static_steps = np.asarray(static_robot_steps[side], dtype=float)
        print(
            f"  static_human_robot_step_rad count={len(static_steps)} "
            f"p50={np.percentile(static_steps, 50):.4f} "
            f"p90={np.percentile(static_steps, 90):.4f} "
            f"p99={np.percentile(static_steps, 99):.4f} "
            f"over_0.08={np.count_nonzero(static_steps > 0.08)}"
        )
    margins = np.asarray(collision_margins, dtype=float)
    print(
        "fixed_base_collision_margin_m "
        f"p10={np.percentile(margins, 10):.4f} "
        f"p50={np.percentile(margins, 50):.4f} "
        f"warning_frames={np.count_nonzero(margins < 0.005)} "
        f"severe_frames={np.count_nonzero(margins < -0.020)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

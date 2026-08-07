"""Offline arm-direction diagnostics for a BODY_38/GMR replay."""

from __future__ import annotations

import argparse
from collections import Counter
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
BRIDGE_ROOT = PROJECT_ROOT / "isaaclab_bridge"
if str(BRIDGE_ROOT) not in sys.path:
    sys.path.insert(0, str(BRIDGE_ROOT))

from body38_to_gmr import Body38ToGMR  # noqa: E402


def body_position(model, data, name: str) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise KeyError(name)
    return np.asarray(data.xpos[body_id], dtype=np.float64).copy()


def wrist_body_name(model: mujoco.MjModel, side: str) -> str:
    """Select the endpoint of the loaded official G1 variant."""
    official_23 = f"{side}_wrist_roll_rubber_hand"
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, official_23) >= 0:
        return official_23
    return f"{side}_wrist_yaw_link"


def angle_deg(first: np.ndarray, second: np.ndarray) -> float:
    denominator = max(float(np.linalg.norm(first) * np.linalg.norm(second)), 1e-9)
    return float(np.degrees(np.arccos(np.clip(np.dot(first, second) / denominator, -1.0, 1.0))))


def percentile(values: list[float], q: float) -> float | None:
    return float(np.percentile(values, q)) if values else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("recording", type=Path)
    parser.add_argument("retarget_npz", type=Path)
    parser.add_argument("--g1-xml", type=Path, required=True)
    args = parser.parse_args()

    source: dict[int, dict] = {}
    with args.recording.open("r", encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            if record.get("schema") == "zed_body38_g1_reference/v1":
                source[int(record["frame_index"])] = record

    result = np.load(args.retarget_npz)
    indices = result["source_frame_index"].astype(int)
    joint_names = [str(name) for name in result["joint_names_29"]]
    q_rows = np.asarray(result["q_g1_29dof"], dtype=np.float64)
    full_qpos_rows = (
        np.asarray(result["q_g1_full_qpos"], dtype=np.float64)
        if "q_g1_full_qpos" in result.files
        else None
    )
    model = mujoco.MjModel.from_xml_path(str(args.g1_xml))
    data = mujoco.MjData(model)
    q_addresses = []
    for name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        q_addresses.append(int(model.jnt_qposadr[joint_id]))

    adapter = Body38ToGMR(fixed_stance=True)
    direction_errors: list[float] = []
    endpoint_errors: list[float] = []
    robot_behind_m: list[float] = []
    human_behind_m: list[float] = []
    raised_frames = 0
    excessive_behind = 0
    sample: dict[str, object] | None = None
    contact_pairs: Counter[str] = Counter()
    contact_min_distance: dict[str, float] = {}
    for row_number, (frame_index, q) in enumerate(zip(indices, q_rows)):
        record = source.get(int(frame_index))
        if record is None:
            continue
        adapted = adapter.adapt(record)
        if adapted is None:
            continue
        if full_qpos_rows is not None:
            data.qpos[:] = full_qpos_rows[row_number]
        for address, value in zip(q_addresses, q):
            data.qpos[address] = float(value)
        mujoco.mj_forward(model, data)
        for contact in data.contact:
            first_body = int(model.geom_bodyid[int(contact.geom1)])
            second_body = int(model.geom_bodyid[int(contact.geom2)])
            if first_body <= 0 or second_body <= 0 or first_body == second_body:
                continue
            first_name = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_BODY, first_body
            ) or str(first_body)
            second_name = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_BODY, second_body
            ) or str(second_body)
            pair = " | ".join(sorted((first_name, second_name)))
            contact_pairs[pair] += 1
            contact_min_distance[pair] = min(
                contact_min_distance.get(pair, float("inf")), float(contact.dist)
            )
        for side in ("left", "right"):
            required = {
                f"{side}_shoulder", f"{side}_elbow", f"{side}_wrist"
            }
            if not required.issubset(adapted.human_data):
                continue
            human_shoulder = np.asarray(adapted.human_data[f"{side}_shoulder"][0])
            human_elbow = np.asarray(adapted.human_data[f"{side}_elbow"][0])
            human_wrist = np.asarray(adapted.human_data[f"{side}_wrist"][0])
            robot_shoulder = body_position(model, data, f"{side}_shoulder_pitch_link")
            robot_elbow = body_position(model, data, f"{side}_elbow_link")
            robot_wrist = body_position(model, data, wrist_body_name(model, side))
            direction_errors.extend(
                [
                    angle_deg(human_elbow - human_shoulder, robot_elbow - robot_shoulder),
                    angle_deg(human_wrist - human_elbow, robot_wrist - robot_elbow),
                ]
            )
            endpoint_errors.append(float(np.linalg.norm(human_wrist - robot_wrist)))
            human_back = float(max(0.0, human_shoulder[0] - human_wrist[0]))
            robot_back = float(max(0.0, robot_shoulder[0] - robot_wrist[0]))
            human_behind_m.append(human_back)
            robot_behind_m.append(robot_back)
            if human_wrist[2] > human_shoulder[2] + 0.15:
                raised_frames += 1
                if robot_back > human_back + 0.06:
                    excessive_behind += 1
                if sample is None:
                    sample = {
                        "frame": int(frame_index),
                        "side": side,
                        "human_shoulder": human_shoulder.tolist(),
                        "human_elbow": human_elbow.tolist(),
                        "human_wrist": human_wrist.tolist(),
                        "robot_shoulder": robot_shoulder.tolist(),
                        "robot_elbow": robot_elbow.tolist(),
                        "robot_wrist": robot_wrist.tolist(),
                    }

    summary = {
        "evaluated_frames": int(len(indices)),
        "raised_arm_samples": raised_frames,
        "limb_direction_error_deg": {
            "p50": percentile(direction_errors, 50),
            "p95": percentile(direction_errors, 95),
            "max": max(direction_errors) if direction_errors else None,
        },
        "wrist_position_error_m": {
            "p50": percentile(endpoint_errors, 50),
            "p95": percentile(endpoint_errors, 95),
            "max": max(endpoint_errors) if endpoint_errors else None,
        },
        "human_wrist_behind_shoulder_m_p95": percentile(human_behind_m, 95),
        "robot_wrist_behind_shoulder_m_p95": percentile(robot_behind_m, 95),
        "raised_excessive_robot_behind_ratio": (
            excessive_behind / raised_frames if raised_frames else None
        ),
        "first_raised_sample": sample,
        "most_common_contact_pairs": contact_pairs.most_common(12),
        "contact_min_distance_m": {
            pair: contact_min_distance[pair]
            for pair, _ in contact_pairs.most_common(12)
        },
    }
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

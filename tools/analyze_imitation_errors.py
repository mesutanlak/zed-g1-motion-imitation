"""Frame-aligned ZED BODY_38 -> G1 retargeting error analysis.

This is an offline diagnostic only.  It never opens Unitree DDS channels and
never writes robot commands.  It compares the observed human skeleton with
the forward kinematics of a GMR NPZ generated from the same JSONL session.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


def finite_point(value: Any) -> np.ndarray | None:
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.isfinite(array).all():
        return None
    return array


def interior_deg(first: np.ndarray, vertex: np.ndarray, third: np.ndarray) -> float:
    first_ray = first - vertex
    second_ray = third - vertex
    denominator = float(np.linalg.norm(first_ray) * np.linalg.norm(second_ray))
    if denominator < 1.0e-9:
        return float("nan")
    cosine = float(np.clip(np.dot(first_ray, second_ray) / denominator, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def direction_error_deg(first: np.ndarray, second: np.ndarray) -> float:
    denominator = float(np.linalg.norm(first) * np.linalg.norm(second))
    if denominator < 1.0e-9:
        return float("nan")
    cosine = float(np.clip(np.dot(first, second) / denominator, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def load_source(path: Path) -> dict[int, dict[str, Any]]:
    result: dict[int, dict[str, Any]] = {}
    with path.open("r", encoding="utf-8-sig") as stream:
        for line in stream:
            if not line.strip():
                continue
            record = json.loads(line)
            if record.get("schema", "").endswith("/metadata/v1"):
                continue
            if "source_packet" in record:
                record = record["source_packet"]
            if "keypoint_names" not in record:
                continue
            sequence = int(record.get("frame_index", record.get("sequence", -1)))
            result[sequence] = record
    return result


def points_by_name(record: dict[str, Any]) -> dict[str, np.ndarray]:
    names = [str(name) for name in record.get("keypoint_names", [])]
    values = (
        (record.get("pelvis_frame") or {}).get("keypoints_m")
        or record.get("keypoints_3d_filtered_m")
        or record.get("keypoints_3d_m")
        or []
    )
    output: dict[str, np.ndarray] = {}
    for index, name in enumerate(names):
        if index < len(values) and (point := finite_point(values[index])) is not None:
            output[name] = point
    return output


def body_position(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise KeyError(name)
    return np.asarray(data.xpos[body_id], dtype=float).copy()


def percentile(values: list[float], q: float) -> float | None:
    finite = [value for value in values if math.isfinite(value)]
    return float(np.percentile(finite, q)) if finite else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source_jsonl", type=Path)
    parser.add_argument("gmr_npz", type=Path)
    parser.add_argument("--g1-xml", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--output-summary", type=Path, required=True)
    args = parser.parse_args()

    source = load_source(args.source_jsonl)
    result = np.load(args.gmr_npz)
    indices = np.asarray(result["source_frame_index"], dtype=int)
    timestamps = np.asarray(result["timestamp_ns"], dtype=np.int64)
    qpos_rows = np.asarray(result["q_g1_full_qpos"], dtype=float)
    joint_names = [str(name) for name in result["joint_names_23"]]
    q23_rows = np.asarray(result["q_g1_23dof"], dtype=float)

    model = mujoco.MjModel.from_xml_path(str(args.g1_xml))
    data = mujoco.MjData(model)
    rows: list[dict[str, Any]] = []
    for sequence, timestamp_ns, qpos, q23 in zip(
        indices, timestamps, qpos_rows, q23_rows
    ):
        record = source.get(int(sequence))
        if record is None:
            continue
        human = points_by_name(record)
        data.qpos[:] = qpos
        mujoco.mj_forward(model, data)
        row: dict[str, Any] = {
            "source_sequence": int(sequence),
            "timestamp_ns": int(timestamp_ns),
            "body_confidence": record.get("body_confidence"),
            "valid_keypoints": len(human),
        }
        for side in ("left", "right"):
            prefix = side.upper()
            required = (
                f"{prefix}_SHOULDER", f"{prefix}_ELBOW", f"{prefix}_WRIST"
            )
            if all(name in human for name in required):
                hs, he, hw = (human[name] for name in required)
                human_angle = interior_deg(hs, he, hw)
            else:
                hs = he = hw = None
                human_angle = float("nan")
            rs = body_position(model, data, f"{side}_shoulder_pitch_link")
            re = body_position(model, data, f"{side}_elbow_link")
            rw = body_position(model, data, f"{side}_wrist_roll_rubber_hand")
            robot_angle = interior_deg(rs, re, rw)
            row[f"{side}_human_elbow_interior_deg"] = human_angle
            row[f"{side}_g1_fk_elbow_interior_deg"] = robot_angle
            row[f"{side}_elbow_abs_error_deg"] = abs(robot_angle - human_angle)
            row[f"{side}_elbow_joint_rad"] = float(
                q23[joint_names.index(f"{side}_elbow_joint")]
            )
            if hs is not None:
                row[f"{side}_upper_direction_error_deg"] = direction_error_deg(
                    he - hs, re - rs
                )
                row[f"{side}_forearm_direction_error_deg"] = direction_error_deg(
                    hw - he, rw - re
                )
                human_reach = max(float(np.linalg.norm(hw - hs)), 1.0e-9)
                robot_reach = float(np.linalg.norm(rw - rs))
                row[f"{side}_normalized_reach_error"] = abs(
                    robot_reach / 0.2935 - human_reach /
                    max(
                        float(np.linalg.norm(he - hs) + np.linalg.norm(hw - he)),
                        1.0e-9,
                    )
                )
        rows.append(row)

    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({field for row in rows for field in row})
    with args.output_csv.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    summary: dict[str, Any] = {"evaluated_frames": len(rows)}
    for side in ("left", "right"):
        error = [
            float(row.get(f"{side}_elbow_abs_error_deg", float("nan")))
            for row in rows
        ]
        upper = [
            float(row.get(f"{side}_upper_direction_error_deg", float("nan")))
            for row in rows
        ]
        fore = [
            float(row.get(f"{side}_forearm_direction_error_deg", float("nan")))
            for row in rows
        ]
        summary[side] = {
            "elbow_abs_error_deg_p50": percentile(error, 50),
            "elbow_abs_error_deg_p95": percentile(error, 95),
            "elbow_abs_error_deg_max": percentile(error, 100),
            "upper_direction_error_deg_p50": percentile(upper, 50),
            "upper_direction_error_deg_p95": percentile(upper, 95),
            "forearm_direction_error_deg_p50": percentile(fore, 50),
            "forearm_direction_error_deg_p95": percentile(fore, 95),
            "frames_over_20deg": int(
                sum(math.isfinite(value) and value > 20.0 for value in error)
            ),
        }
    args.output_summary.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

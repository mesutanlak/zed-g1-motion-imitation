"""Compare normal IK and policy-powered sections of a Rerun recording."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from motion_pipeline.reference_policy import policy_collision_action_scale_numpy


METRICS = (
    "joint_tracking_rmse_rad",
    "body_tracking_mpjpe_m",
    "left_hand_position_error_m",
    "right_hand_position_error_m",
    "left_elbow_position_error_m",
    "right_elbow_position_error_m",
    "target_jerk_rms_rad_s3",
    "reference_target_error_rms_rad",
    "total_control_ms",
    "policy_raw_action_saturation_rate",
    "policy_clipped_unclipped_l1",
    "policy_residual_rms_rad",
)


def _number(value):
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except (TypeError, ValueError):
        return None


def _stats(values: list[float]) -> dict:
    if not values:
        return {"count": 0}
    array = np.asarray(values, dtype=float)
    return {
        "count": int(array.size),
        "min": float(array.min()),
        "p01": float(np.quantile(array, 0.01)),
        "p10": float(np.quantile(array, 0.10)),
        "mean": float(array.mean()),
        "p50": float(np.quantile(array, 0.50)),
        "p90": float(np.quantile(array, 0.90)),
        "p99": float(np.quantile(array, 0.99)),
        "max": float(array.max()),
    }


def _distance(points: dict, first: str, second: str) -> float | None:
    if first not in points or second not in points:
        return None
    a = np.asarray(points[first], dtype=float)
    b = np.asarray(points[second], dtype=float)
    if a.shape != (3,) or b.shape != (3,) or not np.isfinite(a).all() or not np.isfinite(b).all():
        return None
    return float(np.linalg.norm(a - b))


def analyze(csv_path: Path, jsonl_path: Path) -> dict:
    groups: dict[str, list[dict]] = {"normal_ik": [], "policy_powered": []}
    with csv_path.open(encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            mode = row.get("policy_control_mode")
            if mode in groups:
                groups[mode].append(row)
    output = {"schema": "g1_live_policy_comparison/v1", "modes": {}}
    for mode, rows in groups.items():
        output["modes"][mode] = {
            "frames": len(rows),
            "sequence_first": int(rows[0]["sequence"]) if rows else None,
            "sequence_last": int(rows[-1]["sequence"]) if rows else None,
            "safety_levels": dict(Counter(row.get("safety_level") or "UNKNOWN" for row in rows)),
            "metrics": {
                metric: _stats([value for row in rows if (value := _number(row.get(metric))) is not None])
                for metric in METRICS
            },
        }

    clearances: dict[str, dict[str, list[float]]] = {
        mode: {name: [] for name in (
            "left_wrist_torso_m", "right_wrist_torso_m",
            "left_elbow_torso_m", "right_elbow_torso_m",
            "wrist_to_wrist_m", "elbow_to_elbow_m",
        )}
        for mode in groups
    }
    risk_margins = {
        "left_wrist_torso_m": 0.12,
        "right_wrist_torso_m": 0.12,
        "left_elbow_torso_m": 0.14,
        "right_elbow_torso_m": 0.14,
        "wrist_to_wrist_m": 0.12,
        "elbow_to_elbow_m": 0.16,
    }
    policy_induced_events: list[dict] = []
    guard_left_scales: list[float] = []
    guard_right_scales: list[float] = []
    guard_active_frames = 0
    with jsonl_path.open(encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            packet = record.get("telemetry_packet") or {}
            mode = (packet.get("policy_observation_v1") or {}).get("control_mode")
            if mode not in clearances:
                continue
            points = (packet.get("g1_skeleton") or {}).get("actual_positions_m") or {}
            safe_points = (packet.get("g1_skeleton") or {}).get("safe_positions_m") or {}
            guard = policy_collision_action_scale_numpy(safe_points, points)
            if mode == "policy_powered":
                guard_left_scales.append(float(guard["left_arm_scale"]))
                guard_right_scales.append(float(guard["right_arm_scale"]))
                guard_active_frames += int(bool(guard["active"]))
            pairs = {
                "left_wrist_torso_m": ("left_wrist_roll_rubber_hand", "torso_link"),
                "right_wrist_torso_m": ("right_wrist_roll_rubber_hand", "torso_link"),
                "left_elbow_torso_m": ("left_elbow_link", "torso_link"),
                "right_elbow_torso_m": ("right_elbow_link", "torso_link"),
                "wrist_to_wrist_m": ("left_wrist_roll_rubber_hand", "right_wrist_roll_rubber_hand"),
                "elbow_to_elbow_m": ("left_elbow_link", "right_elbow_link"),
            }
            for name, pair in pairs.items():
                value = _distance(points, *pair)
                if value is not None:
                    clearances[mode][name].append(value)
                    safe_value = _distance(safe_points, *pair)
                    if (
                        mode == "policy_powered"
                        and safe_value is not None
                        and value < risk_margins[name]
                        and safe_value >= risk_margins[name]
                    ):
                        policy = packet.get("policy_observation_v1") or {}
                        policy_induced_events.append(
                            {
                                "sequence": packet.get("sequence"),
                                "pair": name,
                                "safe_clearance_m": safe_value,
                                "actual_clearance_m": value,
                                "residual_rad": policy.get("residual_rad"),
                                "raw_action": policy.get("raw_action"),
                                "collision_guard_left_scale": guard["left_arm_scale"],
                                "collision_guard_right_scale": guard["right_arm_scale"],
                            }
                        )
    for mode in groups:
        output["modes"][mode]["actual_link_clearance"] = {
            name: _stats(values) for name, values in clearances[mode].items()
        }
    output["policy_induced_collision_risk"] = {
        "margins_m": risk_margins,
        "event_count": len(policy_induced_events),
        "events_by_pair": dict(Counter(event["pair"] for event in policy_induced_events)),
        "worst_events": sorted(
            policy_induced_events,
            key=lambda event: event["actual_clearance_m"],
        )[:20],
    }
    risk_arm = {
        "left_wrist_torso_m": "left",
        "left_elbow_torso_m": "left",
        "right_wrist_torso_m": "right",
        "right_elbow_torso_m": "right",
    }
    covered = 0
    for event in policy_induced_events:
        arm = risk_arm.get(event["pair"])
        if arm is None:
            scale = min(
                event["collision_guard_left_scale"],
                event["collision_guard_right_scale"],
            )
        else:
            scale = event[f"collision_guard_{arm}_scale"]
        covered += int(scale < 0.999)
    output["collision_guard_replay"] = {
        "policy_frames": len(guard_left_scales),
        "active_frames": guard_active_frames,
        "active_frame_rate": guard_active_frames / max(len(guard_left_scales), 1),
        "left_arm_scale": _stats(guard_left_scales),
        "right_arm_scale": _stats(guard_right_scales),
        "legacy_risk_events_covered": covered,
        "legacy_risk_event_count": len(policy_induced_events),
    }
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--jsonl", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(analyze(args.csv, args.jsonl), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

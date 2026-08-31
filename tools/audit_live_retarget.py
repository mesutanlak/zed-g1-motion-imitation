#!/usr/bin/env python3
"""Audit whether live G1 targets move consistently with human arm geometry."""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


def distribution(values: list[float]) -> dict[str, float | int | None]:
    data = np.asarray(values, dtype=np.float64)
    return {
        "count": int(data.size),
        "p50": float(np.percentile(data, 50)) if data.size else None,
        "p90": float(np.percentile(data, 90)) if data.size else None,
        "p99": float(np.percentile(data, 99)) if data.size else None,
        "max": float(np.max(data)) if data.size else None,
    }


def angle_deg(first: np.ndarray, second: np.ndarray) -> float:
    denominator = float(np.linalg.norm(first) * np.linalg.norm(second))
    if denominator < 1.0e-9:
        return float("nan")
    cosine = float(np.clip(np.dot(first, second) / denominator, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def human_arm_state(packet: dict[str, Any], side: str) -> np.ndarray | None:
    human = (packet.get("retarget_comparison") or {}).get("human_positions_m") or {}
    try:
        shoulder = np.asarray(human[f"{side}_shoulder"], dtype=np.float64)
        elbow = np.asarray(human[f"{side}_elbow"], dtype=np.float64)
        wrist = np.asarray(human[f"{side}_wrist"], dtype=np.float64)
    except (KeyError, TypeError, ValueError):
        return None
    values = np.concatenate((elbow - shoulder, wrist - elbow))
    return values if np.isfinite(values).all() else None


def load_unique_packets(path: Path) -> list[dict[str, Any]]:
    unique: dict[tuple[int, int], dict[str, Any]] = {}
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            packet = (json.loads(line).get("telemetry_packet") or {})
            if not packet.get("safe_joint_position_rad"):
                continue
            key = (
                int(packet.get("control_session_id", 0)),
                int(packet.get("sequence", -1)),
            )
            if key not in unique or packet.get("isaac_metrics"):
                unique[key] = packet
    return sorted(
        unique.values(), key=lambda packet: int(packet.get("source_timestamp_ns", 0))
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("imitation_jsonl", type=Path)
    parser.add_argument("--static-angle-deg", type=float, default=2.0)
    parser.add_argument("--static-endpoint-m", type=float, default=0.015)
    args = parser.parse_args()

    packets = load_unique_packets(args.imitation_jsonl)
    if len(packets) < 2:
        raise ValueError("At least two unique live retarget packets are required")
    names = [str(name) for name in packets[0]["joint_names"]]
    q = np.asarray([packet["safe_joint_position_rad"] for packet in packets])
    timestamps = np.asarray(
        [int(packet.get("source_timestamp_ns", 0)) for packet in packets],
        dtype=np.float64,
    )
    dt = np.diff(timestamps) / 1.0e9
    dq = np.abs(np.diff(q, axis=0))

    limb_metrics: dict[str, list[float]] = {}
    for side in ("left", "right"):
        for segment in ("upper", "forearm"):
            key = f"{side}_{segment}_direction_error_deg"
            limb_metrics[key] = [
                float(value)
                for packet in packets
                if isinstance(
                    value := (
                        (packet.get("bridge_metrics") or {})
                        .get("limb_direction_error_deg", {})
                        .get(key)
                    ),
                    (int, float),
                )
                and math.isfinite(float(value))
            ]

    scalar_bridge_metrics: dict[str, list[float]] = {}
    for key in (
        "left_direct_arm_ik_rms_m", "right_direct_arm_ik_rms_m",
        "left_gmr_arm_rms_m", "right_gmr_arm_rms_m",
        "ik_upper_relative_residual_m",
    ):
        scalar_bridge_metrics[key] = [
            float(value)
            for packet in packets
            if isinstance(
                value := (packet.get("bridge_metrics") or {}).get(key),
                (int, float),
            )
            and math.isfinite(float(value))
        ]

    conditional_direction: dict[str, dict[str, list[float]]] = {}
    for condition_name, predicate in (
        ("mirror_applied", lambda metrics: bool(metrics.get("mirror_rescue_applied"))),
        ("mirror_not_applied", lambda metrics: not bool(metrics.get("mirror_rescue_applied"))),
        ("right_baseline", lambda metrics: bool(metrics.get("right_arm_baseline_preserved"))),
        ("right_refined", lambda metrics: not bool(metrics.get("right_arm_baseline_preserved"))),
    ):
        grouped: dict[str, list[float]] = {}
        for key in limb_metrics:
            values = []
            for packet in packets:
                metrics = packet.get("bridge_metrics") or {}
                value = (metrics.get("limb_direction_error_deg") or {}).get(key)
                if (
                    predicate(metrics)
                    and isinstance(value, (int, float))
                    and math.isfinite(float(value))
                ):
                    values.append(float(value))
            grouped[key] = values
        conditional_direction[condition_name] = grouped

    categorical: dict[str, Counter[str]] = {}
    for key in (
        "left_arm_pole_source", "right_arm_pole_source",
        "left_elbow_branch_sign", "right_elbow_branch_sign",
        "left_wrist_orientation_quality", "right_wrist_orientation_quality",
        "mirror_rescue_triggered", "mirror_rescue_applied",
        "left_arm_baseline_preserved", "right_arm_baseline_preserved",
        "left_direct_ik_candidate_count", "right_direct_ik_candidate_count",
    ):
        categorical[key] = Counter(
            str((packet.get("bridge_metrics") or {}).get(key))
            for packet in packets
        )

    static_joint_delta: dict[str, list[float]] = {name: [] for name in names}
    inconsistent_steps: list[dict[str, Any]] = []
    human_change_values: list[float] = []
    for index in range(1, len(packets)):
        if not (0.0 < dt[index - 1] <= 0.20):
            continue
        changes = []
        endpoint_changes = []
        valid = True
        for side in ("left", "right"):
            previous = human_arm_state(packets[index - 1], side)
            current = human_arm_state(packets[index], side)
            if previous is None or current is None:
                valid = False
                break
            changes.extend((
                angle_deg(previous[:3], current[:3]),
                angle_deg(previous[3:], current[3:]),
            ))
            endpoint_changes.append(float(np.linalg.norm(
                (current[:3] + current[3:])
                - (previous[:3] + previous[3:])
            )))
        if not valid or not np.isfinite(changes).all():
            continue
        human_change = max(changes)
        human_change_values.append(human_change)
        static = (
            human_change <= args.static_angle_deg
            and max(endpoint_changes) <= args.static_endpoint_m
        )
        if static:
            for joint_name, value in zip(names, dq[index - 1]):
                static_joint_delta[joint_name].append(float(value))
            maximum_delta = float(np.max(dq[index - 1, 13:23]))
            if maximum_delta > 0.08:
                joint_index = 13 + int(np.argmax(dq[index - 1, 13:23]))
                inconsistent_steps.append({
                    "sequence": int(packets[index].get("sequence", -1)),
                    "source_timestamp_ns": int(timestamps[index]),
                    "dt_ms": float(dt[index - 1] * 1000.0),
                    "human_direction_change_deg": human_change,
                    "human_endpoint_change_m": max(endpoint_changes),
                    "robot_joint": names[joint_index],
                    "robot_joint_delta_rad": float(dq[index - 1, joint_index]),
                    "fusion_state": str(
                        (packets[index].get("source_multi_camera") or {}).get(
                            "fusion_state", "UNKNOWN"
                        )
                    ),
                })

    branch_transitions = {}
    for side in ("left", "right"):
        values = [
            str((packet.get("bridge_metrics") or {}).get(
                f"{side}_elbow_branch_sign"
            ))
            for packet in packets
        ]
        branch_transitions[side] = sum(
            current != previous for previous, current in zip(values, values[1:])
        )

    per_joint_delta = {
        name: {
            "all": distribution(dq[:, index].tolist()),
            "human_static": distribution(static_joint_delta[name]),
        }
        for index, name in enumerate(names)
    }
    ranked_static = sorted(
        (
            (name, values["human_static"].get("p99") or 0.0)
            for name, values in per_joint_delta.items()
        ),
        key=lambda item: item[1],
        reverse=True,
    )
    payload = {
        "schema": "g1_live_retarget_audit/v1",
        "unique_packets": len(packets),
        "duration_s": float((timestamps[-1] - timestamps[0]) / 1.0e9),
        "packet_dt_ms": distribution((dt * 1000.0).tolist()),
        "human_direction_change_deg": distribution(human_change_values),
        "limb_direction_error_deg": {
            name: distribution(values) for name, values in limb_metrics.items()
        },
        "bridge_metrics": {
            name: distribution(values)
            for name, values in scalar_bridge_metrics.items()
        },
        "conditional_limb_direction_error_deg": {
            condition: {
                name: distribution(values) for name, values in grouped.items()
            }
            for condition, grouped in conditional_direction.items()
        },
        "categorical": {
            name: dict(counts) for name, counts in categorical.items()
        },
        "elbow_branch_transitions": branch_transitions,
        "worst_human_static_joints": [
            {"joint": name, **per_joint_delta[name]["human_static"]}
            for name, _ in ranked_static[:12]
        ],
        "inconsistent_static_step_count": len(inconsistent_steps),
        "worst_inconsistent_static_steps": sorted(
            inconsistent_steps,
            key=lambda item: item["robot_joint_delta_rad"],
            reverse=True,
        )[:20],
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

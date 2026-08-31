#!/usr/bin/env python3
"""Summarize dual perception, GMR safety and Isaac response from one session."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


def finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def distribution(values: list[float]) -> dict[str, float | int | None]:
    data = np.asarray(values, dtype=float)
    return {
        "count": int(data.size),
        "mean": float(np.mean(data)) if data.size else None,
        "p50": float(np.percentile(data, 50)) if data.size else None,
        "p90": float(np.percentile(data, 90)) if data.size else None,
        "p99": float(np.percentile(data, 99)) if data.size else None,
        "max": float(np.max(data)) if data.size else None,
    }


def csv_rows(path: Path):
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        yield from csv.DictReader(stream)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--frames", type=Path, required=True)
    parser.add_argument("--imitation", type=Path, required=True)
    args = parser.parse_args()

    raw_mode = Counter()
    raw_reasons = Counter()
    raw_failures = Counter()
    raw_runs: list[tuple[str, int]] = []
    previous_mode = None
    run_length = 0
    segment_quality: dict[str, list[float]] = defaultdict(list)
    source_counts = Counter()
    source_by_joint: dict[str, Counter[str]] = defaultdict(Counter)
    raw_metrics: dict[str, list[float]] = defaultdict(list)
    raw_start = raw_end = None
    with args.raw.open("r", encoding="utf-8") as stream:
        next(stream, None)
        for line in stream:
            frame = json.loads(line)
            timestamp = int(frame.get("timestamp_ns", 0))
            raw_start = timestamp if raw_start is None else raw_start
            raw_end = timestamp
            multi = frame.get("multi_camera") or {}
            mode = str(multi.get("mode") or "unknown")
            raw_mode[mode] += 1
            raw_reasons[str(multi.get("selection_reason") or "unknown")] += 1
            raw_failures.update(multi.get("failure_codes") or [])
            if mode == previous_mode:
                run_length += 1
            else:
                if previous_mode is not None:
                    raw_runs.append((previous_mode, run_length))
                previous_mode, run_length = mode, 1
            agreement = multi.get("cross_view_agreement") or {}
            human = frame.get("human_state") or {}
            for key, value in {
                "camera_timestamp_delta_ms": multi.get("camera_timestamp_delta_ms"),
                "cross_view_mpjpe_m": agreement.get("mpjpe_m"),
                "cross_view_p95_m": agreement.get("p95_error_m"),
                "core_disagreement_m": human.get("core_disagreement_m"),
                "pelvis_disagreement_m": agreement.get("pelvis_error_m"),
                "left_wrist_disagreement_m": agreement.get("left_wrist_error_m"),
                "right_wrist_disagreement_m": agreement.get("right_wrist_error_m"),
            }.items():
                number = finite_float(value)
                if number is not None:
                    raw_metrics[key].append(number)
            for name, value in (human.get("segment_quality") or {}).items():
                number = finite_float(value)
                if number is not None:
                    segment_quality[name].append(number)
            for name, source in (human.get("joint_source") or {}).items():
                source_counts[str(source)] += 1
                source_by_joint[name][str(source)] += 1
    if previous_mode is not None:
        raw_runs.append((previous_mode, run_length))

    frame_metrics: dict[str, list[float]] = defaultdict(list)
    frame_modes = Counter()
    for row in csv_rows(args.frames):
        frame_modes[row.get("fusion_mode") or "unknown"] += 1
        for key in (
            "source_hz", "capture_to_send_ms", "camera_fps_min",
            "camera_fps_max", "camera_timestamp_delta_ms",
            "left_arm_quality", "right_arm_quality", "torso_quality",
        ):
            value = finite_float(row.get(key))
            if value is not None:
                frame_metrics[key].append(value)

    imitation_metrics: dict[str, list[float]] = defaultdict(list)
    safety_levels = Counter()
    safety_reasons = Counter()
    for row in csv_rows(args.imitation):
        safety_levels[row.get("safety_level") or "unknown"] += 1
        try:
            safety_reasons.update(json.loads(row.get("safety_reasons") or "[]"))
        except json.JSONDecodeError:
            pass
        for key in (
            "gmr_upper_relative_residual_m", "joint_tracking_rmse_rad",
            "body_tracking_mpjpe_m", "target_jerk_rms_rad_s3",
            "target_jerk_max_rad_s3", "total_control_ms",
            "reference_target_error_rms_rad",
            "reference_target_error_max_rad",
            "left_safe_error_deg", "right_safe_error_deg",
            "left_hand_position_error_m", "right_hand_position_error_m",
            "left_elbow_position_error_m", "right_elbow_position_error_m",
        ):
            value = finite_float(row.get(key))
            if value is not None:
                imitation_metrics[key].append(value)

    summary = {
        "raw": {
            "frames": sum(raw_mode.values()),
            "duration_s": (
                (raw_end - raw_start) / 1.0e9
                if raw_start is not None and raw_end is not None else None
            ),
            "mode_counts": dict(raw_mode),
            "mode_transitions": max(0, len(raw_runs) - 1),
            "run_length_frames": {
                mode: distribution([length for run_mode, length in raw_runs if run_mode == mode])
                for mode in raw_mode
            },
            "selection_reasons": dict(raw_reasons),
            "failure_codes": dict(raw_failures),
            "metrics": {key: distribution(values) for key, values in raw_metrics.items()},
            "segment_quality": {key: distribution(values) for key, values in segment_quality.items()},
            "joint_source_counts": dict(source_counts),
            "joint_sources": {name: dict(counts) for name, counts in source_by_joint.items()},
        },
        "rerun": {
            "frame_mode_counts": dict(frame_modes),
            "frame_metrics": {key: distribution(values) for key, values in frame_metrics.items()},
            "safety_levels": dict(safety_levels),
            "safety_reasons": dict(safety_reasons),
            "imitation_metrics": {
                key: distribution(values) for key, values in imitation_metrics.items()
            },
        },
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

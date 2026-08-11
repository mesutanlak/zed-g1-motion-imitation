#!/usr/bin/env python3
"""Create reproducible single-vs-dual BODY_38 paper metrics from JSONL."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
from statistics import mean, median
from typing import Any, Iterable

import numpy as np


ARMS = {
    "left": ("LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST"),
    "right": ("RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST"),
}
UPPER = (
    "PELVIS", "SPINE_2", "SPINE_3", "NECK",
    "LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST",
    "RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST",
)
BONES = (
    ("LEFT_SHOULDER", "LEFT_ELBOW"), ("LEFT_ELBOW", "LEFT_WRIST"),
    ("RIGHT_SHOULDER", "RIGHT_ELBOW"), ("RIGHT_ELBOW", "RIGHT_WRIST"),
)


def percentile(values: list[float], q: float) -> float | None:
    return float(np.percentile(values, q)) if values else None


def angle_deg(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float | None:
    first, second = a - b, c - b
    denominator = float(np.linalg.norm(first) * np.linalg.norm(second))
    if denominator < 1e-8 or not math.isfinite(denominator):
        return None
    return float(np.degrees(np.arccos(np.clip(np.dot(first, second) / denominator, -1, 1))))


def frames(path: Path) -> Iterable[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            try:
                item = json.loads(line)
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue
            if str(item.get("schema", "")).endswith("metadata/v1"):
                continue
            if "keypoints_3d_filtered_m" in item or "keypoints_3d_m" in item:
                yield item


def analyze(path: Path, label: str) -> dict[str, Any]:
    count = 0
    timestamps: list[int] = []
    visible_ratios: list[float] = []
    upper_ratios: list[float] = []
    bone_lengths: dict[str, list[float]] = {f"{a}-{b}": [] for a, b in BONES}
    elbow_angles = {"left": [], "right": []}
    elbow_steps = {"left": [], "right": []}
    last_elbow: dict[str, float] = {}
    recovery_frames = 0
    bilateral_frames = 0
    source_modes: Counter[str] = Counter()
    contributing_views: list[float] = []
    cross_view_mpjpe: list[float] = []
    fusion_camera_counts: list[float] = []
    fusion_sync_stdev_ms: list[float] = []
    per_camera_received_fps: dict[str, list[float]] = {}
    for item in frames(path):
        names = item.get("keypoint_names", [])
        index = {name: i for i, name in enumerate(names)}
        if not all(name in index for name in UPPER):
            continue
        points = np.asarray(
            item.get("keypoints_3d_filtered_m", item.get("keypoints_3d_m")),
            dtype=np.float64,
        )
        confidence = np.asarray(item.get("keypoint_confidence", []), dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or confidence.shape[0] != points.shape[0]:
            continue
        count += 1
        timestamp = item.get("timestamp_ns")
        if isinstance(timestamp, int):
            timestamps.append(timestamp)
        visible = np.isfinite(points).all(axis=1) & (confidence >= 40.0)
        visible_ratios.append(float(np.mean(visible)))
        upper_ratios.append(float(np.mean(visible[[index[name] for name in UPPER]])))
        for first, second in BONES:
            if visible[index[first]] and visible[index[second]]:
                bone_lengths[f"{first}-{second}"].append(
                    float(np.linalg.norm(points[index[first]] - points[index[second]]))
                )
        for side, (shoulder, elbow, wrist) in ARMS.items():
            if all(visible[index[name]] for name in (shoulder, elbow, wrist)):
                angle = angle_deg(points[index[shoulder]], points[index[elbow]], points[index[wrist]])
                if angle is not None:
                    elbow_angles[side].append(angle)
                    if side in last_elbow:
                        elbow_steps[side].append(abs(angle - last_elbow[side]))
                    last_elbow[side] = angle
        occlusion = item.get("occlusion_analysis", {})
        recovered = occlusion.get("arm_chain_recovered", {})
        recovery_frames += int(any(bool(value) for value in recovered.values()))
        bilateral_frames += int("bilateral_front_arm_occlusion" in occlusion.get("reasons", []))
        multi = item.get("multi_camera", {})
        if multi:
            source_modes[str(multi.get("mode", "unknown"))] += 1
            value = multi.get("contributing_views")
            if isinstance(value, (int, float)):
                contributing_views.append(float(value))
            agreement = (multi.get("cross_view_agreement") or {}).get("mpjpe_m")
            if isinstance(agreement, (int, float)) and math.isfinite(float(agreement)):
                cross_view_mpjpe.append(float(agreement))
            fusion_metrics = multi.get("fusion_metrics") or {}
            fused_count = fusion_metrics.get("mean_camera_fused")
            if isinstance(fused_count, (int, float)) and math.isfinite(float(fused_count)):
                fusion_camera_counts.append(float(fused_count))
            sync_stdev_s = fusion_metrics.get("mean_stdev_between_camera_s")
            if isinstance(sync_stdev_s, (int, float)) and math.isfinite(float(sync_stdev_s)):
                fusion_sync_stdev_ms.append(1000.0 * float(sync_stdev_s))
            for serial, camera_metrics in (fusion_metrics.get("per_camera") or {}).items():
                received_fps = camera_metrics.get("received_fps")
                if isinstance(received_fps, (int, float)) and math.isfinite(float(received_fps)):
                    per_camera_received_fps.setdefault(str(serial), []).append(float(received_fps))

    intervals_ms = [
        (b - a) / 1e6 for a, b in zip(timestamps, timestamps[1:]) if b > a
    ]
    bone_cvs = []
    for values in bone_lengths.values():
        if len(values) > 1 and mean(values) > 1e-6:
            bone_cvs.append(float(np.std(values) / np.mean(values)))
    result = {
        "label": label,
        "path": str(path.resolve()),
        "frames": count,
        "duration_s": ((timestamps[-1] - timestamps[0]) / 1e9 if len(timestamps) > 1 else None),
        "effective_body_fps": (1000.0 / median(intervals_ms) if intervals_ms else None),
        "frame_interval_p95_ms": percentile(intervals_ms, 95),
        "visible_keypoint_ratio_mean": mean(visible_ratios) if visible_ratios else None,
        "upper_body_visible_ratio_mean": mean(upper_ratios) if upper_ratios else None,
        "upper_body_complete_frame_ratio": (
            sum(value >= 0.999 for value in upper_ratios) / len(upper_ratios)
            if upper_ratios else None
        ),
        "arm_bone_length_cv_mean": mean(bone_cvs) if bone_cvs else None,
        "left_elbow_angle_step_p95_deg": percentile(elbow_steps["left"], 95),
        "right_elbow_angle_step_p95_deg": percentile(elbow_steps["right"], 95),
        "arm_recovery_frame_ratio": recovery_frames / count if count else None,
        "bilateral_front_occlusion_frame_ratio": bilateral_frames / count if count else None,
        "source_mode_counts": dict(source_modes),
        "contributing_views_mean": mean(contributing_views) if contributing_views else None,
        "cross_view_mpjpe_mean_m": mean(cross_view_mpjpe) if cross_view_mpjpe else None,
        "cross_view_mpjpe_p95_m": percentile(cross_view_mpjpe, 95),
        "fusion_mean_camera_count": mean(fusion_camera_counts) if fusion_camera_counts else None,
        "fusion_sync_stdev_mean_ms": mean(fusion_sync_stdev_ms) if fusion_sync_stdev_ms else None,
        "fusion_received_fps_mean": {
            serial: mean(values) for serial, values in per_camera_received_fps.items()
        },
    }
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--single", type=Path, required=True)
    parser.add_argument("--dual", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = [analyze(args.single, "single")]
    if args.dual:
        rows.append(analyze(args.dual, "dual"))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    nested = {"source_mode_counts", "fusion_received_fps_mean"}
    flat_keys = [key for key in rows[0] if key not in nested]
    with args.output.with_suffix(".csv").open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=flat_keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in flat_keys})
    print(json.dumps(rows, ensure_ascii=False, indent=2))
    if args.dual is None:
        print("Dual kayıt verilmedi: tek-kamera baz çizgisi üretildi; sonuç artışı iddia edilmedi.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

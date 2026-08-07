"""Deterministic quality benchmark for ZED BODY_38 JSONL recordings.

This tool intentionally evaluates capture/perception data before GMR or Isaac.
It prevents controller tuning from hiding identity, timing, visibility, or
kinematic-consistency regressions in the camera stream.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any, Iterable


BONES = (
    ("LEFT_SHOULDER", "LEFT_ELBOW"),
    ("LEFT_ELBOW", "LEFT_WRIST"),
    ("RIGHT_SHOULDER", "RIGHT_ELBOW"),
    ("RIGHT_ELBOW", "RIGHT_WRIST"),
    ("LEFT_HIP", "LEFT_KNEE"),
    ("LEFT_KNEE", "LEFT_ANKLE"),
    ("RIGHT_HIP", "RIGHT_KNEE"),
    ("RIGHT_KNEE", "RIGHT_ANKLE"),
    ("LEFT_SHOULDER", "RIGHT_SHOULDER"),
    ("LEFT_HIP", "RIGHT_HIP"),
    ("PELVIS", "SPINE_3"),
)
CORE = (
    "PELVIS", "SPINE_3", "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST",
)


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def finite_point(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    if not all(isinstance(v, (int, float)) and math.isfinite(float(v)) for v in value):
        return None
    return tuple(float(v) for v in value)


def iter_frames(path: Path) -> Iterable[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number}: invalid JSON: {exc}") from exc
            if isinstance(record.get("source_packet"), dict):
                record = record["source_packet"]
            if "keypoint_names" in record and (
                "keypoints_3d_filtered_m" in record
                or "keypoints_3d_m" in record
                or "keypoints_3d_raw_m" in record
            ):
                yield record


@dataclass
class CaptureBenchmark:
    recording: str
    frames: int
    duration_s: float
    effective_fps: float | None
    interval_p50_ms: float | None
    interval_p95_ms: float | None
    jitter_p95_ms: float | None
    sequence_gap_frames: int
    body_id_switches: int
    locked_ratio: float | None
    calibration_ready_ratio: float | None
    body_confidence_p50: float | None
    body_confidence_p05: float | None
    visible_keypoint_ratio_mean: float | None
    core_visible_ratio_mean: float | None
    bone_cv_median: float | None
    bone_cv_p95: float | None
    shoulder_hip_swap_risk_frames: int
    torso_overlap_ratio: float | None
    max_joint_speed_p95_m_s: float | None
    teleport_events: int
    quality_level: str
    quality_reasons: list[str]


def benchmark(path: Path, confidence_threshold: float, target_fps: float) -> CaptureBenchmark:
    timestamps: list[int] = []
    sequences: list[int] = []
    body_ids: list[int] = []
    body_confidences: list[float] = []
    visible_ratios: list[float] = []
    core_ratios: list[float] = []
    bone_lengths: dict[str, list[float]] = {
        f"{a}->{b}": [] for a, b in BONES
    }
    locked: list[bool] = []
    calibrated: list[bool] = []
    overlap_frames = 0
    overlap_available = 0
    lateral_order_samples: list[tuple[float, float]] = []
    previous_points: dict[str, tuple[float, float, float]] = {}
    previous_timestamp: int | None = None
    speeds: list[float] = []
    teleport_events = 0
    frame_count = 0

    for frame in iter_frames(path):
        frame_count += 1
        timestamp = int(frame.get("timestamp_ns", 0) or 0)
        if timestamp:
            timestamps.append(timestamp)
        sequence = frame.get("frame_index", frame.get("sequence"))
        if isinstance(sequence, int):
            sequences.append(sequence)
        body_id = frame.get("body_id")
        if isinstance(body_id, int):
            body_ids.append(body_id)
        confidence = frame.get("body_confidence")
        if isinstance(confidence, (int, float)) and math.isfinite(float(confidence)):
            body_confidences.append(float(confidence))

        names = [str(name) for name in frame.get("keypoint_names", [])]
        values = (
            frame.get("keypoints_3d_filtered_m")
            or frame.get("keypoints_3d_m")
            or frame.get("keypoints_3d_raw_m")
            or []
        )
        confidences = frame.get("keypoint_confidence") or []
        points: dict[str, tuple[float, float, float]] = {}
        valid_names: set[str] = set()
        for index, name in enumerate(names):
            point = finite_point(values[index]) if index < len(values) else None
            keypoint_confidence = (
                float(confidences[index])
                if index < len(confidences)
                and isinstance(confidences[index], (int, float))
                else 0.0
            )
            if point is not None and keypoint_confidence >= confidence_threshold:
                points[name] = point
                valid_names.add(name)
        if names:
            visible_ratios.append(len(valid_names) / len(names))
        core_ratios.append(sum(name in valid_names for name in CORE) / len(CORE))

        for first, second in BONES:
            if first in points and second in points:
                bone_lengths[f"{first}->{second}"].append(
                    math.dist(points[first], points[second])
                )
        if all(
            name in points
            for name in (
                "LEFT_SHOULDER", "RIGHT_SHOULDER", "LEFT_HIP", "RIGHT_HIP"
            )
        ):
            # Facing direction changes the expected sign. Learn the dominant
            # lateral ordering from the clip, then count only temporal flips.
            # Wrists are deliberately excluded because crossing arms is valid.
            lateral_order_samples.append((
                points["LEFT_SHOULDER"][1] - points["RIGHT_SHOULDER"][1],
                points["LEFT_HIP"][1] - points["RIGHT_HIP"][1],
            ))

        if previous_timestamp and timestamp > previous_timestamp:
            dt = (timestamp - previous_timestamp) / 1e9
            if 0.005 <= dt <= 0.5:
                for name, point in points.items():
                    if name not in previous_points:
                        continue
                    speed = math.dist(point, previous_points[name]) / dt
                    speeds.append(speed)
                    if speed > 6.0:
                        teleport_events += 1
        previous_points = points
        previous_timestamp = timestamp or previous_timestamp

        selection = frame.get("operator_selection") or {}
        if selection:
            locked.append(selection.get("state") == "LOCKED")
        calibration = frame.get("calibration") or {}
        if calibration:
            calibrated.append(calibration.get("state") == "READY")
        occlusion = frame.get("occlusion_analysis") or {}
        overlap = occlusion.get("arm_torso_overlap") or {}
        if overlap:
            overlap_available += 1
            overlap_frames += int(bool(overlap.get("left") or overlap.get("right")))

    intervals_ms = [
        (second - first) / 1e6
        for first, second in zip(timestamps, timestamps[1:])
        if second > first
    ]
    p50_interval = percentile(intervals_ms, 0.50)
    jitter = (
        [abs(value - p50_interval) for value in intervals_ms]
        if p50_interval is not None else []
    )
    duration_s = (
        (timestamps[-1] - timestamps[0]) / 1e9 if len(timestamps) >= 2 else 0.0
    )
    sequence_gaps = sum(
        max(0, current - previous - 1)
        for previous, current in zip(sequences, sequences[1:])
        if current > previous
    )
    switches = sum(a != b for a, b in zip(body_ids, body_ids[1:]))
    bone_cvs = []
    for values in bone_lengths.values():
        if len(values) >= 10:
            mean = statistics.fmean(values)
            if mean > 1e-6:
                bone_cvs.append(statistics.pstdev(values) / mean)

    swap_risk = 0
    if lateral_order_samples:
        shoulder_sign = math.copysign(
            1.0, statistics.median(value[0] for value in lateral_order_samples)
        )
        hip_sign = math.copysign(
            1.0, statistics.median(value[1] for value in lateral_order_samples)
        )
        swap_risk = sum(
            (shoulder * shoulder_sign < 0.0) or (hip * hip_sign < 0.0)
            for shoulder, hip in lateral_order_samples
        )

    fps = (frame_count - 1) / duration_s if duration_s > 0 and frame_count > 1 else None
    visible_mean = statistics.fmean(visible_ratios) if visible_ratios else None
    core_mean = statistics.fmean(core_ratios) if core_ratios else None
    bone_cv_p95 = percentile(bone_cvs, 0.95)
    reasons: list[str] = []
    if fps is None or fps < 0.90 * target_fps:
        reasons.append("capture_fps_below_target")
    if core_mean is None or core_mean < 0.90:
        reasons.append("core_visibility_below_90pct")
    if bone_cv_p95 is not None and bone_cv_p95 > 0.08:
        reasons.append("bone_length_instability")
    if sequence_gaps:
        reasons.append("source_sequence_gaps")
    if teleport_events:
        reasons.append("keypoint_teleport_events")
    if locked and statistics.fmean(locked) < 0.90:
        reasons.append("operator_not_consistently_locked")
    quality = (
        "RED"
        if frame_count == 0
        else ("GREEN" if not reasons else ("YELLOW" if len(reasons) <= 2 else "RED"))
    )

    return CaptureBenchmark(
        recording=str(path), frames=frame_count, duration_s=duration_s,
        effective_fps=fps, interval_p50_ms=p50_interval,
        interval_p95_ms=percentile(intervals_ms, 0.95),
        jitter_p95_ms=percentile(jitter, 0.95),
        sequence_gap_frames=sequence_gaps, body_id_switches=switches,
        locked_ratio=statistics.fmean(locked) if locked else None,
        calibration_ready_ratio=statistics.fmean(calibrated) if calibrated else None,
        body_confidence_p50=percentile(body_confidences, 0.50),
        body_confidence_p05=percentile(body_confidences, 0.05),
        visible_keypoint_ratio_mean=visible_mean,
        core_visible_ratio_mean=core_mean,
        bone_cv_median=percentile(bone_cvs, 0.50), bone_cv_p95=bone_cv_p95,
        shoulder_hip_swap_risk_frames=swap_risk,
        torso_overlap_ratio=(overlap_frames / overlap_available if overlap_available else None),
        max_joint_speed_p95_m_s=percentile(speeds, 0.95),
        teleport_events=teleport_events, quality_level=quality,
        quality_reasons=reasons,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", nargs="+", type=Path)
    parser.add_argument("--confidence", type=float, default=40.0)
    parser.add_argument("--target-fps", type=float, default=30.0)
    parser.add_argument("--output", type=Path, default=Path("reports/motion_capture_benchmark.json"))
    args = parser.parse_args()
    results = [benchmark(path, args.confidence, args.target_fps) for path in args.recordings]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema": "zed_g1_motion_capture_benchmark/v1",
        "confidence_threshold": args.confidence,
        "target_fps": args.target_fps,
        "results": [asdict(result) for result in results],
    }
    args.output.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    csv_path = args.output.with_suffix(".csv")
    with csv_path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(asdict(results[0]).keys()))
        writer.writeheader()
        for result in results:
            row = asdict(result)
            row["quality_reasons"] = ";".join(row["quality_reasons"])
            writer.writerow(row)
    for result in results:
        fps_text = f"{result.effective_fps:.2f}" if result.effective_fps is not None else "n/a"
        core_text = (
            f"{result.core_visible_ratio_mean:.1%}"
            if result.core_visible_ratio_mean is not None else "n/a"
        )
        bone_text = f"{result.bone_cv_p95:.1%}" if result.bone_cv_p95 is not None else "n/a"
        print(
            f"{Path(result.recording).name}: {result.quality_level} "
            f"fps={fps_text} core={core_text} "
            f"bone_cv_p95={bone_text} reasons={result.quality_reasons}"
        )
    print(f"JSON: {args.output}\nCSV: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

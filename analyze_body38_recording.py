#!/usr/bin/env python3
"""Quality analysis for ZED BODY_38 recordings prepared for G1 retargeting.

The metrics describe observability and temporal consistency. They are not
ground-truth anatomical accuracy and this program never commands a robot.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any


BODY_GROUPS = {
    "torso": ["PELVIS", "SPINE_2", "SPINE_3", "LEFT_HIP", "RIGHT_HIP"],
    "left_arm": ["LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST"],
    "right_arm": ["RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST"],
    "left_leg_core": ["LEFT_HIP", "LEFT_KNEE", "LEFT_ANKLE"],
    "right_leg_core": ["RIGHT_HIP", "RIGHT_KNEE", "RIGHT_ANKLE"],
    "left_foot_support": ["LEFT_ANKLE", "LEFT_HEEL", "LEFT_BIG_TOE"],
    "right_foot_support": ["RIGHT_ANKLE", "RIGHT_HEEL", "RIGHT_BIG_TOE"],
    # Dex3-1 is a three-finger hand: thumb, index and middle.
    "left_dex3_source": [
        "LEFT_WRIST",
        "LEFT_HAND_THUMB_4",
        "LEFT_HAND_INDEX_1",
        "LEFT_HAND_MIDDLE_4",
    ],
    "right_dex3_source": [
        "RIGHT_WRIST",
        "RIGHT_HAND_THUMB_4",
        "RIGHT_HAND_INDEX_1",
        "RIGHT_HAND_MIDDLE_4",
    ],
}

BONES = [
    ("PELVIS", "SPINE_1"),
    ("SPINE_1", "SPINE_2"),
    ("SPINE_2", "SPINE_3"),
    ("SPINE_3", "NECK"),
    ("SPINE_3", "LEFT_SHOULDER"),
    ("SPINE_3", "RIGHT_SHOULDER"),
    ("LEFT_SHOULDER", "LEFT_ELBOW"),
    ("LEFT_ELBOW", "LEFT_WRIST"),
    ("RIGHT_SHOULDER", "RIGHT_ELBOW"),
    ("RIGHT_ELBOW", "RIGHT_WRIST"),
    ("PELVIS", "LEFT_HIP"),
    ("PELVIS", "RIGHT_HIP"),
    ("LEFT_HIP", "LEFT_KNEE"),
    ("LEFT_KNEE", "LEFT_ANKLE"),
    ("RIGHT_HIP", "RIGHT_KNEE"),
    ("RIGHT_KNEE", "RIGHT_ANKLE"),
]


def number(value: Any) -> float | None:
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value)
    return None


def vector(value: Any, size: int) -> list[float] | None:
    if not isinstance(value, list) or len(value) < size:
        return None
    result = [number(component) for component in value[:size]]
    if any(component is None for component in result):
        return None
    return [float(component) for component in result]


def describe(values: list[float], extended: bool = False) -> dict[str, Any]:
    if not values:
        return {
            "count": 0,
            "min": None,
            "median": None,
            "mean": None,
            "max": None,
        }
    result: dict[str, Any] = {
        "count": len(values),
        "min": min(values),
        "median": statistics.median(values),
        "mean": statistics.fmean(values),
        "max": max(values),
    }
    if extended:
        ordered = sorted(values)
        result["std"] = statistics.pstdev(values) if len(values) > 1 else 0.0
        result["p95"] = ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]
        median = float(result["median"])
        result["mad"] = statistics.median(abs(value - median) for value in values)
    return result


def distance(first: list[float], second: list[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def quaternion_angle_deg(first: list[float], second: list[float]) -> float:
    first_norm = math.sqrt(sum(value * value for value in first))
    second_norm = math.sqrt(sum(value * value for value in second))
    if first_norm < 1e-9 or second_norm < 1e-9:
        return math.nan
    dot = abs(
        sum(a * b for a, b in zip(first, second)) / (first_norm * second_norm)
    )
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def parse_recording(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]], int]:
    metadata: dict[str, Any] | None = None
    frames: list[dict[str, Any]] = []
    invalid_lines = 0
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            if not raw_line.strip():
                continue
            try:
                item = json.loads(raw_line)
            except json.JSONDecodeError as exc:
                print(f"UYARI: {line_number}. satır geçersiz: {exc}", file=sys.stderr)
                invalid_lines += 1
                continue
            schema = str(item.get("schema", ""))
            if schema.endswith("/metadata/v1") and metadata is None:
                metadata = item
            elif schema == "zed_body38_g1_reference/v1":
                frames.append(item)
            else:
                invalid_lines += 1
    if metadata is None:
        raise ValueError("Metadata satırı bulunamadı.")
    return metadata, frames, invalid_lines


def readiness_classification(
    duration_s: float,
    effective_fps: float | None,
    group_percent: dict[str, float],
    body_confidence_mean: float | None,
) -> dict[str, Any]:
    upper = min(
        group_percent["torso"],
        group_percent["left_arm"],
        group_percent["right_arm"],
    )
    whole_core = min(
        upper,
        group_percent["left_leg_core"],
        group_percent["right_leg_core"],
    )
    reasons: list[str] = []
    if duration_s < 10:
        reasons.append("Süre 10 saniyeden kısa.")
    if effective_fps is None or effective_fps < 20:
        reasons.append("Etkin kare hızı 20 FPS altında.")
    if body_confidence_mean is None or body_confidence_mean < 50:
        reasons.append("Ortalama gövde güveni 50 altında.")
    if whole_core < 80:
        reasons.append(f"G1 temel tüm-vücut görünürlüğü yalnızca %{whole_core:.1f}.")

    if not reasons:
        label = "whole_body_candidate"
    elif duration_s >= 5 and upper >= 80:
        label = "upper_body_only"
    else:
        label = "partial_or_retake"
    return {
        "label": label,
        "upper_body_observability_percent": upper,
        "whole_body_core_observability_percent": whole_core,
        "reasons": reasons,
        "important": (
            "Bu sınıflandırma algılanabilirlik/tutarlılık ölçer; gerçek eklem "
            "açısı doğruluğunu ölçen motion-capture ground truth değildir."
        ),
    }


def analyze(
    metadata: dict[str, Any],
    frames: list[dict[str, Any]],
    invalid_lines: int,
    threshold: float,
    recommended_distance_min_m: float = 2.0,
    recommended_distance_max_m: float = 4.0,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    names = list(metadata.get("keypoint_names", []))
    if len(names) != 38:
        raise ValueError(f"BODY_38 bekleniyordu; {len(names)} keypoint bulundu.")
    index = {name: position for position, name in enumerate(names)}

    confidence_values = [[] for _ in names]
    valid_counts = [0] * len(names)
    point_speeds = [[] for _ in names]
    point_axes = [[[], [], []] for _ in names]
    point_path_lengths = [0.0 for _ in names]
    joint_quaternion_norm_errors = [[] for _ in names]
    joint_angular_speeds = [[] for _ in names]
    bone_lengths: dict[str, list[float]] = {
        f"{first}--{second}": [] for first, second in BONES
    }
    quaternion_norm_errors: list[float] = []
    quaternion_speeds: list[float] = []
    root_quaternion_norm_errors: list[float] = []
    body_confidences: list[float] = []
    distances: list[float] = []
    recommended_distance_frames = 0
    geometric_angles: dict[str, list[float]] = {}
    imu_frames = 0
    imu_timestamp_offsets_ms: list[float] = []
    group_counts = {name: 0 for name in BODY_GROUPS}
    body_ids: dict[str, int] = {}
    tracking_states: dict[str, int] = {}
    rows: list[dict[str, Any]] = []
    malformed = 0

    previous_points: list[list[float] | None] | None = None
    previous_local_q: list[list[float] | None] | None = None
    previous_timestamp: int | None = None
    previous_body_id: Any = None
    timestamps: list[int] = []
    frame_intervals_ms: list[float] = []
    segment_gaps_s: list[float] = []
    segment_count = 0

    for frame in frames:
        points_raw = frame.get("keypoints_3d_filtered_m")
        confidences_raw = frame.get("keypoint_confidence")
        local_q_raw = frame.get("local_orientation_per_joint_xyzw")
        timestamp = number(frame.get("timestamp_ns"))
        if not (
            isinstance(points_raw, list)
            and len(points_raw) == 38
            and isinstance(confidences_raw, list)
            and len(confidences_raw) == 38
            and timestamp is not None
        ):
            malformed += 1
            continue

        timestamp_int = int(timestamp)
        timestamps.append(timestamp_int)
        points = [vector(value, 3) for value in points_raw]
        confidences = [number(value) for value in confidences_raw]
        valid = [
            points[position] is not None
            and confidences[position] is not None
            and float(confidences[position]) >= threshold
            for position in range(38)
        ]
        for position in range(38):
            if confidences[position] is not None:
                confidence_values[position].append(float(confidences[position]))
            valid_counts[position] += int(valid[position])
            if valid[position]:
                for axis in range(3):
                    point_axes[position][axis].append(points[position][axis])  # type: ignore[index]

        body_id = frame.get("body_id")
        body_ids[str(body_id)] = body_ids.get(str(body_id), 0) + 1
        tracking = str(frame.get("tracking_state"))
        tracking_states[tracking] = tracking_states.get(tracking, 0) + 1
        body_confidence = number(frame.get("body_confidence"))
        if body_confidence is not None:
            body_confidences.append(body_confidence)
        body_distance = number(frame.get("euclidean_distance_m"))
        if body_distance is not None:
            distances.append(body_distance)
            recommended_distance_frames += int(
                recommended_distance_min_m
                <= body_distance
                <= recommended_distance_max_m
            )

        features = frame.get("g1_reference_features", {})
        for angle_name, raw_angle in features.get("geometric_angles", {}).items():
            angle = number(raw_angle)
            if angle is not None:
                geometric_angles.setdefault(angle_name, []).append(angle)

        imu = frame.get("imu")
        if isinstance(imu, dict):
            imu_frames += 1
            imu_timestamp = number(imu.get("timestamp_ns"))
            if imu_timestamp is not None:
                imu_timestamp_offsets_ms.append(
                    (imu_timestamp - timestamp_int) / 1e6
                )

        group_ready: dict[str, bool] = {}
        for group_name, required in BODY_GROUPS.items():
            ready = all(valid[index[name]] for name in required)
            group_ready[group_name] = ready
            group_counts[group_name] += int(ready)

        for first, second in BONES:
            if valid[index[first]] and valid[index[second]]:
                bone_lengths[f"{first}--{second}"].append(
                    distance(points[index[first]], points[index[second]])  # type: ignore[arg-type]
                )

        local_q = (
            [vector(value, 4) for value in local_q_raw]
            if isinstance(local_q_raw, list) and len(local_q_raw) == 38
            else [None] * 38
        )
        for position, quaternion in enumerate(local_q):
            if quaternion is not None:
                norm_error = abs(
                    math.sqrt(sum(value * value for value in quaternion)) - 1.0
                )
                quaternion_norm_errors.append(norm_error)
                joint_quaternion_norm_errors[position].append(norm_error)
        root_q = vector(frame.get("global_root_orientation_xyzw"), 4)
        if root_q is not None:
            root_quaternion_norm_errors.append(
                abs(math.sqrt(sum(value * value for value in root_q)) - 1.0)
            )

        dt = None
        if previous_timestamp is None:
            segment_count = 1
        elif timestamp_int > previous_timestamp:
            candidate_dt = (timestamp_int - previous_timestamp) / 1e9
            if body_id != previous_body_id or candidate_dt > 0.5:
                segment_count += 1
                segment_gaps_s.append(candidate_dt)
            else:
                dt = candidate_dt
                frame_intervals_ms.append(dt * 1000.0)
        if dt is not None:
            for position in range(38):
                if valid[position] and previous_points and previous_points[position]:
                    step_distance = distance(
                        points[position], previous_points[position]  # type: ignore[arg-type]
                    )
                    point_speeds[position].append(step_distance / dt)
                    point_path_lengths[position] += step_distance
                if local_q[position] is not None and previous_local_q:
                    previous_q = previous_local_q[position]
                    if previous_q is not None:
                        angle = quaternion_angle_deg(previous_q, local_q[position])
                        if math.isfinite(angle):
                            angular_speed = angle / dt
                            quaternion_speeds.append(angular_speed)
                            joint_angular_speeds[position].append(angular_speed)

        row: dict[str, Any] = {
            "frame_index": frame.get("frame_index"),
            "timestamp_ns": timestamp_int,
            "body_id": body_id,
            "body_confidence": body_confidence,
            "distance_m": body_distance,
        }
        row.update({f"{name}_ready": ready for name, ready in group_ready.items()})
        rows.append(row)
        previous_points = points
        previous_local_q = local_q
        previous_timestamp = timestamp_int
        previous_body_id = body_id

    usable = len(frames) - malformed
    wall_clock_span_s = (
        (max(timestamps) - min(timestamps)) / 1e9 if len(timestamps) >= 2 else 0.0
    )
    active_duration_s = sum(frame_intervals_ms) / 1000.0
    effective_fps = (
        len(frame_intervals_ms) / active_duration_s
        if active_duration_s > 0
        else None
    )
    wall_clock_fps = (
        (len(timestamps) - 1) / wall_clock_span_s
        if wall_clock_span_s > 0
        else None
    )
    group_percent = {
        name: 100.0 * count / usable if usable else 0.0
        for name, count in group_counts.items()
    }
    confidence_summary = describe(body_confidences, extended=True)
    per_keypoint = {}
    for position, name in enumerate(names):
        per_keypoint[name] = {
            "valid_percent": 100.0 * valid_counts[position] / usable if usable else 0.0,
            "confidence": describe(confidence_values[position], extended=True),
            "frame_to_frame_speed_m_s": describe(
                point_speeds[position], extended=True
            ),
            "local_quaternion_norm_error": describe(
                joint_quaternion_norm_errors[position], extended=True
            ),
            "local_angular_speed_deg_s": describe(
                joint_angular_speeds[position], extended=True
            ),
            "position_range_m": {
                axis: (
                    max(point_axes[position][axis_index])
                    - min(point_axes[position][axis_index])
                    if point_axes[position][axis_index]
                    else None
                )
                for axis_index, axis in enumerate(("x", "y", "z"))
            },
            "observed_path_length_m": point_path_lengths[position],
        }

    bone_summary: dict[str, Any] = {}
    for name, values in bone_lengths.items():
        stats = describe(values, extended=True)
        mean = stats.get("mean")
        std = stats.get("std")
        stats["coefficient_of_variation_percent"] = (
            100.0 * float(std) / float(mean)
            if isinstance(mean, (int, float))
            and isinstance(std, (int, float))
            and mean > 1e-9
            else None
        )
        bone_summary[name] = stats

    classification = readiness_classification(
        active_duration_s,
        effective_fps,
        group_percent,
        confidence_summary.get("mean"),
    )
    median_interval = (
        statistics.median(frame_intervals_ms) if frame_intervals_ms else None
    )
    estimated_dropped_frames = (
        sum(
            max(0, round(interval / median_interval) - 1)
            for interval in frame_intervals_ms
        )
        if median_interval and median_interval > 0
        else 0
    )
    angular_jump_count = sum(value > 500.0 for value in quaternion_speeds)
    summary = {
        "schema": "zed_body38_recording_analysis/v2",
        "source": {
            "body_format": metadata.get("body_format"),
            "zed_sdk_version": metadata.get("zed_sdk_version"),
        },
        "recording": {
            "frame_count": len(frames),
            "usable_frame_count": usable,
            "malformed_frame_count": malformed,
            "invalid_json_or_schema_lines": invalid_lines,
            "duration_s": active_duration_s,
            "wall_clock_span_s": wall_clock_span_s,
            "wall_clock_fps": wall_clock_fps,
            "effective_fps": effective_fps,
            "frame_interval_ms": describe(frame_intervals_ms, extended=True),
            "estimated_dropped_frames": estimated_dropped_frames,
            "segment_count": segment_count,
            "segment_gap_s": describe(segment_gaps_s, extended=True),
            "body_ids": body_ids,
            "tracking_states": tracking_states,
        },
        "threshold": threshold,
        "body_confidence": confidence_summary,
        "distance_m": describe(distances, extended=True),
        "distance_quality": {
            "recommended_range_m": [
                recommended_distance_min_m,
                recommended_distance_max_m,
            ],
            "inside_recommended_range_percent": (
                100.0 * recommended_distance_frames / usable if usable else 0.0
            ),
        },
        "motion_coverage": {
            "geometric_angles_deg": {
                name: describe(values, extended=True)
                for name, values in sorted(geometric_angles.items())
            },
            "note": (
                "Large range indicates motion coverage, not correctness. "
                "Per-keypoint axis ranges and path lengths are under keypoints."
            ),
        },
        "imu": {
            "available_frame_count": imu_frames,
            "coverage_percent": 100.0 * imu_frames / usable if usable else 0.0,
            "sensor_minus_image_timestamp_ms": describe(
                imu_timestamp_offsets_ms, extended=True
            ),
        },
        "native_svo2": {
            "declared_path": metadata.get("native_svo2_path"),
            "declared": bool(metadata.get("native_svo2_path")),
        },
        "group_observability_percent": group_percent,
        "classification": classification,
        "keypoints": per_keypoint,
        "bone_length_consistency_m": bone_summary,
        "orientation_quality": {
            "local_quaternion_norm_error": describe(
                quaternion_norm_errors, extended=True
            ),
            "root_quaternion_norm_error": describe(
                root_quaternion_norm_errors, extended=True
            ),
            "local_angular_speed_deg_s": describe(
                quaternion_speeds, extended=True
            ),
            "angular_jump_over_500_deg_s_count": angular_jump_count,
            "angular_jump_over_500_deg_s_percent": (
                100.0 * angular_jump_count / len(quaternion_speeds)
                if quaternion_speeds
                else 0.0
            ),
            "interpretation": (
                "Yüksek açısal hız gerçek hareket veya takip sıçraması olabilir; "
                "motion-capture referansı olmadan mutlak doğruluk değildir."
            ),
        },
        "dex3_1": {
            "hand": "Unitree Dex3-1, three fingers, 7 actuators per hand",
            "left_source_observability_percent": group_percent["left_dex3_source"],
            "right_source_observability_percent": group_percent["right_dex3_source"],
            "body38_limitation": (
                "BODY_38 her elde yalnızca bilek ile thumb/index/middle/pinky "
                "temsil noktaları verir. Üç parmaklı Dex3-1'in 7 motor açısı bu "
                "noktalardan benzersiz çözülemez. BODY_38 kaba el yönü/açıklığı "
                "için kullanılabilir; iyi parmak taklidi için SVO2 görüntüsünden "
                "ayrı 21-landmark el takibi ve dex-retargeting gerekir."
            ),
        },
        "safety": (
            "Analysis only. No motor commands. Physical G1 deployment requires "
            "joint limits, velocity/acceleration limits, collision checks, "
            "balance control, watchdog, dead-man switch and staged simulation."
        ),
    }
    return summary, rows


def main() -> int:
    parser = argparse.ArgumentParser(
        description="ZED BODY_38 kaydını G1/Dex3 için ayrıntılı analiz eder."
    )
    parser.add_argument("recording", type=Path)
    parser.add_argument("--confidence", type=float, default=50.0)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--distance-min", type=float, default=2.0)
    parser.add_argument("--distance-max", type=float, default=4.0)
    args = parser.parse_args()
    if not args.recording.is_file() or not 0 <= args.confidence <= 100:
        print("Kayıt yolu veya --confidence değeri geçersiz.", file=sys.stderr)
        return 2
    try:
        metadata, frames, invalid = parse_recording(args.recording)
        if not frames:
            raise ValueError("Kayıtta BODY_38 karesi yok.")
        if args.distance_min < 0 or args.distance_max <= args.distance_min:
            raise ValueError("Mesafe aralığı geçersiz.")
        summary, rows = analyze(
            metadata,
            frames,
            invalid,
            args.confidence,
            args.distance_min,
            args.distance_max,
        )
    except (OSError, ValueError) as exc:
        print(f"Analiz başarısız: {exc}", file=sys.stderr)
        return 3

    output_dir = args.output_dir or args.recording.parent / "analysis"
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = args.recording.stem
    summary_path = output_dir / f"{stem}_analysis.json"
    csv_path = output_dir / f"{stem}_frames.csv"
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    with csv_path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    recording = summary["recording"]
    groups = summary["group_observability_percent"]
    result = summary["classification"]
    print(
        f"Kare: {recording['frame_count']} | Süre: {recording['duration_s']:.2f} s "
        f"| FPS: {recording['effective_fps']:.2f}"
    )
    print(
        f"G1 görünürlük: üst gövde %{result['upper_body_observability_percent']:.1f}, "
        f"tüm vücut çekirdek %{result['whole_body_core_observability_percent']:.1f}"
    )
    print(
        f"Dex3 kaynak görünürlüğü: sol %{groups['left_dex3_source']:.1f}, "
        f"sağ %{groups['right_dex3_source']:.1f}"
    )
    print(f"Karar: {result['label']}")
    for reason in result["reasons"]:
        print(f"- {reason}")
    print(f"Özet: {summary_path}")
    print(f"Kare tablosu: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

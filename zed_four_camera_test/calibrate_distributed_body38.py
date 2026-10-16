#!/usr/bin/env python3
"""Estimate static four-camera extrinsics from a captured BODY_38 session.

The input is produced by ``distributed_body38_fusion.py --calibration-record``.
All four cameras must stay fixed and see the same single person.  The result
maps each camera's ``RIGHT_HANDED_Z_UP_X_FWD`` coordinates to the reference
camera's frame; it is consumed only by the application-level fusion receiver.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np

# A script launched by its path has ``zed_four_camera_test`` as sys.path[0],
# whereas motion_pipeline lives at the repository root.  Keep the command-line
# calibration tool independent of the shell's current directory.
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

CORE_NAMES = (
    "PELVIS", "SPINE_1", "SPINE_2", "SPINE_3", "NECK",
    "LEFT_CLAVICLE", "RIGHT_CLAVICLE", "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST",
    "LEFT_HIP", "RIGHT_HIP", "LEFT_KNEE", "RIGHT_KNEE",
    "LEFT_ANKLE", "RIGHT_ANKLE",
)

TORSO_NAMES = (
    "PELVIS", "SPINE_1", "SPINE_2", "SPINE_3", "NECK",
    "LEFT_CLAVICLE", "RIGHT_CLAVICLE", "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_HIP", "RIGHT_HIP",
)

# Stable torso landmarks drive the camera pose.  Distal limbs still contribute
# to validation, but an occluded wrist must not pull a tripod by centimetres.
JOINT_WEIGHTS = {
    "PELVIS": 6.0,
    "SPINE_1": 5.0,
    "SPINE_2": 5.0,
    "SPINE_3": 5.0,
    "NECK": 3.0,
    "LEFT_HIP": 4.0,
    "RIGHT_HIP": 4.0,
    "LEFT_CLAVICLE": 3.0,
    "RIGHT_CLAVICLE": 3.0,
    "LEFT_SHOULDER": 3.0,
    "RIGHT_SHOULDER": 3.0,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dortlu BODY_38 ham kaydindan statik kamera extrinsic olusturur")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--reference-serial", type=int, required=True)
    parser.add_argument("--confidence", type=float, default=55.0)
    parser.add_argument("--max-residual-m", type=float, default=0.16)
    # A distributed capture can contain short periods where one view is
    # geometrically weaker.  Using the whole ordinary 45-90 second session is
    # more stable than an arbitrary first-900-frame prefix; the wrapper caps
    # unusually long sessions explicitly when needed.
    parser.add_argument("--max-samples", type=int, default=5000)
    parser.add_argument("--min-inlier-ratio", type=float, default=0.25)
    parser.add_argument("--min-inlier-points", type=int, default=300)
    parser.add_argument("--min-capture-samples", type=int, default=60)
    parser.add_argument("--max-pelvis-p95-m", type=float, default=0.25)
    parser.add_argument("--max-pair-delta-ms", type=float, default=20.0)
    parser.add_argument("--temporal-offset-search-ms", type=float, default=40.0)
    parser.add_argument("--temporal-offset-step-ms", type=float, default=1.0)
    parser.add_argument("--holdout-ratio", type=float, default=0.25)
    parser.add_argument("--min-holdout-samples", type=int, default=80)
    parser.add_argument("--max-holdout-torso-p95-m", type=float, default=0.12)
    parser.add_argument(
        "--max-holdout-all-p95-m", type=float, default=0.0,
        help="Istege bagli tum-eklem sanity kapisi; 0 yalniz raporlar.",
    )
    parser.add_argument("--max-systematic-bias-m", type=float, default=0.05)
    parser.add_argument("--require-imu-tilt", action="store_true")
    parser.add_argument("--max-imu-tilt-correction-deg", type=float, default=10.0)
    parser.add_argument("--ptp-record", type=Path, action="append", default=[])
    parser.add_argument("--require-identical-source-configuration", action="store_true")
    parser.add_argument(
        "--min-pelvis-motion-m", type=float, default=0.10,
        help=(
            "Referans operator pelvisinin kalibrasyon boyunca yapmasi gereken "
            "asgari saglam hareket genisligi. Yanlis/sabit kisi kilidini yakalar."
        ),
    )
    parser.add_argument(
        "--min-pelvis-motion-ratio", type=float, default=0.60,
        help="Kamera ve referans pelvis hareket genisliklerinin asgari orani.",
    )
    parser.add_argument(
        "--min-pelvis-trajectory-correlation", type=float, default=0.75,
        help="Donusturulmus kamera ile referans pelvis yollari arasindaki asgari korelasyon.",
    )
    parser.add_argument(
        "--world-poses-jsonl",
        type=Path,
        default=None,
        help="Kamera-dunya pozlarini satir bazli yaz. Varsayilan: OUTPUT yaninda *_world_poses.jsonl.",
    )
    return parser.parse_args()


def matrix_to_quaternion_xyzw(matrix: np.ndarray) -> list[float]:
    value = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(value))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quat = np.array([
            (value[2, 1] - value[1, 2]) / scale,
            (value[0, 2] - value[2, 0]) / scale,
            (value[1, 0] - value[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(value)))
        if index == 0:
            scale = math.sqrt(max(1.0e-12, 1.0 + value[0, 0] - value[1, 1] - value[2, 2])) * 2.0
            quat = np.array([0.25 * scale, (value[0, 1] + value[1, 0]) / scale,
                             (value[0, 2] + value[2, 0]) / scale, (value[2, 1] - value[1, 2]) / scale])
        elif index == 1:
            scale = math.sqrt(max(1.0e-12, 1.0 + value[1, 1] - value[0, 0] - value[2, 2])) * 2.0
            quat = np.array([(value[0, 1] + value[1, 0]) / scale, 0.25 * scale,
                             (value[1, 2] + value[2, 1]) / scale, (value[0, 2] - value[2, 0]) / scale])
        else:
            scale = math.sqrt(max(1.0e-12, 1.0 + value[2, 2] - value[0, 0] - value[1, 1])) * 2.0
            quat = np.array([(value[0, 2] + value[2, 0]) / scale, (value[1, 2] + value[2, 1]) / scale,
                             0.25 * scale, (value[1, 0] - value[0, 1]) / scale])
    quat /= max(float(np.linalg.norm(quat)), 1.0e-12)
    return quat.tolist()


def write_world_poses_jsonl(
    path: Path,
    *,
    output: dict[str, Any],
    source_ports: dict[int, int],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [{
        "schema": "zed_camera_world_poses_metadata/v1",
        "created_unix_ns": output["created_unix_ns"],
        "coordinate_system": output["coordinate_system"],
        "units": output["units"],
        "reference_world_serial": output["reference_world_serial"],
        "source_capture": output["source_capture"],
        "transform_semantics": "point_world = rotation_camera_to_world @ point_camera + translation_camera_to_world_m",
    }]
    for serial_text, camera in output["cameras"].items():
        serial = int(serial_text)
        rotation = np.asarray(camera["rotation_camera_to_world"], dtype=np.float64)
        translation = np.asarray(camera["translation_camera_to_world_m"], dtype=np.float64)
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation
        lines.append({
            "schema": "zed_camera_world_pose/v1",
            "serial": serial,
            "source_udp_port": source_ports.get(serial),
            "is_reference": serial == int(output["reference_world_serial"]),
            "camera_optical_origin_world_m": translation.tolist(),
            "rotation_camera_to_world": rotation.tolist(),
            "orientation_camera_to_world_xyzw": matrix_to_quaternion_xyzw(rotation),
            "transform_camera_to_world_4x4": transform.tolist(),
            "fit": camera.get("fit", {}),
        })
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        for item in lines:
            handle.write(json.dumps(item, ensure_ascii=False, allow_nan=False) + "\n")


def read_samples(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    metadata: dict[str, Any] = {}
    samples: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for number, line in enumerate(handle, start=1):
            try:
                item = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"Satir {number} JSON degil: {exc}") from exc
            schema = item.get("schema") if isinstance(item, dict) else None
            if schema == "zed_body38_multihost_calibration_metadata/v1":
                metadata = item
            elif schema == "zed_body38_multihost_calibration_sample/v1":
                samples.append(item)
    if not metadata:
        raise ValueError("Kalibrasyon metadata satiri bulunamadi.")
    if not samples:
        raise ValueError("Kalibrasyon ornek karesi bulunamadi.")
    return metadata, samples


def array(value: Any, shape: tuple[int, ...]) -> np.ndarray | None:
    try:
        result = np.asarray(value, dtype=np.float64)
        if result.shape == shape:
            return result
    except (TypeError, ValueError):
        pass
    return None


def packet_time_ns(packet: dict[str, Any]) -> int:
    return int(
        packet.get("normalized_capture_timestamp_ns")
        or packet.get("source_timestamp_ns")
        or packet.get("receiver_timestamp_ns")
        or 0
    )


def unique_camera_packets(
    samples: list[dict[str, Any]], serial: int, maximum_samples: int
) -> list[dict[str, Any]]:
    unique: dict[tuple[int, int], dict[str, Any]] = {}
    for sample in samples[:maximum_samples]:
        sources = sample.get("sources", {})
        packet = sources.get(str(serial)) if isinstance(sources, dict) else None
        if not isinstance(packet, dict):
            continue
        timestamp = packet_time_ns(packet)
        if timestamp <= 0:
            continue
        sequence = int(packet.get("sequence", -1) or -1)
        unique[(sequence, timestamp)] = packet
    return sorted(unique.values(), key=packet_time_ns)


def pair_camera_packets(
    samples: list[dict[str, Any]],
    *,
    reference_serial: int,
    target_serial: int,
    maximum_samples: int,
    temporal_offset_ms: float,
    maximum_pair_delta_ms: float,
) -> list[tuple[dict[str, Any], dict[str, Any], float]]:
    references = unique_camera_packets(samples, reference_serial, maximum_samples)
    targets = unique_camera_packets(samples, target_serial, maximum_samples)
    reference_times = [packet_time_ns(packet) for packet in references]
    offset_ns = int(round(float(temporal_offset_ms) * 1.0e6))
    maximum_delta_ns = int(round(float(maximum_pair_delta_ms) * 1.0e6))
    paired: list[tuple[dict[str, Any], dict[str, Any], float]] = []
    used_reference: set[tuple[int, int]] = set()
    for target in targets:
        aligned_time = packet_time_ns(target) + offset_ns
        index = bisect_left(reference_times, aligned_time)
        candidates = [value for value in (index - 1, index) if 0 <= value < len(references)]
        if not candidates:
            continue
        best_index = min(candidates, key=lambda value: abs(reference_times[value] - aligned_time))
        delta_ns = abs(reference_times[best_index] - aligned_time)
        reference = references[best_index]
        reference_key = (int(reference.get("sequence", -1) or -1), reference_times[best_index])
        if delta_ns > maximum_delta_ns or reference_key in used_reference:
            continue
        used_reference.add(reference_key)
        paired.append((target, reference, delta_ns / 1.0e6))
    return sorted(paired, key=lambda item: packet_time_ns(item[1]))


def collect_paired_points(
    pairs: list[tuple[dict[str, Any], dict[str, Any], float]],
    *,
    threshold: float,
    names_to_use: tuple[str, ...],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, int]:
    source_rows: list[np.ndarray] = []
    target_rows: list[np.ndarray] = []
    weights: list[float] = []
    pelvis_pairs: list[np.ndarray] = []
    used_frames = 0
    for source_packet, reference_packet, _delta_ms in pairs:
        names = reference_packet.get("keypoint_names")
        if names != source_packet.get("keypoint_names") or not isinstance(names, list):
            continue
        indices = {str(name): index for index, name in enumerate(names)}
        required = [name for name in names_to_use if name in indices]
        reference_points = array(reference_packet.get("keypoints_3d_m"), (len(names), 3))
        source_points = array(source_packet.get("keypoints_3d_m"), (len(names), 3))
        reference_confidence = array(reference_packet.get("keypoint_confidence"), (len(names),))
        source_confidence = array(source_packet.get("keypoint_confidence"), (len(names),))
        if any(item is None for item in (reference_points, source_points, reference_confidence, source_confidence)):
            continue
        keep = [
            name for name in required
            if np.isfinite(reference_points[indices[name]]).all()
            and np.isfinite(source_points[indices[name]]).all()
            and reference_confidence[indices[name]] >= threshold
            and source_confidence[indices[name]] >= threshold
        ]
        if len(keep) < 6:
            continue
        frame_weights = np.asarray([JOINT_WEIGHTS.get(name, 1.0) for name in keep])
        frame_weights /= max(float(np.sum(frame_weights)), 1.0e-12)
        for name, weight in zip(keep, frame_weights):
            index = indices[name]
            source_rows.append(source_points[index])
            target_rows.append(reference_points[index])
            # Every frame has equal total influence; confidence only provides
            # a bounded modulation above the validity threshold.
            confidence_scale = math.sqrt(
                float(source_confidence[index] * reference_confidence[index])
            ) / 100.0
            weights.append(float(weight * np.clip(confidence_scale, 0.55, 1.0)))
        pelvis_index = indices.get("PELVIS")
        if pelvis_index is not None and "PELVIS" in keep:
            pelvis_pairs.append(np.stack((source_points[pelvis_index], reference_points[pelvis_index])))
        used_frames += 1
    return (
        np.asarray(source_rows, dtype=np.float64).reshape((-1, 3)),
        np.asarray(target_rows, dtype=np.float64).reshape((-1, 3)),
        np.asarray(weights, dtype=np.float64),
        np.asarray(pelvis_pairs, dtype=np.float64).reshape((-1, 2, 3)),
        used_frames,
    )


def weighted_rigid_alignment(
    source: np.ndarray,
    target: np.ndarray,
    weights: np.ndarray,
    *,
    maximum_residual_m: float,
) -> tuple[np.ndarray, np.ndarray, dict[str, float | int]] | None:
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3 or len(source) < 12:
        return None
    base = np.asarray(weights, dtype=np.float64)
    if base.shape != (len(source),) or not np.isfinite(base).all() or float(np.sum(base)) <= 0.0:
        return None
    robust = np.ones_like(base)
    rotation = np.eye(3)
    translation = np.zeros(3)
    for _ in range(6):
        effective = base * robust
        total = float(np.sum(effective))
        source_center = np.sum(source * effective[:, None], axis=0) / total
        target_center = np.sum(target * effective[:, None], axis=0) / total
        covariance = (source - source_center).T @ ((target - target_center) * effective[:, None])
        u, _singular, vt = np.linalg.svd(covariance)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0.0:
            vt[-1] *= -1.0
            rotation = vt.T @ u.T
        translation = target_center - rotation @ source_center
        residual = np.linalg.norm(source @ rotation.T + translation - target, axis=1)
        # Huber reweighting preserves every valid torso observation instead of
        # repeatedly trimming away the pelvis until a deceptively small p95 is
        # obtained.
        huber_m = min(0.08, float(maximum_residual_m))
        robust = np.ones_like(residual)
        outside = residual > huber_m
        robust[outside] = huber_m / np.maximum(residual[outside], 1.0e-9)
    residual = np.linalg.norm(source @ rotation.T + translation - target, axis=1)
    inliers = residual <= float(maximum_residual_m)
    if int(np.count_nonzero(inliers)) < 12:
        return None
    selected = residual[inliers]
    return rotation, translation, {
        "paired_keypoints": int(np.count_nonzero(inliers)),
        "rms_m": float(np.sqrt(np.mean(selected * selected))),
        "median_m": float(np.median(selected)),
        "p95_m": float(np.percentile(selected, 95)),
        "all_valid_rms_m": float(np.sqrt(np.mean(residual * residual))),
        "all_valid_median_m": float(np.median(residual)),
        "all_valid_p95_m": float(np.percentile(residual, 95)),
    }


def quaternion_matrix_xyzw(value: Any) -> np.ndarray | None:
    quat = array(value, (4,))
    if quat is None or not np.isfinite(quat).all():
        return None
    norm = float(np.linalg.norm(quat))
    if norm < 1.0e-9:
        return None
    x, y, z, w = quat / norm
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def median_camera_up(
    packets: list[dict[str, Any]]
) -> tuple[np.ndarray | None, int, float | None]:
    values: list[np.ndarray] = []
    world_up = np.asarray([0.0, 0.0, 1.0])
    for packet in packets:
        imu = packet.get("imu")
        matrix = quaternion_matrix_xyzw(imu.get("orientation_xyzw")) if isinstance(imu, dict) else None
        if matrix is not None:
            # IMU orientation is sensor-local to its gravity-aligned world;
            # express that world's up axis in the camera/sensor frame.
            values.append(matrix.T @ world_up)
    if not values:
        return None, 0, None
    up = np.median(np.asarray(values), axis=0)
    norm = float(np.linalg.norm(up))
    if norm <= 1.0e-9:
        return None, len(values), None
    up /= norm
    deviations = np.degrees(np.arccos(np.clip(np.asarray(values) @ up, -1.0, 1.0)))
    return up, len(values), float(np.percentile(deviations, 95))


def rotation_between_vectors(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    a = source / max(float(np.linalg.norm(source)), 1.0e-12)
    b = target / max(float(np.linalg.norm(target)), 1.0e-12)
    cross = np.cross(a, b)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(np.dot(a, b), -1.0, 1.0))
    if sine < 1.0e-9:
        if cosine > 0.0:
            return np.eye(3)
        axis = np.cross(a, np.asarray([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1.0e-6:
            axis = np.cross(a, np.asarray([0.0, 1.0, 0.0]))
        axis /= np.linalg.norm(axis)
        return 2.0 * np.outer(axis, axis) - np.eye(3)
    axis = cross / sine
    skew = np.asarray([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    angle = math.atan2(sine, cosine)
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def apply_imu_tilt_constraint(
    rotation: np.ndarray,
    translation: np.ndarray,
    source: np.ndarray,
    target: np.ndarray,
    weights: np.ndarray,
    source_packets: list[dict[str, Any]],
    reference_packets: list[dict[str, Any]],
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    source_up, source_count, source_stability = median_camera_up(source_packets)
    reference_up, reference_count, reference_stability = median_camera_up(reference_packets)
    if source_up is None or reference_up is None:
        return rotation, translation, {
            "available": False,
            "source_samples": source_count,
            "reference_samples": reference_count,
            "source_up_stability_p95_deg": source_stability,
            "reference_up_stability_p95_deg": reference_stability,
            "correction_deg": None,
        }
    mapped_up = rotation @ source_up
    correction = rotation_between_vectors(mapped_up, reference_up)
    correction_deg = math.degrees(math.acos(float(np.clip(np.dot(mapped_up, reference_up), -1.0, 1.0))))
    constrained = correction @ rotation
    total = float(np.sum(weights))
    constrained_translation = np.sum(
        (target - source @ constrained.T) * weights[:, None], axis=0
    ) / max(total, 1.0e-12)
    return constrained, constrained_translation, {
        "available": True,
        "source_samples": source_count,
        "reference_samples": reference_count,
        "source_up_stability_p95_deg": source_stability,
        "reference_up_stability_p95_deg": reference_stability,
        "correction_deg": correction_deg,
        "residual_tilt_deg": 0.0,
    }


def evaluate_points(
    source: np.ndarray,
    target: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> dict[str, Any]:
    if not len(source):
        return {"count": 0, "rms_m": None, "median_m": None, "p95_m": None, "mean_bias_m": None}
    residual_vector = source @ rotation.T + translation - target
    residual = np.linalg.norm(residual_vector, axis=1)
    return {
        "count": int(len(residual)),
        "rms_m": float(np.sqrt(np.mean(residual * residual))),
        "median_m": float(np.median(residual)),
        "p95_m": float(np.percentile(residual, 95)),
        "mean_bias_m": float(np.linalg.norm(np.mean(residual_vector, axis=0))),
        "mean_bias_xyz_m": np.mean(residual_vector, axis=0).tolist(),
    }


def estimate_temporal_offset_ms(
    samples: list[dict[str, Any]],
    *,
    reference_serial: int,
    target_serial: int,
    threshold: float,
    maximum_samples: int,
    maximum_pair_delta_ms: float,
    search_ms: float,
    step_ms: float,
) -> tuple[float, dict[str, Any]]:
    candidates = np.arange(-search_ms, search_ms + 0.5 * step_ms, step_ms)
    evaluated: list[dict[str, Any]] = []
    for offset in candidates:
        pairs = pair_camera_packets(
            samples,
            reference_serial=reference_serial,
            target_serial=target_serial,
            maximum_samples=maximum_samples,
            temporal_offset_ms=float(offset),
            maximum_pair_delta_ms=maximum_pair_delta_ms,
        )
        if len(pairs) < 20:
            continue
        split = max(12, int(round(0.70 * len(pairs))))
        train = pairs[:split]
        holdout = pairs[split:]
        source, target, weights, _pelvis, _used = collect_paired_points(
            train, threshold=threshold, names_to_use=TORSO_NAMES
        )
        fit = weighted_rigid_alignment(source, target, weights, maximum_residual_m=0.20)
        if fit is None or len(holdout) < 6:
            continue
        rotation, translation, _metrics = fit
        hs, ht, _hw, _hp, _hu = collect_paired_points(
            holdout, threshold=threshold, names_to_use=TORSO_NAMES
        )
        metrics = evaluate_points(hs, ht, rotation, translation)
        if metrics["p95_m"] is None:
            continue
        evaluated.append({
            "offset_ms": float(offset),
            "paired_frames": len(pairs),
            "holdout_torso_p95_m": float(metrics["p95_m"]),
        })
    if not evaluated:
        return 0.0, {"evaluated_candidates": 0, "reason": "insufficient_timestamp_pairs"}
    maximum_count = max(item["paired_frames"] for item in evaluated)
    eligible = [item for item in evaluated if item["paired_frames"] >= 0.80 * maximum_count]
    best = min(
        eligible,
        key=lambda item: (
            item["holdout_torso_p95_m"] + 0.00005 * abs(item["offset_ms"]),
            abs(item["offset_ms"]),
        ),
    )
    return float(best["offset_ms"]), {
        "evaluated_candidates": len(evaluated),
        "maximum_paired_frames": maximum_count,
        "selected": best,
    }


def pelvis_trajectory_metrics(
    pelvis_pairs: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> dict[str, float]:
    """Measure whether both cameras tracked the same moving pelvis.

    A rigid fit over many BODY_38 joints can appear numerically acceptable
    even when one camera stays locked to a different, mostly stationary
    person.  Translation/rotation cannot change trajectory scale; comparing
    the centered pelvis paths therefore catches that cross-person failure.
    """
    if len(pelvis_pairs) < 3:
        return {
            "pelvis_source_motion_m": 0.0,
            "pelvis_reference_motion_m": 0.0,
            "pelvis_motion_scale_ratio": 0.0,
            "pelvis_trajectory_correlation": 0.0,
        }
    source = pelvis_pairs[:, 0] @ rotation.T + translation
    reference = pelvis_pairs[:, 1]

    def robust_motion_extent(values: np.ndarray) -> float:
        centered_radius = np.linalg.norm(
            values - np.median(values, axis=0), axis=1
        )
        return float(
            np.percentile(centered_radius, 95)
            - np.percentile(centered_radius, 5)
        )

    source_motion = robust_motion_extent(source)
    reference_motion = robust_motion_extent(reference)
    maximum_motion = max(source_motion, reference_motion, 1.0e-9)
    motion_ratio = min(source_motion, reference_motion) / maximum_motion
    source_centered = source - np.mean(source, axis=0)
    reference_centered = reference - np.mean(reference, axis=0)
    denominator = float(
        np.linalg.norm(source_centered) * np.linalg.norm(reference_centered)
    )
    correlation = (
        float(np.sum(source_centered * reference_centered) / denominator)
        if denominator > 1.0e-9
        else 0.0
    )
    return {
        "pelvis_source_motion_m": source_motion,
        "pelvis_reference_motion_m": reference_motion,
        "pelvis_motion_scale_ratio": float(np.clip(motion_ratio, 0.0, 1.0)),
        "pelvis_trajectory_correlation": float(np.clip(correlation, -1.0, 1.0)),
    }


def format_metric(value: Any, digits: int = 3) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "N/A"
    return f"{number:.{digits}f}" if math.isfinite(number) else "N/A"


def file_evidence(paths: list[Path]) -> list[dict[str, Any]]:
    result = []
    for raw_path in paths:
        path = raw_path.expanduser().resolve()
        content = path.read_bytes()
        result.append({
            "path": str(path),
            "sha256": hashlib.sha256(content).hexdigest(),
            "bytes": len(content),
        })
    return result


def source_configurations(
    samples: list[dict[str, Any]], serials: list[int]
) -> dict[str, dict[str, Any] | None]:
    result: dict[str, dict[str, Any] | None] = {str(serial): None for serial in serials}
    for sample in samples:
        sources = sample.get("sources", {})
        if not isinstance(sources, dict):
            continue
        for serial in serials:
            packet = sources.get(str(serial))
            config = packet.get("source_configuration") if isinstance(packet, dict) else None
            if isinstance(config, dict) and result[str(serial)] is None:
                result[str(serial)] = config
        if all(value is not None for value in result.values()):
            break
    return result


def main() -> int:
    args = parse_args()
    if (
        not 0.0 <= args.confidence <= 100.0
        or args.max_residual_m <= 0.0
        or not 0.0 < args.min_inlier_ratio <= 1.0
        or args.min_inlier_points < 12
        or args.min_capture_samples < 1
        or args.max_pelvis_p95_m <= 0.0
        or args.max_pair_delta_ms <= 0.0
        or args.temporal_offset_search_ms < 0.0
        or args.temporal_offset_step_ms <= 0.0
        or not 0.05 <= args.holdout_ratio <= 0.50
        or args.min_holdout_samples < 1
        or args.max_holdout_torso_p95_m <= 0.0
        or args.max_holdout_all_p95_m < 0.0
        or args.max_systematic_bias_m <= 0.0
        or args.max_imu_tilt_correction_deg <= 0.0
        or args.min_pelvis_motion_m <= 0.0
        or not 0.0 < args.min_pelvis_motion_ratio <= 1.0
        or not -1.0 <= args.min_pelvis_trajectory_correlation <= 1.0
    ):
        print("HATA: confidence veya max-residual-m gecersiz.", file=sys.stderr)
        return 2
    try:
        metadata, samples = read_samples(args.input.expanduser().resolve())
        ptp_evidence = file_evidence(args.ptp_record)
    except (OSError, ValueError) as exc:
        print(f"HATA: {exc}", file=sys.stderr)
        return 2
    serials = [int(item["serial"]) for item in metadata.get("sources", []) if isinstance(item, dict) and "serial" in item]
    if len(serials) != 4 or len(set(serials)) != 4:
        print("HATA: kayit dort benzersiz ZED serisi icermiyor.", file=sys.stderr)
        return 2
    if args.reference_serial not in serials:
        print(f"HATA: referans ZED {args.reference_serial} kayitta yok.", file=sys.stderr)
        return 2
    configurations = source_configurations(samples, serials)
    available_configurations = [value for value in configurations.values() if value is not None]
    if args.require_identical_source_configuration and (
        len(available_configurations) != 4
        or any(value != available_configurations[0] for value in available_configurations[1:])
    ):
        print(
            "HATA: Dort kaynakta ayni FPS/body-model/depth ayari dogrulanamadi: "
            f"{configurations}",
            file=sys.stderr,
        )
        return 2

    cameras: dict[str, Any] = {
        str(args.reference_serial): {
            "rotation_camera_to_world": np.eye(3).tolist(),
            "translation_camera_to_world_m": [0.0, 0.0, 0.0],
            "fit": {"reference": True},
        }
    }
    failures = 0
    for serial in serials:
        if serial == args.reference_serial:
            continue
        temporal_offset_ms, temporal_search = estimate_temporal_offset_ms(
            samples,
            reference_serial=args.reference_serial,
            target_serial=serial,
            threshold=args.confidence,
            maximum_samples=args.max_samples,
            maximum_pair_delta_ms=args.max_pair_delta_ms,
            search_ms=args.temporal_offset_search_ms,
            step_ms=args.temporal_offset_step_ms,
        )
        pairs = pair_camera_packets(
            samples,
            reference_serial=args.reference_serial,
            target_serial=serial,
            maximum_samples=args.max_samples,
            temporal_offset_ms=temporal_offset_ms,
            maximum_pair_delta_ms=args.max_pair_delta_ms,
        )
        split_index = int(math.floor((1.0 - args.holdout_ratio) * len(pairs)))
        training_pairs = pairs[:split_index]
        holdout_pairs = pairs[split_index:]
        source, target, weights, training_pelvis, used_samples = collect_paired_points(
            training_pairs, threshold=args.confidence, names_to_use=TORSO_NAMES
        )
        fit = weighted_rigid_alignment(
            source, target, weights, maximum_residual_m=args.max_residual_m
        )
        if fit is None:
            print(
                f"HATA: ZED {serial} icin yeterli zaman-eslesmis torso noktasi yok "
                f"(egitim={used_samples}, nokta={len(source)}).",
                file=sys.stderr,
            )
            failures += 1
            continue
        rotation, translation, metrics = fit
        rotation, translation, imu_tilt = apply_imu_tilt_constraint(
            rotation,
            translation,
            source,
            target,
            weights,
            [pair[0] for pair in training_pairs],
            [pair[1] for pair in training_pairs],
        )
        train_torso = evaluate_points(source, target, rotation, translation)
        holdout_source, holdout_target, _holdout_weights, holdout_pelvis, holdout_used = collect_paired_points(
            holdout_pairs, threshold=args.confidence, names_to_use=TORSO_NAMES
        )
        holdout_torso = evaluate_points(
            holdout_source, holdout_target, rotation, translation
        )
        train_all_source, train_all_target, _train_all_weights, _train_all_pelvis, _train_all_used = collect_paired_points(
            training_pairs, threshold=args.confidence, names_to_use=CORE_NAMES
        )
        holdout_all_source, holdout_all_target, _holdout_all_weights, _holdout_all_pelvis, _holdout_all_used = collect_paired_points(
            holdout_pairs, threshold=args.confidence, names_to_use=CORE_NAMES
        )
        training_all_metrics = evaluate_points(
            train_all_source, train_all_target, rotation, translation
        )
        holdout_all_metrics = evaluate_points(
            holdout_all_source, holdout_all_target, rotation, translation
        )
        candidate_points = int(len(source))
        train_residual = np.linalg.norm(
            source @ rotation.T + translation - target, axis=1
        ) if len(source) else np.empty(0)
        paired_keypoints = int(np.count_nonzero(train_residual <= args.max_residual_m))
        inlier_errors = train_residual[train_residual <= args.max_residual_m]
        inlier_ratio = float(paired_keypoints / candidate_points) if candidate_points else 0.0
        pelvis_errors = (
            np.linalg.norm(
                holdout_pelvis[:, 0] @ rotation.T + translation - holdout_pelvis[:, 1],
                axis=1,
            )
            if len(holdout_pelvis)
            else np.empty(0, dtype=np.float64)
        )
        pelvis_median_m = float(np.median(pelvis_errors)) if pelvis_errors.size else None
        pelvis_p95_m = float(np.percentile(pelvis_errors, 95)) if pelvis_errors.size else None
        all_pelvis = np.concatenate((training_pelvis, holdout_pelvis), axis=0)
        trajectory = pelvis_trajectory_metrics(all_pelvis, rotation, translation)
        pair_delta_ms = np.asarray([pair[2] for pair in pairs], dtype=np.float64)
        minimum_imu_samples = max(30, int(math.ceil(0.50 * used_samples)))
        imu_tilt["minimum_required_samples"] = minimum_imu_samples
        imu_ok = bool(
            (not args.require_imu_tilt)
            or (
                imu_tilt.get("available") is True
                and int(imu_tilt.get("source_samples", 0)) >= minimum_imu_samples
                and int(imu_tilt.get("reference_samples", 0)) >= minimum_imu_samples
                and float(imu_tilt.get("correction_deg", math.inf))
                <= args.max_imu_tilt_correction_deg
            )
        )
        holdout_all_p95 = holdout_all_metrics.get("p95_m")
        holdout_torso_p95 = holdout_torso.get("p95_m")
        holdout_bias = holdout_torso.get("mean_bias_m")
        quality_ok = bool(
            used_samples >= args.min_capture_samples
            and holdout_used >= args.min_holdout_samples
            and paired_keypoints >= args.min_inlier_points
            and inlier_ratio >= args.min_inlier_ratio
            and float(train_torso["rms_m"]) <= args.max_residual_m
            and pelvis_p95_m is not None
            and pelvis_p95_m <= args.max_pelvis_p95_m
            and holdout_torso_p95 is not None
            and float(holdout_torso_p95) <= args.max_holdout_torso_p95_m
            and holdout_all_p95 is not None
            and (
                args.max_holdout_all_p95_m <= 0.0
                or float(holdout_all_p95) <= args.max_holdout_all_p95_m
            )
            and holdout_bias is not None
            and float(holdout_bias) <= args.max_systematic_bias_m
            and imu_ok
            and trajectory["pelvis_reference_motion_m"] >= args.min_pelvis_motion_m
            and trajectory["pelvis_motion_scale_ratio"] >= args.min_pelvis_motion_ratio
            and trajectory["pelvis_trajectory_correlation"]
            >= args.min_pelvis_trajectory_correlation
        )
        fit_metrics = {
            "capture_samples": used_samples,
            "total_paired_samples": len(pairs),
            "holdout_capture_samples": holdout_used,
            "holdout_ratio": args.holdout_ratio,
            "candidate_keypoints": candidate_points,
            "paired_keypoints": paired_keypoints,
            "inlier_ratio": inlier_ratio,
            "pelvis_frames": int(pelvis_errors.size),
            "pelvis_median_m": pelvis_median_m,
            "pelvis_p95_m": pelvis_p95_m,
            "temporal_offset_ms": temporal_offset_ms,
            "temporal_offset_search": temporal_search,
            "pair_delta_ms": {
                "median": float(np.median(pair_delta_ms)) if pair_delta_ms.size else None,
                "p95": float(np.percentile(pair_delta_ms, 95)) if pair_delta_ms.size else None,
                "maximum": float(np.max(pair_delta_ms)) if pair_delta_ms.size else None,
            },
            "imu_tilt_constraint": imu_tilt,
            "training_torso_all_valid": train_torso,
            "holdout_torso_all_valid": holdout_torso,
            "training_full_body_all_valid": training_all_metrics,
            "holdout_full_body_all_valid": holdout_all_metrics,
            "quality_gate_passed": quality_ok,
            **trajectory,
            "rms_m": float(np.sqrt(np.mean(inlier_errors ** 2))) if inlier_errors.size else None,
            "median_m": float(np.median(inlier_errors)) if inlier_errors.size else None,
            "p95_m": float(np.percentile(inlier_errors, 95)) if inlier_errors.size else None,
            "all_valid_rms_m": train_torso["rms_m"],
            "all_valid_median_m": train_torso["median_m"],
            "all_valid_p95_m": train_torso["p95_m"],
        }
        cameras[str(serial)] = {
            "rotation_camera_to_world": rotation.tolist(),
            "translation_camera_to_world_m": translation.tolist(),
            "fit": fit_metrics,
        }
        print(
            f"ZED {serial} -> {args.reference_serial} | egitim/holdout="
            f"{used_samples}/{holdout_used} | offset={temporal_offset_ms:+.1f}ms "
            f"| nokta={paired_keypoints}/{candidate_points} "
            f"({100.0 * inlier_ratio:.1f}%) | rms={train_torso['rms_m']:.3f}m "
            f"holdout_torso_p95={format_metric(holdout_torso['p95_m'])}m "
            f"holdout_tum_p95={format_metric(holdout_all_p95)}m | "
            f"pelvis_p95={format_metric(pelvis_p95_m)}m "
            f"| pelvis_hareket={trajectory['pelvis_source_motion_m']:.3f}/"
            f"{trajectory['pelvis_reference_motion_m']:.3f}m "
            f"oran={trajectory['pelvis_motion_scale_ratio']:.2f} "
            f"korelasyon={trajectory['pelvis_trajectory_correlation']:.2f}"
        )
        if not quality_ok:
            print(
                f"HATA: ZED {serial} kalibrasyon kalite kapisini gecemedi "
                f"(egitim >= {args.min_capture_samples}, holdout >= {args.min_holdout_samples}, "
                f"{args.min_inlier_points} nokta, "
                f"%{100.0 * args.min_inlier_ratio:.0f} inlier, pelvis p95 <= "
                f"{args.max_pelvis_p95_m:.2f} m, holdout torso p95 <= "
                f"{args.max_holdout_torso_p95_m:.2f} m, sistematik bias <= "
                f"{args.max_systematic_bias_m:.2f} m, referans pelvis hareketi >= "
                f"{args.min_pelvis_motion_m:.2f} m, hareket orani >= "
                f"{args.min_pelvis_motion_ratio:.2f}, yol korelasyonu >= "
                f"{args.min_pelvis_trajectory_correlation:.2f} gerekli).",
                file=sys.stderr,
            )
            failures += 1
    output = {
        "schema": "zed_body38_distributed_extrinsics/v1",
        "created_unix_ns": time.time_ns(),
        "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
        "units": "meter",
        "reference_world_serial": args.reference_serial,
        "source_capture": str(args.input.expanduser().resolve()),
        "ptp_evidence": ptp_evidence,
        "source_configurations": configurations,
        "method": "weighted_torso_kabsch_imu_temporal_holdout_v2",
        "calibration_policy": {
            "maximum_pair_delta_ms": args.max_pair_delta_ms,
            "temporal_offset_search_ms": args.temporal_offset_search_ms,
            "holdout_ratio": args.holdout_ratio,
            "imu_tilt_required": args.require_imu_tilt,
            "maximum_holdout_torso_p95_m": args.max_holdout_torso_p95_m,
            "maximum_holdout_full_body_p95_m": (
                args.max_holdout_all_p95_m if args.max_holdout_all_p95_m > 0.0 else None
            ),
            "maximum_systematic_bias_m": args.max_systematic_bias_m,
        },
        "cameras": cameras,
        "warning": "Tripodlar veya kameralar hareket ederse bu dosya gecersizdir; yeniden kalibre edin.",
    }
    path = args.output.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    if failures or len(cameras) != 4:
        rejected_path = path.with_name(path.stem + ".rejected_diagnostics.json")
        rejected = dict(output)
        rejected["quality_gate_passed"] = False
        rejected["failure_count"] = failures + max(0, 4 - len(cameras))
        rejected_path.write_text(
            json.dumps(rejected, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(
            "Kalibrasyon dosyasi etkinlestirilmedi; dusuk hatali fit gerekli. "
            f"Tani raporu: {rejected_path}",
            file=sys.stderr,
        )
        return 3

    output["quality_gate_passed"] = True
    path.write_text(json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"KALIBRASYON HAZIR: {path}")
    source_ports = {
        int(item["serial"]): int(item["port"])
        for item in metadata.get("sources", [])
        if isinstance(item, dict) and "serial" in item and "port" in item
    }
    poses_path = (
        args.world_poses_jsonl.expanduser().resolve()
        if args.world_poses_jsonl is not None
        else path.with_name(path.stem + "_world_poses.jsonl")
    )
    write_world_poses_jsonl(poses_path, output=output, source_ports=source_ports)
    print(f"DUNYA POZLARI JSONL HAZIR: {poses_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

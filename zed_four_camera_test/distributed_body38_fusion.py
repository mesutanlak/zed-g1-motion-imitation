#!/usr/bin/env python3
"""Application-level multi-host BODY_38 fusion for the G1 perception path.

This is intentionally *not* a wrapper around ``sl.Fusion`` or ZED360.  It is
the fallback transport for the two-host rig when the SDK Network Fusion path
rejects otherwise valid BODY_38 data.  Each physical ZED is opened by
``zed_g1_skeleton.py`` and sends its normal ``zed_body38_live/v1`` UDP packet
to this process.  After an explicit static extrinsic calibration, this process
transforms and confidence-fuses keypoints, then emits the same downstream
``zed_body38_live/v1`` contract.  It never sends robot commands.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path
import queue
import selectors
import socket
import sys
import threading
import time
from typing import Any

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import msvcrt
except ImportError:
    msvcrt = None

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from motion_pipeline.calibration import to_pelvis_local


BODY38_NAMES = (
    "PELVIS", "SPINE_1", "SPINE_2", "SPINE_3", "NECK", "NOSE",
    "LEFT_EYE", "RIGHT_EYE", "LEFT_EAR", "RIGHT_EAR",
    "LEFT_CLAVICLE", "RIGHT_CLAVICLE", "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST",
    "LEFT_HIP", "RIGHT_HIP", "LEFT_KNEE", "RIGHT_KNEE", "LEFT_ANKLE",
    "RIGHT_ANKLE", "LEFT_BIG_TOE", "RIGHT_BIG_TOE", "LEFT_SMALL_TOE",
    "RIGHT_SMALL_TOE", "LEFT_HEEL", "RIGHT_HEEL", "LEFT_HAND_THUMB_4",
    "RIGHT_HAND_THUMB_4", "LEFT_HAND_INDEX_1", "RIGHT_HAND_INDEX_1",
    "LEFT_HAND_MIDDLE_4", "RIGHT_HAND_MIDDLE_4", "LEFT_HAND_PINKY_1",
    "RIGHT_HAND_PINKY_1",
)
IDX = {name: index for index, name in enumerate(BODY38_NAMES)}


@dataclass(frozen=True)
class InputEndpoint:
    serial: int
    port: int


@dataclass
class Sample:
    serial: int
    packet: dict[str, Any]
    received_ns: int
    sequence: int


@dataclass(frozen=True)
class Extrinsic:
    rotation: np.ndarray
    translation: np.ndarray


@dataclass(frozen=True)
class ExtrinsicSet:
    reference_serial: int
    cameras: dict[int, Extrinsic]


@dataclass
class PreparedView:
    sample: Sample
    extrinsic: Extrinsic
    raw_points: np.ndarray
    aligned_points: np.ndarray
    confidence: np.ndarray
    quality: float
    alignment_translation: np.ndarray
    pose_disagreement_m: float | None
    accepted: bool


CRITICAL_GROUPS = {
    "torso": ("PELVIS", "SPINE_3", "NECK", "LEFT_SHOULDER", "RIGHT_SHOULDER"),
    "left_arm": ("LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST"),
    "right_arm": ("RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST"),
    "left_leg": ("LEFT_HIP", "LEFT_KNEE", "LEFT_ANKLE"),
    "right_leg": ("RIGHT_HIP", "RIGHT_KNEE", "RIGHT_ANKLE"),
}


def sanitize(value: Any) -> Any:
    """Make NumPy and non-finite values valid JSON without changing packets."""
    if isinstance(value, np.ndarray):
        return sanitize(value.tolist())
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, dict):
        return {str(key): sanitize(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [sanitize(item) for item in value]
    return value


def parse_endpoint(value: str) -> InputEndpoint:
    try:
        serial_text, port_text = value.strip().split(":", 1)
        serial, port = int(serial_text), int(port_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Kaynak SERIAL:PORT biciminde olmali.") from exc
    if serial <= 0 or not (1024 <= port <= 65535):
        raise argparse.ArgumentTypeError("Gecerli SERIAL ve 1024-65535 UDP portu verin.")
    return InputEndpoint(serial, port)


def vector(value: Any, length: int) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
        if result.shape == (length,):
            return result
    except (TypeError, ValueError):
        pass
    return np.full(length, np.nan, dtype=np.float64)


def points(value: Any) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
        if result.shape == (38, 3):
            return result
    except (TypeError, ValueError):
        pass
    return np.full((38, 3), np.nan, dtype=np.float64)


def confidence(value: Any) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
        if result.shape == (38,):
            return result
    except (TypeError, ValueError):
        pass
    return np.zeros(38, dtype=np.float64)


def quaternion_xyzw_to_matrix(value: Any) -> np.ndarray:
    x, y, z, w = vector(value, 4)
    norm = float(math.sqrt(x * x + y * y + z * z + w * w))
    if not math.isfinite(norm) or norm < 1.0e-8:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def matrix_to_quaternion_xyzw(matrix: np.ndarray) -> np.ndarray:
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
    return quat / max(float(np.linalg.norm(quat)), 1.0e-12)


def load_extrinsics(path: Path | None, endpoints: list[InputEndpoint]) -> ExtrinsicSet | None:
    if path is None:
        return None
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"Extrinsic dosyasi okunamadi: {path}: {exc}") from exc
    if document.get("schema") != "zed_body38_distributed_extrinsics/v1":
        raise ValueError("Extrinsic schema zed_body38_distributed_extrinsics/v1 olmali.")
    if document.get("coordinate_system") != "RIGHT_HANDED_Z_UP_X_FWD":
        raise ValueError("Extrinsic koordinat sistemi RIGHT_HANDED_Z_UP_X_FWD olmali.")
    cameras = document.get("cameras")
    if not isinstance(cameras, dict):
        raise ValueError("Extrinsic dosyasinda cameras nesnesi yok.")
    try:
        reference_serial = int(document["reference_world_serial"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Extrinsic dosyasinda reference_world_serial gecersiz.") from exc
    result: dict[int, Extrinsic] = {}
    for endpoint in endpoints:
        item = cameras.get(str(endpoint.serial))
        if not isinstance(item, dict):
            raise ValueError(f"Extrinsic dosyasinda ZED {endpoint.serial} yok.")
        rotation = np.asarray(item.get("rotation_camera_to_world"), dtype=np.float64)
        translation = np.asarray(item.get("translation_camera_to_world_m"), dtype=np.float64)
        if rotation.shape != (3, 3) or translation.shape != (3,) or not np.isfinite(rotation).all() or not np.isfinite(translation).all():
            raise ValueError(f"ZED {endpoint.serial} extrinsic matrisi gecersiz.")
        if not np.isclose(np.linalg.det(rotation), 1.0, atol=0.03):
            raise ValueError(f"ZED {endpoint.serial} rotation matrisi proper rotation degil.")
        result[endpoint.serial] = Extrinsic(rotation, translation)
    if reference_serial not in result:
        raise ValueError(f"Referans ZED {reference_serial} dort kaynak arasinda yok.")
    return ExtrinsicSet(reference_serial=reference_serial, cameras=result)


def distance_between(value: np.ndarray, first: str, second: str) -> float | None:
    first_value, second_value = value[IDX[first]], value[IDX[second]]
    if not (np.isfinite(first_value).all() and np.isfinite(second_value).all()):
        return None
    return float(np.linalg.norm(first_value - second_value))


def joint_angle_deg(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float | None:
    if not (np.isfinite(a).all() and np.isfinite(b).all() and np.isfinite(c).all()):
        return None
    ba, bc = a - b, c - b
    denominator = float(np.linalg.norm(ba) * np.linalg.norm(bc))
    if denominator < 1.0e-8:
        return None
    return math.degrees(math.acos(float(np.clip(np.dot(ba, bc) / denominator, -1.0, 1.0))))


def build_g1_features(value: np.ndarray, conf: np.ndarray, threshold: float) -> dict[str, Any]:
    validity: dict[str, bool] = {}
    for group_name, joint_names in CRITICAL_GROUPS.items():
        indexes = [IDX[name] for name in joint_names]
        validity[group_name] = bool(
            np.isfinite(value[indexes]).all() and np.all(conf[indexes] >= threshold)
        )
    return {
        "valid_groups": validity,
        "upper_body_reference_ready": bool(
            validity["torso"] and validity["left_arm"] and validity["right_arm"]
        ),
        "whole_body_reference_ready": bool(all(validity.values())),
        "anthropometry_m": {
            "shoulder_width": distance_between(value, "LEFT_SHOULDER", "RIGHT_SHOULDER"),
            "hip_width": distance_between(value, "LEFT_HIP", "RIGHT_HIP"),
            "left_upper_arm": distance_between(value, "LEFT_SHOULDER", "LEFT_ELBOW"),
            "left_forearm": distance_between(value, "LEFT_ELBOW", "LEFT_WRIST"),
            "right_upper_arm": distance_between(value, "RIGHT_SHOULDER", "RIGHT_ELBOW"),
            "right_forearm": distance_between(value, "RIGHT_ELBOW", "RIGHT_WRIST"),
            "left_thigh": distance_between(value, "LEFT_HIP", "LEFT_KNEE"),
            "left_shank": distance_between(value, "LEFT_KNEE", "LEFT_ANKLE"),
            "right_thigh": distance_between(value, "RIGHT_HIP", "RIGHT_KNEE"),
            "right_shank": distance_between(value, "RIGHT_KNEE", "RIGHT_ANKLE"),
        },
        "geometric_angles": {
            "left_elbow_interior_deg": joint_angle_deg(value[IDX["LEFT_SHOULDER"]], value[IDX["LEFT_ELBOW"]], value[IDX["LEFT_WRIST"]]),
            "right_elbow_interior_deg": joint_angle_deg(value[IDX["RIGHT_SHOULDER"]], value[IDX["RIGHT_ELBOW"]], value[IDX["RIGHT_WRIST"]]),
            "left_knee_interior_deg": joint_angle_deg(value[IDX["LEFT_HIP"]], value[IDX["LEFT_KNEE"]], value[IDX["LEFT_ANKLE"]]),
            "right_knee_interior_deg": joint_angle_deg(value[IDX["RIGHT_HIP"]], value[IDX["RIGHT_KNEE"]], value[IDX["RIGHT_ANKLE"]]),
        },
        "note": "Fused perception features; robot commands are produced only after retargeting and safety gates.",
    }


def synchronized_samples(
    histories: dict[int, deque[Sample]],
    *,
    now_ns: int,
    source_timeout_ns: int,
    maximum_spread_ns: int,
    minimum_sources: int,
) -> tuple[list[Sample], float] | None:
    """Choose the largest, freshest arrival-time-aligned camera bundle.

    Source clocks need not agree for this application fallback: all matching is
    performed on the main PC's UDP receive clock.  A short history prevents a
    fast camera's newest frame from continually outrunning a slower source.
    """
    fresh = {
        serial: [sample for sample in history if now_ns - sample.received_ns <= source_timeout_ns]
        for serial, history in histories.items()
    }
    fresh = {serial: samples for serial, samples in fresh.items() if samples}
    if len(fresh) < minimum_sources:
        return None
    anchors = [sample.received_ns for samples in fresh.values() for sample in samples]
    best: tuple[tuple[int, int, int], list[Sample], float] | None = None
    for anchor_ns in anchors:
        selected = [min(samples, key=lambda item: abs(item.received_ns - anchor_ns)) for samples in fresh.values()]
        selected = [item for item in selected if abs(item.received_ns - anchor_ns) <= maximum_spread_ns]
        if len(selected) < minimum_sources:
            continue
        spread_ns = max(item.received_ns for item in selected) - min(item.received_ns for item in selected)
        if spread_ns > maximum_spread_ns:
            continue
        # Prefer more cameras, then the newest valid bundle.  Using spread as
        # the second key can pin the selector to an older, unusually tight
        # bundle until it ages out of source_timeout_ns, throttling a 15 Hz
        # input to only a few fused packets per second.
        score = (len(selected), max(item.received_ns for item in selected), -spread_ns)
        if best is None or score > best[0]:
            best = (score, selected, spread_ns / 1.0e6)
    return (best[1], best[2]) if best is not None else None


def sample_quality(packet: dict[str, Any], minimum_confidence: float) -> float:
    xyz = points(packet.get("keypoints_3d_m"))
    conf = confidence(packet.get("keypoint_confidence"))
    visible = np.isfinite(xyz).all(axis=1) & (conf >= minimum_confidence)
    upper = [IDX[name] for name in ("SPINE_3", "NECK", "LEFT_SHOULDER", "RIGHT_SHOULDER", "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST")]
    upper_visible = float(np.mean(visible[upper]))
    overall_visible = float(np.mean(visible))
    mean_conf = float(np.mean(conf[visible])) / 100.0 if np.any(visible) else 0.0
    body_conf = float(packet.get("body_confidence", 0.0) or 0.0) / 100.0
    return float(np.clip(0.50 * upper_visible + 0.25 * overall_visible + 0.15 * mean_conf + 0.10 * body_conf, 0.0, 1.0))


def transformed_points(packet: dict[str, Any], extrinsic: Extrinsic) -> np.ndarray:
    value = points(packet.get("keypoints_3d_m"))
    valid = np.isfinite(value).all(axis=1)
    result = value.copy()
    result[valid] = value[valid] @ extrinsic.rotation.T + extrinsic.translation
    return result


def prepare_aligned_views(
    views: list[tuple[Sample, Extrinsic]],
    *,
    minimum_confidence: float,
    maximum_pose_disagreement_m: float = 0.22,
    maximum_alignment_translation_m: float = 1.0,
) -> list[PreparedView]:
    """Reject cross-person views and correct residual translation drift.

    Static extrinsics remain the source of camera orientation.  A BODY-based
    calibration can nevertheless leave a sizeable translation bias when the
    person was not perfectly still.  Translation does not change articulation,
    so each accepted view is shifted to the robust pelvis consensus before
    per-joint fusion.  Root-relative torso/limb shape guards against aligning
    and mixing two different people merely because both have a valid pelvis.
    """
    raw: list[tuple[Sample, Extrinsic, np.ndarray, np.ndarray, float]] = []
    for sample, extrinsic in views:
        raw.append((
            sample,
            extrinsic,
            transformed_points(sample.packet, extrinsic),
            confidence(sample.packet.get("keypoint_confidence")),
            sample_quality(sample.packet, minimum_confidence),
        ))
    if not raw:
        return []

    core_names = {
        name for group in CRITICAL_GROUPS.values() for name in group
    } | {"SPINE_1", "SPINE_2", "LEFT_HIP", "RIGHT_HIP"}
    core_indexes = np.asarray([IDX[name] for name in sorted(core_names)], dtype=int)
    pair_errors = np.full((len(raw), len(raw)), np.nan, dtype=np.float64)
    np.fill_diagonal(pair_errors, 0.0)
    for first in range(len(raw)):
        first_xyz, first_conf = raw[first][2], raw[first][3]
        if not np.isfinite(first_xyz[IDX["PELVIS"]]).all():
            continue
        first_relative = first_xyz - first_xyz[IDX["PELVIS"]]
        for second in range(first + 1, len(raw)):
            second_xyz, second_conf = raw[second][2], raw[second][3]
            if not np.isfinite(second_xyz[IDX["PELVIS"]]).all():
                continue
            second_relative = second_xyz - second_xyz[IDX["PELVIS"]]
            valid = (
                np.isfinite(first_relative[core_indexes]).all(axis=1)
                & np.isfinite(second_relative[core_indexes]).all(axis=1)
                & (first_conf[core_indexes] >= minimum_confidence)
                & (second_conf[core_indexes] >= minimum_confidence)
            )
            if np.count_nonzero(valid) < 5:
                continue
            error = float(np.mean(np.linalg.norm(
                first_relative[core_indexes][valid]
                - second_relative[core_indexes][valid],
                axis=1,
            )))
            pair_errors[first, second] = pair_errors[second, first] = error

    medoid_scores = []
    for index in range(len(raw)):
        finite = pair_errors[index][np.isfinite(pair_errors[index])]
        medoid_scores.append(float(np.median(finite)) if finite.size > 1 else math.inf)
    medoid = int(np.argmin(medoid_scores)) if any(math.isfinite(value) for value in medoid_scores) else int(np.argmax([item[4] for item in raw]))
    accepted: list[bool] = []
    disagreement: list[float | None] = []
    for index in range(len(raw)):
        value = pair_errors[medoid, index]
        finite_value = float(value) if math.isfinite(float(value)) else None
        disagreement.append(finite_value)
        accepted.append(index == medoid or (finite_value is not None and finite_value <= maximum_pose_disagreement_m))

    accepted_pelvis = [
        item[2][IDX["PELVIS"]]
        for index, item in enumerate(raw)
        if accepted[index] and np.isfinite(item[2][IDX["PELVIS"]]).all()
    ]
    pelvis_consensus = (
        np.median(np.asarray(accepted_pelvis), axis=0)
        if accepted_pelvis
        else np.full(3, np.nan, dtype=np.float64)
    )
    if np.isfinite(pelvis_consensus).all():
        for index, item in enumerate(raw):
            pelvis = item[2][IDX["PELVIS"]]
            if (
                accepted[index]
                and np.isfinite(pelvis).all()
                and float(np.linalg.norm(pelvis_consensus - pelvis))
                > maximum_alignment_translation_m
            ):
                accepted[index] = False
        # A badly split calibration (or two simultaneous operators) can put
        # the coordinate-wise median farther than the safety gate from every
        # view.  Always preserve the pose medoid as a safe single-view
        # fallback so the emitted skeleton cannot collapse to all-NaN values.
        if not any(accepted):
            accepted[medoid] = True
        accepted_pelvis = [
            item[2][IDX["PELVIS"]]
            for index, item in enumerate(raw)
            if accepted[index] and np.isfinite(item[2][IDX["PELVIS"]]).all()
        ]
        if accepted_pelvis:
            pelvis_consensus = np.median(np.asarray(accepted_pelvis), axis=0)
    result: list[PreparedView] = []
    for index, (sample, extrinsic, xyz, conf, quality) in enumerate(raw):
        pelvis = xyz[IDX["PELVIS"]]
        correction = (
            pelvis_consensus - pelvis
            if accepted[index] and np.isfinite(pelvis_consensus).all() and np.isfinite(pelvis).all()
            else np.zeros(3, dtype=np.float64)
        )
        aligned = xyz.copy()
        valid = np.isfinite(aligned).all(axis=1)
        aligned[valid] += correction
        result.append(PreparedView(
            sample=sample,
            extrinsic=extrinsic,
            raw_points=xyz,
            aligned_points=aligned,
            confidence=conf,
            quality=quality,
            alignment_translation=correction,
            pose_disagreement_m=disagreement[index],
            accepted=accepted[index],
        ))
    return result


def fuse_prepared_keypoints(
    prepared: list[PreparedView],
    *,
    minimum_confidence: float,
    maximum_spread_m: float,
) -> tuple[np.ndarray, np.ndarray, list[int]]:
    result = np.full((38, 3), np.nan, dtype=np.float64)
    result_confidence = np.zeros(38, dtype=np.float64)
    contribution_count: list[int] = []
    accepted = [item for item in prepared if item.accepted]
    for joint in range(38):
        candidates: list[tuple[np.ndarray, float, float]] = []
        for item in accepted:
            if np.isfinite(item.aligned_points[joint]).all() and item.confidence[joint] >= minimum_confidence:
                candidates.append((item.aligned_points[joint], float(item.confidence[joint]), item.quality))
        if not candidates:
            contribution_count.append(0)
            continue
        median = np.median(np.asarray([item[0] for item in candidates]), axis=0)
        inliers = [item for item in candidates if float(np.linalg.norm(item[0] - median)) <= maximum_spread_m]
        if not inliers:
            inliers = [max(candidates, key=lambda item: item[1] * item[2])]
        weights = np.asarray([max(1.0e-4, (item[1] / 100.0) ** 2 * max(item[2], 0.05)) for item in inliers])
        joint_values = np.asarray([item[0] for item in inliers])
        result[joint] = np.average(joint_values, axis=0, weights=weights)
        result_confidence[joint] = float(np.average(np.asarray([item[1] for item in inliers]), weights=weights))
        contribution_count.append(len(inliers))
    return result, result_confidence, contribution_count


def fuse_keypoints(
    views: list[tuple[Sample, Extrinsic]],
    *,
    minimum_confidence: float,
    maximum_spread_m: float,
) -> tuple[np.ndarray, np.ndarray, list[int]]:
    """Robustly average joints after static camera-to-world transforms.

    Per-joint medians remove a single bad/occluded view before confidence
    weighting.  A joint visible in only one calibrated camera is retained,
    which is important for wrists and the BODY_38 hand proxy joints.
    """
    prepared = prepare_aligned_views(
        views,
        minimum_confidence=minimum_confidence,
    )
    return fuse_prepared_keypoints(
        prepared,
        minimum_confidence=minimum_confidence,
        maximum_spread_m=maximum_spread_m,
    )


def compact_sample(sample: Sample) -> dict[str, Any]:
    packet = sample.packet
    return sanitize({
        "serial": sample.serial,
        "source_timestamp_ns": int(packet.get("timestamp_ns", 0) or 0),
        "receiver_timestamp_ns": sample.received_ns,
        "sequence": sample.sequence,
        "body_confidence": packet.get("body_confidence", 0.0),
        "keypoint_names": packet.get("keypoint_names", BODY38_NAMES),
        "keypoints_3d_m": packet.get("keypoints_3d_m"),
        "keypoint_confidence": packet.get("keypoint_confidence"),
    })


def event_rate(events: deque[float], now: float, window_s: float = 2.0) -> float:
    while events and now - events[0] > window_s:
        events.popleft()
    if len(events) < 2:
        return 0.0
    elapsed = events[-1] - events[0]
    return (len(events) - 1) / elapsed if elapsed > 1.0e-6 else 0.0


def cross_view_agreement(
    prepared: list[tuple[int, np.ndarray, np.ndarray]],
    fused: np.ndarray,
    minimum_confidence: float,
) -> dict[str, Any]:
    errors: list[float] = []
    per_joint: dict[str, float | None] = {}
    for joint, name in enumerate(BODY38_NAMES):
        joint_errors: list[float] = []
        if np.isfinite(fused[joint]).all():
            for _serial, xyz, conf in prepared:
                if np.isfinite(xyz[joint]).all() and conf[joint] >= minimum_confidence:
                    joint_errors.append(float(np.linalg.norm(xyz[joint] - fused[joint])))
        per_joint[name] = float(np.mean(joint_errors)) if joint_errors else None
        errors.extend(joint_errors)
    finite = np.asarray(errors, dtype=np.float64)
    return sanitize({
        "mpjpe_m": float(np.mean(finite)) if finite.size else None,
        "p95_error_m": float(np.percentile(finite, 95)) if finite.size else None,
        "pelvis_error_m": per_joint["PELVIS"],
        "left_wrist_error_m": per_joint["LEFT_WRIST"],
        "right_wrist_error_m": per_joint["RIGHT_WRIST"],
        "per_joint_error_m": per_joint,
    })


def compact_live_packet(packet: dict[str, Any]) -> dict[str, Any]:
    """Remove analysis-only arrays so the real-time GMR datagram stays small."""
    result = dict(packet)
    multi = dict(result.get("multi_camera") or {})
    compact_views = []
    for view in multi.get("per_camera") or []:
        item = dict(view)
        item.pop("keypoints_3d_fusion_m", None)
        item.pop("keypoint_confidence", None)
        compact_views.append(item)
    multi["per_camera"] = compact_views
    multi.pop("camera_pose_fusion_from_local", None)
    result["multi_camera"] = multi
    return result


def draw_four_preview(
    frames: dict[int, np.ndarray],
    endpoints: list[InputEndpoint],
    source_metrics: dict[int, dict[str, Any]],
    *,
    connected: list[int],
    body_fresh: list[int],
    contributing: list[int],
    output_hz: float,
    arrival_spread_ms: float,
    recording: bool,
    recorded: int,
    record_dropped: int,
) -> np.ndarray | None:
    if cv2 is None:
        return None
    tile_width, tile_height = 640, 360
    tiles: list[np.ndarray] = []
    now_ns = time.time_ns()
    for endpoint in endpoints:
        source = frames.get(endpoint.serial)
        if source is None:
            tile = np.zeros((tile_height, tile_width, 3), dtype=np.uint8)
            cv2.putText(tile, "ONIZLEME BEKLENIYOR", (145, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 180, 255), 2, cv2.LINE_AA)
        else:
            tile = source
            if tile.ndim == 2:
                tile = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)
            tile = cv2.resize(tile, (tile_width, tile_height), interpolation=cv2.INTER_AREA)
        metric = source_metrics.get(endpoint.serial, {})
        age_ms = (now_ns - int(metric.get("last_body_ns", 0))) / 1.0e6 if metric.get("last_body_ns") else math.inf
        body_fps = float(metric.get("body_fps", 0.0))
        rx_fps = float(metric.get("rx_fps", 0.0))
        capture_ms = metric.get("capture_to_receive_ms")
        capture_text = f"{float(capture_ms):.1f}ms" if isinstance(capture_ms, (int, float)) and math.isfinite(float(capture_ms)) else "n/a"
        color = (40, 220, 80) if endpoint.serial in body_fresh else (0, 165, 255)
        cv2.rectangle(tile, (0, 0), (tile_width, 62), (18, 18, 18), -1)
        cv2.putText(tile, f"ZED {endpoint.serial}  BODY {body_fps:.1f} fps  RX {rx_fps:.1f} fps", (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.57, color, 2, cv2.LINE_AA)
        cv2.putText(tile, f"durum={metric.get('status', 'YOK')}  gecikme={capture_text}  body_yasi={age_ms:.0f}ms", (12, 51), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (235, 235, 235), 1, cv2.LINE_AA)
        tiles.append(tile)
    while len(tiles) < 4:
        tiles.append(np.zeros((tile_height, tile_width, 3), dtype=np.uint8))
    grid = np.vstack((np.hstack((tiles[0], tiles[1])), np.hstack((tiles[2], tiles[3]))))
    footer = np.zeros((82, grid.shape[1], 3), dtype=np.uint8)
    spread = f"{arrival_spread_ms:.1f} ms" if math.isfinite(arrival_spread_ms) else "n/a"
    cv2.putText(footer, f"4-ZED BODY_38 | bagli={len(connected)}/4 | body_taze={len(body_fresh)}/4 | fusion_katki={len(contributing)}/4 | fusion={output_hz:.1f} fps | yayilim={spread}", (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.61, (80, 235, 235), 2, cv2.LINE_AA)
    cv2.putText(footer, f"REC={'ON' if recording else 'OFF'} kare={recorded} drop={record_dropped} | S: kayit ac/kapat | Q/ESC: cikis", (16, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.57, (60, 220, 80) if recording else (200, 200, 200), 2, cv2.LINE_AA)
    return np.vstack((grid, footer))


def make_output_packet(
    views: list[tuple[Sample, Extrinsic]],
    *,
    minimum_confidence: float,
    maximum_spread_m: float,
    output_sequence: int,
    reference_serial: int,
    arrival_spread_ms: float,
    source_metrics: dict[int, dict[str, Any]] | None = None,
    connected_serials: list[int] | None = None,
    effective_output_hz: float = 0.0,
    record_queue_depth: int = 0,
    record_dropped: int = 0,
    maximum_pose_disagreement_m: float = 0.22,
    maximum_alignment_translation_m: float = 1.0,
) -> dict[str, Any]:
    prepared = prepare_aligned_views(
        views,
        minimum_confidence=minimum_confidence,
        maximum_pose_disagreement_m=maximum_pose_disagreement_m,
        maximum_alignment_translation_m=maximum_alignment_translation_m,
    )
    accepted = [item for item in prepared if item.accepted]
    if not accepted:
        accepted = prepared
    best_view = max(accepted, key=lambda item: item.quality)
    best_sample, best_extrinsic = best_view.sample, best_view.extrinsic
    packet = dict(best_sample.packet)
    fused, fused_confidence, contributions = fuse_prepared_keypoints(
        prepared,
        minimum_confidence=minimum_confidence,
        maximum_spread_m=maximum_spread_m,
    )
    pelvis = fused[IDX["PELVIS"]]
    source_orientation = quaternion_xyzw_to_matrix(packet.get("global_root_orientation_xyzw"))
    try:
        pelvis_local, pelvis_origin, pelvis_rotation = to_pelvis_local(fused, IDX)
    except ValueError:
        pelvis_local = np.full_like(fused, np.nan)
        pelvis_origin = np.full(3, np.nan)
        pelvis_rotation = np.full((3, 3), np.nan)
    shoulder_width = distance_between(fused, "LEFT_SHOULDER", "RIGHT_SHOULDER")
    shoulder_normalized = (
        pelvis_local / shoulder_width
        if shoulder_width is not None and shoulder_width > 0.05
        else np.full_like(fused, np.nan)
    )
    fused_features = build_g1_features(fused, fused_confidence, minimum_confidence)
    raw_agreement_views = [
        (item.sample.serial, item.raw_points, item.confidence)
        for item in prepared
    ]
    aligned_agreement_views = [
        (item.sample.serial, item.aligned_points, item.confidence)
        for item in accepted
    ]
    accepted_serials = [item.sample.serial for item in accepted]
    excluded_serials = [item.sample.serial for item in prepared if not item.accepted]
    metrics = source_metrics or {}
    view_descriptions = []
    camera_poses: dict[str, Any] = {}
    now_ns = time.time_ns()
    for item in prepared:
        sample, extrinsic = item.sample, item.extrinsic
        source_metric = metrics.get(sample.serial, {})
        view_descriptions.append(sanitize({
            "serial_number": sample.serial,
            "body_id": sample.packet.get("body_id"),
            "unique_object_id": sample.packet.get("unique_object_id"),
            "sequence": sample.sequence,
            "body_confidence": sample.packet.get("body_confidence", 0.0),
            "source_timestamp_ns": int(sample.packet.get("timestamp_ns", 0) or 0),
            "receiver_timestamp_ns": sample.received_ns,
            "receiver_age_ms": (now_ns - sample.received_ns) / 1.0e6,
            "quality_score": item.quality,
            "accepted_for_fusion": item.accepted,
            "pose_disagreement_to_medoid_m": item.pose_disagreement_m,
            "dynamic_alignment_translation_m": item.alignment_translation,
            "keypoints_3d_fusion_m": item.aligned_points,
            "keypoint_confidence": item.confidence,
            "source_metrics": source_metric,
        }))
        camera_poses[str(sample.serial)] = sanitize({
            "rotation_camera_to_world": extrinsic.rotation,
            "translation_camera_to_world_m": extrinsic.translation,
        })
    agreement = cross_view_agreement(raw_agreement_views, fused, minimum_confidence)
    aligned_agreement = cross_view_agreement(aligned_agreement_views, fused, minimum_confidence)
    calibration = dict(packet.get("calibration") or {})
    profile = dict(calibration.get("profile") or {})
    neutral = np.asarray(profile.get("neutral_pelvis_rotation_matrix"), dtype=np.float64)
    if neutral.shape == (3, 3) and np.isfinite(neutral).all():
        profile["neutral_pelvis_rotation_matrix"] = (best_extrinsic.rotation @ neutral).tolist()
    if profile:
        calibration["profile"] = profile
    packet.update(sanitize({
        "schema": "zed_body38_live/v1",
        "source_serial": 0,
        "sequence": output_sequence,
        "timestamp_ns": time.time_ns(),
        "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
        "units": "meter",
        "root_position_m": pelvis,
        "global_root_orientation_xyzw": matrix_to_quaternion_xyzw(best_extrinsic.rotation @ source_orientation),
        "keypoint_names": BODY38_NAMES,
        "keypoints_3d_raw_m": fused,
        "keypoints_3d_m": fused,
        "keypoint_confidence": fused_confidence,
        "root_relative_keypoints_m": pelvis_local,
        "shoulder_width_normalized_keypoints": shoulder_normalized,
        "pelvis_frame": {
            "coordinate_system": "PELVIS_LOCAL_X_FWD_Y_LEFT_Z_UP",
            "reference_frame": "FUSION_WORLD",
            # Keep the legacy keys because the existing GMR consumer uses
            # them generically even when the parent frame is fusion WORLD.
            "origin_camera_m": pelvis_origin,
            "rotation_camera_from_pelvis": pelvis_rotation,
            "origin_world_m": pelvis_origin,
            "rotation_world_from_pelvis": pelvis_rotation,
            "relative_neutral_yaw_rad": None,
            "keypoints_m": pelvis_local,
        },
        "calibration": calibration,
        "reference_ready": {
            "upper_body": fused_features["upper_body_reference_ready"],
            "whole_body": fused_features["whole_body_reference_ready"],
        },
        "g1_reference_features": fused_features,
        "fusion": {
            "implementation": "application_level_weighted_body38/v1",
            "reference_world_serial": reference_serial,
            "contributing_serials": accepted_serials,
            "excluded_pose_serials": excluded_serials,
            "best_orientation_serial": best_sample.serial,
            "per_joint_contributions": contributions,
            "maximum_joint_spread_m": maximum_spread_m,
            "arrival_spread_ms": arrival_spread_ms,
        },
        "multi_camera": {
            "mode": "FOUR_FUSED" if len(accepted) == 4 else "PARTIAL_FUSED",
            "contributing_views": len(accepted),
            "contributing_serials": accepted_serials,
            "excluded_pose_serials": excluded_serials,
            "connected_serials": connected_serials or [sample.serial for sample, _ in views],
            "configured_serials": sorted(metrics) if metrics else [sample.serial for sample, _ in views],
            "camera_timestamp_delta_ms": arrival_spread_ms,
            "camera_sync_ok": bool(arrival_spread_ms <= 110.0),
            "fused_quality_score": float(np.mean(fused_confidence) / 100.0),
            "best_single_quality_score": sample_quality(best_sample.packet, minimum_confidence),
            "cross_view_agreement": agreement,
            "post_alignment_agreement": aligned_agreement,
            "fusion_metrics": {
                "mean_camera_fused": float(np.mean(contributions)),
                "mean_stdev_between_camera_s": arrival_spread_ms / 1000.0,
                "per_camera": metrics,
            },
            "per_camera": view_descriptions,
            "camera_pose_fusion_from_local": camera_poses,
            "failure_codes": [] if len(accepted) == 4 else (["CROSS_PERSON_OR_POSE_OUTLIER"] if excluded_serials else ["PARTIAL_CAMERA_SET"]),
        },
        "transport_metrics": {
            "source_interval_ms": 1000.0 / effective_output_hz if effective_output_hz > 0.0 else None,
            "effective_output_hz": effective_output_hz,
            "capture_to_send_ms": (now_ns - int(best_sample.packet.get("timestamp_ns", 0) or now_ns)) / 1.0e6,
            "record_queue_depth": record_queue_depth,
            "record_dropped": record_dropped,
            "source_mode": "distributed_four_zed_body38",
        },
        "latency_trace_ns": {
            "t0_capture_ns": int(best_sample.packet.get("timestamp_ns", 0) or 0),
            "t2_windows_udp_receive_ns": best_sample.received_ns,
            "t3_application_fused_ns": time.time_ns(),
        },
    }))
    return packet


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Iki-host, dort-ZED application BODY_38 fusion alicisi")
    parser.add_argument("--source", type=parse_endpoint, action="append", required=True, help="Her kamera icin SERIAL:UDP_PORT; dort kez verin.")
    parser.add_argument("--preview-source", type=parse_endpoint, action="append", default=[], help="Canli JPEG icin SERIAL:UDP_PORT; arayuz icin dort kez verin.")
    parser.add_argument("--bind", default="0.0.0.0", help="Dinlenecek PC IPv4 adresi (varsayilan tum arayuzler).")
    parser.add_argument("--extrinsics", type=Path, default=None, help="Kalibre edilmis zed_body38_distributed_extrinsics/v1 JSON dosyasi.")
    parser.add_argument("--output-host", default="", help="Birlesik BODY_38 UDP hedefi; bos ise cikis yayini kapali.")
    parser.add_argument("--output-port", type=int, default=15050)
    parser.add_argument("--output-max-hz", type=float, default=15.0)
    parser.add_argument("--monitor-host", default="", help="Tam analiz/Rerun BODY_38 kopyasi.")
    parser.add_argument("--monitor-port", type=int, default=15052)
    parser.add_argument("--monitor-max-hz", type=float, default=15.0)
    parser.add_argument("--ros-host", default="", help="Kompakt ROS/WSL BODY_38 kopyasi.")
    parser.add_argument("--ros-port", type=int, default=15054)
    parser.add_argument("--ros-max-hz", type=float, default=15.0)
    parser.add_argument("--confidence", type=float, default=45.0)
    parser.add_argument("--max-sync-ms", type=float, default=110.0, help="Ana PC'ye varis zamanina gore azami dortlu paket yayilimi.")
    parser.add_argument("--source-timeout-ms", type=float, default=750.0, help="Bu sureden eski BODY_38 kaynagini taze sayma.")
    parser.add_argument("--minimum-sources", type=int, choices=(2, 3, 4), default=2, help="Calisma aninda cikis icin gereken en az taze kamera.")
    parser.add_argument("--max-joint-spread-m", type=float, default=0.30)
    parser.add_argument("--max-pose-disagreement-m", type=float, default=0.22, help="Farkli kisi/poz gorunumunu dislamak icin pelvis-yerel govde MPJPE esigi.")
    parser.add_argument("--max-alignment-translation-m", type=float, default=1.0, help="Dinamik pelvis hizalamasinda farkli kisiyi elemek icin azami ceviri.")
    parser.add_argument("--calibration-record", type=Path, default=None, help="Dortlu senkron ham BODY_38 karelerini JSONL olarak kaydet.")
    parser.add_argument("--calibration-max-hz", type=float, default=12.0)
    parser.add_argument("--record", action="store_true", help="Fusion JSONL kaydini baslangicta ac.")
    parser.add_argument("--output-dir", type=Path, default=REPOSITORY_ROOT / "recordings")
    parser.add_argument("--record-stem", default="four_body38_fusion")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--preview-hz", type=float, default=10.0, help="2x2 arayuz yenileme hizi.")
    parser.add_argument("--duration", type=float, default=0.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    endpoints: list[InputEndpoint] = args.source
    preview_endpoints: list[InputEndpoint] = args.preview_source
    if len(endpoints) != 4 or len({item.serial for item in endpoints}) != 4 or len({item.port for item in endpoints}) != 4:
        print("HATA: tam dort farkli SERIAL:PORT kaynagi gerekli.", file=sys.stderr)
        return 2
    if preview_endpoints and (
        len(preview_endpoints) != 4
        or {item.serial for item in preview_endpoints} != {item.serial for item in endpoints}
        or len({item.port for item in preview_endpoints}) != 4
    ):
        print("HATA: onizleme icin ayni dort seriye ait dort farkli SERIAL:PORT gerekli.", file=sys.stderr)
        return 2
    if preview_endpoints and not args.headless and cv2 is None:
        print("HATA: dortlu arayuz icin opencv-python kurulu olmali.", file=sys.stderr)
        return 2
    if (
        not 0.0 <= args.confidence <= 100.0
        or args.max_sync_ms <= 0.0
        or args.source_timeout_ms <= 0.0
        or args.max_joint_spread_m <= 0.0
        or args.max_pose_disagreement_m <= 0.0
        or args.max_alignment_translation_m <= 0.0
        or args.output_max_hz <= 0.0
        or args.monitor_max_hz <= 0.0
        or args.ros_max_hz <= 0.0
        or args.preview_hz <= 0.0
    ):
        print("HATA: confidence, max-sync-ms, source-timeout-ms veya max-joint-spread-m gecersiz.", file=sys.stderr)
        return 2
    resolved_extrinsics = args.extrinsics.expanduser().resolve() if args.extrinsics else None
    try:
        extrinsics = load_extrinsics(resolved_extrinsics, endpoints)
    except ValueError as exc:
        print(f"HATA: {exc}", file=sys.stderr)
        return 2
    if extrinsics is None:
        print("UYARI: extrinsic yok. Kayit yapilabilir ancak guvenli olarak sadece en iyi tek kamera paketlenecek.")
    elif args.calibration_record is not None:
        print("BILGI: Kalibrasyon kaydinda tum 4 kamera zorunludur; --minimum-sources yalniz canli cikisi etkiler.")

    selector = selectors.DefaultSelector()
    sockets: list[socket.socket] = []
    try:
        for endpoint in endpoints:
            receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            receiver.bind((args.bind, endpoint.port))
            receiver.setblocking(False)
            selector.register(receiver, selectors.EVENT_READ, data=("body", endpoint))
            sockets.append(receiver)
            print(f"DINLE | ZED {endpoint.serial} | {args.bind}:{endpoint.port}")
        for endpoint in preview_endpoints:
            receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            receiver.bind((args.bind, endpoint.port))
            receiver.setblocking(False)
            selector.register(receiver, selectors.EVENT_READ, data=("preview", endpoint))
            sockets.append(receiver)
            print(f"ONIZLE | ZED {endpoint.serial} | {args.bind}:{endpoint.port}")
    except OSError as exc:
        print(f"HATA: UDP portu acilamadi: {exc}", file=sys.stderr)
        for item in sockets:
            item.close()
        return 3

    output_targets: dict[str, tuple[tuple[str, int], float]] = {}
    if args.output_host:
        output_targets["gmr"] = ((args.output_host, args.output_port), args.output_max_hz)
    if args.monitor_host:
        output_targets["monitor"] = ((args.monitor_host, args.monitor_port), args.monitor_max_hz)
    if args.ros_host:
        output_targets["ros"] = ((args.ros_host, args.ros_port), args.ros_max_hz)
    output_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if output_targets else None
    last_target_send = {name: 0.0 for name in output_targets}
    record_file = None
    if args.calibration_record:
        path = args.calibration_record.expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        record_file = path.open("w", encoding="utf-8", buffering=1)
        record_file.write(json.dumps(sanitize({
            "schema": "zed_body38_multihost_calibration_metadata/v1",
            "created_unix_ns": time.time_ns(),
            "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
            "units": "meter",
            "sources": [{"serial": item.serial, "port": item.port} for item in endpoints],
            "maximum_arrival_spread_ms": args.max_sync_ms,
            "instruction": "Tripodlar sabitken ortak gorus alaninda 20-30 saniye T-pozda sakin durun.",
        }), ensure_ascii=False, allow_nan=False) + "\n")
        print(f"HAM KALIBRASYON KAYDI: {path}")

    args.output_dir = args.output_dir.expanduser().resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    record_path = args.output_dir / f"{args.record_stem}_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
    fusion_record_file = None
    recording = False
    recorded = 0
    record_dropped = 0
    record_queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=300)
    writer_stop = threading.Event()
    writer_thread: threading.Thread | None = None
    writer_errors: list[str] = []

    def writer_loop() -> None:
        nonlocal recorded
        while not writer_stop.is_set() or not record_queue.empty():
            try:
                item = record_queue.get(timeout=0.10)
            except queue.Empty:
                continue
            try:
                if fusion_record_file is not None:
                    fusion_record_file.write(json.dumps(sanitize(item), ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n")
                    recorded += 1
            except Exception as exc:
                writer_errors.append(str(exc))
            finally:
                record_queue.task_done()

    def set_recording(enabled: bool) -> None:
        nonlocal fusion_record_file, writer_thread, recording
        if enabled and fusion_record_file is None:
            fusion_record_file = record_path.open("w", encoding="utf-8", buffering=1024 * 1024)
            extrinsic_document = None
            if resolved_extrinsics is not None:
                try:
                    extrinsic_document = json.loads(resolved_extrinsics.read_text(encoding="utf-8"))
                except (OSError, json.JSONDecodeError):
                    extrinsic_document = None
            header = sanitize({
                "schema": "zed_four_body38_g1_reference/metadata/v1",
                "created_unix_ns": time.time_ns(),
                "body_format": "BODY_38",
                "keypoint_names": BODY38_NAMES,
                "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
                "units": "meter",
                "serial_numbers": [item.serial for item in endpoints],
                "body_sources": [{"serial": item.serial, "port": item.port} for item in endpoints],
                "preview_sources": [{"serial": item.serial, "port": item.port} for item in preview_endpoints],
                "extrinsics_path": str(resolved_extrinsics) if resolved_extrinsics else None,
                "extrinsics": extrinsic_document,
                "safety": "Perception only; contains no physical robot motor commands.",
            })
            fusion_record_file.write(json.dumps(header, ensure_ascii=False, allow_nan=False) + "\n")
            writer_thread = threading.Thread(target=writer_loop, name="four-body38-jsonl-writer", daemon=True)
            writer_thread.start()
        recording = enabled
        print(f"4-ZED fusion kayit {'ACIK' if enabled else 'KAPALI'}: {record_path} ({recorded} kare, drop={record_dropped})", flush=True)

    histories: dict[int, deque[Sample]] = {item.serial: deque(maxlen=16) for item in endpoints}
    last_any_packet_ns: dict[int, int] = {}
    last_body_packet_ns: dict[int, int] = {}
    last_source_status: dict[int, str] = {}
    per_source_input = {item.serial: 0 for item in endpoints}
    per_source_status = {item.serial: 0 for item in endpoints}
    last_bundle: tuple[tuple[int, int], ...] | None = None
    last_output_at = 0.0
    last_record_at = 0.0
    output_sequence = 0
    input_packets = 0
    invalid_packets = 0
    status_packets = 0
    fused_packets = 0
    raw_records = 0
    last_fused_serials: list[int] = []
    last_arrival_spread_ms = math.nan
    preview_frames: dict[int, np.ndarray] = {}
    preview_received_ns: dict[int, int] = {}
    any_events = {item.serial: deque(maxlen=120) for item in endpoints}
    body_events = {item.serial: deque(maxlen=120) for item in endpoints}
    output_events: deque[float] = deque(maxlen=120)
    source_metrics: dict[int, dict[str, Any]] = {item.serial: {} for item in endpoints}
    last_preview_at = 0.0
    started = time.monotonic()
    last_status = started
    if args.record:
        set_recording(True)
    print("HAZIR | Q/ESC: cikis | S: fusion JSONL kayit ac/kapat")
    try:
        while True:
            if msvcrt is not None and msvcrt.kbhit():
                pressed = msvcrt.getwch().lower()
                if pressed in ("q", "\x1b"):
                    break
                if pressed == "s":
                    set_recording(not recording)
            events = selector.select(timeout=0.20)
            for key, _ in events:
                kind, endpoint = key.data
                if kind == "preview":
                    # JPEG preview is presentation-only.  If decoding falls
                    # behind, drain the socket and decode just the newest
                    # datagram instead of spending fusion time on stale video.
                    latest_preview: bytes | None = None
                    while True:
                        try:
                            latest_preview, _sender = key.fileobj.recvfrom(65535)
                        except BlockingIOError:
                            break
                        except OSError:
                            break
                    if cv2 is not None and latest_preview is not None:
                        encoded_image = np.frombuffer(latest_preview, dtype=np.uint8)
                        decoded = cv2.imdecode(encoded_image, cv2.IMREAD_COLOR)
                        if decoded is not None:
                            preview_frames[endpoint.serial] = decoded
                            preview_received_ns[endpoint.serial] = time.time_ns()
                    continue
                while True:
                    try:
                        payload, _sender = key.fileobj.recvfrom(65535)
                    except BlockingIOError:
                        break
                    except OSError:
                        break
                    try:
                        document = json.loads(payload.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        invalid_packets += 1
                        continue
                    if not isinstance(document, dict):
                        invalid_packets += 1
                        continue
                    received_ns = time.time_ns()
                    last_any_packet_ns[endpoint.serial] = received_ns
                    any_events[endpoint.serial].append(time.monotonic())
                    if document.get("schema") == "zed_body38_live/status/v1":
                        # A strict/monitor source reports frame-integrity state
                        # on the same port.  It is not a body payload and must
                        # never make the receiver's real malformed-packet count
                        # look like a network fault.
                        status_packets += 1
                        per_source_status[endpoint.serial] += 1
                        last_source_status[endpoint.serial] = str(document.get("status", "STATUS"))
                        continue
                    if document.get("schema") != "zed_body38_live/v1":
                        invalid_packets += 1
                        continue
                    declared_serial = int(document.get("source_serial", endpoint.serial) or endpoint.serial)
                    if declared_serial not in (0, endpoint.serial):
                        invalid_packets += 1
                        print(f"UYARI: port {endpoint.port} ZED {endpoint.serial} beklerken {declared_serial} paketi geldi; atlandi.", file=sys.stderr)
                        continue
                    if points(document.get("keypoints_3d_m")).shape != (38, 3):
                        invalid_packets += 1
                        continue
                    histories[endpoint.serial].append(Sample(
                        serial=endpoint.serial,
                        packet=document,
                        received_ns=received_ns,
                        sequence=int(document.get("sequence", 0) or 0),
                    ))
                    last_body_packet_ns[endpoint.serial] = received_ns
                    body_events[endpoint.serial].append(time.monotonic())
                    last_source_status[endpoint.serial] = "BODY"
                    per_source_input[endpoint.serial] += 1
                    input_packets += 1

            now_ns = time.time_ns()
            selection = synchronized_samples(
                histories,
                now_ns=now_ns,
                source_timeout_ns=int(args.source_timeout_ms * 1.0e6),
                maximum_spread_ns=int(args.max_sync_ms * 1.0e6),
                minimum_sources=args.minimum_sources,
            )
            if selection is not None:
                samples, arrival_spread_ms = selection
                marker = tuple(sorted((sample.serial, sample.sequence) for sample in samples))
                is_new = marker != last_bundle
                now = time.monotonic()
                if is_new:
                    last_bundle = marker
                    if record_file is not None and len(samples) == len(endpoints) and now - last_record_at >= 0.98 / args.calibration_max_hz:
                        record_file.write(json.dumps(sanitize({
                            "schema": "zed_body38_multihost_calibration_sample/v1",
                            "recorded_unix_ns": time.time_ns(),
                            "arrival_spread_ms": arrival_spread_ms,
                            "sources": {str(sample.serial): compact_sample(sample) for sample in samples},
                        }), ensure_ascii=False, allow_nan=False) + "\n")
                        raw_records += 1
                        last_record_at = now
                    if extrinsics is not None and now - last_output_at >= 0.98 / args.output_max_hz:
                        views = [(sample, extrinsics.cameras[sample.serial]) for sample in samples]
                        status_now_ns = time.time_ns()
                        online_serials = sorted(serial for serial, stamp in last_any_packet_ns.items() if status_now_ns - stamp <= int(args.source_timeout_ms * 1.0e6))
                        for sample in samples:
                            capture_ns = int(sample.packet.get("timestamp_ns", 0) or 0)
                            trace = sample.packet.get("latency_trace_ns") or {}
                            if isinstance(trace, dict):
                                capture_ns = int(trace.get("t0_capture_ns", capture_ns) or capture_ns)
                            latency_ms = (sample.received_ns - capture_ns) / 1.0e6 if capture_ns else None
                            source_metrics[sample.serial].update({
                                "status": last_source_status.get(sample.serial, "BODY"),
                                "body_fps": event_rate(body_events[sample.serial], now),
                                "rx_fps": event_rate(any_events[sample.serial], now),
                                "capture_to_receive_ms": latency_ms,
                                "last_body_ns": last_body_packet_ns.get(sample.serial),
                                "source_transport": sample.packet.get("transport_metrics") or {},
                            })
                        effective_hz = event_rate(output_events, now)
                        packet = make_output_packet(
                            views,
                            minimum_confidence=args.confidence,
                            maximum_spread_m=args.max_joint_spread_m,
                            output_sequence=output_sequence,
                            reference_serial=extrinsics.reference_serial,
                            arrival_spread_ms=arrival_spread_ms,
                            source_metrics=source_metrics,
                            connected_serials=online_serials,
                            effective_output_hz=effective_hz,
                            record_queue_depth=record_queue.qsize(),
                            record_dropped=record_dropped,
                            maximum_pose_disagreement_m=args.max_pose_disagreement_m,
                            maximum_alignment_translation_m=args.max_alignment_translation_m,
                        )
                        compact = compact_live_packet(packet)
                        compact_encoded = json.dumps(compact, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
                        full_encoded = json.dumps(packet, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
                        if len(compact_encoded) > 60000:
                            print(f"UYARI: kompakt fusion UDP paketi cok buyuk ({len(compact_encoded)} byte).", file=sys.stderr)
                        else:
                            # Control is deliberately sent before analysis and disk work.
                            if output_socket is not None:
                                for target_name in ("gmr", "monitor", "ros"):
                                    target_config = output_targets.get(target_name)
                                    if target_config is None:
                                        continue
                                    target, target_hz = target_config
                                    if now - last_target_send[target_name] < 0.98 / target_hz:
                                        continue
                                    outgoing = full_encoded if target_name == "monitor" and len(full_encoded) <= 60000 else compact_encoded
                                    try:
                                        output_socket.sendto(outgoing, target)
                                        last_target_send[target_name] = now
                                    except OSError as exc:
                                        print(f"UYARI: {target_name} UDP gonderilemedi: {exc}", file=sys.stderr)
                            output_sequence += 1
                            fused_packets += 1
                            output_events.append(now)
                            last_fused_serials = sorted(int(value) for value in (packet.get("fusion") or {}).get("contributing_serials", []))
                            last_arrival_spread_ms = arrival_spread_ms
                            last_output_at = now
                            if recording:
                                try:
                                    record_queue.put_nowait(packet)
                                except queue.Full:
                                    record_dropped += 1

            now = time.monotonic()
            status_now_ns = time.time_ns()
            timeout_ns = int(args.source_timeout_ms * 1.0e6)
            online_serials = sorted(serial for serial, stamp in last_any_packet_ns.items() if status_now_ns - stamp <= timeout_ns)
            body_serials = sorted(serial for serial, stamp in last_body_packet_ns.items() if status_now_ns - stamp <= timeout_ns)
            for endpoint in endpoints:
                metric = source_metrics[endpoint.serial]
                metric.update({
                    "status": last_source_status.get(endpoint.serial, "YOK"),
                    "body_fps": event_rate(body_events[endpoint.serial], now),
                    "rx_fps": event_rate(any_events[endpoint.serial], now),
                    "last_body_ns": last_body_packet_ns.get(endpoint.serial),
                    "preview_age_ms": (status_now_ns - preview_received_ns[endpoint.serial]) / 1.0e6 if endpoint.serial in preview_received_ns else None,
                })
            if preview_endpoints and not args.headless and now - last_preview_at >= 0.98 / args.preview_hz:
                preview = draw_four_preview(
                    preview_frames, endpoints, source_metrics,
                    connected=online_serials,
                    body_fresh=body_serials,
                    contributing=last_fused_serials,
                    output_hz=event_rate(output_events, now),
                    arrival_spread_ms=last_arrival_spread_ms,
                    recording=recording,
                    recorded=recorded,
                    record_dropped=record_dropped,
                )
                if preview is not None:
                    cv2.imshow("Four ZED 2i BODY_38 Fusion - G1", preview)
                    key_code = cv2.waitKey(1) & 0xFF
                    if key_code in (ord("q"), ord("Q"), 27):
                        break
                    if key_code in (ord("s"), ord("S")):
                        set_recording(not recording)
                last_preview_at = now

            if now - last_status >= 1.0:
                details = ", ".join(
                    f"{item.serial}:{last_source_status.get(item.serial, 'YOK')}/body={source_metrics[item.serial].get('body_fps', 0.0):.1f}fps/rx={source_metrics[item.serial].get('rx_fps', 0.0):.1f}fps"
                    for item in endpoints
                )
                print(
                    f"DURUM | bagli={len(online_serials)}/4 [{', '.join(map(str, online_serials)) or 'yok'}] "
                    f"| body_taze={len(body_serials)}/4 [{', '.join(map(str, body_serials)) or 'yok'}] | input={input_packets} "
                    f"| ham_kayit={raw_records} | fusion_cikis={fused_packets} "
                    f"| son_katki=[{', '.join(map(str, last_fused_serials)) or 'yok'}] "
                    f"| fusion_fps={event_rate(output_events, now):.1f} "
                    f"| yayilim_ms={last_arrival_spread_ms:.1f} "
                    f"| rec={recorded}/drop={record_dropped}/q={record_queue.qsize()} "
                    f"| durum={status_packets} | gecersiz={invalid_packets} | {details}",
                    flush=True,
                )
                last_status = now
            if args.duration > 0.0 and now - started >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if record_file is not None:
            record_file.close()
        if output_socket is not None:
            output_socket.close()
        if fusion_record_file is not None:
            recording = False
            record_queue.join()
            writer_stop.set()
            if writer_thread is not None:
                writer_thread.join(timeout=5.0)
            fusion_record_file.flush()
            fusion_record_file.close()
        if cv2 is not None and not args.headless:
            cv2.destroyAllWindows()
        for item in sockets:
            try:
                selector.unregister(item)
            except Exception:
                pass
            item.close()
        selector.close()
    print(f"Bitti | input={input_packets} | ham_kayit={raw_records} | fusion_cikis={fused_packets} | kayit={recorded} | drop={record_dropped}")
    if fusion_record_file is not None:
        print(f"FUSION JSONL: {record_path}")
    if writer_errors:
        print(f"UYARI: JSONL yazici hatasi: {writer_errors[-1]}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

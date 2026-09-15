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
from itertools import combinations
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
from hand_tracking.contracts import validate_hand_packet
from hand_tracking.fusion import CameraPose, fuse_hand_packets, interpolate_landmarks
from hand_tracking.normalization import PalmNormalizer
from hand_tracking.retargeting import Dex3Retargeter, OfficialDexRetargetingSolver


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
    capture_timeline_ns: int | None = None
    source_clock_offset_ns: int | None = None
    source_host_id: str = ""


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
    rejection_reason: str | None = None


CRITICAL_GROUPS = {
    "torso": ("PELVIS", "SPINE_3", "NECK", "LEFT_SHOULDER", "RIGHT_SHOULDER"),
    "left_arm": ("LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST"),
    "right_arm": ("RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST"),
    "left_leg": ("LEFT_HIP", "LEFT_KNEE", "LEFT_ANKLE"),
    "right_leg": ("RIGHT_HIP", "RIGHT_KNEE", "RIGHT_ANKLE"),
}

# Identity association must not reject a useful camera merely because an arm
# is hidden by the torso.  Limbs are fused joint-by-joint below; only stable
# torso landmarks decide whether two views belong to the same pose/person.
ASSOCIATION_BODY_NAMES = (
    "PELVIS", "SPINE_1", "SPINE_2", "SPINE_3", "NECK",
    "LEFT_CLAVICLE", "RIGHT_CLAVICLE", "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_HIP", "RIGHT_HIP",
)
ARM_CHAIN_NAMES = {
    "left": ("LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST"),
    "right": ("RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST"),
}
ARM_JOINT_NAMES = {
    "left": (
        "LEFT_CLAVICLE", "LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST",
        "LEFT_HAND_THUMB_4", "LEFT_HAND_INDEX_1", "LEFT_HAND_MIDDLE_4",
        "LEFT_HAND_PINKY_1",
    ),
    "right": (
        "RIGHT_CLAVICLE", "RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST",
        "RIGHT_HAND_THUMB_4", "RIGHT_HAND_INDEX_1", "RIGHT_HAND_MIDDLE_4",
        "RIGHT_HAND_PINKY_1",
    ),
}
ARM_SIDE_BY_INDEX = {
    IDX[name]: side
    for side, names in ARM_JOINT_NAMES.items()
    for name in names
}


class ClockOffsetEstimator:
    """Map independent sender wall clocks onto the receiver timeline.

    Windows hosts can differ by tens of milliseconds even after ``w32tm``.
    The lower envelope of ``receive - send`` contains host clock offset plus
    minimum network transit; subtracting it yields a stable, non-negative
    application latency and comparable capture timestamps without pretending
    that NTP provides hardware synchronization.
    """

    def __init__(self, window: int = 300, quantile: float = 0.05) -> None:
        self.window = max(20, int(window))
        self.quantile = float(np.clip(quantile, 0.0, 0.25))
        self._deltas: dict[str, deque[int]] = {}

    def update(
        self,
        source_host_id: str,
        packet: dict[str, Any],
        received_ns: int,
    ) -> tuple[int | None, int | None, dict[str, float | None]]:
        trace = packet.get("latency_trace_ns") or {}
        capture_ns = int(trace.get("t0_capture_ns", packet.get("timestamp_ns", 0)) or 0)
        send_ns = int(trace.get("t2_windows_udp_send_ns", 0) or 0)
        if capture_ns <= 0 or send_ns <= 0 or send_ns < capture_ns:
            return None, None, {
                "raw_clock_mixed_capture_to_receive_ms": None,
                "clock_offset_estimate_ms": None,
                "corrected_capture_to_receive_ms": None,
                "network_queue_ms": None,
            }
        history = self._deltas.setdefault(str(source_host_id), deque(maxlen=self.window))
        history.append(int(received_ns) - send_ns)
        ordered = np.sort(np.asarray(history, dtype=np.int64))
        index = int(round((len(ordered) - 1) * self.quantile))
        offset_ns = int(ordered[index])
        corrected_capture_ns = capture_ns + offset_ns
        return corrected_capture_ns, offset_ns, {
            "raw_clock_mixed_capture_to_receive_ms": (received_ns - capture_ns) / 1.0e6,
            "clock_offset_estimate_ms": offset_ns / 1.0e6,
            "corrected_capture_to_receive_ms": max(0.0, (received_ns - corrected_capture_ns) / 1.0e6),
            "network_queue_ms": max(0.0, (received_ns - (send_ns + offset_ns)) / 1.0e6),
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


def valid_body38_payload(document: dict[str, Any]) -> bool:
    """Reject malformed/non-BODY_38 datagrams before they enter histories."""
    if document.get("schema") != "zed_body38_live/v1":
        return False
    if tuple(document.get("keypoint_names") or ()) != BODY38_NAMES:
        return False
    try:
        raw_points = np.asarray(document.get("keypoints_3d_m"), dtype=np.float64)
        raw_confidence = np.asarray(
            document.get("keypoint_confidence"), dtype=np.float64
        )
    except (TypeError, ValueError):
        return False
    if raw_points.shape != (38, 3) or raw_confidence.shape != (38,):
        return False
    finite_joints = np.isfinite(raw_points).all(axis=1)
    # A body datagram needs a finite pelvis and enough torso/limb evidence to
    # be meaningful.  Individual occluded joints may legitimately be null.
    return bool(finite_joints[IDX["PELVIS"]] and np.count_nonzero(finite_joints) >= 5)


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
    if document.get("units") != "meter":
        raise ValueError("Extrinsic birimi meter olmali.")
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
        if (
            not np.isclose(np.linalg.det(rotation), 1.0, atol=0.03)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=0.03)
        ):
            raise ValueError(f"ZED {endpoint.serial} rotation matrisi proper rotation degil.")
        result[endpoint.serial] = Extrinsic(rotation, translation)
    if reference_serial not in result:
        raise ValueError(f"Referans ZED {reference_serial} dort kaynak arasinda yok.")
    reference = result[reference_serial]
    if not (
        np.allclose(reference.rotation, np.eye(3), atol=0.03)
        and np.allclose(reference.translation, np.zeros(3), atol=0.03)
    ):
        raise ValueError(
            "Referans ZED pozu identity degil. ZED360 pozlarini secilen "
            "reference camera frame'ine rebase ederek yeniden donusturun."
        )
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


def sample_timeline_ns(sample: Sample) -> int:
    """Comparable capture time, falling back to the receiver clock."""
    return int(sample.capture_timeline_ns or sample.received_ns)


def world_forward_in_workspace(
    sample: Sample,
    extrinsic: Extrinsic,
    minimum_x_m: float,
    maximum_x_m: float,
) -> tuple[bool, float | None]:
    """Check the pelvis in the calibrated reference camera's X-forward frame.

    Camera-local range cannot describe a four-corner rig: the same operator
    may be 2.7 m from one ZED and 4.8 m from the opposite ZED.  The calibrated
    world frame makes the project's 2--4 m operator volume unambiguous and
    rejects the background person near X=6.5 m.
    """
    xyz = transformed_points(sample.packet, extrinsic)
    pelvis = xyz[IDX["PELVIS"]]
    if not np.isfinite(pelvis).all():
        return False, None
    forward_x_m = float(pelvis[0])
    return bool(minimum_x_m <= forward_x_m <= maximum_x_m), forward_x_m


def workspace_membership_with_hysteresis(
    forward_x_m: float | None,
    *,
    was_inside: bool,
    minimum_x_m: float,
    maximum_x_m: float,
    hysteresis_m: float,
) -> bool:
    """Use expanded exit bounds so boundary noise does not flap contributors."""
    if forward_x_m is None or not math.isfinite(float(forward_x_m)):
        return False
    margin = max(0.0, float(hysteresis_m)) if was_inside else 0.0
    return bool(
        minimum_x_m - margin
        <= float(forward_x_m)
        <= maximum_x_m + margin
    )


def temporal_compensate_samples(
    selected: list[Sample],
    histories: dict[int, deque[Sample]],
    *,
    minimum_confidence: float,
    maximum_prediction_ms: float = 70.0,
    maximum_joint_speed_m_s: float = 5.0,
    maximum_displacement_m: float = 0.25,
) -> list[Sample]:
    """Predict older views to the newest common capture instant.

    ZED 2i USB cameras are not hardware triggered.  At 15 fps, otherwise
    valid views can describe an arm almost one frame apart.  This bounded
    constant-velocity correction is applied only where two consecutive,
    confident observations of the same locked BODY id exist.  It therefore
    reduces phase smear without manufacturing long drop-out trajectories.
    """
    if not selected:
        return []
    target_ns = max(sample_timeline_ns(sample) for sample in selected)
    maximum_prediction_ns = int(max(0.0, maximum_prediction_ms) * 1.0e6)
    result: list[Sample] = []
    for sample in selected:
        current_ns = sample_timeline_ns(sample)
        lag_ns = max(0, min(target_ns - current_ns, maximum_prediction_ns))
        packet = dict(sample.packet)
        predicted_ms = 0.0
        predicted_joints = 0
        if lag_ns > 0:
            previous_candidates = [
                item for item in histories.get(sample.serial, ())
                if item.sequence != sample.sequence
                and sample_timeline_ns(item) < current_ns
                and item.packet.get("body_id") == sample.packet.get("body_id")
            ]
            if previous_candidates:
                previous = max(previous_candidates, key=sample_timeline_ns)
                delta_ns = current_ns - sample_timeline_ns(previous)
                if 10_000_000 <= delta_ns <= 250_000_000:
                    current_points = points(sample.packet.get("keypoints_3d_m"))
                    previous_points = points(previous.packet.get("keypoints_3d_m"))
                    current_conf = confidence(sample.packet.get("keypoint_confidence"))
                    previous_conf = confidence(previous.packet.get("keypoint_confidence"))
                    valid = (
                        np.isfinite(current_points).all(axis=1)
                        & np.isfinite(previous_points).all(axis=1)
                        & (current_conf >= minimum_confidence)
                        & (previous_conf >= minimum_confidence)
                    )
                    velocity = (current_points - previous_points) / (delta_ns / 1.0e9)
                    speed = np.linalg.norm(velocity, axis=1)
                    scale = np.ones(38, dtype=np.float64)
                    too_fast = speed > maximum_joint_speed_m_s
                    scale[too_fast] = maximum_joint_speed_m_s / np.maximum(speed[too_fast], 1.0e-9)
                    displacement = velocity * scale[:, None] * (lag_ns / 1.0e9)
                    displacement_norm = np.linalg.norm(displacement, axis=1)
                    too_far = displacement_norm > maximum_displacement_m
                    displacement[too_far] *= (
                        maximum_displacement_m
                        / np.maximum(displacement_norm[too_far], 1.0e-9)
                    )[:, None]
                    predicted = current_points.copy()
                    predicted[valid] += displacement[valid]
                    packet["keypoints_3d_m"] = predicted.tolist()
                    packet["_fusion_temporal_prediction_ms"] = lag_ns / 1.0e6
                    packet["_fusion_temporal_prediction_joints"] = int(np.count_nonzero(valid))
                    predicted_ms = lag_ns / 1.0e6
                    predicted_joints = int(np.count_nonzero(valid))
        if predicted_ms == 0.0:
            packet["_fusion_temporal_prediction_ms"] = 0.0
            packet["_fusion_temporal_prediction_joints"] = predicted_joints
        # Every returned pose describes this common instant, even when a
        # particular source needed no prediction.  Keeping the target private
        # until make_output_packet() prevents the fused packet from inheriting
        # the timestamp of whichever camera happened to have the best score.
        packet["_fusion_target_capture_ns"] = target_ns
        result.append(Sample(
            serial=sample.serial,
            packet=packet,
            received_ns=sample.received_ns,
            sequence=sample.sequence,
            capture_timeline_ns=sample.capture_timeline_ns,
            source_clock_offset_ns=sample.source_clock_offset_ns,
            source_host_id=sample.source_host_id,
        ))
    return result


def synchronized_samples(
    histories: dict[int, deque[Sample]],
    *,
    now_ns: int,
    source_timeout_ns: int,
    maximum_spread_ns: int,
    minimum_sources: int,
    preferred_full_set_spread_ns: int = 40_000_000,
) -> tuple[list[Sample], float] | None:
    """Choose the largest, freshest clock-normalized capture-time bundle."""
    fresh = {
        serial: [sample for sample in history if now_ns - sample.received_ns <= source_timeout_ns]
        for serial, history in histories.items()
    }
    fresh = {serial: samples for serial, samples in fresh.items() if samples}
    if len(fresh) < minimum_sources:
        return None
    anchors = [sample_timeline_ns(sample) for samples in fresh.values() for sample in samples]
    best: tuple[tuple[int, int, int, int], list[Sample], float] | None = None
    bundles: list[tuple[list[Sample], int, int, int]] = []
    for anchor_ns in anchors:
        candidates = [min(samples, key=lambda item: abs(sample_timeline_ns(item) - anchor_ns)) for samples in fresh.values()]
        candidates = [item for item in candidates if abs(sample_timeline_ns(item) - anchor_ns) <= maximum_spread_ns]
        if len(candidates) < minimum_sources:
            continue
        # Four cameras make exhaustive subsets cheap (at most 11 candidates).
        # This matters when three cameras have entered the new 15 Hz cycle but
        # the fourth still has a previous-cycle frame only 66 ms behind: the
        # fresh 3-view bundle must beat that smeared 4-view bundle.
        for count in range(minimum_sources, len(candidates) + 1):
            for subset in combinations(candidates, count):
                selected = list(subset)
                spread_ns = (
                    max(sample_timeline_ns(item) for item in selected)
                    - min(sample_timeline_ns(item) for item in selected)
                )
                if spread_ns > maximum_spread_ns:
                    continue
                oldest_ns = min(sample_timeline_ns(item) for item in selected)
                newest_ns = max(sample_timeline_ns(item) for item in selected)
                bundles.append((selected, spread_ns, oldest_ns, newest_ns))
                # Advance on the newest *common* instant (the oldest member of
                # the bundle), then prefer more cameras and a tighter spread.
                score = (
                    oldest_ns,
                    len(selected),
                    -spread_ns,
                    newest_ns,
                )
                if best is None or score > best[0]:
                    best = (score, selected, spread_ns / 1.0e6)
    if best is not None and bundles:
        # Once all online cameras are inside a tight part of the same capture
        # cycle, prefer that larger bundle over a three-view subset that is
        # only a few milliseconds newer.  Without this bounded coherence
        # preference, asynchronous UDP arrival can repeatedly emit 3/4 just
        # before the fourth 15 Hz packet arrives.  A genuinely previous-cycle
        # fourth frame (roughly 66 ms old) still loses to the fresh 3-view set.
        # At 15 Hz two independently started hosts can have a stable phase
        # offset close to half a frame (~33 ms).  Forty milliseconds still
        # cannot mistake the previous 66.7 ms capture cycle for the current
        # one, while allowing that legitimate fourth camera to participate.
        coherent_ns = min(maximum_spread_ns, preferred_full_set_spread_ns)
        preferred = max(
            bundles,
            key=lambda item: (len(item[0]), item[2], -item[1], item[3]),
        )
        best_oldest_ns = best[0][0]
        if (
            len(preferred[0]) > len(best[1])
            and preferred[1] <= coherent_ns
            and best_oldest_ns - preferred[2] <= coherent_ns
        ):
            best = (
                (
                    preferred[2], len(preferred[0]),
                    -preferred[1], preferred[3],
                ),
                preferred[0],
                preferred[1] / 1.0e6,
            )
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

    core_indexes = np.asarray(
        [IDX[name] for name in ASSOCIATION_BODY_NAMES], dtype=int
    )
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
            rejection_reason=(
                None
                if accepted[index]
                else (
                    "CALIBRATION_DRIFT"
                    if disagreement[index] is not None
                    and disagreement[index] <= maximum_pose_disagreement_m
                    else "CROSS_PERSON_OR_POSE_OUTLIER"
                )
            ),
        ))
    return result


def covariance_weight(packet: dict[str, Any], joint: int) -> float:
    """Convert optional SDK BODY_38 covariance to a bounded reliability."""
    try:
        values = np.asarray(packet.get("keypoints_covariance"), dtype=np.float64)
    except (TypeError, ValueError):
        return 1.0
    if values.ndim != 2 or values.shape[0] != 38 or joint >= values.shape[0]:
        return 1.0
    row = values[joint]
    if row.size >= 6:
        diagonal = row[[0, 3, 5]]
    elif row.size >= 3:
        diagonal = row[:3]
    else:
        return 1.0
    if not np.isfinite(diagonal).all():
        return 1.0
    mean_variance = max(0.0, float(np.mean(diagonal)))
    # ZED reports an all-zero covariance for keypoints with missing depth or
    # outside the image.  It means "unavailable", not perfect certainty.
    if mean_variance <= 1.0e-12:
        return 0.25
    standard_deviation_m = math.sqrt(mean_variance)
    return float(np.clip(1.0 / (1.0 + (standard_deviation_m / 0.06) ** 2), 0.05, 1.0))


def normalized_quaternion_xyzw(value: Any) -> np.ndarray | None:
    candidate = vector(value, 4)
    if not np.isfinite(candidate).all():
        return None
    norm = float(np.linalg.norm(candidate))
    if norm < 1.0e-8:
        return None
    return candidate / norm


def weighted_quaternion_average_xyzw(
    values: list[np.ndarray],
    weights: list[float],
) -> np.ndarray:
    """Average quaternions after resolving the q/-q hemisphere ambiguity."""
    if not values:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    reference_index = int(np.argmax(np.asarray(weights, dtype=np.float64)))
    reference = values[reference_index]
    aligned = np.asarray([
        -value if float(np.dot(value, reference)) < 0.0 else value
        for value in values
    ])
    averaged = np.average(
        aligned,
        axis=0,
        weights=np.maximum(np.asarray(weights, dtype=np.float64), 1.0e-6),
    )
    norm = float(np.linalg.norm(averaged))
    return averaged / norm if norm >= 1.0e-8 else reference.copy()


def fuse_body_orientations(
    accepted: list[PreparedView],
    contribution_serials: list[list[int]],
    *,
    minimum_confidence: float,
) -> tuple[np.ndarray, np.ndarray, list[int | None], dict[str, int | None]]:
    """Fuse BODY_38 rotations from the same views selected for each joint.

    ZED local joint rotations are relative to the BODY_38 parent, so they are
    camera-independent and can be averaged directly.  The global root is first
    transformed into the calibrated world.  This keeps a torso-occluded camera
    from supplying arm rotations after clear front/rear views supplied the arm
    positions.
    """
    by_serial = {item.sample.serial: item for item in accepted}
    local_output = np.tile(
        np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64), (38, 1)
    )
    orientation_serials: list[int | None] = []
    root_values: list[np.ndarray] = []
    root_weights: list[float] = []
    root_serials: list[int] = []

    for joint in range(38):
        side = ARM_SIDE_BY_INDEX.get(joint)
        values: list[np.ndarray] = []
        weights: list[float] = []
        serials: list[int] = []
        for serial in contribution_serials[joint]:
            item = by_serial.get(serial)
            if item is None:
                continue
            raw_orientations = item.sample.packet.get(
                "local_orientation_per_joint_xyzw"
            ) or []
            if joint >= len(raw_orientations):
                continue
            value = normalized_quaternion_xyzw(raw_orientations[joint])
            if value is None:
                continue
            visibility_weight = 1.0
            if side:
                state = arm_view_state(item, side, minimum_confidence)
                if not state["clear"]:
                    visibility_weight = 0.12 if state["overlap"] else 0.25
                    if state["recovered"]:
                        visibility_weight *= 0.60
            weight = (
                max(float(item.confidence[joint]), 0.0) / 100.0
            ) ** 2 * max(item.quality, 0.05) * visibility_weight * covariance_weight(
                item.sample.packet, joint
            )
            values.append(value)
            weights.append(max(weight, 1.0e-6))
            serials.append(serial)
        if values:
            local_output[joint] = weighted_quaternion_average_xyzw(values, weights)
            orientation_serials.append(serials[int(np.argmax(weights))])
        else:
            orientation_serials.append(None)

        if joint == IDX["PELVIS"]:
            for value, weight, serial in zip(values, weights, serials):
                item = by_serial[serial]
                source_root = quaternion_xyzw_to_matrix(
                    item.sample.packet.get("global_root_orientation_xyzw")
                )
                root_values.append(
                    matrix_to_quaternion_xyzw(item.extrinsic.rotation @ source_root)
                )
                root_weights.append(weight)
                root_serials.append(serial)

    global_root = weighted_quaternion_average_xyzw(root_values, root_weights)
    arm_sources = {
        side: next(
            (
                orientation_serials[IDX[name]]
                for name in reversed(ARM_CHAIN_NAMES[side])
                if orientation_serials[IDX[name]] is not None
            ),
            None,
        )
        for side in ARM_CHAIN_NAMES
    }
    if root_serials:
        arm_sources["root"] = root_serials[int(np.argmax(root_weights))]
    else:
        arm_sources["root"] = None
    return local_output, global_root, orientation_serials, arm_sources


def arm_view_state(
    item: PreparedView,
    side: str,
    minimum_confidence: float,
) -> dict[str, Any]:
    chain = [IDX[name] for name in ARM_CHAIN_NAMES[side]]
    complete = bool(
        np.isfinite(item.aligned_points[chain]).all()
        and np.all(item.confidence[chain] >= minimum_confidence)
    )
    analysis = item.sample.packet.get("occlusion_analysis") or {}
    overlap_map = analysis.get("arm_torso_overlap") or {}
    recovered_map = analysis.get("arm_chain_recovered") or {}
    overlap = bool(overlap_map.get(side, overlap_map.get(side.upper(), False)))
    recovered = bool(recovered_map.get(side, recovered_map.get(side.upper(), False)))
    return {
        "complete": complete,
        "overlap": overlap,
        "recovered": recovered,
        "clear": bool(complete and not overlap and not recovered),
        "chain_min_confidence": float(np.min(item.confidence[chain])) if complete else 0.0,
    }


def _fuse_prepared_keypoints(
    prepared: list[PreparedView],
    *,
    minimum_confidence: float,
    maximum_spread_m: float,
) -> tuple[np.ndarray, np.ndarray, list[int], list[list[int]], dict[str, Any]]:
    result = np.full((38, 3), np.nan, dtype=np.float64)
    result_confidence = np.zeros(38, dtype=np.float64)
    contribution_count: list[int] = []
    contribution_serials: list[list[int]] = []
    accepted = [item for item in prepared if item.accepted]
    arm_states = {
        side: {
            item.sample.serial: arm_view_state(item, side, minimum_confidence)
            for item in accepted
        }
        for side in ARM_CHAIN_NAMES
    }
    for joint in range(38):
        candidates: list[dict[str, Any]] = []
        side = ARM_SIDE_BY_INDEX.get(joint)
        for item in accepted:
            if np.isfinite(item.aligned_points[joint]).all() and item.confidence[joint] >= minimum_confidence:
                state = arm_states[side][item.sample.serial] if side else None
                visibility_weight = 1.0
                if state is not None and not state["clear"]:
                    # A recovered/torso-overlapped chain remains available as
                    # a bounded fallback, but it cannot outvote a clear rear
                    # or front camera for this anatomical arm.
                    visibility_weight = 0.12 if state["overlap"] else 0.25
                    if state["recovered"]:
                        visibility_weight *= 0.60
                candidates.append({
                    "point": item.aligned_points[joint],
                    "confidence": float(item.confidence[joint]),
                    "quality": item.quality,
                    "serial": item.sample.serial,
                    "clear": bool(state["clear"]) if state is not None else True,
                    "visibility_weight": visibility_weight,
                    "covariance_weight": covariance_weight(item.sample.packet, joint),
                })
        if not candidates:
            contribution_count.append(0)
            contribution_serials.append([])
            continue
        clear = [item for item in candidates if item["clear"]] if side else []
        if len(clear) >= 2:
            # Two independent clear views are enough to suppress front/back
            # torso-occluded estimates completely.
            pool = clear
            center = np.median(np.asarray([item["point"] for item in clear]), axis=0)
        elif len(clear) == 1:
            # One clear view is the anatomical anchor. Torso-overlapped rear
            # cameras may support it only when they agree tightly; otherwise
            # several low-quality occluded estimates can still drag an arm
            # through the torso despite their smaller individual weights.
            support_radius_m = max(
                0.06, min(0.12, 0.55 * maximum_spread_m)
            )
            pool = [
                item for item in candidates
                if item["clear"]
                or float(np.linalg.norm(item["point"] - clear[0]["point"]))
                <= support_radius_m
            ]
            center = clear[0]["point"]
        else:
            pool = candidates
            center = np.median(np.asarray([item["point"] for item in candidates]), axis=0)
        inliers = [
            item for item in pool
            if float(np.linalg.norm(item["point"] - center)) <= maximum_spread_m
        ]
        if not inliers:
            inliers = [max(
                pool,
                key=lambda item: (
                    item["confidence"] * item["quality"]
                    * item["visibility_weight"] * item["covariance_weight"]
                ),
            )]
        weights = np.asarray([
            max(
                1.0e-4,
                (item["confidence"] / 100.0) ** 2
                * max(item["quality"], 0.05)
                * item["visibility_weight"]
                * item["covariance_weight"],
            )
            for item in inliers
        ])
        joint_values = np.asarray([item["point"] for item in inliers])
        result[joint] = np.average(joint_values, axis=0, weights=weights)
        result_confidence[joint] = float(np.average(
            np.asarray([item["confidence"] for item in inliers]), weights=weights
        ))
        contribution_count.append(len(inliers))
        contribution_serials.append(sorted(int(item["serial"]) for item in inliers))

    evidence: dict[str, Any] = {}
    for side, states in arm_states.items():
        selected = {
            serial
            for name in ARM_JOINT_NAMES[side]
            for serial in contribution_serials[IDX[name]]
        }
        selected_motion = {
            serial
            for name in (f"{side.upper()}_ELBOW", f"{side.upper()}_WRIST")
            for serial in contribution_serials[IDX[name]]
        }
        evidence[side] = {
            "supporting_views": sum(bool(state["complete"]) for state in states.values()),
            "reliable_clear_views": sum(bool(state["clear"]) for state in states.values()),
            "supporting_serials": sorted(serial for serial, state in states.items() if state["complete"]),
            "clear_serials": sorted(serial for serial, state in states.items() if state["clear"]),
            "overlapped_serials": sorted(serial for serial, state in states.items() if state["overlap"]),
            "recovered_serials": sorted(serial for serial, state in states.items() if state["recovered"]),
            "selected_serials": sorted(selected),
            "selected_motion_serials": sorted(selected_motion),
            "occlusion_rejected_serials": sorted(
                serial
                for serial, state in states.items()
                if state["complete"] and serial not in selected_motion
            ),
        }
    return result, result_confidence, contribution_count, contribution_serials, evidence


def fuse_prepared_keypoints(
    prepared: list[PreparedView],
    *,
    minimum_confidence: float,
    maximum_spread_m: float,
) -> tuple[np.ndarray, np.ndarray, list[int]]:
    fused, fused_confidence, counts, _serials, _evidence = _fuse_prepared_keypoints(
        prepared,
        minimum_confidence=minimum_confidence,
        maximum_spread_m=maximum_spread_m,
    )
    return fused, fused_confidence, counts


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
        "normalized_capture_timestamp_ns": sample_timeline_ns(sample),
        "source_clock_offset_ns": sample.source_clock_offset_ns,
        "source_host_id": sample.source_host_id,
        "receiver_timestamp_ns": sample.received_ns,
        "sequence": sample.sequence,
        "body_id": packet.get("body_id"),
        "unique_object_id": packet.get("unique_object_id"),
        "body_confidence": packet.get("body_confidence", 0.0),
        "keypoint_names": packet.get("keypoint_names", BODY38_NAMES),
        "keypoints_3d_m": packet.get("keypoints_3d_m"),
        "keypoint_confidence": packet.get("keypoint_confidence"),
        "keypoints_covariance": packet.get("keypoints_covariance"),
        "operator_selection": packet.get("operator_selection"),
        "distance_quality": packet.get("distance_quality"),
        "euclidean_distance_m": packet.get("euclidean_distance_m"),
        "occlusion_analysis": packet.get("occlusion_analysis"),
    })


def event_rate(events: deque[float], now: float, window_s: float = 2.0) -> float:
    while events and now - events[0] > window_s:
        events.popleft()
    if len(events) < 2:
        return 0.0
    elapsed = events[-1] - events[0]
    return (len(events) - 1) / elapsed if elapsed > 1.0e-6 else 0.0


def metric_text(value: Any, suffix: str = "", precision: int = 1) -> str:
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return f"{float(value):.{precision}f}{suffix}"
    return "n/a"


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


UDP_FLOAT_DECIMALS = 6
UDP_SAFE_DATAGRAM_BYTES = 60_000


def quantize_udp_floats(value: Any) -> Any:
    """Bound JSON float size without changing the lossless recording packet.

    Six decimal places represent one micrometre for metre-valued coordinates,
    far below ZED depth uncertainty.  Python's default JSON encoder otherwise
    emits up to 17 significant digits for every BODY_38 coordinate and can
    turn a valid four-view frame into a 60+ kB UDP datagram.
    """
    if isinstance(value, dict):
        return {key: quantize_udp_floats(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [quantize_udp_floats(item) for item in value]
    if isinstance(value, (float, np.floating)):
        number = float(value)
        return round(number, UDP_FLOAT_DECIMALS) if math.isfinite(number) else None
    return value


def _compact_agreement(value: Any) -> dict[str, Any]:
    agreement = dict(value or {})
    return {
        key: agreement.get(key)
        for key in (
            "mpjpe_m", "p95_error_m", "pelvis_error_m",
            "left_wrist_error_m", "right_wrist_error_m",
        )
        if key in agreement
    }


def compact_live_packet(packet: dict[str, Any]) -> dict[str, Any]:
    """Build the small real-time BODY_38 contract consumed by GMR/Isaac.

    The lossless packet intentionally contains four raw skeletons and rich
    diagnostics.  Starting from ``dict(packet)`` made newly-added diagnostics
    silently leak into the control datagram.  Use an allow-list here so future
    analysis fields cannot stop the real-time control path again.
    """
    keep = (
        "schema", "source_serial", "source_host_id", "sequence",
        "timestamp_ns", "coordinate_system", "units", "body_id",
        "unique_object_id", "tracking_state", "action_state",
        "body_confidence", "root_position_m", "global_root_orientation_xyzw",
        "keypoint_names", "keypoints_3d_raw_m", "keypoints_3d_m",
        "keypoint_confidence", "local_orientation_per_joint_xyzw",
        "pelvis_frame", "operator_selection", "calibration",
        "occlusion_analysis", "perception_metrics", "human_state",
        "control_mode_request", "latency_trace_ns", "transport_metrics",
    )
    result = {key: packet[key] for key in keep if key in packet}

    fusion = dict(packet.get("fusion") or {})
    result["fusion"] = {
        key: fusion.get(key)
        for key in (
            "implementation", "reference_world_serial", "contributing_serials",
            "excluded_pose_serials", "workspace_excluded_serials",
            "best_orientation_serial", "metadata_source_serial",
            "arm_orientation_sources", "arm_evidence",
            "maximum_joint_spread_m", "capture_spread_ms", "capture_stdev_ms",
            "selection_capture_spread_ms", "full_set_wait_applied_ms",
            "arrival_spread_ms",
        )
        if key in fusion
    }

    multi = dict(packet.get("multi_camera") or {})
    compact_multi = {
        key: multi.get(key)
        for key in (
            "mode", "contributing_views", "contributing_serials",
            "excluded_pose_serials", "workspace_excluded_serials",
            "connected_serials", "configured_serials",
            "camera_timestamp_delta_ms", "timestamp_basis", "camera_sync_ok",
            "fused_quality_score", "best_single_quality_score",
            "failure_codes", "arm_evidence",
        )
        if key in multi
    }
    compact_multi["cross_view_agreement"] = _compact_agreement(
        multi.get("cross_view_agreement")
    )
    compact_multi["post_alignment_agreement"] = _compact_agreement(
        multi.get("post_alignment_agreement")
    )
    fusion_metrics = dict(multi.get("fusion_metrics") or {})
    compact_multi["fusion_metrics"] = {
        key: fusion_metrics.get(key)
        for key in (
            "mean_camera_fused", "mean_stdev_between_camera_s",
            "capture_range_between_camera_ms",
        )
        if key in fusion_metrics
    }
    compact_views = []
    for view in multi.get("per_camera") or []:
        source_metrics = dict(view.get("source_metrics") or {})
        compact_view = {
            key: view.get(key)
            for key in (
                "serial_number", "body_id", "unique_object_id", "sequence",
                "body_confidence", "source_timestamp_ns",
                "normalized_capture_timestamp_ns", "source_clock_offset_ns",
                "source_host_id", "receiver_timestamp_ns", "receiver_age_ms",
                "quality_score", "accepted_for_fusion", "rejection_reason",
                "pose_disagreement_to_medoid_m",
                "dynamic_alignment_translation_m", "temporal_prediction_ms",
                "temporal_prediction_joints",
            )
            if key in view
        }
        compact_view["source_metrics"] = {
            key: source_metrics.get(key)
            for key in (
                "status", "body_fps", "rx_fps", "received_fps",
                "capture_to_receive_ms", "received_latency_ms",
                "corrected_capture_to_receive_ms", "network_queue_ms",
                "source_host_id",
            )
            if key in source_metrics
        }
        compact_views.append(compact_view)
    compact_multi["per_camera"] = compact_views
    result["multi_camera"] = compact_multi
    return quantize_udp_floats(result)


def analysis_live_packet(packet: dict[str, Any]) -> dict[str, Any]:
    """Build the bounded four-view packet consumed by live Rerun.

    The lossless JSONL recorder keeps the complete fusion document.  Rerun
    needs the fused BODY_38 skeleton, calibrated per-camera skeletons and
    timing/quality metrics, but not covariance tensors, duplicate source
    packets or every calibration diagnostic.  Keeping an explicit allow-list
    prevents the analysis socket from becoming a 55-60 kB bottleneck while
    preserving everything drawn or exported by ``rerun_analysis``.
    """
    result = compact_live_packet(packet)
    multi = dict(packet.get("multi_camera") or {})
    compact_multi = dict(result.get("multi_camera") or {})
    for key in ("camera_pose_fusion_from_local", "source_operator_selections"):
        if key in multi:
            compact_multi[key] = multi[key]

    source_views = {
        int(view.get("serial_number")): view
        for view in multi.get("per_camera") or []
        if isinstance(view, dict) and view.get("serial_number") is not None
    }
    analysis_views: list[dict[str, Any]] = []
    for compact_view in compact_multi.get("per_camera") or []:
        view = dict(compact_view)
        source = source_views.get(int(view.get("serial_number", -1)), {})
        for key in (
            "keypoints_3d_fusion_m", "keypoint_confidence",
            "operator_selection", "occlusion_analysis", "distance_quality",
            "euclidean_distance_m",
        ):
            if key in source:
                view[key] = source[key]
        analysis_views.append(view)
    compact_multi["per_camera"] = analysis_views

    fusion_metrics = dict(multi.get("fusion_metrics") or {})
    compact_fusion_metrics = dict(compact_multi.get("fusion_metrics") or {})
    if "per_camera" in fusion_metrics:
        compact_fusion_metrics["per_camera"] = fusion_metrics["per_camera"]
    compact_multi["fusion_metrics"] = compact_fusion_metrics
    result["multi_camera"] = compact_multi
    # Hand data is analysis-only. It never enlarges the BODY_38 GMR/ROS control
    # datagram. The sender already falls back to compact body when this exceeds
    # the safe UDP size; lossless JSONL still retains every hand observation.
    for key in ("hand_tracking", "dex3_targets"):
        if key in packet:
            result[key] = packet[key]
    return quantize_udp_floats(result)


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
        capture_ms = metric.get("corrected_capture_to_receive_ms", metric.get("capture_to_receive_ms"))
        capture_text = f"{float(capture_ms):.1f}ms" if isinstance(capture_ms, (int, float)) and math.isfinite(float(capture_ms)) else "n/a"
        queue_ms = metric.get("network_queue_ms")
        queue_text = f"{float(queue_ms):.1f}ms" if isinstance(queue_ms, (int, float)) and math.isfinite(float(queue_ms)) else "n/a"
        world_x = metric.get("world_forward_x_m")
        world_x_text = f"{float(world_x):.2f}m" if isinstance(world_x, (int, float)) and math.isfinite(float(world_x)) else "n/a"
        workspace_state = str(metric.get("workspace_state", "BEKLE"))
        color = (40, 220, 80) if endpoint.serial in body_fresh else (0, 165, 255)
        selection_state = str(metric.get("operator_state", "YOK"))
        locked_id = metric.get("locked_body_id")
        detected_count = int(metric.get("detected_body_count", 0) or 0)
        preview_age_ms = metric.get("preview_age_ms")
        preview_age_text = (
            f"{float(preview_age_ms):.0f}ms"
            if isinstance(preview_age_ms, (int, float))
            and math.isfinite(float(preview_age_ms))
            else "yok"
        )
        lock_text = str(locked_id) if locked_id is not None else "-"
        selection_color = (
            (40, 220, 80)
            if selection_state == "LOCKED"
            else (0, 165, 255)
            if selection_state in {"ACQUIRING", "LOST"}
            else (150, 150, 150)
        )
        # One small translucent console replaces the old 82 px opaque banner.
        # Source previews are intentionally transmitted without their local
        # diagnostics overlay, so the operator's head and shoulders stay clear.
        console_width, console_height = 355, 55
        roi = tile[:console_height, :console_width]
        shade = np.zeros_like(roi)
        cv2.addWeighted(shade, 0.56, roi, 0.44, 0.0, dst=roi)
        cv2.putText(tile, f"ZED {endpoint.serial}  B {body_fps:.1f}  RX {rx_fps:.1f}", (7, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.36, color, 1, cv2.LINE_AA)
        cv2.putText(tile, f"{selection_state} id={lock_text} kisi={detected_count} oniz={preview_age_text}", (7, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.33, selection_color, 1, cv2.LINE_AA)
        cv2.putText(tile, f"{metric.get('status', 'YOK')} X={world_x_text}/{workspace_state} lat={capture_text} q={queue_text}", (7, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.31, (225, 225, 225), 1, cv2.LINE_AA)
        tiles.append(tile)
    while len(tiles) < 4:
        tiles.append(np.zeros((tile_height, tile_width, 3), dtype=np.uint8))
    grid = np.vstack((np.hstack((tiles[0], tiles[1])), np.hstack((tiles[2], tiles[3]))))
    footer = np.zeros((58, grid.shape[1], 3), dtype=np.uint8)
    spread = f"{arrival_spread_ms:.1f} ms" if math.isfinite(arrival_spread_ms) else "n/a"
    cv2.putText(footer, f"4-ZED | bagli={len(connected)}/4 taze={len(body_fresh)}/4 katki={len(contributing)}/4 fusion={output_hz:.1f}fps yayilim={spread}", (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (80, 235, 235), 1, cv2.LINE_AA)
    cv2.putText(footer, f"REC={'ON' if recording else 'OFF'} kare={recorded} drop={record_dropped} | S kayit | Q/ESC cikis", (12, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (60, 220, 80) if recording else (200, 200, 200), 1, cv2.LINE_AA)
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
    maximum_capture_spread_ms: float = 80.0,
    workspace_excluded_serials: list[int] | None = None,
    workspace_x_min_m: float = 2.0,
    workspace_x_max_m: float = 4.0,
    full_set_wait_applied_ms: float = 0.0,
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
    best_sample = best_view.sample
    packet = dict(best_sample.packet)
    # A source covariance describes that camera's estimate, not the fused
    # joint.  Do not mislabel the best camera covariance as fused covariance.
    packet.pop("keypoints_covariance", None)
    packet.pop("_fusion_temporal_prediction_ms", None)
    packet.pop("_fusion_temporal_prediction_joints", None)
    packet.pop("_fusion_target_capture_ns", None)
    fused, fused_confidence, contributions, contribution_serials, arm_evidence = _fuse_prepared_keypoints(
        prepared,
        minimum_confidence=minimum_confidence,
        maximum_spread_m=maximum_spread_m,
    )
    pelvis = fused[IDX["PELVIS"]]
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
    rejection_codes = sorted({
        str(item.rejection_reason)
        for item in prepared
        if not item.accepted and item.rejection_reason
    })
    workspace_excluded_serials = sorted(
        int(value) for value in (workspace_excluded_serials or [])
    )
    if workspace_excluded_serials:
        rejection_codes.append("OUTSIDE_OPERATOR_WORKSPACE")
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
            "normalized_capture_timestamp_ns": sample_timeline_ns(sample),
            "source_clock_offset_ns": sample.source_clock_offset_ns,
            "source_host_id": sample.source_host_id,
            "receiver_timestamp_ns": sample.received_ns,
            "receiver_age_ms": (now_ns - sample.received_ns) / 1.0e6,
            "quality_score": item.quality,
            "accepted_for_fusion": item.accepted,
            "rejection_reason": item.rejection_reason,
            "pose_disagreement_to_medoid_m": item.pose_disagreement_m,
            "dynamic_alignment_translation_m": item.alignment_translation,
            "keypoints_3d_fusion_m": item.aligned_points,
            "keypoint_confidence": item.confidence,
            "keypoints_covariance": sample.packet.get("keypoints_covariance"),
            "occlusion_analysis": sample.packet.get("occlusion_analysis"),
            "distance_quality": sample.packet.get("distance_quality"),
            "euclidean_distance_m": sample.packet.get("euclidean_distance_m"),
            "operator_selection": sample.packet.get("operator_selection"),
            "temporal_prediction_ms": sample.packet.get("_fusion_temporal_prediction_ms", 0.0),
            "temporal_prediction_joints": sample.packet.get("_fusion_temporal_prediction_joints", 0),
            "source_metrics": source_metric,
            "source_latency_trace_ns": sample.packet.get("latency_trace_ns"),
        }))
        camera_poses[str(sample.serial)] = sanitize({
            "rotation_camera_to_world": extrinsic.rotation,
            "translation_camera_to_world_m": extrinsic.translation,
        })
    agreement = cross_view_agreement(raw_agreement_views, fused, minimum_confidence)
    aligned_agreement = cross_view_agreement(aligned_agreement_views, fused, minimum_confidence)
    # BODY_38's operator calibration is a human scale/neutral-pose profile,
    # not the multi-camera extrinsic calibration.  One complete profile is
    # sufficient after every view has already been transformed into the
    # validated common world frame.  Requiring *all* contributing cameras to
    # be READY deadlocks distributed fusion whenever an occluded/rear camera
    # reports COLLECTING or FAILED, even though another camera has a valid
    # profile.  Prefer a READY profile (reference camera first), and expose
    # the other source states as diagnostics rather than making them a global
    # control gate.
    ready_profile_views = [
        item
        for item in accepted
        if (
            (item.sample.packet.get("calibration") or {}).get("state") == "READY"
            and bool((item.sample.packet.get("calibration") or {}).get("profile"))
        )
    ]
    metadata_pool = ready_profile_views or accepted
    metadata_view = max(
        metadata_pool,
        key=lambda item: (
            item.sample.serial == reference_serial,
            int((item.sample.packet.get("calibration") or {}).get("sample_count", 0) or 0),
            item.quality,
        ),
    )
    metadata_sample = metadata_view.sample
    metadata_extrinsic = metadata_view.extrinsic
    calibration = dict(metadata_sample.packet.get("calibration") or {})
    profile = dict(calibration.get("profile") or {})
    neutral = np.asarray(profile.get("neutral_pelvis_rotation_matrix"), dtype=np.float64)
    if neutral.shape == (3, 3) and np.isfinite(neutral).all():
        profile["neutral_pelvis_rotation_matrix"] = (metadata_extrinsic.rotation @ neutral).tolist()
    if profile:
        calibration["profile"] = profile
    calibration["profile_source_serial"] = metadata_sample.serial
    calibration_states = {
        str(item.sample.serial): (item.sample.packet.get("calibration") or {}).get(
            "state", "MISSING"
        )
        for item in accepted
    }
    calibration["source_states"] = calibration_states
    calibration["ready_source_serials"] = sorted(
        int(serial)
        for serial, state in calibration_states.items()
        if state == "READY"
    )
    calibration["ready_profile_source_serials"] = sorted(
        item.sample.serial for item in ready_profile_views
    )
    if ready_profile_views:
        calibration["state"] = "READY"
        calibration["progress"] = 1.0
        calibration["reason"] = "ready_profile_in_calibrated_fusion_world"
    else:
        calibration["state"] = "ACQUIRING"
        calibration["progress"] = min(
            [
                float((item.sample.packet.get("calibration") or {}).get("progress", 0.0) or 0.0)
                for item in accepted
            ]
            or [0.0]
        )
    operator_sources = {
        str(item.sample.serial): dict(
            item.sample.packet.get("operator_selection") or {}
        )
        for item in accepted
    }
    locked_sources = sorted(
        int(serial)
        for serial, selection in operator_sources.items()
        if selection.get("state") == "LOCKED"
    )
    operator_selection = {
        "state": (
            "LOCKED"
            if accepted and len(locked_sources) == len(accepted)
            else "ACQUIRING"
        ),
        "locked_body_id": 0,
        "locked_unique_object_id": "four-camera-fused-operator",
        "missing_frames": max(
            [int(value.get("missing_frames", 0) or 0) for value in operator_sources.values()]
            or [0]
        ),
        "acquisition_frames": min(
            [int(value.get("acquisition_frames", 0) or 0) for value in operator_sources.values()]
            or [0]
        ),
        "reason": "calibrated_world_workspace_multiview_lock",
        "automatic_handover": False,
        "locked_source_serials": locked_sources,
        "source_states": {
            serial: selection.get("state", "MISSING")
            for serial, selection in operator_sources.items()
        },
    }
    fused_occlusion = dict(metadata_sample.packet.get("occlusion_analysis") or {})
    fused_occlusion.update({
        "arm_torso_overlap": {
            side: int(details["reliable_clear_views"]) == 0
            for side, details in arm_evidence.items()
        },
        "arm_chain_recovered": {
            side: bool(
                int(details["reliable_clear_views"]) == 0
                and details["recovered_serials"]
            )
            for side, details in arm_evidence.items()
        },
        "multiview_arm_evidence": arm_evidence,
        "fusion_rule": "per_joint_clear_view_then_confidence_covariance_weighted",
    })
    local_orientations, global_root_orientation, orientation_serials, orientation_sources = (
        fuse_body_orientations(
            accepted,
            contribution_serials,
            minimum_confidence=minimum_confidence,
        )
    )
    # temporal_compensate_samples writes the common target into every view.
    # Direct unit tests and old recordings lack it, so fall back to the newest
    # accepted normalized capture timestamp.
    corrected_capture_ns = max(
        int(
            item.sample.packet.get(
                "_fusion_target_capture_ns", sample_timeline_ns(item.sample)
            )
            or sample_timeline_ns(item.sample)
        )
        for item in accepted
    )
    contributor_capture_times = [
        sample_timeline_ns(item.sample) for item in accepted
    ]
    contributor_spread_ms = (
        (max(contributor_capture_times) - min(contributor_capture_times)) / 1.0e6
        if contributor_capture_times
        else float(arrival_spread_ms)
    )
    contributor_stdev_ms = (
        float(np.std(np.asarray(contributor_capture_times, dtype=np.float64)))
        / 1.0e6
        if contributor_capture_times
        else None
    )
    fused_at_ns = time.time_ns()
    body_confidence = float(np.average(
        np.asarray([
            float(item.sample.packet.get("body_confidence", 0.0) or 0.0)
            for item in accepted
        ]),
        weights=np.maximum(
            np.asarray([item.quality for item in accepted], dtype=np.float64),
            0.05,
        ),
    ))
    packet.update(sanitize({
        "schema": "zed_body38_live/v1",
        "source_serial": 0,
        "source_host_id": "fusion_receiver",
        "sequence": output_sequence,
        "timestamp_ns": corrected_capture_ns,
        "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
        "units": "meter",
        "root_position_m": pelvis,
        "global_root_orientation_xyzw": global_root_orientation,
        "local_orientation_per_joint_xyzw": local_orientations,
        "body_confidence": body_confidence,
        "body_id": 0,
        "tracking_state": "OK",
        "keypoint_names": BODY38_NAMES,
        "keypoints_3d_raw_m": fused,
        "keypoints_3d_m": fused,
        "keypoint_confidence": fused_confidence,
        "root_relative_keypoints_m": pelvis_local,
        # ZED local-position and 2D arrays belong to one physical camera and
        # are not meaningful after world-space per-joint fusion.
        "local_position_per_joint_m": None,
        "keypoints_2d_px": None,
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
        "operator_selection": operator_selection,
        "perception_metrics": metadata_sample.packet.get("perception_metrics"),
        "control_mode_request": metadata_sample.packet.get("control_mode_request"),
        "action_state": metadata_sample.packet.get("action_state"),
        "imu": None,
        "distance_quality": {
            "recommended_min_m": workspace_x_min_m,
            "recommended_max_m": workspace_x_max_m,
            "inside_recommended_range": bool(
                np.isfinite(pelvis).all()
                and workspace_x_min_m <= float(pelvis[0]) <= workspace_x_max_m
            ),
            "basis": "CALIBRATED_REFERENCE_WORLD_X",
        },
        "euclidean_distance_m": (
            float(np.linalg.norm(pelvis)) if np.isfinite(pelvis).all() else None
        ),
        "occlusion_analysis": fused_occlusion,
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
            "workspace_excluded_serials": workspace_excluded_serials,
            "best_orientation_serial": orientation_sources.get("root"),
            "metadata_source_serial": metadata_sample.serial,
            "per_joint_orientation_serials": orientation_serials,
            "arm_orientation_sources": {
                side: orientation_sources.get(side) for side in ARM_CHAIN_NAMES
            },
            "per_joint_contributions": contributions,
            "per_joint_contributing_serials": contribution_serials,
            "arm_evidence": arm_evidence,
            "maximum_joint_spread_m": maximum_spread_m,
            "capture_spread_ms": contributor_spread_ms,
            "capture_stdev_ms": contributor_stdev_ms,
            "selection_capture_spread_ms": arrival_spread_ms,
            "full_set_wait_applied_ms": full_set_wait_applied_ms,
            # Backward-compatible name used by existing Rerun summaries.  In
            # v2 it is the clock-normalized capture spread, not socket arrival.
            "arrival_spread_ms": contributor_spread_ms,
        },
        "multi_camera": {
            "mode": "FOUR_FUSED" if len(accepted) == 4 else "PARTIAL_FUSED",
            "contributing_views": len(accepted),
            "contributing_serials": accepted_serials,
            "excluded_pose_serials": excluded_serials,
            "workspace_excluded_serials": workspace_excluded_serials,
            "connected_serials": connected_serials or [sample.serial for sample, _ in views],
            "configured_serials": sorted(metrics) if metrics else [sample.serial for sample, _ in views],
            "camera_timestamp_delta_ms": contributor_spread_ms,
            "timestamp_basis": "clock_offset_corrected_capture_time",
            "camera_sync_ok": bool(contributor_spread_ms <= maximum_capture_spread_ms),
            "fused_quality_score": float(np.mean(fused_confidence) / 100.0),
            "best_single_quality_score": sample_quality(best_sample.packet, minimum_confidence),
            "cross_view_agreement": agreement,
            "post_alignment_agreement": aligned_agreement,
            "fusion_metrics": {
                "mean_camera_fused": float(np.mean(contributions)),
                "mean_stdev_between_camera_s": (
                    contributor_stdev_ms / 1000.0
                    if contributor_stdev_ms is not None else None
                ),
                "capture_range_between_camera_ms": contributor_spread_ms,
                "per_camera": metrics,
            },
            "per_camera": view_descriptions,
            "source_operator_selections": operator_sources,
            "arm_evidence": arm_evidence,
            "camera_pose_fusion_from_local": camera_poses,
            "failure_codes": sorted(set(
                rejection_codes
                if rejection_codes
                else ([] if len(accepted) == 4 else ["PARTIAL_CAMERA_SET"])
            )),
        },
        "transport_metrics": {
            "source_interval_ms": 1000.0 / effective_output_hz if effective_output_hz > 0.0 else None,
            "effective_output_hz": effective_output_hz,
            "capture_to_send_ms": max(0.0, (fused_at_ns - corrected_capture_ns) / 1.0e6),
            "record_queue_depth": record_queue_depth,
            "record_dropped": record_dropped,
            "source_mode": "distributed_four_zed_body38",
        },
        "latency_trace_ns": {
            "t0_capture_ns": corrected_capture_ns,
            "t2_windows_udp_receive_ns": max(
                item.sample.received_ns for item in accepted
            ),
            "t3_application_fused_ns": fused_at_ns,
            # Replaced immediately before serialization/socket send in main;
            # this value keeps direct callers and offline tests well formed.
            "t2_windows_udp_send_ns": fused_at_ns,
        },
    }))
    return packet


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Iki-host, dort-ZED application BODY_38 fusion alicisi")
    parser.add_argument("--source", type=parse_endpoint, action="append", required=True, help="Her kamera icin SERIAL:UDP_PORT; dort kez verin.")
    parser.add_argument("--hand-source", type=parse_endpoint, action="append", default=[], help="21-landmark kanali icin SERIAL:UDP_PORT; el takibinde dort kez verin.")
    parser.add_argument("--hand-tracking", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--dex3-retargeting", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--hand-max-age-ms", type=float, default=70.0)
    parser.add_argument("--hand-max-spread-ms", type=float, default=40.0)
    parser.add_argument("--hand-single-view-depth", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--dex3-config", type=Path, default=REPOSITORY_ROOT / "config" / "g1_23dof_dex3.json")
    parser.add_argument("--dex3-official-root", type=Path, default=None, help="Opsiyonel resmi unitreerobotics/xr_teleoperate checkout yolu")
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
    parser.add_argument("--max-sync-ms", type=float, default=80.0, help="Saat-ofseti duzeltilmis capture zamanina gore azami paket yayilimi.")
    parser.add_argument("--preferred-full-set-spread-ms", type=float, default=40.0, help="Dort kamerayi ayni 15 Hz dongusu sayip 3 gorunume tercih eden sikilik esigi.")
    parser.add_argument("--full-set-wait-ms", type=float, default=20.0, help="Diger tum kaynaklar canliyken dorduncu yeni kare icin azami bekleme.")
    parser.add_argument("--source-timeout-ms", type=float, default=250.0, help="Bu sureden eski BODY_38 kaynagini taze sayma.")
    parser.add_argument("--minimum-sources", type=int, choices=(2, 3, 4), default=2, help="Calisma aninda cikis icin gereken en az taze kamera.")
    parser.add_argument("--max-joint-spread-m", type=float, default=0.30)
    parser.add_argument("--max-pose-disagreement-m", type=float, default=0.22, help="Farkli kisi/poz gorunumunu dislamak icin pelvis-yerel govde MPJPE esigi.")
    parser.add_argument("--max-alignment-translation-m", type=float, default=0.25, help="Iyi kalibrasyonda izin verilen azami dinamik pelvis cevirisi; asilirsa gorus dislanir.")
    parser.add_argument("--max-temporal-prediction-ms", type=float, default=70.0, help="Eski kamera gorusunu son capture anina tasimak icin azami tahmin (ms).")
    parser.add_argument("--workspace-x-min-m", type=float, default=2.0, help="Referans ZED dunya X ekseninde operator hacminin baslangici (m).")
    parser.add_argument("--workspace-x-max-m", type=float, default=4.0, help="Referans ZED dunya X ekseninde operator hacminin sonu (m).")
    parser.add_argument("--workspace-hysteresis-m", type=float, default=0.15, help="Iceride kilitli operator icin calisma alani cikis toleransi (m).")
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
    hand_endpoints: list[InputEndpoint] = args.hand_source
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
    if args.hand_tracking and (
        len(hand_endpoints) != 4
        or {item.serial for item in hand_endpoints} != {item.serial for item in endpoints}
        or len({item.port for item in hand_endpoints}) != 4
    ):
        print("HATA: el takibi icin ayni dort seriye ait dort farkli --hand-source gerekli.", file=sys.stderr)
        return 2
    if preview_endpoints and not args.headless and cv2 is None:
        print("HATA: dortlu arayuz icin opencv-python kurulu olmali.", file=sys.stderr)
        return 2
    if (
        not 0.0 <= args.confidence <= 100.0
        or args.max_sync_ms <= 0.0
        or not 0.0 <= args.preferred_full_set_spread_ms <= args.max_sync_ms
        or not 0.0 <= args.full_set_wait_ms <= 50.0
        or args.source_timeout_ms <= 0.0
        or args.max_joint_spread_m <= 0.0
        or args.max_pose_disagreement_m <= 0.0
        or args.max_alignment_translation_m <= 0.0
        or args.max_temporal_prediction_ms < 0.0
        or args.workspace_x_min_m < 0.0
        or args.workspace_x_max_m <= args.workspace_x_min_m
        or not 0.0 <= args.workspace_hysteresis_m <= 1.0
        or args.output_max_hz <= 0.0
        or args.monitor_max_hz <= 0.0
        or args.ros_max_hz <= 0.0
        or args.preview_hz <= 0.0
        or args.hand_max_age_ms <= 0.0
        or args.hand_max_spread_ms <= 0.0
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
        for endpoint in hand_endpoints:
            receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            receiver.bind((args.bind, endpoint.port))
            receiver.setblocking(False)
            selector.register(receiver, selectors.EVENT_READ, data=("hand", endpoint))
            sockets.append(receiver)
            print(f"EL21 | ZED {endpoint.serial} | {args.bind}:{endpoint.port}")
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
            "maximum_capture_spread_ms": args.max_sync_ms,
            "timestamp_basis": "clock_offset_corrected_capture_time",
            "instruction": (
                "Ortamda tek kisi olsun. Tum ortak calisma hacmini yavasca dolasin; "
                "grid noktalarinda T/A ve bukulu-dirsek pozlarinda 1-2 saniye durun."
            ),
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
                "hand_sources": [{"serial": item.serial, "port": item.port} for item in hand_endpoints],
                "hand_schema": "zed_operator_hand/v1" if args.hand_tracking else None,
                "extrinsics_path": str(resolved_extrinsics) if resolved_extrinsics else None,
                "extrinsics": extrinsic_document,
                "operator_workspace": {
                    "frame": "REFERENCE_CAMERA_WORLD_X_FWD",
                    "x_min_m": args.workspace_x_min_m,
                    "x_max_m": args.workspace_x_max_m,
                    "hysteresis_m": args.workspace_hysteresis_m,
                },
                "safety": "Perception only; contains no physical robot motor commands.",
            })
            fusion_record_file.write(json.dumps(header, ensure_ascii=False, allow_nan=False) + "\n")
            writer_thread = threading.Thread(target=writer_loop, name="four-body38-jsonl-writer", daemon=True)
            writer_thread.start()
        recording = enabled
        print(f"4-ZED fusion kayit {'ACIK' if enabled else 'KAPALI'}: {record_path} ({recorded} kare, drop={record_dropped})", flush=True)

    histories: dict[int, deque[Sample]] = {item.serial: deque(maxlen=32) for item in endpoints}
    hand_histories: dict[int, deque[dict[str, Any]]] = {item.serial: deque(maxlen=32) for item in endpoints}
    palm_normalizer = PalmNormalizer()
    try:
        dex3_solver = OfficialDexRetargetingSolver(args.dex3_official_root) if args.dex3_official_root else None
        dex3_retargeter = Dex3Retargeter(args.dex3_config, solver=dex3_solver) if args.hand_tracking and args.dex3_retargeting else None
    except (OSError, RuntimeError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"HATA: Dex3 config yuklenemedi: {exc}", file=sys.stderr)
        return 2
    clock_offsets = ClockOffsetEstimator(window=300, quantile=0.05)
    last_any_packet_ns: dict[int, int] = {}
    last_body_packet_ns: dict[int, int] = {}
    last_source_status: dict[int, str] = {}
    per_source_input = {item.serial: 0 for item in endpoints}
    per_source_status = {item.serial: 0 for item in endpoints}
    last_calibration_bundle: tuple[tuple[int, int], ...] | None = None
    last_output_bundle: tuple[tuple[int, int], ...] | None = None
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
    last_gmr_gate = "BEKLE"
    partial_set_wait_started_at: float | None = None
    last_full_set_wait_applied_ms = 0.0
    preview_frames: dict[int, np.ndarray] = {}
    preview_received_ns: dict[int, int] = {}
    any_events = {item.serial: deque(maxlen=120) for item in endpoints}
    body_events = {item.serial: deque(maxlen=120) for item in endpoints}
    output_events: deque[float] = deque(maxlen=120)
    source_metrics: dict[int, dict[str, Any]] = {item.serial: {} for item in endpoints}
    workspace_inside_state: dict[int, bool] = {
        item.serial: False for item in endpoints
    }
    last_preview_at = 0.0
    analysis_fallback_warned = False
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
            select_timeout_s = 0.20
            if partial_set_wait_started_at is not None:
                wait_remaining_s = (
                    args.full_set_wait_ms / 1000.0
                    - (time.monotonic() - partial_set_wait_started_at)
                )
                select_timeout_s = min(
                    select_timeout_s, max(0.001, wait_remaining_s)
                )
            events = selector.select(timeout=select_timeout_s)
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
                if kind == "hand":
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
                        valid, _reason = validate_hand_packet(document)
                        if not valid or int(document.get("camera_serial", 0)) != endpoint.serial:
                            invalid_packets += 1
                            continue
                        latest_body = histories[endpoint.serial][-1] if histories[endpoint.serial] else None
                        offset_ns = latest_body.source_clock_offset_ns if latest_body is not None else 0
                        document["_normalized_capture_timestamp_ns"] = int(document["capture_timestamp_ns"]) + int(offset_ns)
                        hand_histories[endpoint.serial].append(document)
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
                        operator_selection = document.get("operator_selection") or {}
                        source_metrics[endpoint.serial]["operator_state"] = str(
                            operator_selection.get("state", "YOK")
                        )
                        source_metrics[endpoint.serial]["locked_body_id"] = (
                            operator_selection.get("locked_body_id")
                        )
                        source_metrics[endpoint.serial]["detected_body_count"] = int(
                            document.get("detected_body_count", 0) or 0
                        )
                        continue
                    if document.get("schema") != "zed_body38_live/v1":
                        invalid_packets += 1
                        continue
                    declared_serial = int(document.get("source_serial", endpoint.serial) or endpoint.serial)
                    if declared_serial not in (0, endpoint.serial):
                        invalid_packets += 1
                        print(f"UYARI: port {endpoint.port} ZED {endpoint.serial} beklerken {declared_serial} paketi geldi; atlandi.", file=sys.stderr)
                        continue
                    if not valid_body38_payload(document):
                        invalid_packets += 1
                        continue
                    source_host_id = str(
                        document.get("source_host_id") or f"legacy-{endpoint.serial}"
                    )
                    capture_timeline_ns, source_clock_offset_ns, clock_metrics = clock_offsets.update(
                        source_host_id, document, received_ns
                    )
                    source_metrics[endpoint.serial].update(clock_metrics)
                    source_metrics[endpoint.serial]["source_host_id"] = source_host_id
                    source_metrics[endpoint.serial]["capture_to_receive_ms"] = clock_metrics.get(
                        "corrected_capture_to_receive_ms"
                    )
                    operator_selection = document.get("operator_selection") or {}
                    source_metrics[endpoint.serial]["operator_state"] = str(
                        operator_selection.get("state", "YOK")
                    )
                    source_metrics[endpoint.serial]["locked_body_id"] = (
                        operator_selection.get("locked_body_id")
                    )
                    source_metrics[endpoint.serial]["detected_body_count"] = int(
                        document.get("detected_body_count", 1) or 1
                    )
                    histories[endpoint.serial].append(Sample(
                        serial=endpoint.serial,
                        packet=document,
                        received_ns=received_ns,
                        sequence=int(document.get("sequence", 0) or 0),
                        capture_timeline_ns=capture_timeline_ns,
                        source_clock_offset_ns=source_clock_offset_ns,
                        source_host_id=source_host_id,
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
                preferred_full_set_spread_ns=int(
                    args.preferred_full_set_spread_ms * 1.0e6
                ),
            )
            if selection is None:
                partial_set_wait_started_at = None
            if selection is not None:
                samples, arrival_spread_ms = selection
                workspace_samples = samples
                workspace_excluded: list[int] = []
                if extrinsics is not None:
                    workspace_samples = []
                    for sample in samples:
                        _strict_inside, forward_x_m = world_forward_in_workspace(
                            sample,
                            extrinsics.cameras[sample.serial],
                            args.workspace_x_min_m,
                            args.workspace_x_max_m,
                        )
                        inside = workspace_membership_with_hysteresis(
                            forward_x_m,
                            was_inside=workspace_inside_state[sample.serial],
                            minimum_x_m=args.workspace_x_min_m,
                            maximum_x_m=args.workspace_x_max_m,
                            hysteresis_m=args.workspace_hysteresis_m,
                        )
                        workspace_inside_state[sample.serial] = inside
                        source_metrics[sample.serial]["world_forward_x_m"] = forward_x_m
                        source_metrics[sample.serial]["workspace_hysteresis_m"] = (
                            args.workspace_hysteresis_m
                        )
                        source_metrics[sample.serial]["workspace_state"] = (
                            "INSIDE" if inside else "OUTSIDE"
                        )
                        if inside:
                            workspace_samples.append(sample)
                        else:
                            workspace_excluded.append(sample.serial)
                    if len(workspace_samples) < args.minimum_sources:
                        last_fused_serials = []
                now = time.monotonic()
                calibration_marker = tuple(sorted(
                    (sample.serial, sample.sequence) for sample in samples
                ))
                calibration_is_new = calibration_marker != last_calibration_bundle
                if calibration_is_new:
                    last_calibration_bundle = calibration_marker
                    if record_file is not None and len(samples) == len(endpoints) and now - last_record_at >= 0.80 / args.calibration_max_hz:
                        record_file.write(json.dumps(sanitize({
                            "schema": "zed_body38_multihost_calibration_sample/v1",
                            "recorded_unix_ns": time.time_ns(),
                            "arrival_spread_ms": arrival_spread_ms,
                            "sources": {str(sample.serial): compact_sample(sample) for sample in samples},
                        }), ensure_ascii=False, allow_nan=False) + "\n")
                        raw_records += 1
                        last_record_at = now
                if (
                    extrinsics is not None
                    and len(workspace_samples) >= args.minimum_sources
                    and now - last_output_at >= 0.80 / args.output_max_hz
                ):
                    # If all four publishers are alive but the selector has
                    # only received three frames from this capture cycle,
                    # wait a small bounded interval for the fourth datagram.
                    # Do not wait when a four-view bundle was already formed
                    # and a camera was explicitly rejected by the workspace
                    # or pose gate; latency cannot repair that condition.
                    fresh_body_count = sum(
                        now_ns - stamp
                        <= int(args.source_timeout_ms * 1.0e6)
                        for stamp in last_body_packet_ns.values()
                    )
                    should_wait_for_four = bool(
                        args.full_set_wait_ms > 0.0
                        and len(samples) < len(endpoints)
                        and len(workspace_samples) == len(samples)
                        and fresh_body_count == len(endpoints)
                    )
                    if should_wait_for_four:
                        if partial_set_wait_started_at is None:
                            partial_set_wait_started_at = now
                        elapsed_wait_ms = (
                            now - partial_set_wait_started_at
                        ) * 1000.0
                        if elapsed_wait_ms < args.full_set_wait_ms:
                            continue
                        last_full_set_wait_applied_ms = elapsed_wait_ms
                        partial_set_wait_started_at = None
                    else:
                        if partial_set_wait_started_at is not None:
                            last_full_set_wait_applied_ms = (
                                now - partial_set_wait_started_at
                            ) * 1000.0
                        else:
                            last_full_set_wait_applied_ms = 0.0
                        partial_set_wait_started_at = None
                    compensated_samples = temporal_compensate_samples(
                        workspace_samples,
                        histories,
                        minimum_confidence=args.confidence,
                        maximum_prediction_ms=args.max_temporal_prediction_ms,
                    )
                    views = [
                        (sample, extrinsics.cameras[sample.serial])
                        for sample in compensated_samples
                    ]
                    status_now_ns = time.time_ns()
                    online_serials = sorted(
                        serial for serial, stamp in last_any_packet_ns.items()
                        if status_now_ns - stamp <= int(args.source_timeout_ms * 1.0e6)
                    )
                    for sample in compensated_samples:
                        source_metrics[sample.serial].update({
                            "status": last_source_status.get(sample.serial, "BODY"),
                            "body_fps": event_rate(body_events[sample.serial], now),
                            "rx_fps": event_rate(any_events[sample.serial], now),
                            "received_fps": event_rate(body_events[sample.serial], now),
                            "capture_to_receive_ms": source_metrics[sample.serial].get(
                                "corrected_capture_to_receive_ms"
                            ),
                            "received_latency_ms": source_metrics[sample.serial].get(
                                "corrected_capture_to_receive_ms"
                            ),
                            "temporal_prediction_ms": sample.packet.get(
                                "_fusion_temporal_prediction_ms", 0.0
                            ),
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
                        maximum_capture_spread_ms=args.max_sync_ms,
                        workspace_excluded_serials=workspace_excluded,
                        workspace_x_min_m=args.workspace_x_min_m,
                        workspace_x_max_m=args.workspace_x_max_m,
                        full_set_wait_applied_ms=last_full_set_wait_applied_ms,
                    )
                    if args.hand_tracking and extrinsics is not None:
                        target_capture_ns = int(packet["timestamp_ns"])
                        selected_hand_packets: list[dict[str, Any]] = []
                        expected_operator_ids = {
                            sample.serial: (sample.packet.get("operator_selection") or {}).get("locked_body_id")
                            for sample in compensated_samples
                        }
                        for serial, items in hand_histories.items():
                            if not items:
                                continue
                            normalized_items: list[dict[str, Any]] = []
                            for source_item in items:
                                aligned_item = dict(source_item)
                                aligned_item["capture_timestamp_ns"] = int(source_item.get("_normalized_capture_timestamp_ns", source_item["capture_timestamp_ns"]))
                                aligned_item["_source_capture_timestamp_ns"] = aligned_item["capture_timestamp_ns"]
                                normalized_items.append(aligned_item)
                            normalized_items.sort(key=lambda item: item["capture_timestamp_ns"])
                            expected_id = expected_operator_ids.get(serial)
                            normalized_items = [
                                item for item in normalized_items
                                if expected_id is not None
                                and (item.get("operator") or {}).get("body_id") == expected_id
                            ]
                            if not normalized_items:
                                continue
                            past = [item for item in normalized_items if item["capture_timestamp_ns"] <= target_capture_ns]
                            aligned = None
                            if len(past) >= 2:
                                aligned = interpolate_landmarks(
                                    past[-2], past[-1], target_capture_ns,
                                    maximum_prediction_ms=args.hand_max_age_ms,
                                )
                            elif past:
                                aligned = past[-1]
                            else:
                                aligned = min(normalized_items, key=lambda item: abs(item["capture_timestamp_ns"] - target_capture_ns))
                            if aligned is not None and abs(int(aligned["capture_timestamp_ns"]) - target_capture_ns) / 1e6 <= args.hand_max_age_ms:
                                selected_hand_packets.append(aligned)
                        camera_poses = {
                            serial: CameraPose(value.rotation, value.translation)
                            for serial, value in extrinsics.cameras.items()
                        }
                        fused_hands = fuse_hand_packets(
                            selected_hand_packets, camera_poses,
                            target_timestamp_ns=target_capture_ns,
                            maximum_age_ms=args.hand_max_age_ms,
                            maximum_capture_spread_ms=args.hand_max_spread_ms,
                            single_view_depth=args.hand_single_view_depth,
                        )
                        fused_hands["per_camera"] = selected_hand_packets
                        dex3_targets: dict[str, Any] = {"physical_robot_output_enabled": False}
                        for hand in fused_hands["hands"]:
                            side = hand["side"]
                            normalized = None
                            confidence = 0.0
                            if hand.get("valid"):
                                try:
                                    normalized_hand = palm_normalizer.normalize(hand["landmarks_world_m"], side)
                                    normalized = normalized_hand.landmarks
                                    confidence = float(np.mean([item["confidence"] for item in hand["landmark_quality"]]))
                                    hand["normalization"] = {
                                        "landmarks": normalized.tolist(),
                                        "wrist_world_m": normalized_hand.wrist_world_m.tolist(),
                                        "rotation_world_from_palm": normalized_hand.rotation_world_from_palm.tolist(),
                                        "scale_m": normalized_hand.scale_m,
                                        "canonical": "+X thumbward, +Y wrist-to-middle, +Z right-handed normal",
                                    }
                                except ValueError as exc:
                                    hand["valid"] = False
                                    hand["rejection_reasons"].append(str(exc))
                            if dex3_retargeter is not None:
                                dex3_targets[side] = dex3_retargeter.update(side, normalized, target_capture_ns, confidence)
                        if "left" in dex3_targets and "right" in dex3_targets:
                            dex3_targets["q_left_dex3"] = dex3_targets["left"]["safe_q_rad"]
                            dex3_targets["q_right_dex3"] = dex3_targets["right"]["safe_q_rad"]
                            dex3_targets["q_body"] = None
                            dex3_targets["q_target"] = None
                            dex3_targets["composition_status"] = "AWAITING_SEPARATE_23DOF_BODY_RETARGET_TARGET"
                        packet["hand_tracking"] = fused_hands
                        packet["dex3_targets"] = dex3_targets
                    calibration_state = str(
                        (packet.get("calibration") or {}).get("state", "MISSING")
                    )
                    operator_state = str(
                        (packet.get("operator_selection") or {}).get("state", "MISSING")
                    )
                    last_gmr_gate = (
                        "READY"
                        if calibration_state == "READY" and operator_state == "LOCKED"
                        else f"BEKLE({calibration_state}/{operator_state})"
                    )
                    accepted_views = [
                        view
                        for view in (packet.get("multi_camera") or {}).get(
                            "per_camera", []
                        )
                        if view.get("accepted_for_fusion")
                    ]
                    output_marker = tuple(sorted(
                        (
                            int(view.get("serial_number", 0) or 0),
                            int(view.get("sequence", 0) or 0),
                        )
                        for view in accepted_views
                    ))
                    should_emit = bool(
                        len(accepted_views) >= args.minimum_sources
                        and output_marker
                        and output_marker != last_output_bundle
                    )
                    if should_emit:
                        # Stamp as close as possible to JSON serialization and
                        # sendto(). The GMR bridge uses t2-t0 for the Windows
                        # perception/Fusion latency.
                        send_ready_ns = time.time_ns()
                        packet["latency_trace_ns"]["t2_windows_udp_send_ns"] = send_ready_ns
                        packet["transport_metrics"]["capture_to_send_ms"] = max(
                            0.0,
                            (
                                send_ready_ns
                                - int(packet["latency_trace_ns"]["t0_capture_ns"])
                            )
                            / 1.0e6,
                        )
                        compact = compact_live_packet(packet)
                        compact_encoded = json.dumps(
                            compact, ensure_ascii=False, allow_nan=False,
                            separators=(",", ":"),
                        ).encode("utf-8")
                        analysis_encoded = json.dumps(
                            analysis_live_packet(packet),
                            ensure_ascii=False, allow_nan=False,
                            separators=(",", ":"),
                        ).encode("utf-8")
                        if len(compact_encoded) > UDP_SAFE_DATAGRAM_BYTES:
                            print(
                                f"HATA: kontrol fusion UDP paketi cok buyuk "
                                f"({len(compact_encoded)} byte).",
                                file=sys.stderr,
                            )
                        else:
                            if (
                                len(analysis_encoded) > UDP_SAFE_DATAGRAM_BYTES
                                and not analysis_fallback_warned
                            ):
                                print(
                                    "UYARI: ayrintili Rerun UDP paketi siniri asti; "
                                    "kontrol-guvenli pakete dusuldu. Lossless JSONL "
                                    "kaydi tam kalir.",
                                    file=sys.stderr,
                                )
                                analysis_fallback_warned = True
                            # Control is deliberately sent before analysis and disk work.
                            if output_socket is not None:
                                for target_name in ("gmr", "monitor", "ros"):
                                    target_config = output_targets.get(target_name)
                                    if target_config is None:
                                        continue
                                    target, target_hz = target_config
                                    if now - last_target_send[target_name] < 0.80 / target_hz:
                                        continue
                                    outgoing = (
                                        analysis_encoded
                                        if target_name == "monitor"
                                        and len(analysis_encoded) <= UDP_SAFE_DATAGRAM_BYTES
                                        else compact_encoded
                                    )
                                    try:
                                        output_socket.sendto(outgoing, target)
                                        last_target_send[target_name] = now
                                    except OSError as exc:
                                        print(
                                            f"UYARI: {target_name} UDP gonderilemedi: {exc}",
                                            file=sys.stderr,
                                        )
                            output_sequence += 1
                            fused_packets += 1
                            output_events.append(now)
                            last_fused_serials = sorted(
                                int(value)
                                for value in (packet.get("fusion") or {}).get(
                                    "contributing_serials", []
                                )
                            )
                            last_arrival_spread_ms = float(
                                (packet.get("fusion") or {}).get(
                                    "capture_spread_ms", arrival_spread_ms
                                )
                            )
                            last_output_at = now
                            last_output_bundle = output_marker
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
                    "received_fps": event_rate(body_events[endpoint.serial], now),
                    "received_latency_ms": metric.get("corrected_capture_to_receive_ms"),
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
                    f"{item.serial}:{last_source_status.get(item.serial, 'YOK')}"
                    f"/body={source_metrics[item.serial].get('body_fps', 0.0):.1f}fps"
                    f"/rx={source_metrics[item.serial].get('rx_fps', 0.0):.1f}fps"
                    f"/lat={metric_text(source_metrics[item.serial].get('corrected_capture_to_receive_ms'), 'ms')}"
                    f"/net={metric_text(source_metrics[item.serial].get('network_queue_ms'), 'ms')}"
                    f"/clk={metric_text(source_metrics[item.serial].get('clock_offset_estimate_ms'), 'ms')}"
                    f"/secim={source_metrics[item.serial].get('operator_state', 'YOK')}"
                    f"/id={source_metrics[item.serial].get('locked_body_id', '-')}"
                    f"/kisi={source_metrics[item.serial].get('detected_body_count', 0)}"
                    f"/preview={metric_text(source_metrics[item.serial].get('preview_age_ms'), 'ms', 0)}"
                    f"/X={metric_text(source_metrics[item.serial].get('world_forward_x_m'), 'm', 2)}"
                    f"/{source_metrics[item.serial].get('workspace_state', 'BEKLE')}"
                    for item in endpoints
                )
                print(
                    f"DURUM | bagli={len(online_serials)}/4 [{', '.join(map(str, online_serials)) or 'yok'}] "
                    f"| body_taze={len(body_serials)}/4 [{', '.join(map(str, body_serials)) or 'yok'}] | input={input_packets} "
                    f"| ham_kayit={raw_records} | fusion_cikis={fused_packets} "
                    f"| son_katki=[{', '.join(map(str, last_fused_serials)) or 'yok'}] "
                    f"| fusion_fps={event_rate(output_events, now):.1f} "
                    f"| gmr_kapi={last_gmr_gate} "
                    f"| yayilim_ms={last_arrival_spread_ms:.1f} "
                    f"| dortlu_bekleme_ms={last_full_set_wait_applied_ms:.1f} "
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

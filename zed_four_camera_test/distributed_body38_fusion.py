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
import selectors
import socket
import sys
import time
from typing import Any

import numpy as np

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
        # Prefer more cameras, then tighter synchronization, then newer data.
        score = (len(selected), -spread_ns, max(item.received_ns for item in selected))
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
    result = np.full((38, 3), np.nan, dtype=np.float64)
    result_confidence = np.zeros(38, dtype=np.float64)
    contribution_count: list[int] = []
    prepared = []
    for sample, extrinsic in views:
        packet_conf = confidence(sample.packet.get("keypoint_confidence"))
        prepared.append((transformed_points(sample.packet, extrinsic), packet_conf, sample_quality(sample.packet, minimum_confidence)))
    for joint in range(38):
        candidates: list[tuple[np.ndarray, float, float]] = []
        for xyz, conf, quality in prepared:
            if np.isfinite(xyz[joint]).all() and conf[joint] >= minimum_confidence:
                candidates.append((xyz[joint], float(conf[joint]), quality))
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


def make_output_packet(
    views: list[tuple[Sample, Extrinsic]],
    *,
    minimum_confidence: float,
    maximum_spread_m: float,
    output_sequence: int,
    reference_serial: int,
    arrival_spread_ms: float,
) -> dict[str, Any]:
    best_sample, best_extrinsic = max(
        views,
        key=lambda item: sample_quality(item[0].packet, minimum_confidence),
    )
    packet = dict(best_sample.packet)
    fused, fused_confidence, contributions = fuse_keypoints(
        views,
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
            "contributing_serials": [sample.serial for sample, _ in views],
            "best_orientation_serial": best_sample.serial,
            "per_joint_contributions": contributions,
            "maximum_joint_spread_m": maximum_spread_m,
            "arrival_spread_ms": arrival_spread_ms,
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
    parser.add_argument("--bind", default="0.0.0.0", help="Dinlenecek PC IPv4 adresi (varsayilan tum arayuzler).")
    parser.add_argument("--extrinsics", type=Path, default=None, help="Kalibre edilmis zed_body38_distributed_extrinsics/v1 JSON dosyasi.")
    parser.add_argument("--output-host", default="", help="Birlesik BODY_38 UDP hedefi; bos ise cikis yayini kapali.")
    parser.add_argument("--output-port", type=int, default=15050)
    parser.add_argument("--output-max-hz", type=float, default=15.0)
    parser.add_argument("--confidence", type=float, default=45.0)
    parser.add_argument("--max-sync-ms", type=float, default=110.0, help="Ana PC'ye varis zamanina gore azami dortlu paket yayilimi.")
    parser.add_argument("--source-timeout-ms", type=float, default=750.0, help="Bu sureden eski BODY_38 kaynagini taze sayma.")
    parser.add_argument("--minimum-sources", type=int, choices=(2, 3, 4), default=2, help="Calisma aninda cikis icin gereken en az taze kamera.")
    parser.add_argument("--max-joint-spread-m", type=float, default=0.30)
    parser.add_argument("--calibration-record", type=Path, default=None, help="Dortlu senkron ham BODY_38 karelerini JSONL olarak kaydet.")
    parser.add_argument("--calibration-max-hz", type=float, default=12.0)
    parser.add_argument("--duration", type=float, default=0.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    endpoints: list[InputEndpoint] = args.source
    if len(endpoints) != 4 or len({item.serial for item in endpoints}) != 4 or len({item.port for item in endpoints}) != 4:
        print("HATA: tam dort farkli SERIAL:PORT kaynagi gerekli.", file=sys.stderr)
        return 2
    if not 0.0 <= args.confidence <= 100.0 or args.max_sync_ms <= 0.0 or args.source_timeout_ms <= 0.0 or args.max_joint_spread_m <= 0.0:
        print("HATA: confidence, max-sync-ms, source-timeout-ms veya max-joint-spread-m gecersiz.", file=sys.stderr)
        return 2
    try:
        extrinsics = load_extrinsics(args.extrinsics.resolve() if args.extrinsics else None, endpoints)
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
            selector.register(receiver, selectors.EVENT_READ, data=endpoint)
            sockets.append(receiver)
            print(f"DINLE | ZED {endpoint.serial} | {args.bind}:{endpoint.port}")
    except OSError as exc:
        print(f"HATA: UDP portu acilamadi: {exc}", file=sys.stderr)
        for item in sockets:
            item.close()
        return 3

    output_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.output_host else None
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
    started = time.monotonic()
    last_status = started
    try:
        while True:
            events = selector.select(timeout=0.20)
            for key, _ in events:
                endpoint: InputEndpoint = key.data
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
                        packet = make_output_packet(
                            views,
                            minimum_confidence=args.confidence,
                            maximum_spread_m=args.max_joint_spread_m,
                            output_sequence=output_sequence,
                            reference_serial=extrinsics.reference_serial,
                            arrival_spread_ms=arrival_spread_ms,
                        )
                        encoded = json.dumps(packet, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
                        if len(encoded) <= 60000:
                            output_sequence += 1
                            fused_packets += 1
                            last_fused_serials = sorted(sample.serial for sample in samples)
                            last_arrival_spread_ms = arrival_spread_ms
                            last_output_at = now
                            if output_socket is not None:
                                output_socket.sendto(encoded, (args.output_host, args.output_port))
                        else:
                            print(f"UYARI: birlesik UDP paketi cok buyuk ({len(encoded)} byte).", file=sys.stderr)

            now = time.monotonic()
            if now - last_status >= 1.0:
                status_now_ns = time.time_ns()
                timeout_ns = int(args.source_timeout_ms * 1.0e6)
                online_serials = sorted(serial for serial, stamp in last_any_packet_ns.items() if status_now_ns - stamp <= timeout_ns)
                body_serials = sorted(serial for serial, stamp in last_body_packet_ns.items() if status_now_ns - stamp <= timeout_ns)
                details = ", ".join(
                    f"{item.serial}:{last_source_status.get(item.serial, 'YOK')}/b{per_source_input[item.serial]}/s{per_source_status[item.serial]}"
                    for item in endpoints
                )
                print(
                    f"DURUM | bagli={len(online_serials)}/4 [{', '.join(map(str, online_serials)) or 'yok'}] "
                    f"| body_taze={len(body_serials)}/4 [{', '.join(map(str, body_serials)) or 'yok'}] | input={input_packets} "
                    f"| ham_kayit={raw_records} | fusion_cikis={fused_packets} "
                    f"| son_katki=[{', '.join(map(str, last_fused_serials)) or 'yok'}] "
                    f"| yayilim_ms={last_arrival_spread_ms:.1f} "
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
        for item in sockets:
            try:
                selector.unregister(item)
            except Exception:
                pass
            item.close()
        selector.close()
    print(f"Bitti | input={input_packets} | ham_kayit={raw_records} | fusion_cikis={fused_packets}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

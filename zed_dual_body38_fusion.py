#!/usr/bin/env python3
"""Dual ZED 2i BODY_38 Fusion source for the existing G1 upper-body path.

This is deliberately separate from ``zed_g1_skeleton.py``.  It follows the
official Stereolabs local multi-camera publish/subscribe workflow and emits the
same ``zed_body38_live/v1`` UDP contract consumed by the existing GMR/Isaac
bridge.  It never sends commands directly to a physical robot.
"""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import queue
import socket
import threading
import time
from types import SimpleNamespace
from typing import Any

try:
    import msvcrt
except ImportError:
    msvcrt = None

from zed_g1_skeleton import (
    BODY38_EDGES,
    BODY38_NAMES,
    DEX3_JOINT_ORDER,
    DEX3_LIMITS_RAD,
    G1_23DOF_JOINT_ORDER,
    IDX,
    ConfidenceAwareLowPass,
    build_record,
    camera_metadata,
    cv2,
    draw_skeleton,
    finite_vector,
    quaternion_xyzw_to_matrix,
    sanitize_for_json,
    sl,
    np,
)
from motion_pipeline.arm_chain import ArmChainOptimizer, ArmChainResult
from motion_pipeline.calibration import (
    CalibrationManager,
    pelvis_frame,
    stabilize_pelvis_rotation,
)
from motion_pipeline.human_state import (
    CORE_NAMES,
    ConfidenceAwareHumanStateEstimator,
    HumanStateResult,
    load_human_state_config,
    quaternion_continuous,
)
from motion_pipeline.metrics import PerceptionMetrics
from motion_pipeline.multiview import (
    MultiViewDecision,
    ViewQuality,
    arm_evidence,
    body_quality,
    choose_output,
    closest_timestamp_samples,
    keypoint_agreement,
    torso_overlap_2d,
)
from motion_pipeline.operator_selector import OperatorSelector


class BodyAdapter(SimpleNamespace):
    """Stable Python snapshot of a temporary PyZED BodyData wrapper."""


def _array(body: Any, name: str, shape: tuple[int, ...], fill: float = math.nan) -> np.ndarray:
    try:
        value = np.asarray(getattr(body, name), dtype=np.float64)
        if value.shape == shape:
            return value.copy()
    except Exception:
        pass
    return np.full(shape, fill, dtype=np.float64)


def adapt_body(
    body: Any,
    *,
    canonical_id: int | None = None,
    keypoint_2d: np.ndarray | None = None,
    source: str,
    source_serial: int | None,
) -> BodyAdapter:
    points = _array(body, "keypoint", (38, 3))
    pixels = (
        np.asarray(keypoint_2d, dtype=np.float64).copy()
        if keypoint_2d is not None and np.asarray(keypoint_2d).shape == (38, 2)
        else _array(body, "keypoint_2d", (38, 2), -1.0)
    )
    return BodyAdapter(
        id=int(body.id if canonical_id is None else canonical_id),
        original_id=int(body.id),
        unique_object_id=str(getattr(body, "unique_object_id", "")),
        tracking_state=getattr(body, "tracking_state", sl.OBJECT_TRACKING_STATE.OK),
        action_state=getattr(body, "action_state", "UNKNOWN"),
        confidence=float(getattr(body, "confidence", 0.0)),
        position=_array(body, "position", (3,)),
        keypoint=points,
        keypoint_2d=pixels,
        keypoint_confidence=_array(body, "keypoint_confidence", (38,), 0.0),
        global_root_orientation=_array(body, "global_root_orientation", (4,), 0.0),
        local_position_per_joint=_array(body, "local_position_per_joint", (38, 3)),
        local_orientation_per_joint=_array(body, "local_orientation_per_joint", (38, 4)),
        source=source,
        source_serial=source_serial,
    )


def _matrix_to_quaternion_xyzw(matrix: np.ndarray) -> np.ndarray:
    """Convert a proper rotation matrix to a normalized XYZW quaternion."""
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
            scale = math.sqrt(1.0 + value[0, 0] - value[1, 1] - value[2, 2]) * 2.0
            quat = np.array([
                0.25 * scale,
                (value[0, 1] + value[1, 0]) / scale,
                (value[0, 2] + value[2, 0]) / scale,
                (value[2, 1] - value[1, 2]) / scale,
            ])
        elif index == 1:
            scale = math.sqrt(1.0 + value[1, 1] - value[0, 0] - value[2, 2]) * 2.0
            quat = np.array([
                (value[0, 1] + value[1, 0]) / scale,
                0.25 * scale,
                (value[1, 2] + value[2, 1]) / scale,
                (value[0, 2] - value[2, 0]) / scale,
            ])
        else:
            scale = math.sqrt(1.0 + value[2, 2] - value[0, 0] - value[1, 1]) * 2.0
            quat = np.array([
                (value[0, 2] + value[2, 0]) / scale,
                (value[1, 2] + value[2, 1]) / scale,
                0.25 * scale,
                (value[1, 0] - value[0, 1]) / scale,
            ])
    return quat / max(float(np.linalg.norm(quat)), 1.0e-12)


def transform_body_to_fusion_frame(
    body: Any,
    *,
    rotation: np.ndarray,
    translation: np.ndarray,
    source_serial: int,
) -> BodyAdapter:
    """Transform a sender-local BODY_38 snapshot with its ZED360 pose.

    Fusion's per-camera debug retrieval is not a synchronized data source and
    may return only one camera even while the official fused body uses both.
    Local sender detections are therefore transformed by the exact calibration
    pose passed to ``Fusion.subscribe`` for diagnostics, association and the
    emergency single-camera fallback. The official fused body remains the
    primary control source.
    """
    result = adapt_body(
        body, source="calibrated_camera", source_serial=source_serial
    )
    valid = np.isfinite(result.keypoint).all(axis=1)
    result.keypoint[valid] = (
        result.keypoint[valid] @ rotation.T + translation
    )
    if np.isfinite(result.position).all():
        result.position = rotation @ result.position + translation
    root = quaternion_xyzw_to_matrix(result.global_root_orientation)
    if np.isfinite(root).all():
        result.global_root_orientation = _matrix_to_quaternion_xyzw(
            rotation @ root
        )
    return result


def _anchor(body: Any) -> np.ndarray:
    position = finite_vector(getattr(body, "position", None), 3)
    if position is not None:
        return position
    points = np.asarray(getattr(body, "keypoint", []), dtype=np.float64)
    if points.shape == (38, 3) and np.isfinite(points[IDX["PELVIS"]]).all():
        return points[IDX["PELVIS"]]
    return np.full(3, np.nan)


def _valid_body(body: Any, threshold: float) -> bool:
    try:
        return (
            body.tracking_state == sl.OBJECT_TRACKING_STATE.OK
            and float(body.confidence) >= threshold
            and np.isfinite(np.asarray(body.keypoint, dtype=np.float64)[IDX["PELVIS"]]).all()
        )
    except Exception:
        return False


def _quality(body: Any, serial: int, threshold: float) -> ViewQuality:
    return body_quality(
        serial_number=serial,
        body_id=int(body.id),
        points=np.asarray(body.keypoint, dtype=np.float64),
        confidence=np.asarray(body.keypoint_confidence, dtype=np.float64),
        body_confidence=float(body.confidence),
        index=IDX,
        threshold=threshold,
    )


def _body_shape(body: Any) -> dict[str, Any]:
    points = np.asarray(getattr(body, "keypoint", []), dtype=np.float64)
    if points.shape != (38, 3):
        return {}
    def distance(first: str, second: str) -> float | None:
        value = points[[IDX[first], IDX[second]]]
        return float(np.linalg.norm(value[1] - value[0])) if np.isfinite(value).all() else None
    pelvis = points[IDX["PELVIS"]]
    neck = points[IDX["NECK"]]
    ankles = points[[IDX["LEFT_ANKLE"], IDX["RIGHT_ANKLE"]]]
    height = None
    if np.isfinite(neck).all() and np.isfinite(ankles).all():
        height = float(np.linalg.norm(neck - np.mean(ankles, axis=0)))
    torso = neck - pelvis if np.isfinite(neck).all() and np.isfinite(pelvis).all() else None
    if torso is not None:
        torso /= max(float(np.linalg.norm(torso)), 1.0e-9)
    return {
        "height": height,
        "shoulder_width": distance("LEFT_SHOULDER", "RIGHT_SHOULDER"),
        "pelvis_width": distance("LEFT_HIP", "RIGHT_HIP"),
        "torso": torso,
    }


def _associated_body(bodies: list[Any], reference: Any, maximum_m: float = 0.80) -> Any | None:
    """Associate camera-local IDs using world pose and body morphology."""
    anchor = _anchor(reference)
    reference_shape = _body_shape(reference)
    valid = []
    for body in bodies:
        value = _anchor(body)
        if np.isfinite(value).all() and np.isfinite(anchor).all():
            pelvis_distance = float(np.linalg.norm(value - anchor))
            shape = _body_shape(body)
            cost = pelvis_distance
            for name, weight, scale in (
                ("height", 0.18, 0.30),
                ("shoulder_width", 0.14, 0.12),
                ("pelvis_width", 0.08, 0.10),
            ):
                first, second = reference_shape.get(name), shape.get(name)
                if first is not None and second is not None:
                    cost += weight * min(abs(first - second) / scale, 2.0)
            first_torso, second_torso = reference_shape.get("torso"), shape.get("torso")
            if first_torso is not None and second_torso is not None:
                cost += 0.12 * (1.0 - float(np.clip(np.dot(first_torso, second_torso), -1.0, 1.0)))
            valid.append((cost, pelvis_distance, body))
    if not valid:
        return None
    _cost, distance, body = min(valid, key=lambda item: item[0])
    return body if distance <= maximum_m else None


def _view_description(
    serial: int,
    body: Any,
    threshold: float,
    *,
    capture_timestamp_ns: int | None = None,
    receive_timestamp_ns: int | None = None,
) -> dict[str, object]:
    confidence = np.asarray(body.keypoint_confidence, dtype=np.float64)
    pixels = np.asarray(body.keypoint_2d, dtype=np.float64)
    points = np.asarray(body.keypoint, dtype=np.float64)
    arm_confidence = {}
    arm_overlap = {}
    for side in ("left", "right"):
        upper = side.upper()
        arm_confidence[side] = [
            float(confidence[IDX[f"{upper}_SHOULDER"]]),
            float(confidence[IDX[f"{upper}_ELBOW"]]),
            float(confidence[IDX[f"{upper}_WRIST"]]),
        ]
        arm_overlap[side] = torso_overlap_2d(pixels, IDX, upper)
    return {
        "serial_number": int(serial),
        "body_id": int(body.id),
        "capture_timestamp_ns": capture_timestamp_ns,
        "receive_timestamp_ns": receive_timestamp_ns,
        "quality": sanitize_for_json(_quality(body, serial, threshold).__dict__),
        # Preserve each calibrated view for offline association/extrinsic
        # auditing. This is intentionally the fusion/world frame, not the
        # sender-local BASELINK frame that caused the old false disagreement.
        "keypoints_3d_fusion_m": sanitize_for_json(points.tolist()),
        "keypoint_confidence": sanitize_for_json(confidence.tolist()),
        "arm_confidence": arm_confidence,
        "arm_overlap": arm_overlap,
    }


def _fusion_metrics_snapshot(fusion: Any) -> dict[str, object] | None:
    """Return JSON-safe official Fusion timing/participation metrics.

    ZED SDK minor releases do not all expose exactly the same CameraMetrics
    fields, so this deliberately reads them defensively. A metrics API failure
    must never interrupt the live control source.
    """
    try:
        status, metrics = fusion.get_process_metrics()
        if status != sl.FUSION_ERROR_CODE.SUCCESS:
            return None
        per_camera: dict[str, object] = {}
        for identifier, camera in metrics.camera_individual_stats.items():
            serial = int(getattr(identifier, "serial_number", 0))
            per_camera[str(serial)] = {
                "is_present": bool(getattr(camera, "is_present", False)),
                "received_fps": float(getattr(camera, "received_fps", math.nan)),
                "received_latency_ms": 1000.0 * float(
                    getattr(camera, "received_latency", math.nan)
                ),
                "synced_latency_ms": 1000.0 * float(
                    getattr(camera, "synced_latency", math.nan)
                ),
                "delta_timestamp_ms": 1000.0 * float(
                    getattr(camera, "delta_ts", math.nan)
                ),
                "body_detection_ratio": float(
                    getattr(camera, "ratio_detection", math.nan)
                ),
            }
        return sanitize_for_json({
            "mean_camera_fused": float(
                getattr(metrics, "mean_camera_fused", math.nan)
            ),
            "mean_stdev_between_camera_s": float(
                getattr(metrics, "mean_stdev_between_camera", math.nan)
            ),
            "per_camera": per_camera,
        })
    except Exception:
        return None


def _camera_frame(zed: Any, image: Any) -> np.ndarray | None:
    """Retrieve an independent copy of a sender's current left image."""
    try:
        if zed.retrieve_image(image, sl.VIEW.LEFT) != sl.ERROR_CODE.SUCCESS:
            return None
        frame = np.asarray(image.get_data())
        if frame.ndim != 3 or frame.shape[0] == 0 or frame.shape[1] == 0:
            return None
        frame = frame.copy()
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame
    except Exception:
        return None


def _draw_dual_preview(
    frames: dict[int, np.ndarray],
    bodies_by_serial: dict[int, list[Any]],
    selected_raw_ids: dict[int, int],
    confidence_threshold: float,
    *,
    fusion_mode: str,
    operator_state: str,
    contributing_views: int,
    recording: bool,
    recorded: int,
    rig_state: str = "UNKNOWN",
    target_height: int = 540,
) -> np.ndarray | None:
    """Build one side-by-side diagnostic window without affecting Fusion."""
    panels: list[np.ndarray] = []
    for serial in sorted(frames):
        frame = frames[serial].copy()
        selected_id = selected_raw_ids.get(serial)
        for body in bodies_by_serial.get(serial, []):
            draw_skeleton(
                frame,
                body,
                selected_id is not None and int(body.id) == int(selected_id),
                confidence_threshold,
            )
        scale = target_height / max(1, frame.shape[0])
        width = max(1, int(round(frame.shape[1] * scale)))
        panel = cv2.resize(frame, (width, target_height), interpolation=cv2.INTER_AREA)
        cv2.rectangle(panel, (0, 0), (panel.shape[1], 42), (18, 18, 18), -1)
        cv2.putText(
            panel, f"ZED 2i S/N {serial} | BODY_38", (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA,
        )
        panels.append(panel)
    if not panels:
        return None
    height = max(panel.shape[0] for panel in panels)
    panels = [
        cv2.copyMakeBorder(panel, 0, height - panel.shape[0], 0, 0, cv2.BORDER_CONSTANT)
        for panel in panels
    ]
    canvas = cv2.hconcat(panels)
    header = np.full((74, canvas.shape[1], 3), 22, dtype=np.uint8)
    state_color = (0, 220, 0) if operator_state in ("LOCKED", "CALIBRATING") else (0, 180, 255)
    cv2.putText(
        header,
        f"DUAL ZED FUSION | source={fusion_mode} | operator={operator_state} "
        f"| evidence_views={contributing_views} "
        f"| REC={'ON' if recording else 'OFF'} {recorded}",
        (12, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.66, state_color, 2, cv2.LINE_AA,
    )
    cv2.putText(
        header,
        f"rig={rig_state} | S: JSONL kayit  R: operator/kalibrasyon reset  Q/ESC: cikis",
        (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (220, 220, 220), 1, cv2.LINE_AA,
    )
    return cv2.vconcat([header, canvas])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="İki ZED 2i ile ortak BODY_38 -> mevcut G1 GMR/Isaac UDP kaynağı")
    parser.add_argument("--fusion-config", type=Path, required=True)
    parser.add_argument(
        "--teleop-config", type=Path,
        default=Path(__file__).parent / "config" / "dual_teleoperation.json",
        help="Eklem-bazli fusion, anatomi, retargeting ve safety ayarlari.",
    )
    parser.add_argument("--fps", type=int, choices=(15, 30, 60), default=30)
    parser.add_argument("--model", choices=("fast", "medium", "accurate"), default="medium")
    parser.add_argument(
        "--depth-mode",
        choices=("ultra", "neural-light", "neural", "performance"),
        default="ultra",
    )
    parser.add_argument("--confidence", type=float, default=40.0)
    parser.add_argument("--fusion-smoothing", type=float, default=0.10)
    parser.add_argument("--filter-tau", type=float, default=0.0)
    parser.add_argument("--operator-acquire-frames", type=int, default=10)
    parser.add_argument(
        "--fallback-serial", type=int, default=33773329,
        help="Kalibrasyon geçersizken korunacak tek ve sabit kamera.",
    )
    parser.add_argument(
        "--max-cross-view-mpjpe", type=float, default=0.12,
        help="Fusion kabulü için iki kamera arasındaki azami ortalama eklem hatası (m).",
    )
    parser.add_argument(
        "--max-camera-sync-ms", type=float, default=50.0,
        help="Tanısal iki-kamera eşleştirmesinde izin verilen azami zaman farkı.",
    )
    parser.add_argument("--calibration-seconds", type=float, default=4.0)
    parser.add_argument("--camera-timeout", type=float, default=5.0)
    parser.add_argument("--stream-host", default="")
    parser.add_argument("--stream-port", type=int, default=15050)
    parser.add_argument("--stream-max-hz", type=float, default=30.0)
    parser.add_argument("--monitor-host", default="")
    parser.add_argument("--monitor-port", type=int, default=15052)
    parser.add_argument("--monitor-max-hz", type=float, default=30.0)
    parser.add_argument("--ros-host", default="")
    parser.add_argument("--ros-port", type=int, default=15054)
    parser.add_argument("--ros-max-hz", type=float, default=30.0)
    parser.add_argument("--record", action="store_true")
    parser.add_argument(
        "--headless", action="store_true",
        help="Kamera/iskelet onizleme penceresini kapat.",
    )
    parser.add_argument(
        "--preview-hz", type=float, default=15.0,
        help="Onizleme goruntu kopyalama/cizim hizi; kontrol akisini sinirlamaz.",
    )
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).parent / "recordings")
    parser.add_argument("--duration", type=float, default=0.0)
    return parser.parse_args()


def _enum_success(value: Any, success: Any) -> bool:
    return value == success


def main() -> int:
    args = parse_args()
    config_path = args.fusion_config.expanduser().resolve()
    if not config_path.is_file():
        print(f"Fusion kalibrasyon dosyası bulunamadı: {config_path}")
        return 2
    teleop_config_path = args.teleop_config.expanduser().resolve()
    try:
        teleop_config = load_human_state_config(teleop_config_path)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"Dual teleoperation config okunamadi: {teleop_config_path}: {exc}")
        return 2
    fusion_configs = sl.read_fusion_configuration_file(
        str(config_path), sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD, sl.UNIT.METER
    )
    if len(fusion_configs) < 2:
        print("Kalibrasyon dosyasında en az iki geçerli ZED bulunmalı.")
        return 2
    calibration_pose = {
        int(conf.serial_number): (
            np.asarray(conf.pose.get_rotation_matrix().r, dtype=np.float64).copy(),
            np.asarray(conf.pose.get_translation().get(), dtype=np.float64).copy(),
        )
        for conf in fusion_configs
    }

    model_map = {
        "fast": sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST,
        "medium": sl.BODY_TRACKING_MODEL.HUMAN_BODY_MEDIUM,
        "accurate": sl.BODY_TRACKING_MODEL.HUMAN_BODY_ACCURATE,
    }
    depth_map = {
        "ultra": sl.DEPTH_MODE.ULTRA,
        "neural-light": sl.DEPTH_MODE.NEURAL_LIGHT,
        "neural": sl.DEPTH_MODE.NEURAL,
        "performance": sl.DEPTH_MODE.PERFORMANCE,
    }
    shared = sl.CommunicationParameters()
    shared.set_for_shared_memory()
    senders: dict[int, Any] = {}
    sender_bodies: dict[int, Any] = {}
    sender_images: dict[int, Any] = {}
    latest_frames: dict[int, np.ndarray] = {}
    last_sender_ok: dict[int, float] = {}
    metadata: dict[str, object] = {}

    for conf in fusion_configs:
        serial = int(conf.serial_number)
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.HD720
        init.camera_fps = int(args.fps)
        init.depth_mode = depth_map[args.depth_mode]
        init.coordinate_units = sl.UNIT.METER
        init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init.depth_maximum_distance = 8.0
        init.input = conf.input_type
        init.set_from_serial_number(serial)
        if hasattr(init, "async_grab_camera_recovery"):
            init.async_grab_camera_recovery = True
        zed = sl.Camera()
        status = zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(f"UYARI: ZED {serial} açılamadı: {status}")
            continue
        positional = sl.PositionalTrackingParameters()
        positional.set_as_static = True
        status = zed.enable_positional_tracking(positional)
        if status != sl.ERROR_CODE.SUCCESS:
            print(f"UYARI: ZED {serial} positional tracking açılamadı: {status}")
            zed.close()
            continue
        parameters = sl.BodyTrackingParameters()
        parameters.detection_model = model_map[args.model]
        parameters.body_format = sl.BODY_FORMAT.BODY_38
        parameters.body_selection = sl.BODY_KEYPOINTS_SELECTION.FULL
        # Follow Stereolabs' local Fusion sample: identity tracking belongs to
        # the Fusion subscriber. Sender-side tracking/fitting duplicates work,
        # reduces throughput and can hand Fusion already-smoothed identities.
        parameters.enable_tracking = False
        parameters.enable_body_fitting = False
        parameters.enable_segmentation = False
        parameters.max_range = 8.0
        # MEDIUM is the quality path requested for teleoperation. Reduced
        # precision is retained only for the explicitly selected FAST mode.
        parameters.allow_reduced_precision_inference = args.model == "fast"
        parameters.prediction_timeout_s = 0.20
        status = zed.enable_body_tracking(parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            print(f"UYARI: ZED {serial} BODY_38 açılamadı: {status}")
            zed.close()
            continue
        status = zed.start_publishing(shared)
        if status != sl.ERROR_CODE.SUCCESS:
            print(f"UYARI: ZED {serial} Fusion yayını açılamadı: {status}")
            zed.close()
            continue
        senders[serial] = zed
        sender_bodies[serial] = sl.Bodies()
        sender_images[serial] = sl.Mat()
        last_sender_ok[serial] = time.monotonic()
        metadata[str(serial)] = camera_metadata(zed)
        print(f"ZED {serial}: BODY_38 yayıncı HAZIR")

    if not senders:
        print("Hiçbir ZED yayıncısı açılamadı.")
        return 3

    fusion_init = sl.InitFusionParameters()
    fusion_init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    fusion_init.coordinate_units = sl.UNIT.METER
    fusion_init.output_performance_metrics = True
    fusion_init.verbose = True
    fusion_init.timeout_period_number = 5
    # Use the SDK's synchronization defaults, as in the official local
    # multi-camera body-tracking sample. The old hand-tuned window rejected
    # otherwise usable pairs and left Fusion with one camera most of the time.
    fusion = sl.Fusion()
    status = fusion.init(fusion_init)
    if status != sl.FUSION_ERROR_CODE.SUCCESS:
        print(f"Fusion başlatılamadı: {status}")
        for zed in senders.values():
            zed.close()
        return 4

    identifiers: dict[int, Any] = {}
    configs_by_serial = {int(conf.serial_number): conf for conf in fusion_configs}
    for serial in senders:
        conf = configs_by_serial[serial]
        uuid = sl.CameraIdentifier()
        uuid.serial_number = serial
        status = fusion.subscribe(
            uuid, conf.communication_parameters, conf.pose, conf.override_gravity
        )
        if status == sl.FUSION_ERROR_CODE.SUCCESS:
            identifiers[serial] = uuid
            print(f"ZED {serial}: Fusion abonesi HAZIR")
        else:
            print(f"UYARI: ZED {serial} Fusion abonesi açılamadı: {status}")
    if not identifiers:
        print("Fusion hiçbir kameraya abone olamadı.")
        fusion.close()
        for zed in senders.values():
            zed.close()
        return 5

    fusion_parameters = sl.BodyTrackingFusionParameters()
    fusion_parameters.enable_tracking = True
    fusion_parameters.enable_body_fitting = False
    status = fusion.enable_body_tracking(fusion_parameters)
    if status != sl.FUSION_ERROR_CODE.SUCCESS:
        print(f"Fusion BODY_38 başlatılamadı: {status}")
        return 6
    fusion_runtime = sl.BodyTrackingFusionRuntimeParameters()
    # Match Stereolabs' official local multi-camera example. Fusion may keep a
    # tracked identity through a momentary one-view occlusion; our explicit
    # cross-view validation below still prevents a bad calibration from
    # entering the control stream.
    # Keep the SDK's global body identity alive through a one-camera delivery
    # gap.  The official sample does not require two cameras at this stage;
    # our timestamped per-view layer below is what decides whether a frame may
    # be labelled/fused as dual. Requiring two here made harmless asynchronous
    # USB deliveries appear as ``fusion_wait`` and repeatedly destroyed body
    # association even though both camera histories were current.
    fusion_runtime.skeleton_minimum_allowed_camera = 1
    fusion_runtime.skeleton_minimum_allowed_keypoints = 7
    fusion_runtime.skeleton_smoothing = float(np.clip(args.fusion_smoothing, 0.0, 1.0))
    fused_bodies = sl.Bodies()
    latest_local_lists: dict[int, list[BodyAdapter]] = {
        serial: [] for serial in identifiers
    }
    latest_raw_lists: dict[int, list[BodyAdapter]] = {
        serial: [] for serial in identifiers
    }
    latest_raw_time = {serial: -math.inf for serial in identifiers}
    latest_capture_ns: dict[int, int] = {serial: 0 for serial in identifiers}
    latest_receive_ns: dict[int, int] = {serial: 0 for serial in identifiers}
    raw_history: dict[int, deque[tuple[int, tuple[int, list[BodyAdapter], float]]]] = {
        serial: deque(maxlen=4) for serial in identifiers
    }
    # Fusion's transformed per-camera BodyData `is_new` flags can lag the
    # local grabs by several host loops.  At 30 FPS the former 83 ms window
    # was shorter than the observed SDK delivery offset (~78 ms), so normal
    # scheduling jitter collapsed almost every frame to one-view evidence.
    # Keep snapshots briefly enough for human motion, but long enough to span
    # two SDK body deliveries at the requested frame rate.
    raw_sync_window_s = max(2.5 / float(args.fps), 0.20)
    # Extrinsics describe a physically static rig. They are immutable for the
    # whole session and are passed exactly once to Fusion.subscribe above.
    # Never estimate/update camera pose from an articulated moving person.
    rig_metrics: dict[str, object] = {
        "state": "STATIC_CONFIG",
        "accepted_updates": 0,
        "source": "fusion_configuration_file",
    }
    bad_calibration_frames = 0
    good_calibration_frames = 0
    fusion_latched = False
    fusion_lock_frames = 0
    last_dual_good_monotonic = -math.inf

    low_pass = ConfidenceAwareLowPass(args.filter_tau)
    human_estimator = ConfidenceAwareHumanStateEstimator(
        BODY38_NAMES, BODY38_EDGES, teleop_config
    )
    selector = OperatorSelector(args.operator_acquire_frames)
    calibrator = CalibrationManager(args.calibration_seconds)
    arm_optimizer = ArmChainOptimizer(hold_s=0.35, recovery_mode="akc", branch_confirm_frames=4)
    perception = PerceptionMetrics()
    raw_identity: dict[int, int] = {}
    last_operator_anchor: np.ndarray | None = None
    previous_root_orientation: np.ndarray | None = None
    previous_local_orientations = np.full((38, 4), np.nan)
    previous_pelvis_rotation: np.ndarray | None = None
    last_preview_s = -math.inf

    targets: dict[tuple[str, int], float] = {}
    if args.stream_host:
        targets[(args.stream_host, args.stream_port)] = args.stream_max_hz
    if args.monitor_host:
        targets[(args.monitor_host, args.monitor_port)] = args.monitor_max_hz
    if args.ros_host:
        targets[(args.ros_host, args.ros_port)] = args.ros_max_hz
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if targets else None
    last_send = {target: 0.0 for target in targets}

    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = time.strftime("zed_dual_body38_%Y%m%d_%H%M%S")
    record_path = args.output_dir / f"{stem}.jsonl"
    record_file = None
    recording = False
    recorded = 0
    record_dropped = 0
    record_queue: queue.Queue[dict[str, object]] = queue.Queue(maxsize=300)
    record_writer_stop = threading.Event()
    record_writer: threading.Thread | None = None
    record_writer_errors: list[str] = []
    policy_powered_requested = False
    control_mode_revision = 0
    control_mode_session_ns = time.time_ns()

    def record_writer_loop() -> None:
        """Serialize/write diagnostics away from the real-time control path."""

        nonlocal recorded
        while not record_writer_stop.is_set() or not record_queue.empty():
            try:
                item = record_queue.get(timeout=0.10)
            except queue.Empty:
                continue
            try:
                if record_file is not None:
                    record_file.write(
                        json.dumps(
                            sanitize_for_json(item),
                            ensure_ascii=False,
                            allow_nan=False,
                        )
                        + "\n"
                    )
                    recorded += 1
            except Exception as exc:
                record_writer_errors.append(str(exc))
            finally:
                record_queue.task_done()

    def set_recording(enabled: bool) -> None:
        nonlocal record_file, recording, record_writer
        if enabled and record_file is None:
            # A large user-space buffer avoids a flush for every BODY_38 frame.
            # The writer thread keeps JSON serialization and OneDrive I/O out
            # of the camera -> GMR critical path.
            record_file = record_path.open(
                "w", encoding="utf-8", buffering=1024 * 1024
            )
            header = {
                "schema": "zed_dual_body38_g1_reference/metadata/v1",
                "created_unix_ns": time.time_ns(),
                "zed_sdk_version": sl.Camera.get_sdk_version(),
                "fusion_config": str(config_path),
                "teleoperation_config": str(teleop_config_path),
                "serial_numbers": sorted(senders),
                "camera": metadata,
                "body_format": "BODY_38",
                "keypoint_names": BODY38_NAMES,
                "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
                "units": "meter",
                "g1_23dof_joint_order": G1_23DOF_JOINT_ORDER,
                "dex3_1": {"joint_order_per_hand": DEX3_JOINT_ORDER, "joint_limits_rad": DEX3_LIMITS_RAD},
                "safety": "Perception only; contains no physical robot motor commands.",
            }
            record_file.write(json.dumps(sanitize_for_json(header), ensure_ascii=False) + "\n")
            record_writer = threading.Thread(
                target=record_writer_loop,
                name="zed-body38-jsonl-writer",
                daemon=True,
            )
            record_writer.start()
        recording = enabled
        print(
            f"Dual BODY_38 kayıt {'AÇIK' if enabled else 'KAPALI'}: "
            f"{record_path} ({recorded} kare, drop={record_dropped})"
        )

    def handle_key(key: str | int | None) -> bool:
        """Handle keys from either the PowerShell console or preview window."""
        nonlocal last_operator_anchor, previous_root_orientation, previous_local_orientations
        nonlocal previous_pelvis_rotation
        nonlocal fusion_latched, fusion_lock_frames, last_dual_good_monotonic
        nonlocal policy_powered_requested, control_mode_revision
        if key is None or key == -1:
            return False
        if isinstance(key, int):
            if key == 27:
                return True
            if key < 0 or key > 255:
                return False
            normalized = chr(key).lower()
        else:
            normalized = key.lower()
        if normalized in ("q", "\x1b"):
            return True
        if normalized == "s":
            set_recording(not recording)
        elif normalized == "p":
            policy_powered_requested = not policy_powered_requested
            control_mode_revision += 1
            print(
                "P: "
                + (
                    "POLICY POWERED istendi (BODY_38/GMR IK + residual)."
                    if policy_powered_requested
                    else "NORMAL IK istendi."
                ),
                flush=True,
            )
        elif normalized == "r":
            selector.reset(); calibrator.reset(); arm_optimizer.reset(); low_pass.reset(); human_estimator.reset()
            raw_identity.clear(); last_operator_anchor = None
            previous_root_orientation = None
            previous_local_orientations[:] = np.nan
            previous_pelvis_rotation = None
            fusion_latched = False
            fusion_lock_frames = 0
            last_dual_good_monotonic = -math.inf
            print("R: dual operator lock, calibration and arm memory reset.")
        return False

    def show_preview(
        raw_lists: dict[int, list[Any]],
        *,
        fusion_mode: str,
        operator_state: str,
        contributing_views: int,
    ) -> bool:
        nonlocal last_preview_s
        if args.headless:
            return False
        preview_period_s = 1.0 / max(1.0, float(args.preview_hz))
        preview_now = time.monotonic()
        if preview_now - last_preview_s < preview_period_s:
            return False
        last_preview_s = preview_now
        preview = _draw_dual_preview(
            latest_frames, raw_lists, raw_identity, args.confidence,
            fusion_mode=fusion_mode,
            operator_state=operator_state,
            contributing_views=contributing_views,
            recording=recording,
            recorded=recorded,
            rig_state=str(rig_metrics["state"]),
        )
        if preview is None:
            return False
        cv2.imshow("Dual ZED 2i BODY_38 Fusion - G1", preview)
        return handle_key(cv2.waitKey(1) & 0xFF)

    # zed.grab() blocks until that camera has a new image. Running two USB
    # cameras serially therefore turns small phase differences into a full
    # extra frame of latency. Keep the official INTRA_PROCESS Fusion workflow,
    # but let each independent Camera instance acquire/publish on its own
    # thread. The main thread remains the sole owner of Fusion and all
    # retargeting state.
    sender_stop = threading.Event()
    sender_wakeup = threading.Event()
    sender_state: dict[int, dict[str, Any]] = {
        serial: {
            "lock": threading.Lock(),
            "grab_generation": 0,
            "body_generation": 0,
            "frame_generation": 0,
            "last_ok_monotonic": time.monotonic(),
            "last_grab_status": None,
            "local_bodies": [],
            "raw_bodies": [],
            "capture_ns": 0,
            "receive_ns": 0,
            "arrival_monotonic": -math.inf,
            "frame": None,
        }
        for serial in senders
    }

    def sender_loop(serial: int, zed: Any) -> None:
        state = sender_state[serial]
        next_preview_s = 0.0
        preview_period_s = 1.0 / max(1.0, float(args.preview_hz))
        rotation, translation = calibration_pose[serial]
        bodies_value = sender_bodies[serial]
        image_value = sender_images[serial]
        runtime = sl.BodyTrackingRuntimeParameters()
        runtime.detection_confidence_threshold = float(args.confidence)
        runtime.minimum_keypoints_threshold = 8
        runtime.skeleton_smoothing = 0.05
        while not sender_stop.is_set():
            grab = zed.grab()
            arrival_monotonic = time.monotonic()
            if grab != sl.ERROR_CODE.SUCCESS:
                with state["lock"]:
                    state["last_grab_status"] = grab
                time.sleep(0.001)
                continue

            frame = None
            if not args.headless and arrival_monotonic >= next_preview_s:
                frame = _camera_frame(zed, image_value)
                next_preview_s = arrival_monotonic + preview_period_s

            retrieve = zed.retrieve_bodies(bodies_value, runtime)
            has_new_body = (
                retrieve == sl.ERROR_CODE.SUCCESS
                and getattr(bodies_value, "is_new", False)
            )
            local_bodies: list[BodyAdapter] | None = None
            raw_bodies: list[BodyAdapter] | None = None
            capture_ns = 0
            receive_ns = 0
            if has_new_body:
                local_bodies = [
                    adapt_body(
                        body,
                        source="sender_local",
                        source_serial=serial,
                    )
                    for body in bodies_value.body_list
                ]
                raw_bodies = [
                    transform_body_to_fusion_frame(
                        body,
                        rotation=rotation,
                        translation=translation,
                        source_serial=serial,
                    )
                    for body in bodies_value.body_list
                ]
                capture_ns = int(
                    zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
                )
                receive_ns = time.time_ns()

            with state["lock"]:
                state["grab_generation"] += 1
                state["last_ok_monotonic"] = arrival_monotonic
                state["last_grab_status"] = grab
                if frame is not None:
                    state["frame"] = frame
                    state["frame_generation"] += 1
                if has_new_body:
                    state["body_generation"] += 1
                    state["local_bodies"] = local_bodies
                    state["raw_bodies"] = raw_bodies
                    state["capture_ns"] = capture_ns
                    state["receive_ns"] = receive_ns
                    state["arrival_monotonic"] = arrival_monotonic
            sender_wakeup.set()

    sender_threads = [
        threading.Thread(
            target=sender_loop,
            args=(serial, zed),
            name=f"zed-acquire-{serial}",
            daemon=True,
        )
        for serial, zed in senders.items()
    ]
    consumed_grab_generation = {serial: 0 for serial in senders}
    consumed_body_generation = {serial: 0 for serial in senders}
    consumed_frame_generation = {serial: 0 for serial in senders}
    last_sender_warning = {serial: -math.inf for serial in senders}
    for thread in sender_threads:
        thread.start()

    if args.record:
        set_recording(True)
    print("Hazır. Q/ESC: çıkış | S: JSONL kayıt | P: Normal IK/Policy Powered | R: reset")
    print("Not: Tek kamera sistemi ayrı kalır; bu süreç yalnız dual Fusion kaynağıdır.")

    frame_index = 0
    output_sequence = 0
    started = time.monotonic()
    previous_timestamp_ns: int | None = None
    output_times: deque[float] = deque()
    fusion_metrics: dict[str, object] | None = None
    last_fusion_metrics_s = 0.0
    try:
        while True:
            now = time.monotonic()
            if args.duration > 0 and now - started >= args.duration:
                break
            if msvcrt is not None and msvcrt.kbhit():
                if handle_key(msvcrt.getwch()):
                    break

            # Wait briefly for a publisher instead of spinning Fusion or
            # replaying the same camera sample. Every generation is consumed
            # at most once; the raw history therefore stays timestamp-clean.
            sender_wakeup.wait(timeout=0.005)
            sender_wakeup.clear()
            active = 0
            new_grabs = 0
            active_window_s = max(3.0 / max(float(args.fps), 1.0), 0.15)
            for serial, state in sender_state.items():
                with state["lock"]:
                    snapshot = dict(state)
                last_ok = float(snapshot["last_ok_monotonic"])
                last_sender_ok[serial] = last_ok
                if now - last_ok <= active_window_s:
                    active += 1
                elif (
                    now - last_ok > 1.0
                    and now - last_sender_warning[serial] >= 1.0
                ):
                    print(
                        f"UYARI: ZED {serial} kare kesintisi; diğer kamera ile "
                        f"devam ediliyor ({snapshot['last_grab_status']})."
                    )
                    last_sender_warning[serial] = now

                grab_generation = int(snapshot["grab_generation"])
                if grab_generation > consumed_grab_generation[serial]:
                    new_grabs += grab_generation - consumed_grab_generation[serial]
                    consumed_grab_generation[serial] = grab_generation

                frame_generation = int(snapshot["frame_generation"])
                if frame_generation > consumed_frame_generation[serial]:
                    frame = snapshot["frame"]
                    if frame is not None:
                        latest_frames[serial] = frame
                    consumed_frame_generation[serial] = frame_generation

                body_generation = int(snapshot["body_generation"])
                if body_generation <= consumed_body_generation[serial]:
                    continue
                consumed_body_generation[serial] = body_generation
                latest_local_lists[serial] = snapshot["local_bodies"]
                latest_raw_lists[serial] = snapshot["raw_bodies"]
                latest_capture_ns[serial] = int(snapshot["capture_ns"])
                latest_receive_ns[serial] = int(snapshot["receive_ns"])
                latest_raw_time[serial] = float(snapshot["arrival_monotonic"])
                raw_history[serial].append((
                    latest_capture_ns[serial],
                    (
                        latest_receive_ns[serial],
                        latest_raw_lists[serial],
                        latest_raw_time[serial],
                    ),
                ))
            if active == 0 or new_grabs == 0:
                time.sleep(0.001)
                continue
            preview_lists = {
                serial: (
                    latest_raw_lists[serial]
                    if now - latest_raw_time[serial] <= raw_sync_window_s
                    else []
                )
                for serial in identifiers
            }
            if fusion.process() != sl.FUSION_ERROR_CODE.SUCCESS:
                if show_preview(
                    preview_lists,
                    fusion_mode=(
                        "joint_fusion" if fusion_latched else "fusion_wait"
                    ),
                    operator_state=(
                        "LOCKED" if selector.locked_id is not None else "WAITING"
                    ),
                    contributing_views=active,
                ):
                    break
                continue
            if now - last_fusion_metrics_s >= 1.0:
                snapshot = _fusion_metrics_snapshot(fusion)
                if snapshot is not None:
                    fusion_metrics = snapshot
                last_fusion_metrics_s = now
            fusion.retrieve_bodies(fused_bodies, fusion_runtime)

            fused_list = (
                list(fused_bodies.body_list)
                if fused_bodies.is_new else []
            )
            # Sender-local snapshots above are transformed by the same ZED360
            # poses used for Fusion subscription. They are diagnostic and
            # fallback inputs; the SDK's official fused body stays primary.
            synchronized = closest_timestamp_samples(
                raw_history,
                preferred_delta_ns=int(
                    float(teleop_config["fusion"]["preferred_sync_ms"]) * 1.0e6
                ),
                after_timestamp_ns=previous_timestamp_ns,
            )
            active_capture_ns: dict[int, int] = {}
            active_receive_ns: dict[int, int] = {}
            raw_lists: dict[int, list[BodyAdapter]] = {}
            for serial in identifiers:
                sample = synchronized.get(serial)
                if sample is None:
                    raw_lists[serial] = []
                    continue
                capture_ns, payload = sample
                receive_ns, bodies, arrival_monotonic = payload
                if now - arrival_monotonic > raw_sync_window_s:
                    raw_lists[serial] = []
                    continue
                raw_lists[serial] = bodies
                active_capture_ns[serial] = int(capture_ns)
                active_receive_ns[serial] = int(receive_ns)
            active_timestamps = {
                serial: active_capture_ns[serial]
                for serial, bodies in raw_lists.items()
                if bodies and active_capture_ns.get(serial, 0) > 0
            }
            camera_timestamp_delta_ms = (
                (max(active_timestamps.values()) - min(active_timestamps.values()))
                / 1.0e6
                if len(active_timestamps) >= 2
                else None
            )
            camera_sync_ok = (
                camera_timestamp_delta_ms <= args.max_camera_sync_ms
                if camera_timestamp_delta_ms is not None
                else None
            )
            # Diagnostics/validation must never compare "latest camera 1" to
            # a stale camera-2 pose. Fusion itself remains synchronized by the
            # SDK; this gate applies only to our association and safety layer.
            if len(active_timestamps) >= 2 and camera_sync_ok is False:
                newest = max(active_timestamps, key=active_timestamps.get)
                raw_lists = {
                    serial: bodies if serial == newest else []
                    for serial, bodies in raw_lists.items()
                }
            candidate_entries: list[tuple[BodyAdapter, ViewQuality, dict[int, Any]]] = []
            for fused in fused_list:
                if not _valid_body(fused, args.confidence):
                    continue
                matches = {
                    serial: match
                    for serial, bodies in raw_lists.items()
                    if (match := _associated_body(bodies, fused)) is not None
                }
                if not matches:
                    # An unassociated fused skeleton cannot be validated or
                    # tied to the locked operator. Wait for association and
                    # use the explicit best-single acquisition path below.
                    continue
                # Once a single-view acquisition has locked the operator,
                # remap an associated Fusion identity to that canonical ID.
                # This permits a seamless return to validated dual Fusion
                # without unlocking or handing over to a bystander.
                canonical_id = int(fused.id)
                if selector.locked_id is not None and any(
                    raw_identity.get(serial) == int(match.id)
                    for serial, match in matches.items()
                ):
                    canonical_id = int(selector.locked_id)
                best_match = max(
                    matches.items(),
                    key=lambda item: _quality(item[1], item[0], args.confidence).score,
                    default=(None, None),
                )
                pixels = (
                    np.asarray(best_match[1].keypoint_2d, dtype=np.float64)
                    if best_match[1] is not None else None
                )
                adapter = adapt_body(
                    fused, canonical_id=canonical_id, keypoint_2d=pixels,
                    source="fusion", source_serial=None,
                )
                candidate_entries.append((
                    adapter, _quality(adapter, 0, args.confidence), matches
                ))

            candidates = [entry[0] for entry in candidate_entries]
            # If the official fused body temporarily disappears, only mapped
            # per-camera IDs belonging to the already locked operator may act
            # as fallback. This prevents a silent handover to a bystander.
            fallback_entries: list[tuple[BodyAdapter, ViewQuality, dict[int, Any]]] = []
            if selector.locked_id is not None and not any(
                int(body.id) == selector.locked_id for body in candidates
            ):
                locked_matches: dict[int, Any] = {}
                for serial, raw_id in raw_identity.items():
                    match = next((b for b in raw_lists.get(serial, []) if int(b.id) == raw_id), None)
                    if match is None or not _valid_body(match, args.confidence):
                        continue
                    locked_matches[serial] = match
                    adapter = adapt_body(
                        match, canonical_id=selector.locked_id,
                        source="single_fallback", source_serial=serial,
                    )
                    # Fusion and per-camera trackers use different UUID
                    # namespaces. Preserve the canonical operator UUID so the
                    # fail-closed selector accepts this view as the same person
                    # rather than treating it as an automatic handover.
                    adapter.unique_object_id = selector.locked_unique_id or ""
                    quality = _quality(adapter, serial, args.confidence)
                    if last_operator_anchor is not None:
                        decision = choose_output(
                            fused=None, singles=[quality],
                            previous_anchor_m=last_operator_anchor,
                        )
                        if decision.mode == "hold":
                            continue
                    fallback_entries.append((adapter, quality, locked_matches))
                if fallback_entries:
                    best_fallback = max(fallback_entries, key=lambda entry: entry[1].score)
                    # Preserve every synchronized, associated observation.
                    # The old code attached only the winning camera here and
                    # therefore threw away the second view whenever the SDK's
                    # fused BodyData skipped one delivery.
                    best_fallback = (best_fallback[0], best_fallback[1], dict(locked_matches))
                    candidates.append(best_fallback[0])
                    candidate_entries.append(best_fallback)
            elif selector.locked_id is None and not candidates:
                # Fusion can need several frames to establish a global ID.
                # Acquire from exactly one best calibrated sender meanwhile;
                # never average unassociated people or camera IDs.
                acquisition_entries: list[
                    tuple[BodyAdapter, ViewQuality, dict[int, Any]]
                ] = []
                for serial, bodies in raw_lists.items():
                    for body in bodies:
                        if not _valid_body(body, args.confidence):
                            continue
                        adapter = adapt_body(
                            body, source="single_fallback",
                            source_serial=serial,
                        )
                        acquisition_entries.append((
                            adapter,
                            _quality(adapter, serial, args.confidence),
                            {serial: body},
                        ))
                if acquisition_entries:
                    preferred = [
                        item for item in acquisition_entries
                        if item[0].source_serial == args.fallback_serial
                    ]
                    best_acquisition = max(
                        preferred or acquisition_entries,
                        key=lambda item: item[1].score,
                    )
                    candidates.append(best_acquisition[0])
                    candidate_entries.append(best_acquisition)

            selection = selector.update(
                candidates,
                valid=lambda body: _valid_body(body, args.confidence),
                distance=lambda body: float(np.linalg.norm(_anchor(body))),
            )
            selected = selection.body
            entry = next((item for item in candidate_entries if item[0] is selected), None)
            if selected is None or entry is None:
                if (
                    fusion_latched
                    and selection.state.value == "LOST"
                    and (now - last_dual_good_monotonic) * 1000.0
                    > float(teleop_config["fusion"]["fusion_lock_release_ms"])
                ):
                    fusion_latched = False
                    fusion_lock_frames = 0
                if show_preview(
                    preview_lists,
                    fusion_mode=("joint_fusion" if fusion_latched else "waiting"),
                    operator_state=selection.state.value,
                    contributing_views=0,
                ):
                    break
                frame_index += 1
                continue
            selected_quality, matches = entry[1], entry[2]

            # Low-quality fusion may be worse than a clear calibrated view.
            single_qualities = [
                _quality(body, serial, args.confidence)
                for serial, body in matches.items()
            ]
            agreement = None
            if len(matches) >= 2:
                first, second = list(matches.values())[:2]
                agreement = keypoint_agreement(
                    np.asarray(first.keypoint), np.asarray(second.keypoint),
                    np.asarray(first.keypoint_confidence),
                    np.asarray(second.keypoint_confidence),
                    args.confidence,
                )
                first_points = np.asarray(first.keypoint, dtype=np.float64)
                second_points = np.asarray(second.keypoint, dtype=np.float64)
                first_conf = np.asarray(
                    first.keypoint_confidence, dtype=np.float64
                )
                second_conf = np.asarray(
                    second.keypoint_confidence, dtype=np.float64
                )
                valid = (
                    np.isfinite(first_points).all(axis=1)
                    & np.isfinite(second_points).all(axis=1)
                    & (first_conf >= args.confidence)
                    & (second_conf >= args.confidence)
                )
                errors = np.linalg.norm(first_points - second_points, axis=1)
                agreement["per_joint_error_m"] = {
                    name: (float(errors[index]) if valid[index] else None)
                    for index, name in enumerate(BODY38_NAMES)
                }
                agreement["pelvis_error_m"] = (
                    float(errors[IDX["PELVIS"]])
                    if valid[IDX["PELVIS"]] else None
                )
                agreement["left_wrist_error_m"] = (
                    float(errors[IDX["LEFT_WRIST"]])
                    if valid[IDX["LEFT_WRIST"]] else None
                )
                agreement["right_wrist_error_m"] = (
                    float(errors[IDX["RIGHT_WRIST"]])
                    if valid[IDX["RIGHT_WRIST"]] else None
                )
            agreement_m = (
                agreement.get("mpjpe_m") if agreement is not None else None
            )
            has_dual_evidence = len(matches) >= 2
            core_disagreement_m = None
            if has_dual_evidence:
                first, second = list(matches.values())[:2]
                first_points = np.asarray(first.keypoint, dtype=np.float64)
                second_points = np.asarray(second.keypoint, dtype=np.float64)
                first_conf = np.asarray(first.keypoint_confidence, dtype=np.float64)
                second_conf = np.asarray(second.keypoint_confidence, dtype=np.float64)
                core_ids = [IDX[name] for name in CORE_NAMES if name in IDX]
                core_valid = (
                    np.isfinite(first_points[core_ids]).all(axis=1)
                    & np.isfinite(second_points[core_ids]).all(axis=1)
                    & (first_conf[core_ids] >= args.confidence)
                    & (second_conf[core_ids] >= args.confidence)
                )
                if np.any(core_valid):
                    core_errors = np.linalg.norm(
                        first_points[core_ids] - second_points[core_ids], axis=1
                    )[core_valid]
                    core_disagreement_m = float(np.median(core_errors))
                    if agreement is not None:
                        agreement["core_median_error_m"] = core_disagreement_m
            calibration_agreement_ok = bool(
                has_dual_evidence
                and isinstance(core_disagreement_m, (int, float))
                and math.isfinite(float(core_disagreement_m))
                and float(core_disagreement_m)
                <= float(teleop_config["fusion"]["core_calibration_max_m"])
            )
            if calibration_agreement_ok:
                fusion_lock_frames += 1
                last_dual_good_monotonic = now
                if fusion_lock_frames >= int(
                    teleop_config["fusion"]["fusion_lock_acquire_frames"]
                ):
                    fusion_latched = True
            else:
                fusion_lock_frames = max(0, fusion_lock_frames - 1)
            if has_dual_evidence and isinstance(core_disagreement_m, (int, float)):
                rig_metrics["last_cross_view_mpjpe_m"] = float(agreement_m) if isinstance(agreement_m, (int, float)) else None
                rig_metrics["last_core_median_error_m"] = float(core_disagreement_m)
                if not calibration_agreement_ok:
                    bad_calibration_frames += 1
                    good_calibration_frames = 0
                else:
                    good_calibration_frames += 1
                    bad_calibration_frames = max(0, bad_calibration_frames - 1)
                if bad_calibration_frames >= 5:
                    rig_metrics["state"] = "CALIBRATION_BAD"
                elif good_calibration_frames >= 30:
                    rig_metrics["state"] = "STATIC_CONFIG"
            rig_metrics["bad_agreement_frames"] = bad_calibration_frames
            human_result: HumanStateResult | None = None
            # Once a static rig has acquired a valid dual lock, tolerate a
            # very short disagreement burst. The per-joint estimator still
            # outlier-gates every landmark, so this does not blindly average
            # incompatible points. A persistent five-frame disagreement is a
            # real calibration failure and falls back to one camera.
            transient_dual_ok = bool(
                has_dual_evidence
                and fusion_latched
                and bad_calibration_frames < 5
            )
            dual_estimator_allowed = bool(
                has_dual_evidence
                and (calibration_agreement_ok or transient_dual_ok)
            )
            estimator_inputs = dict(matches) if dual_estimator_allowed else {}
            if not estimator_inputs and matches:
                # A poor core transform is a calibration failure, not an
                # invitation to average incompatible coordinate frames. Keep
                # one deterministic camera while retaining temporal state.
                serial = (
                    int(args.fallback_serial)
                    if int(args.fallback_serial) in matches
                    else int(max(matches, key=lambda value: _quality(
                        matches[value], value, args.confidence
                    ).score))
                )
                estimator_inputs = {serial: matches[serial]}
            if estimator_inputs:
                estimator_timestamps = {
                    serial: active_capture_ns.get(serial, 0)
                    for serial in estimator_inputs
                }
                estimator_target_ns = max(estimator_timestamps.values())
                if (
                    previous_timestamp_ns is not None
                    and estimator_target_ns <= previous_timestamp_ns
                ):
                    if show_preview(
                        preview_lists,
                        fusion_mode=(
                            "joint_fusion" if fusion_latched else "latest_hold"
                        ),
                        operator_state=selection.state.value,
                        contributing_views=len(matches),
                    ):
                        break
                    frame_index += 1
                    continue
                fusion_input_timestamp_ns = time.time_ns()
                try:
                    human_result = human_estimator.update(
                        estimator_inputs,
                        estimator_timestamps,
                        camera_positions={
                            serial: calibration_pose[serial][1]
                            for serial in estimator_inputs
                        },
                        target_timestamp_ns=estimator_target_ns,
                    )
                except ValueError:
                    human_result = None
                fusion_finish_timestamp_ns = time.time_ns()
            if human_result is None:
                if show_preview(
                    preview_lists,
                    fusion_mode="hold",
                    operator_state=selection.state.value,
                    contributing_views=len(matches),
                ):
                    break
                frame_index += 1
                continue
            # Report what actually entered the estimator. The previous latch-
            # based label could claim joint fusion while only one camera was
            # used, which made diagnostics misleading.
            source_mode = (
                "joint_fusion"
                if len(estimator_inputs) >= 2
                else "single_fallback"
            )
            fusion_state = (
                "LOCKED_DUAL"
                if len(estimator_inputs) >= 2 and calibration_agreement_ok
                else "LOCKED_DUAL_TRANSIENT_GUARD"
                if len(estimator_inputs) >= 2 and transient_dual_ok
                else "LOCKED_CALIBRATION_GUARD"
                if has_dual_evidence
                else "LOCKED_PARTIALLY_VISIBLE"
                if fusion_latched
                else "SINGLE_ACQUISITION"
            )
            source_serial = None if len(estimator_inputs) >= 2 else next(iter(estimator_inputs))
            base_body = selected if source_serial is None else estimator_inputs[source_serial]
            selected = adapt_body(
                base_body,
                canonical_id=int(selection.body_id),
                source=source_mode,
                source_serial=source_serial,
            )
            selected.keypoint = human_result.points.copy()
            selected.keypoint_confidence = human_result.confidence.copy()
            selected.position = human_result.points[IDX["PELVIS"]].copy()
            quaternion_flip_corrected = bool(
                previous_root_orientation is not None
                and np.isfinite(selected.global_root_orientation).all()
                and float(np.dot(
                    previous_root_orientation, selected.global_root_orientation
                )) < 0.0
            )
            selected.global_root_orientation = quaternion_continuous(
                previous_root_orientation, selected.global_root_orientation
            )
            if np.isfinite(selected.global_root_orientation).all():
                previous_root_orientation = selected.global_root_orientation.copy()
            continuous_local = np.full((38, 4), np.nan)
            for joint in range(38):
                previous = (
                    previous_local_orientations[joint]
                    if np.isfinite(previous_local_orientations[joint]).all()
                    else None
                )
                if (
                    previous is not None
                    and np.isfinite(selected.local_orientation_per_joint[joint]).all()
                    and float(np.dot(
                        previous, selected.local_orientation_per_joint[joint]
                    )) < 0.0
                ):
                    quaternion_flip_corrected = True
                continuous_local[joint] = quaternion_continuous(
                    previous, selected.local_orientation_per_joint[joint]
                )
                if np.isfinite(continuous_local[joint]).all():
                    previous_local_orientations[joint] = continuous_local[joint]
            selected.local_orientation_per_joint = continuous_local
            decision = MultiViewDecision(
                mode=source_mode,
                selected_serial=source_serial,
                reason=(
                    "per_joint_confidence_fusion"
                    if source_mode == "joint_fusion"
                    else "calibration_or_view_fallback_stable_camera"
                ),
                fused_score=selected_quality.score,
                best_single_score=max((item.score for item in single_qualities), default=None),
            )

            if matches:
                for serial, body in matches.items():
                    raw_identity[serial] = int(body.id)
            last_operator_anchor = _anchor(selected).copy()
            timestamp_ns = max(
                (active_capture_ns.get(serial, 0) for serial in estimator_inputs),
                default=time.time_ns(),
            )
            if timestamp_ns <= 0:
                timestamp_ns = time.time_ns()
            timestamp_s = timestamp_ns / 1e9
            raw = np.asarray(selected.keypoint, dtype=np.float64)
            confidence = np.asarray(selected.keypoint_confidence, dtype=np.float64)
            filtered = low_pass.update(
                int(selection.body_id), raw, confidence,
                args.confidence, timestamp_s,
            )
            view_descriptions = [
                _view_description(
                    serial,
                    body,
                    args.confidence,
                    capture_timestamp_ns=active_capture_ns.get(serial),
                    receive_timestamp_ns=active_receive_ns.get(serial),
                )
                for serial, body in matches.items()
            ]
            evidence = {
                side: arm_evidence(view_descriptions, side=side, threshold=args.confidence)
                for side in ("left", "right")
            }
            if selected.source in ("fusion", "joint_fusion"):
                # Joint fusion already applies timestamp compensation,
                # anatomical projection and elbow-plane continuity. Re-solving
                # its elbows from one camera's 2-D pixels would mix frames.
                arm_result = ArmChainResult(
                    points=filtered.copy(),
                    overlap={"left": False, "right": False},
                    recovered={"left": False, "right": False},
                    reasons=("confidence_aware_human_state_preserved",),
                )
            else:
                arm_result = arm_optimizer.update(
                    timestamp_s=timestamp_s,
                    points_3d=filtered,
                    points_2d=np.asarray(selected.keypoint_2d, dtype=np.float64),
                    confidence=confidence,
                    index=IDX,
                    threshold=args.confidence,
                    calibration=calibrator.profile,
                    multiview_evidence=evidence,
                )
            filtered = arm_result.points
            calibration = calibrator.update(
                operator_id=int(selection.body_id), timestamp_s=timestamp_s,
                points=filtered, confidence=confidence, index=IDX,
                confidence_threshold=args.confidence,
            )
            record = build_record(
                selected, filtered, timestamp_ns, frame_index,
                args.confidence, imu=None,
            )
            try:
                pelvis_origin, measured_pelvis_rotation = pelvis_frame(filtered, IDX)
                pelvis_rotation = stabilize_pelvis_rotation(
                    previous_pelvis_rotation, measured_pelvis_rotation,
                )
                previous_pelvis_rotation = pelvis_rotation.copy()
                pelvis_local = (
                    np.asarray(filtered, dtype=np.float64) - pelvis_origin
                ) @ pelvis_rotation
            except ValueError:
                pelvis_local = np.full_like(filtered, np.nan)
                pelvis_origin = np.full(3, np.nan)
                pelvis_rotation = np.full((3, 3), np.nan)
            record["operator_selection"] = {
                "state": selection.state.value,
                "locked_body_id": selector.locked_id,
                "locked_unique_object_id": selector.locked_unique_id,
                "missing_frames": selection.missing_frames,
                "acquisition_frames": selection.acquisition_frames,
                "reason": selection.reason,
                "automatic_handover": False,
            }
            record["calibration"] = {
                "state": calibration.state, "progress": calibration.progress,
                "elapsed_s": calibration.elapsed_s, "sample_count": calibration.sample_count,
                "reason": calibration.reason, "profile": calibration.profile,
            }
            record["pelvis_frame"] = {
                "coordinate_system": "PELVIS_LOCAL_X_FWD_Y_LEFT_Z_UP",
                "origin_camera_m": pelvis_origin,
                "rotation_camera_from_pelvis": pelvis_rotation,
                "relative_neutral_yaw_rad": None,
                "keypoints_m": pelvis_local,
            }
            record["occlusion_analysis"] = {
                "torso_polygon_names": ["LEFT_SHOULDER", "RIGHT_SHOULDER", "RIGHT_HIP", "LEFT_HIP"],
                "arm_torso_overlap": arm_result.overlap,
                "arm_chain_recovered": arm_result.recovered,
                "arm_recovery_mode": "akc_event_triggered_after_multiview",
                "multiview_arm_evidence": evidence,
                "akc_candidate_confidence": arm_result.candidate_confidence,
                "akc_candidate_cost": arm_result.candidate_cost,
                "elbow_branch_sign": arm_result.branch_sign,
                "reasons": list(arm_result.reasons),
            }
            record["perception_metrics"] = perception.update(
                timestamp_s=timestamp_s, points=pelvis_local, confidence=confidence,
                index=IDX, threshold=args.confidence, overlap=arm_result.overlap,
                calibration_profile=calibration.profile,
            )
            record["human_state"] = human_result.as_dict(BODY38_NAMES)
            record["human_state"]["retargeting_policy"] = teleop_config["retargeting"]
            record["control_mode_request"] = {
                "session_ns": control_mode_session_ns,
                "revision": control_mode_revision,
                "policy_powered": policy_powered_requested,
            }
            record["multi_camera"] = {
                "mode": selected.source,
                "fusion_state": fusion_state,
                "fusion_latched": fusion_latched,
                "selection_reason": decision.reason,
                "fused_quality_score": decision.fused_score,
                "best_single_quality_score": decision.best_single_score,
                "fusion_config": str(config_path),
                "connected_serials": sorted(identifiers),
                "contributing_views": len(matches),
                "selected_single_serial": selected.source_serial,
                "per_camera": view_descriptions,
                "cross_view_agreement": agreement,
                "calibration_agreement_ok": calibration_agreement_ok,
                "camera_timestamp_delta_ms": camera_timestamp_delta_ms,
                "camera_sync_ok": camera_sync_ok,
                "latency_trace_ns": {
                    "camera_capture_ns": dict(estimator_timestamps),
                    "body_tracking_finish_ns": {
                        serial: active_receive_ns.get(serial)
                        for serial in estimator_inputs
                    },
                    "fusion_input_ns": fusion_input_timestamp_ns,
                    "fusion_finish_ns": fusion_finish_timestamp_ns,
                },
                "failure_codes": [
                    code for code, active in (
                        ("CAMERA_TIME_MISMATCH", camera_sync_ok is False),
                        ("DUAL_EVIDENCE_MISSING", not has_dual_evidence),
                        (
                            "CAMERA_CALIBRATION_DISAGREEMENT",
                            has_dual_evidence and not calibration_agreement_ok,
                        ),
                        ("QUATERNION_FLIP_CORRECTED", quaternion_flip_corrected),
                    ) if active
                ] + list(human_result.failure_codes),
                "arm_evidence": evidence,
                "fusion_metrics": fusion_metrics,
                "rig_extrinsics": rig_metrics,
                "camera_pose_fusion_from_local": {
                    str(serial): {
                        "rotation": sanitize_for_json(rotation.tolist()),
                        "translation_m": sanitize_for_json(translation.tolist()),
                    }
                    for serial, (rotation, translation) in calibration_pose.items()
                },
            }
            processing_ns = time.time_ns()
            output_times.append(now)
            while output_times and now - output_times[0] > 1.0:
                output_times.popleft()
            effective_output_hz = (
                (len(output_times) - 1)
                / max(output_times[-1] - output_times[0], 1.0e-6)
                if len(output_times) >= 2 else 0.0
            )
            # Preserve full per-camera BODY_38 arrays in the lossless JSONL,
            # but keep the live UDP datagram compact. Large UDP fragments were
            # adding latency and whole-frame loss when Isaac/Rerun/ROS were all
            # active at once.
            live_multi_camera = dict(record["multi_camera"])
            live_multi_camera["per_camera"] = [
                {
                    key: value
                    for key, value in view.items()
                    if key not in (
                        "keypoints_3d_fusion_m", "keypoint_confidence"
                    )
                }
                for view in view_descriptions
            ]
            live_multi_camera.pop("camera_pose_fusion_from_local", None)
            udp_send_ns = time.time_ns()
            packet = sanitize_for_json({
                "schema": "zed_body38_live/v1",
                "sequence": output_sequence,
                "source_frame_index": frame_index,
                "timestamp_ns": timestamp_ns,
                "coordinate_system": record["coordinate_system"], "units": "meter",
                "body_id": record["body_id"], "tracking_state": record["tracking_state"],
                "action_state": record["action_state"], "body_confidence": record["body_confidence"],
                "root_position_m": record["root_position_m"],
                "global_root_orientation_xyzw": record["global_root_orientation_xyzw"],
                "keypoint_names": BODY38_NAMES,
                "keypoints_2d_px": record["keypoints_2d_px"],
                "keypoints_3d_raw_m": record["keypoints_3d_raw_m"],
                "keypoints_3d_m": record["keypoints_3d_filtered_m"],
                "keypoint_confidence": record["keypoint_confidence"],
                "local_orientation_per_joint_xyzw": record["local_orientation_per_joint_xyzw"],
                "local_position_per_joint_m": record["local_position_per_joint_m"],
                "root_relative_keypoints_m": record["root_relative_keypoints_m"],
                "pelvis_frame": record["pelvis_frame"],
                "operator_selection": record["operator_selection"],
                "calibration": record["calibration"],
                "occlusion_analysis": record["occlusion_analysis"],
                "perception_metrics": record["perception_metrics"],
                "human_state": record["human_state"],
                "control_mode_request": record["control_mode_request"],
                "multi_camera": live_multi_camera,
                "shoulder_width_normalized_keypoints": record["shoulder_width_normalized_keypoints"],
                "reference_ready": {
                    "upper_body": record["g1_reference_features"]["upper_body_reference_ready"],
                    "whole_body": record["g1_reference_features"]["whole_body_reference_ready"],
                },
                "g1_reference_features": record["g1_reference_features"],
                "euclidean_distance_m": record["euclidean_distance_m"],
                "imu": None,
                "latency_trace_ns": {
                    "t0_capture_ns": timestamp_ns,
                    "t1_zed_processing_done_ns": processing_ns,
                    # This is sampled immediately before compact serialization
                    # and send; unlike the old value it excludes recording and
                    # full diagnostic-packet work.
                    "t2_windows_udp_send_ns": udp_send_ns,
                },
                "transport_metrics": {
                    "source_interval_ms": (
                        (timestamp_ns - previous_timestamp_ns) / 1e6
                        if previous_timestamp_ns and timestamp_ns > previous_timestamp_ns else None
                    ),
                    "capture_to_send_ms": max(0.0, (udp_send_ns - timestamp_ns) / 1e6),
                    "stream_limit_hz": args.stream_max_hz,
                    "source_mode": selected.source,
                    "camera_acquisition": "parallel_per_camera",
                    "preview_hz": 0.0 if args.headless else float(args.preview_hz),
                    "record_queue_depth": record_queue.qsize(),
                    "record_dropped": record_dropped,
                    "effective_output_hz": effective_output_hz,
                },
            })
            payload = json.dumps(packet, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")

            # The compact GMR datagram is the real-time product. Send it before
            # building Rerun/ROS diagnostics or serializing the lossless JSONL.
            control_target = (
                (args.stream_host, args.stream_port)
                if args.stream_host else None
            )
            if (
                udp is not None
                and control_target is not None
                and len(payload) <= 60000
                and now - last_send[control_target]
                >= 0.98 / max(targets[control_target], 1.0)
            ):
                udp.sendto(payload, control_target)
                last_send[control_target] = now

            analysis_payload = payload
            monitor_target = (
                (args.monitor_host, args.monitor_port)
                if args.monitor_host else None
            )
            monitor_due = (
                monitor_target is not None
                and now - last_send[monitor_target]
                >= 0.98 / max(targets[monitor_target], 1.0)
            )
            if monitor_due:
                analysis_multi_camera = dict(live_multi_camera)
                analysis_multi_camera["per_camera"] = view_descriptions
                analysis_packet = dict(packet)
                analysis_packet["multi_camera"] = analysis_multi_camera
                candidate_payload = json.dumps(
                    sanitize_for_json(analysis_packet),
                    ensure_ascii=False, allow_nan=False, separators=(",", ":"),
                ).encode("utf-8")
                if len(candidate_payload) <= 60000:
                    analysis_payload = candidate_payload
            if udp is not None and len(payload) <= 60000:
                for target, hz in targets.items():
                    if target == control_target:
                        continue
                    if now - last_send[target] >= 0.98 / max(hz, 1.0):
                        outgoing = (
                            analysis_payload
                            if target == (args.monitor_host, args.monitor_port)
                            else payload
                        )
                        udp.sendto(outgoing, target)
                        last_send[target] = now

            if recording and record_file is not None:
                record["transport_metrics"] = packet["transport_metrics"]
                try:
                    record_queue.put_nowait(record)
                except queue.Full:
                    # Never let slow OneDrive/disk I/O increase robot latency.
                    # The count is printed and can be used to reject an
                    # incomplete training recording.
                    record_dropped += 1
            previous_timestamp_ns = timestamp_ns
            output_sequence += 1
            if show_preview(
                preview_lists,
                fusion_mode=selected.source,
                operator_state=selection.state.value,
                contributing_views=len(matches),
            ):
                break
            frame_index += 1
            if frame_index % max(1, args.fps * 2) == 0:
                print(
                    f"dual={selected.source} operator={selection.state.value} "
                    f"views={len(matches)} valid={np.count_nonzero(np.isfinite(filtered).all(axis=1))}/38 "
                    f"cal={calibration.state} output={effective_output_hz:.1f}Hz "
                    f"record_q={record_queue.qsize()}"
                )
    finally:
        sender_stop.set()
        sender_wakeup.set()
        for thread in sender_threads:
            thread.join(timeout=2.0)
        record_writer_stop.set()
        if record_writer is not None:
            record_queue.join()
            record_writer.join(timeout=5.0)
        if record_file is not None:
            record_file.flush(); record_file.close()
        if udp is not None:
            udp.close()
        fusion.close()
        for zed in senders.values():
            try:
                zed.stop_publishing(); zed.disable_body_tracking(); zed.disable_positional_tracking(); zed.close()
            except Exception:
                pass
        cv2.destroyAllWindows()
        if record_writer_errors:
            print(
                "UYARI: JSONL writer hatalari: "
                + "; ".join(record_writer_errors[:3])
            )
        print(
            f"Dual Fusion kapatıldı. Kayıtlı BODY_38: {recorded}, "
            f"record_drop={record_dropped}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

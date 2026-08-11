#!/usr/bin/env python3
"""Dual ZED 2i BODY_38 Fusion source for the existing G1 upper-body path.

This is deliberately separate from ``zed_g1_skeleton.py``.  It follows the
official Stereolabs local multi-camera publish/subscribe workflow and emits the
same ``zed_body38_live/v1`` UDP contract consumed by the existing GMR/Isaac
bridge.  It never sends commands directly to a physical robot.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import socket
import time
from types import SimpleNamespace
from typing import Any

try:
    import msvcrt
except ImportError:
    msvcrt = None

from zed_g1_skeleton import (
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
    sanitize_for_json,
    sl,
    np,
)
from motion_pipeline.arm_chain import ArmChainOptimizer
from motion_pipeline.calibration import CalibrationManager, to_pelvis_local
from motion_pipeline.metrics import PerceptionMetrics
from motion_pipeline.multiview import (
    MultiViewDecision,
    ViewQuality,
    arm_evidence,
    body_quality,
    choose_output,
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


def _nearest_body(bodies: list[Any], anchor: np.ndarray, maximum_m: float = 0.80) -> Any | None:
    valid = []
    for body in bodies:
        value = _anchor(body)
        if np.isfinite(value).all() and np.isfinite(anchor).all():
            valid.append((float(np.linalg.norm(value - anchor)), body))
    if not valid:
        return None
    distance, body = min(valid, key=lambda item: item[0])
    return body if distance <= maximum_m else None


def _view_description(serial: int, body: Any, threshold: float) -> dict[str, object]:
    confidence = np.asarray(body.keypoint_confidence, dtype=np.float64)
    pixels = np.asarray(body.keypoint_2d, dtype=np.float64)
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
        "quality": sanitize_for_json(_quality(body, serial, threshold).__dict__),
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
        f"| views={contributing_views} | REC={'ON' if recording else 'OFF'} {recorded}",
        (12, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.66, state_color, 2, cv2.LINE_AA,
    )
    cv2.putText(
        header, "S: JSONL kayit  R: operator/kalibrasyon reset  Q/ESC: cikis",
        (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (220, 220, 220), 1, cv2.LINE_AA,
    )
    return cv2.vconcat([header, canvas])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="İki ZED 2i ile ortak BODY_38 -> mevcut G1 GMR/Isaac UDP kaynağı")
    parser.add_argument("--fusion-config", type=Path, required=True)
    parser.add_argument("--fps", type=int, choices=(15, 30, 60), default=30)
    parser.add_argument("--model", choices=("fast", "medium", "accurate"), default="medium")
    parser.add_argument("--depth-mode", choices=("neural-light", "neural", "performance"), default="neural-light")
    parser.add_argument("--confidence", type=float, default=40.0)
    parser.add_argument("--fusion-smoothing", type=float, default=0.10)
    parser.add_argument("--filter-tau", type=float, default=0.0)
    parser.add_argument("--operator-acquire-frames", type=int, default=10)
    parser.add_argument("--calibration-seconds", type=float, default=4.0)
    parser.add_argument("--camera-timeout", type=float, default=5.0)
    parser.add_argument("--stream-host", default="")
    parser.add_argument("--stream-port", type=int, default=15050)
    parser.add_argument("--stream-max-hz", type=float, default=30.0)
    parser.add_argument("--monitor-host", default="")
    parser.add_argument("--monitor-port", type=int, default=15052)
    parser.add_argument("--monitor-max-hz", type=float, default=15.0)
    parser.add_argument("--ros-host", default="")
    parser.add_argument("--ros-port", type=int, default=15054)
    parser.add_argument("--ros-max-hz", type=float, default=30.0)
    parser.add_argument("--record", action="store_true")
    parser.add_argument(
        "--headless", action="store_true",
        help="Kamera/iskelet onizleme penceresini kapat.",
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
    fusion_configs = sl.read_fusion_configuration_file(
        str(config_path), sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD, sl.UNIT.METER
    )
    if len(fusion_configs) < 2:
        print("Kalibrasyon dosyasında en az iki geçerli ZED bulunmalı.")
        return 2

    model_map = {
        "fast": sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST,
        "medium": sl.BODY_TRACKING_MODEL.HUMAN_BODY_MEDIUM,
        "accurate": sl.BODY_TRACKING_MODEL.HUMAN_BODY_ACCURATE,
    }
    depth_map = {
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
        parameters.enable_tracking = True
        parameters.enable_body_fitting = True
        parameters.enable_segmentation = False
        parameters.max_range = 8.0
        parameters.allow_reduced_precision_inference = True
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
    synchronization = sl.SynchronizationParameter()
    source_period_ms = 1000.0 / float(args.fps)
    synchronization.windows_size = source_period_ms
    synchronization.maximum_lateness = 2.0 * source_period_ms
    synchronization.data_source_timeout = max(3.0 * source_period_ms, 200.0)
    # Never silently reuse a stale arm pose. If one source misses the current
    # window, official Fusion may still use the fresh calibrated camera because
    # skeleton_minimum_allowed_camera is one.
    synchronization.keep_last_data = False
    fusion_init.synchronization_parameters = synchronization
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
    # Official fallback behavior: one calibrated camera is enough to retain
    # the fused identity when the second view is temporarily unavailable.
    fusion_runtime.skeleton_minimum_allowed_camera = 1
    fusion_runtime.skeleton_minimum_allowed_keypoints = 8
    fusion_runtime.skeleton_smoothing = float(np.clip(args.fusion_smoothing, 0.0, 1.0))
    sender_runtime = sl.BodyTrackingRuntimeParameters()
    sender_runtime.detection_confidence_threshold = float(args.confidence)
    sender_runtime.minimum_keypoints_threshold = 8
    sender_runtime.skeleton_smoothing = 0.05
    fused_bodies = sl.Bodies()
    raw_bodies = {serial: sl.Bodies() for serial in identifiers}

    low_pass = ConfidenceAwareLowPass(args.filter_tau)
    selector = OperatorSelector(args.operator_acquire_frames)
    calibrator = CalibrationManager(args.calibration_seconds)
    arm_optimizer = ArmChainOptimizer(hold_s=0.35, recovery_mode="akc", branch_confirm_frames=4)
    perception = PerceptionMetrics()
    raw_identity: dict[int, int] = {}
    last_operator_anchor: np.ndarray | None = None

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

    def set_recording(enabled: bool) -> None:
        nonlocal record_file, recording
        if enabled and record_file is None:
            record_file = record_path.open("w", encoding="utf-8", buffering=1)
            header = {
                "schema": "zed_dual_body38_g1_reference/metadata/v1",
                "created_unix_ns": time.time_ns(),
                "zed_sdk_version": sl.Camera.get_sdk_version(),
                "fusion_config": str(config_path),
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
        recording = enabled
        print(f"Dual BODY_38 kayıt {'AÇIK' if enabled else 'KAPALI'}: {record_path} ({recorded} kare)")

    def handle_key(key: str | int | None) -> bool:
        """Handle keys from either the PowerShell console or preview window."""
        nonlocal last_operator_anchor
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
        elif normalized == "r":
            selector.reset(); calibrator.reset(); arm_optimizer.reset(); low_pass.reset()
            raw_identity.clear(); last_operator_anchor = None
            print("R: dual operator lock, calibration and arm memory reset.")
        return False

    def show_preview(
        raw_lists: dict[int, list[Any]],
        *,
        fusion_mode: str,
        operator_state: str,
        contributing_views: int,
    ) -> bool:
        if args.headless:
            return False
        preview = _draw_dual_preview(
            latest_frames, raw_lists, raw_identity, args.confidence,
            fusion_mode=fusion_mode,
            operator_state=operator_state,
            contributing_views=contributing_views,
            recording=recording,
            recorded=recorded,
        )
        if preview is None:
            return False
        cv2.imshow("Dual ZED 2i BODY_38 Fusion - G1", preview)
        return handle_key(cv2.waitKey(1) & 0xFF)

    if args.record:
        set_recording(True)
    print("Hazır. Q/ESC: çıkış | S: JSONL kayıt | R: operatör/kalibrasyon/kol hafızası sıfırla")
    print("Not: Tek kamera sistemi ayrı kalır; bu süreç yalnız dual Fusion kaynağıdır.")

    frame_index = 0
    started = time.monotonic()
    previous_timestamp_ns: int | None = None
    fusion_metrics: dict[str, object] | None = None
    last_fusion_metrics_s = 0.0
    try:
        while True:
            now = time.monotonic()
            if args.duration > 0 and now - started >= args.duration:
                break
            if msvcrt is not None and msvcrt.kbhit():
                key = msvcrt.getwch().lower()
                if key in ("q", "\x1b"):
                    break
                if key == "s":
                    set_recording(not recording)
                elif key == "r":
                    selector.reset(); calibrator.reset(); arm_optimizer.reset(); low_pass.reset()
                    raw_identity.clear(); last_operator_anchor = None
                    print("R: dual operatör kilidi, kalibrasyon ve kol hafızası sıfırlandı.")

            active = 0
            for serial, zed in senders.items():
                grab = zed.grab()
                if grab == sl.ERROR_CODE.SUCCESS:
                    active += 1
                    last_sender_ok[serial] = now
                    frame = _camera_frame(zed, sender_images[serial])
                    if frame is not None:
                        latest_frames[serial] = frame
                    zed.retrieve_bodies(sender_bodies[serial], sender_runtime)
                elif now - last_sender_ok[serial] > 1.0:
                    print(f"UYARI: ZED {serial} kare kesintisi; diğer kamera ile devam ediliyor ({grab}).")
                    last_sender_ok[serial] = now - 0.9
            if active == 0:
                time.sleep(0.002)
                continue
            preview_lists = {
                serial: (
                    list(value.body_list)
                    if getattr(value, "is_new", False) else []
                )
                for serial, value in sender_bodies.items()
            }
            if fusion.process() != sl.FUSION_ERROR_CODE.SUCCESS:
                if show_preview(
                    preview_lists,
                    fusion_mode="fusion_wait",
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
            for serial, uuid in identifiers.items():
                fusion.retrieve_bodies(
                    raw_bodies[serial], fusion_runtime, uuid,
                    sl.FUSION_REFERENCE_FRAME.BASELINK,
                )

            fused_list = list(fused_bodies.body_list) if fused_bodies.is_new else []
            raw_lists = {
                serial: (list(value.body_list) if value.is_new else [])
                for serial, value in raw_bodies.items()
            }
            candidate_entries: list[tuple[BodyAdapter, ViewQuality, dict[int, Any]]] = []
            for fused in fused_list:
                if not _valid_body(fused, args.confidence):
                    continue
                fused_anchor = _anchor(fused)
                matches = {
                    serial: match
                    for serial, bodies in raw_lists.items()
                    if (match := _nearest_body(bodies, fused_anchor)) is not None
                }
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
                    fused, keypoint_2d=pixels, source="fusion", source_serial=None
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
                for serial, raw_id in raw_identity.items():
                    match = next((b for b in raw_lists.get(serial, []) if int(b.id) == raw_id), None)
                    if match is None or not _valid_body(match, args.confidence):
                        continue
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
                    fallback_entries.append((adapter, quality, {serial: match}))
                if fallback_entries:
                    best_fallback = max(fallback_entries, key=lambda entry: entry[1].score)
                    candidates.append(best_fallback[0])
                    candidate_entries.append(best_fallback)

            selection = selector.update(
                candidates,
                valid=lambda body: _valid_body(body, args.confidence),
                distance=lambda body: float(np.linalg.norm(_anchor(body))),
            )
            selected = selection.body
            entry = next((item for item in candidate_entries if item[0] is selected), None)
            if selected is None or entry is None:
                if show_preview(
                    preview_lists,
                    fusion_mode="waiting",
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
            decision: MultiViewDecision = choose_output(
                fused=selected_quality if selected.source == "fusion" else None,
                singles=single_qualities,
                previous_anchor_m=last_operator_anchor,
            )
            if decision.mode == "single_fallback" and decision.selected_serial in matches:
                serial = int(decision.selected_serial)
                selected = adapt_body(
                    matches[serial], canonical_id=int(selection.body_id),
                    source="single_fallback", source_serial=serial,
                )
            elif decision.mode == "hold":
                if show_preview(
                    preview_lists,
                    fusion_mode="hold",
                    operator_state=selection.state.value,
                    contributing_views=len(matches),
                ):
                    break
                frame_index += 1
                continue

            if matches:
                for serial, body in matches.items():
                    raw_identity[serial] = int(body.id)
            last_operator_anchor = _anchor(selected).copy()
            timestamp_ns = int(fused_bodies.timestamp.get_nanoseconds())
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
                _view_description(serial, body, args.confidence)
                for serial, body in matches.items()
            ]
            evidence = {
                side: arm_evidence(view_descriptions, side=side, threshold=args.confidence)
                for side in ("left", "right")
            }
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
                pelvis_local, pelvis_origin, pelvis_rotation = to_pelvis_local(filtered, IDX)
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
            agreement = None
            if len(matches) >= 2:
                first, second = list(matches.values())[:2]
                agreement = keypoint_agreement(
                    np.asarray(first.keypoint), np.asarray(second.keypoint),
                    np.asarray(first.keypoint_confidence), np.asarray(second.keypoint_confidence),
                    args.confidence,
                )
            record["multi_camera"] = {
                "mode": selected.source,
                "selection_reason": decision.reason,
                "fusion_config": str(config_path),
                "connected_serials": sorted(identifiers),
                "contributing_views": len(matches),
                "selected_single_serial": selected.source_serial,
                "per_camera": view_descriptions,
                "cross_view_agreement": agreement,
                "arm_evidence": evidence,
                "fusion_metrics": fusion_metrics,
            }
            if recording and record_file is not None:
                record_file.write(json.dumps(sanitize_for_json(record), ensure_ascii=False, allow_nan=False) + "\n")
                recorded += 1

            processing_ns = time.time_ns()
            packet = sanitize_for_json({
                "schema": "zed_body38_live/v1",
                "sequence": frame_index,
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
                "multi_camera": record["multi_camera"],
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
                    "t2_windows_udp_send_ns": processing_ns,
                },
                "transport_metrics": {
                    "source_interval_ms": (
                        (timestamp_ns - previous_timestamp_ns) / 1e6
                        if previous_timestamp_ns and timestamp_ns > previous_timestamp_ns else None
                    ),
                    "capture_to_send_ms": max(0.0, (processing_ns - timestamp_ns) / 1e6),
                    "stream_limit_hz": args.stream_max_hz,
                    "source_mode": selected.source,
                },
            })
            payload = json.dumps(packet, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
            if udp is not None and len(payload) <= 60000:
                for target, hz in targets.items():
                    if now - last_send[target] >= 0.98 / max(hz, 1.0):
                        udp.sendto(payload, target)
                        last_send[target] = now
            previous_timestamp_ns = timestamp_ns
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
                    f"cal={calibration.state}"
                )
    finally:
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
        print(f"Dual Fusion kapatıldı. Kayıtlı BODY_38: {recorded}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

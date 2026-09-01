#!/usr/bin/env python3
"""Four-camera, two-host ZED BODY_38 Fusion acceptance test.

The main PC opens the cameras listed with ``--local-serial`` and subscribes to
the laptop cameras listed with ``--remote-camera``.  It only prints/writes
fused BODY_38 telemetry; it has no G1, ROS, Isaac, or actuator output.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import pyzed.sl as sl

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from zed_four_camera_test.fusion_config_io import read_fusion_configuration_file


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Iki hosttan dort ZED BODY_38 Fusion kabul testi")
    parser.add_argument("--fusion-config", type=Path, required=True, help="ZED360 ile uretilmis 4-kamera JSON")
    parser.add_argument("--local-serial", type=int, action="append", required=True, help="Ana PC'ye USB ile bagli ZED seri numarasi")
    parser.add_argument(
        "--remote-camera",
        action="append",
        required=True,
        metavar="SERIAL@IP:PORT",
        help="Laptop publisher kamerasi. Ornek: 12345678@192.168.50.11:30000",
    )
    parser.add_argument("--fps", type=int, choices=(15, 30, 60), default=15)
    parser.add_argument("--model", choices=("fast", "medium", "accurate"), default="fast")
    parser.add_argument(
        "--depth-mode",
        choices=("performance", "neural-light", "neural", "ultra"),
        default="neural-light",
    )
    parser.add_argument("--duration", type=float, default=180.0, help="Saniye; 0 sonsuz calisma demektir.")
    parser.add_argument("--record", type=Path, default=None, help="Fused BODY_38 JSONL kayit yolu")
    parser.add_argument("--min-cameras", type=int, choices=(1, 2, 3, 4), default=2)
    return parser.parse_args()


def parse_remote(value: str) -> tuple[int, str, int]:
    try:
        serial_text, endpoint = value.split("@", maxsplit=1)
        ip, port_text = endpoint.rsplit(":", maxsplit=1)
        serial, port = int(serial_text), int(port_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Uzak kamera bicimi SERIAL@IP:PORT olmali.") from exc
    if serial <= 0 or not ip or not 1 <= port <= 65535 or port % 2:
        raise argparse.ArgumentTypeError("Gecersiz uzak kamera seri numarasi, IP veya port; port cift olmali.")
    return serial, ip, port


def finite_list(value: Any) -> list[Any]:
    array = np.asarray(value, dtype=np.float64)
    return np.where(np.isfinite(array), array, None).tolist()


def json_safe(value: Any) -> Any:
    """Convert the SDK metrics' NaN/Inf values to portable JSON nulls."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    return value


def fusion_metrics(fusion: sl.Fusion) -> dict[str, Any] | None:
    try:
        status, metrics = fusion.get_process_metrics()
        if status != sl.FUSION_ERROR_CODE.SUCCESS:
            return None
        cameras: dict[str, Any] = {}
        for identifier, camera in metrics.camera_individual_stats.items():
            serial = str(int(getattr(identifier, "serial_number", 0)))
            cameras[serial] = {
                "present": bool(getattr(camera, "is_present", False)),
                "received_fps": float(getattr(camera, "received_fps", math.nan)),
                "received_latency_ms": 1000.0 * float(getattr(camera, "received_latency", math.nan)),
                "sync_delta_ms": 1000.0 * float(getattr(camera, "delta_ts", math.nan)),
            }
        return {
            "mean_camera_fused": float(getattr(metrics, "mean_camera_fused", math.nan)),
            "mean_stdev_between_camera_ms": 1000.0 * float(
                getattr(metrics, "mean_stdev_between_camera", math.nan)
            ),
            "cameras": cameras,
        }
    except Exception:
        return None


def main() -> int:
    args = parse_args()
    config_path = args.fusion_config.expanduser().resolve()
    if not config_path.is_file():
        print(f"Fusion JSON bulunamadi: {config_path}", file=sys.stderr)
        return 2
    try:
        remote_specs = [parse_remote(item) for item in args.remote_camera]
    except argparse.ArgumentTypeError as exc:
        print(f"Gecersiz --remote-camera: {exc}", file=sys.stderr)
        return 2
    remote = {serial: (ip, port) for serial, ip, port in remote_specs}
    local = set(args.local_serial)
    if (
        len(local) != len(args.local_serial)
        or len(remote) != len(remote_specs)
        or local.intersection(remote)
    ):
        print("Yerel/uzak kamera seri numaralari benzersiz olmali.", file=sys.stderr)
        return 2

    configs = read_fusion_configuration_file(
        config_path, sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP, sl.UNIT.METER
    )
    by_serial = {int(conf.serial_number): conf for conf in configs}
    expected = local.union(remote)
    missing = sorted(expected.difference(by_serial))
    if missing:
        print(f"Bu seri numaralari ZED360 JSON dosyasinda yok: {missing}", file=sys.stderr)
        return 2
    if len(expected) != 4:
        print("Bu kabul testi tam olarak 4 farkli kamera bekler.", file=sys.stderr)
        return 2

    model_map = {
        "fast": sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST,
        "medium": sl.BODY_TRACKING_MODEL.HUMAN_BODY_MEDIUM,
        "accurate": sl.BODY_TRACKING_MODEL.HUMAN_BODY_ACCURATE,
    }
    depth_map = {
        "performance": sl.DEPTH_MODE.PERFORMANCE,
        "neural-light": sl.DEPTH_MODE.NEURAL_LIGHT,
        "neural": sl.DEPTH_MODE.NEURAL,
        "ultra": sl.DEPTH_MODE.ULTRA,
    }

    local_cameras: dict[int, sl.Camera] = {}
    try:
        for serial in sorted(local):
            init = sl.InitParameters()
            init.set_from_serial_number(serial)
            init.camera_resolution = sl.RESOLUTION.HD720
            init.camera_fps = args.fps
            init.depth_mode = depth_map[args.depth_mode]
            init.coordinate_units = sl.UNIT.METER
            init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
            init.depth_maximum_distance = 8.0
            if hasattr(init, "async_grab_camera_recovery"):
                init.async_grab_camera_recovery = True
            camera = sl.Camera()
            status = camera.open(init)
            if status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"ZED {serial} acilamadi: {status}")
            positional = sl.PositionalTrackingParameters()
            positional.set_as_static = True
            status = camera.enable_positional_tracking(positional)
            if status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"ZED {serial} positional tracking acilamadi: {status}")
            body = sl.BodyTrackingParameters()
            body.detection_model = model_map[args.model]
            body.body_format = sl.BODY_FORMAT.BODY_38
            body.body_selection = sl.BODY_KEYPOINTS_SELECTION.FULL
            body.enable_tracking = False
            body.enable_body_fitting = False
            body.enable_segmentation = False
            body.max_range = 8.0
            body.allow_reduced_precision_inference = args.model == "fast"
            status = camera.enable_body_tracking(body)
            if status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"ZED {serial} BODY_38 acilamadi: {status}")
            communication = sl.CommunicationParameters()
            communication.set_for_shared_memory()
            status = camera.start_publishing(communication)
            if status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"ZED {serial} shared-memory yayini acilamadi: {status}")
            by_serial[serial].communication_parameters = communication
            local_cameras[serial] = camera
            print(f"YEREL HAZIR | ZED {serial} | HD720@{args.fps} BODY_38")

        for serial, (ip, port) in remote.items():
            communication = sl.CommunicationParameters()
            communication.set_for_local_network(port, ip)
            by_serial[serial].communication_parameters = communication
            print(f"UZAK BEKLENIYOR | ZED {serial} | {ip}:{port}")

        fusion_init = sl.InitFusionParameters()
        fusion_init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        fusion_init.coordinate_units = sl.UNIT.METER
        fusion_init.output_performance_metrics = True
        fusion_init.verbose = True
        # HD720@15 source period is ~66.7 ms; 150 ms avoids dropping a valid
        # network body packet merely because it misses the 50 ms SDK default.
        fusion_init.timeout_period_number = 15
        fusion = sl.Fusion()
        status = fusion.init(fusion_init)
        if status != sl.FUSION_ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Fusion baslatilamadi: {status}")

        subscribed = []
        for serial in sorted(expected):
            conf = by_serial[serial]
            identifier = sl.CameraIdentifier()
            identifier.serial_number = serial
            status = fusion.subscribe(identifier, conf.communication_parameters, conf.pose, conf.override_gravity)
            if status != sl.FUSION_ERROR_CODE.SUCCESS:
                raise RuntimeError(f"Fusion ZED {serial} aboneligi basarisiz: {status}")
            subscribed.append(serial)
        params = sl.BodyTrackingFusionParameters()
        params.enable_tracking = True
        params.enable_body_fitting = False
        status = fusion.enable_body_tracking(params)
        if status != sl.FUSION_ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Fusion BODY_38 baslatilamadi: {status}")
        runtime = sl.BodyTrackingFusionRuntimeParameters()
        runtime.skeleton_minimum_allowed_camera = args.min_cameras
        runtime.skeleton_minimum_allowed_keypoints = 7
        runtime.skeleton_smoothing = 0.10
        bodies = sl.Bodies()
        print(f"FUSION HAZIR | kameralar={subscribed} | minimum_katki={args.min_cameras}")

        output = None
        if args.record:
            args.record.parent.mkdir(parents=True, exist_ok=True)
            output = args.record.open("w", encoding="utf-8")
            print(f"Kayit: {args.record}")
        running = True

        def stop(_signal: int, _frame: object) -> None:
            nonlocal running
            running = False

        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        started = time.monotonic()
        last_report = started
        fused_frames = 0
        accepted_frames = 0
        while running and (args.duration <= 0.0 or time.monotonic() - started < args.duration):
            for camera in local_cameras.values():
                camera.grab()
            status = fusion.process()
            if status != sl.FUSION_ERROR_CODE.SUCCESS:
                time.sleep(0.002)
                continue
            fusion.retrieve_bodies(bodies, runtime)
            fused_frames += 1
            if bodies.body_list:
                accepted_frames += 1
            if output:
                record = {
                    "timestamp_ns": time.time_ns(),
                    "body_count": len(bodies.body_list),
                    "bodies": [
                        {
                            "id": int(body.id),
                            "confidence": float(body.confidence),
                            "keypoint_3d_m": finite_list(body.keypoint),
                            "keypoint_confidence": finite_list(body.keypoint_confidence),
                        }
                        for body in bodies.body_list
                    ],
                    "metrics": fusion_metrics(fusion),
                }
                output.write(json.dumps(json_safe(record), allow_nan=False) + "\n")
            now = time.monotonic()
            if now - last_report >= 1.0:
                elapsed = now - last_report
                metrics = fusion_metrics(fusion) or {}
                present = sum(
                    1 for item in metrics.get("cameras", {}).values() if item.get("present")
                )
                print(
                    f"FUSION | fps={fused_frames / elapsed:.1f} | body_frames={accepted_frames / elapsed:.1f} "
                    f"| cameras_present={present}/4 | bodies={len(bodies.body_list)} "
                    f"| mean_fused={metrics.get('mean_camera_fused', math.nan):.2f}"
                )
                fused_frames = 0
                accepted_frames = 0
                last_report = now
            time.sleep(0.001)
        if output:
            output.close()
        fusion.close()
    except RuntimeError as exc:
        print(f"HATA: {exc}", file=sys.stderr)
        return 3
    finally:
        for camera in local_cameras.values():
            camera.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Publish one or more local ZED 2i BODY_38 streams to a Fusion host.

This is deliberately a test-only edge publisher.  It never sends G1 commands,
does not open a Fusion subscriber, and publishes only the SDK Fusion payload
over a wired local network.  A separate host runs ``zed_four_body38_fusion_test``.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from dataclasses import dataclass

import pyzed.sl as sl


@dataclass
class Publisher:
    serial: int
    port: int
    camera: sl.Camera
    bodies: sl.Bodies
    frames: int = 0
    last_report: float = 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Yerel ZED 2i kameralarini ag uzerinden BODY_38 Fusion yayincisi yapar."
    )
    parser.add_argument(
        "--camera",
        action="append",
        required=True,
        metavar="SERIAL:PORT",
        help="Her yerel kamera icin seri numarasi ve benzersiz, cift UDP portu. Ornek: 12345678:30000",
    )
    parser.add_argument("--fps", type=int, choices=(15, 30, 60), default=15)
    parser.add_argument("--model", choices=("fast", "medium", "accurate"), default="fast")
    parser.add_argument(
        "--depth-mode",
        choices=("performance", "neural-light", "neural", "ultra"),
        default="neural-light",
    )
    parser.add_argument("--duration", type=float, default=0.0, help="0 sonsuz calisma demektir.")
    return parser.parse_args()


def parse_camera(value: str) -> tuple[int, int]:
    try:
        serial_text, port_text = value.split(":", maxsplit=1)
        serial, port = int(serial_text), int(port_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Kamera bicimi SERIAL:PORT olmali.") from exc
    if serial <= 0 or not 1 <= port <= 65535 or port % 2:
        raise argparse.ArgumentTypeError("Seri pozitif; port 1-65535 araliginda ve cift olmali.")
    return serial, port


def main() -> int:
    args = parse_args()
    try:
        requested = [parse_camera(item) for item in args.camera]
    except argparse.ArgumentTypeError as exc:
        print(f"Gecersiz --camera: {exc}", file=sys.stderr)
        return 2
    if len({serial for serial, _ in requested}) != len(requested):
        print("Ayni kamera seri numarasi birden fazla kez verildi.", file=sys.stderr)
        return 2
    if len({port for _, port in requested}) != len(requested):
        print("Her kamera icin farkli bir ag portu gerekir.", file=sys.stderr)
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

    publishers: list[Publisher] = []
    for serial, port in requested:
        init = sl.InitParameters()
        init.set_from_serial_number(serial)
        init.camera_resolution = sl.RESOLUTION.HD720
        init.camera_fps = args.fps
        init.depth_mode = depth_map[args.depth_mode]
        init.coordinate_units = sl.UNIT.METER
        init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init.depth_maximum_distance = 8.0
        if hasattr(init, "async_grab_camera_recovery"):
            init.async_grab_camera_recovery = True

        camera = sl.Camera()
        status = camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(f"ZED {serial} acilamadi: {status}", file=sys.stderr)
            continue

        positional = sl.PositionalTrackingParameters()
        positional.set_as_static = True
        status = camera.enable_positional_tracking(positional)
        if status != sl.ERROR_CODE.SUCCESS:
            print(f"ZED {serial} positional tracking acilamadi: {status}", file=sys.stderr)
            camera.close()
            continue

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
            print(f"ZED {serial} BODY_38 acilamadi: {status}", file=sys.stderr)
            camera.close()
            continue

        communication = sl.CommunicationParameters()
        communication.set_for_local_network(port)
        status = camera.start_publishing(communication)
        if status != sl.ERROR_CODE.SUCCESS:
            print(f"ZED {serial} ag yayini acilamadi: {status}", file=sys.stderr)
            camera.close()
            continue

        publishers.append(Publisher(serial, port, camera, sl.Bodies(), last_report=time.monotonic()))
        print(f"HAZIR | ZED {serial} | BODY_38 | UDP port {port} | HD720@{args.fps}")

    if not publishers:
        print("Hicbir yerel kamera yayina baslayamadi.", file=sys.stderr)
        return 3

    running = True

    def stop(_signal: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    runtime = sl.BodyTrackingRuntimeParameters()
    started = time.monotonic()
    try:
        while running and (args.duration <= 0.0 or time.monotonic() - started < args.duration):
            for publisher in publishers:
                status = publisher.camera.grab()
                if status == sl.ERROR_CODE.SUCCESS:
                    publisher.camera.retrieve_bodies(publisher.bodies, runtime)
                    publisher.frames += 1
                now = time.monotonic()
                if now - publisher.last_report >= 1.0:
                    elapsed = max(now - publisher.last_report, 1.0e-6)
                    print(
                        f"ZED {publisher.serial} | publisher_fps={publisher.frames / elapsed:.1f} "
                        f"| bodies={len(publisher.bodies.body_list)} | port={publisher.port}"
                    )
                    publisher.frames = 0
                    publisher.last_report = now
            time.sleep(0.001)
    finally:
        for publisher in publishers:
            publisher.camera.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

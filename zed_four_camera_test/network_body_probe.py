#!/usr/bin/env python3
"""Check raw body payloads received from network ZED Fusion publishers.

This is a diagnostic receiver, not a calibrator.  It subscribes to all camera
publishers over the local network and reports the raw body count that the ZED
Fusion SDK can actually retrieve for each individual camera.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

import pyzed.sl as sl


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Agdaki ZED publisher'lardan gelen ham iskeletleri dogrudan kontrol eder."
    )
    parser.add_argument(
        "--camera",
        action="append",
        required=True,
        metavar="SERIAL@IP:PORT",
        help="Ornek: 12345678@192.168.50.11:30000",
    )
    parser.add_argument("--duration", type=float, default=20.0)
    return parser.parse_args()


def parse_camera(value: str) -> tuple[int, str, int]:
    try:
        serial_text, endpoint = value.split("@", maxsplit=1)
        ip, port_text = endpoint.rsplit(":", maxsplit=1)
        serial, port = int(serial_text), int(port_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Kamera bicimi SERIAL@IP:PORT olmali.") from exc
    if serial <= 0 or not ip or not 1 <= port <= 65535 or port % 2:
        raise argparse.ArgumentTypeError("Seri/IP/port gecersiz; port cift olmali.")
    return serial, ip, port


def main() -> int:
    args = parse_args()
    try:
        cameras = [parse_camera(item) for item in args.camera]
    except argparse.ArgumentTypeError as exc:
        print(f"HATA: {exc}", file=sys.stderr)
        return 2
    if len({serial for serial, _, _ in cameras}) != len(cameras):
        print("HATA: Ayni seri numarasi birden fazla kez verildi.", file=sys.stderr)
        return 2

    fusion = sl.Fusion()
    init = sl.InitFusionParameters()
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init.coordinate_units = sl.UNIT.METER
    init.output_performance_metrics = True
    init.verbose = True
    status = fusion.init(init)
    if status != sl.FUSION_ERROR_CODE.SUCCESS:
        print(f"HATA: Fusion baslatilamadi: {status}", file=sys.stderr)
        return 3

    identifiers: dict[int, sl.CameraIdentifier] = {}
    for serial, ip, port in cameras:
        communication = sl.CommunicationParameters()
        communication.set_for_local_network(port, ip)
        identifier = sl.CameraIdentifier()
        identifier.serial_number = serial
        status = fusion.subscribe(identifier, communication, sl.Transform())
        if status != sl.FUSION_ERROR_CODE.SUCCESS:
            print(f"HATA: {serial} aboneligi basarisiz: {status}", file=sys.stderr)
            fusion.close()
            return 3
        identifiers[serial] = identifier
        print(f"ABONE | ZED {serial} | {ip}:{port}")

    body_params = sl.BodyTrackingFusionParameters()
    body_params.enable_tracking = False
    body_params.enable_body_fitting = False
    status = fusion.enable_body_tracking(body_params)
    if status != sl.FUSION_ERROR_CODE.SUCCESS:
        print(f"HATA: Fusion Body Tracking baslatilamadi: {status}", file=sys.stderr)
        fusion.close()
        return 3

    runtime = sl.BodyTrackingFusionRuntimeParameters()
    runtime.skeleton_minimum_allowed_camera = 1
    runtime.skeleton_minimum_allowed_keypoints = 7
    running = True

    def stop(_signal: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    started = time.monotonic()
    last_report = 0.0
    new_data_frames = 0
    try:
        while running and time.monotonic() - started < args.duration:
            process_status = fusion.process()
            if process_status == sl.FUSION_ERROR_CODE.SUCCESS:
                new_data_frames += 1
            now = time.monotonic()
            if now - last_report >= 1.0:
                counts: list[str] = []
                for serial, identifier in identifiers.items():
                    bodies = sl.Bodies()
                    retrieve_status = fusion.retrieve_bodies(bodies, runtime, identifier)
                    counts.append(f"{serial}={len(bodies.body_list)} ({retrieve_status})")
                print(
                    f"PROBE | fusion_new_frames={new_data_frames} | raw_bodies: "
                    + ", ".join(counts)
                )
                new_data_frames = 0
                last_report = now
            time.sleep(0.001)
    finally:
        fusion.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

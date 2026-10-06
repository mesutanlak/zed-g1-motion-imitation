#!/usr/bin/env python3
"""Capture one high-resolution ZED SVO2 for a fixed planar reference board."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import pyzed.sl as sl


RESOLUTIONS = {
    "hd720": sl.RESOLUTION.HD720,
    "hd1080": sl.RESOLUTION.HD1080,
    "hd2k": sl.RESOLUTION.HD2K,
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resolution", choices=tuple(RESOLUTIONS), default="hd720")
    parser.add_argument("--fps", type=int, choices=(15, 30, 60), default=30)
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument(
        "--compression", choices=("h265", "h264", "lossless"), default="h265"
    )
    args = parser.parse_args()
    if args.seconds < 5.0:
        raise SystemExit("--seconds en az 5 olmali")
    allowed_fps = {
        "hd2k": {15},
        "hd1080": {15, 30},
        "hd720": {15, 30, 60},
    }
    if args.fps not in allowed_fps[args.resolution]:
        raise SystemExit(
            f"{args.resolution} icin desteklenmeyen FPS: {args.fps}; "
            f"izin verilen={sorted(allowed_fps[args.resolution])}"
        )

    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise SystemExit(f"Var olan SVO2 dosyasinin ustune yazilmaz: {output}")

    init = sl.InitParameters()
    init.set_from_serial_number(args.serial)
    init.camera_resolution = RESOLUTIONS[args.resolution]
    init.camera_fps = args.fps
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.sdk_verbose = 1
    if hasattr(init, "async_grab_camera_recovery"):
        init.async_grab_camera_recovery = True

    camera = sl.Camera()
    opened = camera.open(init)
    if opened != sl.ERROR_CODE.SUCCESS:
        raise SystemExit(f"ZED {args.serial} acilamadi: {opened}")

    compression = {
        "h265": sl.SVO_COMPRESSION_MODE.H265,
        "h264": sl.SVO_COMPRESSION_MODE.H264,
        "lossless": sl.SVO_COMPRESSION_MODE.LOSSLESS,
    }[args.compression]
    recording_enabled = False
    frames = 0
    started_unix_ns = time.time_ns()
    started = time.monotonic()
    clean_shutdown = False
    try:
        result = camera.enable_recording(sl.RecordingParameters(str(output), compression))
        if result != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"SVO2 kaydi baslatilamadi: {result}")
        recording_enabled = True
        information = camera.get_camera_information()
        configuration = information.camera_configuration
        print(
            f"REFERENCE SVO2 ACILDI | serial={args.serial} "
            f"resolution={configuration.resolution.width}x{configuration.resolution.height} "
            f"fps={configuration.fps:g} duration={args.seconds:g}s | {output}"
        )
        while time.monotonic() - started < args.seconds:
            status = camera.grab()
            if status == sl.ERROR_CODE.SUCCESS:
                frames += 1
            else:
                print(f"ZED {args.serial} grab uyarisi: {status}")
        clean_shutdown = True
    except KeyboardInterrupt:
        clean_shutdown = True
        print(f"ZED {args.serial}: kullanici durdurdu; SVO2 guvenli kapatiliyor")
    finally:
        if recording_enabled:
            camera.disable_recording()
        camera.close()

    metadata = {
        "schema": "zed_fixed_reference_capture/v1",
        "serial": args.serial,
        "svo2": str(output),
        "resolution": args.resolution,
        "fps": args.fps,
        "requested_seconds": args.seconds,
        "captured_frames": frames,
        "started_unix_ns": started_unix_ns,
        "ended_unix_ns": time.time_ns(),
        "clean_shutdown": clean_shutdown,
        "board": {
            "target_type": "checkerboard",
            "inner_corners": [17, 12],
            "square_m": 0.02,
            "outer_pattern_m": [0.36, 0.26],
        },
    }
    metadata_path = output.with_suffix(".capture.json")
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(f"REFERENCE SVO2 KAPANDI | serial={args.serial} frames={frames} | {output}")
    return 0 if frames >= max(1, int(args.fps * args.seconds * 0.8)) else 1


if __name__ == "__main__":
    raise SystemExit(main())

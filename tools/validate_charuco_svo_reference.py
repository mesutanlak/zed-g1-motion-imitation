#!/usr/bin/env python3
"""Validate ZED extrinsics and metric scale against a static ChArUco board.

This is deliberately offline: it cannot steal GPU/USB time from live control.
The board must remain fixed while all supplied SVO2 recordings are captured.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
from typing import Any

import cv2
import numpy as np
import pyzed.sl as sl


def serial_from_path(path: Path) -> int:
    match = re.search(r"_(\d{8})_", path.name)
    if not match:
        raise ValueError(f"Dosya adindan ZED seri numarasi okunamadi: {path.name}")
    return int(match.group(1))


def transform_from_extrinsics(document: dict[str, Any], serial: int) -> np.ndarray:
    item = document["cameras"][str(serial)]
    value = np.eye(4, dtype=np.float64)
    value[:3, :3] = np.asarray(item["rotation_camera_to_world"], dtype=np.float64)
    value[:3, 3] = np.asarray(item["translation_camera_to_world_m"], dtype=np.float64)
    return value


def rotation_error_deg(first: np.ndarray, second: np.ndarray) -> float:
    relative = first.T @ second
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("svo2", type=Path, nargs="+")
    parser.add_argument("--extrinsics", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--squares-x", type=int, default=7)
    parser.add_argument("--squares-y", type=int, default=5)
    parser.add_argument("--square-mm", type=float, default=55.0)
    parser.add_argument("--marker-mm", type=float, default=41.0)
    parser.add_argument("--frame-stride", type=int, default=15)
    parser.add_argument("--minimum-corners", type=int, default=8)
    args = parser.parse_args()

    extrinsics = json.loads(args.extrinsics.read_text(encoding="utf-8-sig"))
    if extrinsics.get("schema") != "zed_body38_distributed_extrinsics/v1":
        raise SystemExit("Desteklenmeyen extrinsic semasi")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    board = cv2.aruco.CharucoBoard(
        (args.squares_x, args.squares_y),
        args.square_mm / 1000.0,
        args.marker_mm / 1000.0,
        dictionary,
    )
    detector = cv2.aruco.CharucoDetector(board)
    board_points = board.getChessboardCorners().reshape(-1, 3)
    observations: list[dict[str, Any]] = []

    for svo_path in args.svo2:
        serial = serial_from_path(svo_path)
        if str(serial) not in extrinsics.get("cameras", {}):
            raise SystemExit(f"Extrinsicte kamera yok: {serial}")
        input_type = sl.InputType()
        input_type.set_from_svo_file(str(svo_path.resolve()))
        init = sl.InitParameters(input_t=input_type, svo_real_time_mode=False)
        init.depth_mode = sl.DEPTH_MODE.NONE
        camera = sl.Camera()
        status = camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise SystemExit(f"SVO acilamadi {svo_path}: {status}")
        try:
            information = camera.get_camera_information()
            calibration = information.camera_configuration.calibration_parameters.left_cam
            matrix = np.asarray([
                [calibration.fx, 0.0, calibration.cx],
                [0.0, calibration.fy, calibration.cy],
                [0.0, 0.0, 1.0],
            ], dtype=np.float64)
            distortion = np.zeros(5, dtype=np.float64)  # VIEW.LEFT is rectified
            image = sl.Mat()
            frame = 0
            while camera.grab() == sl.ERROR_CODE.SUCCESS:
                if frame % max(1, args.frame_stride):
                    frame += 1
                    continue
                camera.retrieve_image(image, sl.VIEW.LEFT)
                bgr = np.asarray(image.get_data())[:, :, :3]
                corners, ids, _marker_corners, _marker_ids = detector.detectBoard(bgr)
                if ids is not None and len(ids) >= args.minimum_corners:
                    image_points = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
                    object_points = board_points[np.asarray(ids).reshape(-1)]
                    ok, rvec, tvec = cv2.solvePnP(
                        object_points,
                        image_points,
                        matrix,
                        distortion,
                        flags=cv2.SOLVEPNP_IPPE,
                    )
                    if ok:
                        rotation, _ = cv2.Rodrigues(rvec)
                        camera_from_board = np.eye(4, dtype=np.float64)
                        camera_from_board[:3, :3] = rotation
                        camera_from_board[:3, 3] = np.asarray(tvec).reshape(3)
                        world_from_board = (
                            transform_from_extrinsics(extrinsics, serial)
                            @ camera_from_board
                        )
                        projected, _ = cv2.projectPoints(
                            object_points, rvec, tvec, matrix, distortion
                        )
                        reprojection = np.linalg.norm(
                            projected.reshape(-1, 2) - image_points, axis=1
                        )
                        observations.append({
                            "serial": serial,
                            "svo2": str(svo_path.resolve()),
                            "frame": frame,
                            "corner_count": int(len(ids)),
                            "reprojection_rms_px": float(
                                np.sqrt(np.mean(np.square(reprojection)))
                            ),
                            "world_from_board": world_from_board.tolist(),
                        })
                frame += 1
        finally:
            camera.close()

    if not observations:
        raise SystemExit("ChArUco algilanamadi; board gorunurlugunu ve olculeri kontrol edin")
    translations = np.asarray(
        [item["world_from_board"] for item in observations], dtype=np.float64
    )[:, :3, 3]
    reference_translation = np.median(translations, axis=0)
    translation_errors = np.linalg.norm(translations - reference_translation, axis=1)
    rotations = np.asarray(
        [item["world_from_board"] for item in observations], dtype=np.float64
    )[:, :3, :3]
    reference_rotation = rotations[0]
    rotation_errors = np.asarray(
        [rotation_error_deg(reference_rotation, value) for value in rotations]
    )
    report = {
        "schema": "zed_charuco_metric_reference/v1",
        "board": {
            "dictionary": "DICT_5X5_1000",
            "squares": [args.squares_x, args.squares_y],
            "square_m": args.square_mm / 1000.0,
            "marker_m": args.marker_mm / 1000.0,
        },
        "extrinsics": str(args.extrinsics.resolve()),
        "observation_count": len(observations),
        "camera_count": len({item["serial"] for item in observations}),
        "translation_error_m": {
            "median": float(np.median(translation_errors)),
            "p95": float(np.percentile(translation_errors, 95)),
            "max": float(np.max(translation_errors)),
        },
        "rotation_error_deg": {
            "median": float(np.median(rotation_errors)),
            "p95": float(np.percentile(rotation_errors, 95)),
            "max": float(np.max(rotation_errors)),
        },
        "reprojection_rms_px": {
            "median": float(np.median([item["reprojection_rms_px"] for item in observations])),
            "p95": float(np.percentile([item["reprojection_rms_px"] for item in observations], 95)),
        },
        "observations": observations,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key != "observations"}, indent=2))
    print(f"Rapor: {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

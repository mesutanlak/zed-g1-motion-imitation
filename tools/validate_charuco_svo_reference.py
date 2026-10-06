#!/usr/bin/env python3
"""Validate ZED extrinsics and metric scale against a static planar board.

This is deliberately offline: it cannot steal GPU/USB time from live control.
The board must remain fixed while all supplied SVO2 recordings are captured.
Both ChArUco and complete checkerboard targets are supported.
"""

from __future__ import annotations

import argparse
import json
import math
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


def zed_from_opencv_optical() -> np.ndarray:
    """Map OpenCV image/PnP axes to ZED RIGHT_HANDED_Z_UP_X_FWD axes."""
    value = np.eye(4, dtype=np.float64)
    # OpenCV optical: X right, Y down, Z forward.
    # ZED configured source: X forward, Y left, Z up.
    value[:3, :3] = np.asarray(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        dtype=np.float64,
    )
    return value


def rotation_error_deg(first: np.ndarray, second: np.ndarray) -> float:
    relative = first.T @ second
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def rotation_medoid(rotations: np.ndarray) -> np.ndarray:
    """Return an observed rotation with the smallest total geodesic error."""
    totals = [
        sum(rotation_error_deg(candidate, other) for other in rotations)
        for candidate in rotations
    ]
    return rotations[int(np.argmin(totals))]


def checkerboard_object_points(
    corners_x: int, corners_y: int, square_m: float
) -> np.ndarray:
    points = np.zeros((corners_x * corners_y, 3), dtype=np.float64)
    points[:, :2] = (
        np.mgrid[0:corners_x, 0:corners_y].T.reshape(-1, 2) * square_m
    )
    return points


def order_quad_points(points: np.ndarray) -> np.ndarray:
    """Order a convex image quadrilateral clockwise, starting near top-left."""
    value = np.asarray(points, dtype=np.float32).reshape(4, 2)
    center = np.mean(value, axis=0)
    angles = np.arctan2(value[:, 1] - center[1], value[:, 0] - center[0])
    value = value[np.argsort(angles)]
    return np.roll(value, -int(np.argmin(np.sum(value, axis=1))), axis=0)


def detect_board_outer_quad(
    bgr: np.ndarray,
    roi_normalized: list[float],
    expected_area_fraction: float,
    expected_quad_normalized: list[list[float]] | None = None,
) -> np.ndarray | None:
    """Detect the fixed white board boundary inside a rig-specific ROI.

    This is deliberately a coarse fallback. It is useful for displacement
    monitoring, but it must never be treated as a full-corner metric target.
    """
    height, width = bgr.shape[:2]
    if len(roi_normalized) != 4:
        return None
    x0, y0, x1, y1 = (
        np.asarray(roi_normalized, dtype=np.float64)
        * np.asarray([width, height, width, height], dtype=np.float64)
    ).astype(int)
    x0, x1 = sorted((max(0, x0), min(width, x1)))
    y0, y1 = sorted((max(0, y0), min(height, y1)))
    if x1 - x0 < 32 or y1 - y0 < 32:
        return None
    gray = cv2.cvtColor(bgr[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    expected_area = expected_area_fraction * width * height
    best: tuple[float, np.ndarray] | None = None
    for threshold in (210, 200, 190, 180, 170):
        mask = cv2.inRange(gray, threshold, 255)
        for nominal_kernel in (9, 15, 21):
            kernel_size = max(5, round(nominal_kernel * width / 1280))
            kernel_size += 1 - kernel_size % 2
            closed = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                cv2.getStructuringElement(
                    cv2.MORPH_RECT, (kernel_size, kernel_size)
                ),
            )
            contours, _ = cv2.findContours(
                closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for contour in contours:
                area = float(cv2.contourArea(contour))
                if not 0.003 * width * height <= area <= 0.015 * width * height:
                    continue
                perimeter = cv2.arcLength(contour, True)
                quad = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
                if len(quad) != 4 or not cv2.isContourConvex(quad):
                    continue
                rect_width, rect_height = cv2.minAreaRect(quad)[1]
                aspect = max(rect_width, rect_height) / max(
                    1.0, min(rect_width, rect_height)
                )
                if not 1.4 <= aspect <= 4.0:
                    continue
                score = abs(math.log(area / expected_area)) + 0.002 * abs(
                    threshold - 195
                )
                points = quad.reshape(-1, 2).astype(np.float32)
                points += np.asarray([x0, y0], dtype=np.float32)
                ordered = order_quad_points(points)
                if expected_quad_normalized:
                    expected = np.asarray(
                        expected_quad_normalized, dtype=np.float32
                    ) * np.asarray([width, height], dtype=np.float32)
                    normalized_rms = float(
                        np.sqrt(np.mean(np.square(ordered - expected)))
                        / np.hypot(width, height)
                    )
                    # The board is permanently fixed. Prefer the expected image
                    # neighborhood strongly enough that another bright floor or
                    # furniture quadrilateral cannot win on area alone.
                    score += 250.0 * normalized_rms
                if best is None or score < best[0]:
                    best = (score, ordered)
    return None if best is None else best[1]


def main(default_board_config: Path | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("svo2", type=Path, nargs="+")
    parser.add_argument("--extrinsics", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--board-config", type=Path, default=default_board_config)
    parser.add_argument(
        "--target-type",
        choices=("charuco", "checkerboard"),
        default=None,
    )
    parser.add_argument("--squares-x", type=int, default=None)
    parser.add_argument("--squares-y", type=int, default=None)
    parser.add_argument("--corners-x", type=int, default=None)
    parser.add_argument("--corners-y", type=int, default=None)
    parser.add_argument("--square-mm", type=float, default=None)
    parser.add_argument("--marker-mm", type=float, default=None)
    parser.add_argument("--frame-stride", type=int, default=15)
    parser.add_argument("--minimum-corners", type=int, default=8)
    args = parser.parse_args()

    board_config: dict[str, Any] = {}
    if args.board_config is not None:
        board_config = json.loads(
            args.board_config.expanduser().read_text(encoding="utf-8-sig")
        )
        if board_config.get("schema") != "zed_fixed_planar_reference/v1":
            raise SystemExit("Desteklenmeyen sabit referans board semasi")
    inner_corners = board_config.get("inner_corners") or [17, 12]
    args.target_type = args.target_type or board_config.get("target_type") or "charuco"
    args.squares_x = args.squares_x or 7
    args.squares_y = args.squares_y or 5
    args.corners_x = args.corners_x or int(inner_corners[0])
    args.corners_y = args.corners_y or int(inner_corners[1])
    args.square_mm = (
        args.square_mm
        if args.square_mm is not None
        else float(board_config.get("square_m", 0.055)) * 1000.0
    )
    args.marker_mm = args.marker_mm if args.marker_mm is not None else 41.0
    detection_config = board_config.get("detection") or {}
    roi_by_serial = detection_config.get("roi_normalized_by_serial") or {}
    expected_quad_by_serial = (
        detection_config.get("expected_outer_quad_normalized_by_serial") or {}
    )
    outer_pattern = board_config.get("outer_pattern_m") or [
        (args.corners_x + 1) * args.square_mm / 1000.0,
        (args.corners_y + 1) * args.square_mm / 1000.0,
    ]
    outer_object_points = np.asarray(
        [
            [-float(outer_pattern[0]) / 2.0, -float(outer_pattern[1]) / 2.0, 0.0],
            [float(outer_pattern[0]) / 2.0, -float(outer_pattern[1]) / 2.0, 0.0],
            [float(outer_pattern[0]) / 2.0, float(outer_pattern[1]) / 2.0, 0.0],
            [-float(outer_pattern[0]) / 2.0, float(outer_pattern[1]) / 2.0, 0.0],
        ],
        dtype=np.float64,
    )

    extrinsics = json.loads(args.extrinsics.read_text(encoding="utf-8-sig"))
    if extrinsics.get("schema") != "zed_body38_distributed_extrinsics/v1":
        raise SystemExit("Desteklenmeyen extrinsic semasi")
    if args.square_mm <= 0.0:
        raise SystemExit("--square-mm pozitif olmali")
    dictionary = None
    detector = None
    if args.target_type == "charuco":
        if not 0.0 < args.marker_mm < args.square_mm:
            raise SystemExit("--marker-mm pozitif ve --square-mm degerinden kucuk olmali")
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        board = cv2.aruco.CharucoBoard(
            (args.squares_x, args.squares_y),
            args.square_mm / 1000.0,
            args.marker_mm / 1000.0,
            dictionary,
        )
        detector = cv2.aruco.CharucoDetector(board)
        board_points = board.getChessboardCorners().reshape(-1, 3)
    else:
        if args.corners_x < 3 or args.corners_y < 3:
            raise SystemExit("Checkerboard icin en az 3x3 ic kose gerekli")
        board_points = checkerboard_object_points(
            args.corners_x, args.corners_y, args.square_mm / 1000.0
        )
    observations: list[dict[str, Any]] = []
    attempted_frames: dict[int, int] = {}
    detected_frames: dict[int, int] = {}

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
                attempted_frames[serial] = attempted_frames.get(serial, 0) + 1
                if args.target_type == "charuco":
                    assert detector is not None
                    corners, ids, _marker_corners, _marker_ids = detector.detectBoard(bgr)
                    detected = ids is not None and len(ids) >= args.minimum_corners
                    if detected:
                        image_points = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
                        object_points = board_points[np.asarray(ids).reshape(-1)]
                        corner_count = int(len(ids))
                else:
                    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                    detected, corners = cv2.findChessboardCornersSB(
                        gray,
                        (args.corners_x, args.corners_y),
                        flags=(
                            cv2.CALIB_CB_NORMALIZE_IMAGE
                            | cv2.CALIB_CB_EXHAUSTIVE
                            | cv2.CALIB_CB_ACCURACY
                        ),
                    )
                    if detected:
                        image_points = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
                        object_points = board_points
                        corner_count = int(len(image_points))
                        detection_kind = "full_checkerboard"
                    elif bool(detection_config.get("outer_quad_fallback")):
                        quad = detect_board_outer_quad(
                            bgr,
                            roi_by_serial.get(str(serial), []),
                            float(detection_config.get("expected_area_fraction", 0.006)),
                            expected_quad_by_serial.get(str(serial)),
                        )
                        if quad is not None:
                            detected = True
                            image_points = np.asarray(quad, dtype=np.float64)
                            object_points = outer_object_points
                            corner_count = 4
                            detection_kind = "outer_quad_coarse"
                if args.target_type == "charuco" and detected:
                    detection_kind = "charuco"
                if detected:
                    detected_frames[serial] = detected_frames.get(serial, 0) + 1
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
                            @ zed_from_opencv_optical()
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
                            "corner_count": corner_count,
                            "detection_kind": detection_kind,
                            "reprojection_rms_px": float(
                                np.sqrt(np.mean(np.square(reprojection)))
                            ),
                            "world_from_board": world_from_board.tolist(),
                            "outer_quad_image_points_px": (
                                image_points.tolist()
                                if detection_kind == "outer_quad_coarse"
                                else None
                            ),
                        })
                frame += 1
        finally:
            camera.close()

    if not observations:
        raise SystemExit(
            f"{args.target_type} algilanamadi; board gorunurlugunu, cozunurlugu "
            "ve olculeri kontrol edin"
        )
    translations = np.asarray(
        [item["world_from_board"] for item in observations], dtype=np.float64
    )[:, :3, 3]
    reference_translation = np.median(translations, axis=0)
    translation_errors = np.linalg.norm(translations - reference_translation, axis=1)
    rotations = np.asarray(
        [item["world_from_board"] for item in observations], dtype=np.float64
    )[:, :3, :3]
    reference_rotation = rotation_medoid(rotations)
    rotation_errors = np.asarray(
        [rotation_error_deg(reference_rotation, value) for value in rotations]
    )
    reference_normal = reference_rotation[:, 2]
    normal_errors = np.asarray([
        float(
            np.degrees(
                np.arccos(
                    np.clip(abs(float(np.dot(reference_normal, value[:, 2]))), 0.0, 1.0)
                )
            )
        )
        for value in rotations
    ])
    per_camera = {}
    for serial in sorted(attempted_frames):
        camera_observations = [item for item in observations if item["serial"] == serial]
        if not camera_observations:
            per_camera[str(serial)] = {
                "attempted_frames": attempted_frames[serial],
                "detected_frames": 0,
                "detection_ratio": 0.0,
                "reprojection_rms_px_median": None,
                "detection_kinds": [],
                "world_board_center_median_m": None,
                "temporal_translation_jitter_p95_m": None,
                "outer_quad_median_px": None,
            }
            continue
        reprojection = [item["reprojection_rms_px"] for item in camera_observations]
        camera_translations = np.asarray(
            [item["world_from_board"] for item in camera_observations],
            dtype=np.float64,
        )[:, :3, 3]
        camera_translation_median = np.median(camera_translations, axis=0)
        camera_jitter = np.linalg.norm(
            camera_translations - camera_translation_median, axis=1
        )
        camera_quads = [
            item["outer_quad_image_points_px"]
            for item in camera_observations
            if item["outer_quad_image_points_px"] is not None
        ]
        per_camera[str(serial)] = {
            "attempted_frames": attempted_frames[serial],
            "detected_frames": detected_frames.get(serial, 0),
            "detection_ratio": detected_frames.get(serial, 0) / attempted_frames[serial],
            "reprojection_rms_px_median": (
                float(np.median(reprojection)) if reprojection else None
            ),
            "detection_kinds": sorted(
                {item["detection_kind"] for item in camera_observations}
            ),
            "world_board_center_median_m": camera_translation_median.tolist(),
            "temporal_translation_jitter_p95_m": float(
                np.percentile(camera_jitter, 95)
            ),
            "outer_quad_median_px": (
                np.median(np.asarray(camera_quads), axis=0).tolist()
                if camera_quads
                else None
            ),
        }
    board_description = {
        "target_type": args.target_type,
        "square_m": args.square_mm / 1000.0,
    }
    if args.target_type == "charuco":
        board_description.update({
            "dictionary": "DICT_5X5_1000",
            "squares": [args.squares_x, args.squares_y],
            "marker_m": args.marker_mm / 1000.0,
        })
    else:
        board_description.update({
            "inner_corners": [args.corners_x, args.corners_y],
            "outer_pattern_m": [
                (args.corners_x + 1) * args.square_mm / 1000.0,
                (args.corners_y + 1) * args.square_mm / 1000.0,
            ],
        })
    report = {
        "schema": "zed_planar_metric_reference/v2",
        "board": board_description,
        "board_config": (
            str(args.board_config.expanduser().resolve())
            if args.board_config is not None
            else None
        ),
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
        "plane_normal_error_deg": {
            "median": float(np.median(normal_errors)),
            "p95": float(np.percentile(normal_errors, 95)),
            "max": float(np.max(normal_errors)),
        },
        "reprojection_rms_px": {
            "median": float(np.median([item["reprojection_rms_px"] for item in observations])),
            "p95": float(np.percentile([item["reprojection_rms_px"] for item in observations], 95)),
        },
        "world_from_board_reference": {
            "translation_m": reference_translation.tolist(),
            "rotation": reference_rotation.tolist(),
        },
        "metric_grade": (
            "FULL_CORNER"
            if all(item["detection_kind"] != "outer_quad_coarse" for item in observations)
            else "OUTER_QUAD_COARSE"
        ),
        "per_camera": per_camera,
        "observations": observations,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key != "observations"}, indent=2))
    print(f"Rapor: {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

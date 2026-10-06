from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np


TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

from validate_charuco_svo_reference import (  # noqa: E402
    checkerboard_object_points,
    detect_board_outer_quad,
    order_quad_points,
    rotation_medoid,
    zed_from_opencv_optical,
)


def test_checkerboard_metric_geometry_matches_fixed_board() -> None:
    points = checkerboard_object_points(17, 12, 0.020)
    assert points.shape == (204, 3)
    assert np.allclose(points.min(axis=0), [0.0, 0.0, 0.0])
    assert np.allclose(points.max(axis=0), [0.320, 0.220, 0.0])


def test_rotation_medoid_rejects_single_outlier() -> None:
    identity = np.eye(3)
    near, _ = cv2.Rodrigues(np.asarray([0.0, 0.0, 0.01]))
    outlier, _ = cv2.Rodrigues(np.asarray([0.0, 0.0, 1.2]))
    selected = rotation_medoid(np.asarray([identity, near, outlier]))
    assert np.allclose(selected, near)


def test_optical_to_zed_axis_mapping() -> None:
    rotation = zed_from_opencv_optical()[:3, :3]
    assert np.allclose(rotation @ [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
    assert np.allclose(rotation @ [1.0, 0.0, 0.0], [0.0, -1.0, 0.0])
    assert np.allclose(rotation @ [0.0, 1.0, 0.0], [0.0, 0.0, -1.0])


def test_quad_order_is_unique_and_clockwise() -> None:
    points = np.asarray([[90, 80], [10, 10], [80, 10], [20, 90]], np.float32)
    ordered = order_quad_points(points)
    assert len({tuple(value) for value in ordered}) == 4
    assert np.allclose(ordered[0], [10, 10])


def test_fixed_board_outer_quad_detector() -> None:
    image = np.full((720, 1280, 3), 90, dtype=np.uint8)
    quad = np.asarray([[760, 590], [940, 600], [930, 665], [745, 650]], np.int32)
    cv2.fillConvexPoly(image, quad, (245, 245, 245))
    for x in range(765, 925, 18):
        for y in range(600, 650, 14):
            cv2.circle(image, (x, y), 3, (20, 20, 20), -1)
    expected = (quad / np.asarray([1280.0, 720.0])).tolist()
    detected = detect_board_outer_quad(
        image, [0.54, 0.78, 0.76, 0.98], 0.006, expected
    )
    assert detected is not None
    assert detected.shape == (4, 2)

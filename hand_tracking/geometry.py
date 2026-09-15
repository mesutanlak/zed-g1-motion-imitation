from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    distortion: tuple[float, ...] = ()

    @classmethod
    def from_mapping(cls, value: dict) -> "CameraIntrinsics":
        size = value.get("image_size") or {}
        return cls(
            fx=float(value["fx"]), fy=float(value["fy"]),
            cx=float(value["cx"]), cy=float(value["cy"]),
            width=int(size.get("width", 1280)), height=int(size.get("height", 720)),
            distortion=tuple(float(x) for x in (value.get("distortion") or [])),
        )


def deproject_pixel(pixel: Sequence[float], depth_m: float, intrinsics: CameraIntrinsics) -> np.ndarray:
    if not np.isfinite(depth_m) or depth_m <= 0.0:
        raise ValueError("depth must be finite and positive")
    u, v = map(float, pixel[:2])
    # BODY_38 keypoint pixels and retrieved left depth are already rectified by
    # the ZED SDK, so pinhole deprojection is correct for this buffer.
    return np.array([
        depth_m,
        -(u - intrinsics.cx) * depth_m / intrinsics.fx,
        -(v - intrinsics.cy) * depth_m / intrinsics.fy,
    ])


def camera_ray(pixel: Sequence[float], intrinsics: CameraIntrinsics) -> np.ndarray:
    point = deproject_pixel(pixel, 1.0, intrinsics)
    return point / np.linalg.norm(point)


def robust_depth_patch(
    depth_image_m: np.ndarray,
    pixel: Sequence[float],
    *,
    wrist_depth_m: float | None,
    radius_px: int = 3,
    maximum_wrist_delta_m: float = 0.35,
    minimum_m: float = 0.2,
    maximum_m: float = 8.0,
) -> tuple[float | None, float]:
    """Median/trimmed patch sampling rejects wall/background pixels."""
    image = np.asarray(depth_image_m, dtype=float)
    if image.ndim != 2:
        return None, 0.0
    u, v = (int(round(float(x))) for x in pixel[:2])
    y0, y1 = max(0, v - radius_px), min(image.shape[0], v + radius_px + 1)
    x0, x1 = max(0, u - radius_px), min(image.shape[1], u + radius_px + 1)
    values = image[y0:y1, x0:x1].reshape(-1)
    valid = np.isfinite(values) & (values >= minimum_m) & (values <= maximum_m)
    if wrist_depth_m is not None and np.isfinite(wrist_depth_m):
        valid &= np.abs(values - wrist_depth_m) <= maximum_wrist_delta_m
    values = np.sort(values[valid])
    if values.size < 3:
        return None, 0.0
    valid_count = int(values.size)
    trim = int(values.size * 0.15)
    if trim and values.size - 2 * trim >= 3:
        values = values[trim:-trim]
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    confidence = float(np.clip((valid_count / ((radius_px * 2 + 1) ** 2)) * np.exp(-mad / 0.03), 0.0, 1.0))
    return median, confidence


def transform_point_camera_to_world(point: Sequence[float], rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return np.asarray(rotation, dtype=float) @ np.asarray(point, dtype=float) + np.asarray(translation, dtype=float)


def transform_ray_camera_to_world(ray: Sequence[float], rotation: np.ndarray) -> np.ndarray:
    result = np.asarray(rotation, dtype=float) @ np.asarray(ray, dtype=float)
    return result / np.linalg.norm(result)


def triangulate_rays(origins: Iterable[Sequence[float]], directions: Iterable[Sequence[float]]) -> tuple[np.ndarray, float]:
    """Least-squares closest point to N calibrated rays."""
    origins_array = [np.asarray(item, dtype=float) for item in origins]
    directions_array = [np.asarray(item, dtype=float) / np.linalg.norm(item) for item in directions]
    if len(origins_array) < 2 or len(origins_array) != len(directions_array):
        raise ValueError("at least two matched rays are required")
    a = np.zeros((3, 3), dtype=float)
    b = np.zeros(3, dtype=float)
    eye = np.eye(3)
    for origin, direction in zip(origins_array, directions_array):
        projection = eye - np.outer(direction, direction)
        a += projection
        b += projection @ origin
    if np.linalg.cond(a) > 1e8:
        raise ValueError("degenerate ray geometry")
    point = np.linalg.solve(a, b)
    distances = [np.linalg.norm(np.cross(point - origin, direction)) for origin, direction in zip(origins_array, directions_array)]
    return point, float(np.sqrt(np.mean(np.square(distances))))


def project_world_point(point_world: Sequence[float], rotation_camera_to_world: np.ndarray, translation_camera_to_world: np.ndarray, intrinsics: CameraIntrinsics) -> np.ndarray:
    rotation = np.asarray(rotation_camera_to_world, dtype=float)
    local = rotation.T @ (np.asarray(point_world, dtype=float) - np.asarray(translation_camera_to_world, dtype=float))
    if local[0] <= 1e-6:
        raise ValueError("point is behind camera")
    return np.array([
        intrinsics.cx - intrinsics.fx * local[1] / local[0],
        intrinsics.cy - intrinsics.fy * local[2] / local[0],
    ])

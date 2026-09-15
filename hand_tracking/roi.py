from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class RoiConfig:
    forearm_scale: float = 1.45
    handward_offset: float = 0.32
    minimum_px: int = 96
    maximum_px: int = 420
    fallback_px: int = 180


def clipped_hand_roi(
    wrist_px: Sequence[float],
    elbow_px: Sequence[float] | None,
    image_size: tuple[int, int],
    config: RoiConfig,
) -> tuple[int, int, int, int] | None:
    """Build a wrist ROI extended away from the elbow, clipped to the image."""
    width, height = map(int, image_size)
    wrist = np.asarray(wrist_px, dtype=float)[:2]
    if wrist.shape != (2,) or not np.isfinite(wrist).all() or width <= 0 or height <= 0:
        return None
    elbow = np.asarray(elbow_px, dtype=float)[:2] if elbow_px is not None else None
    if elbow is not None and elbow.shape == (2,) and np.isfinite(elbow).all():
        vector = wrist - elbow
        forearm_px = float(np.linalg.norm(vector))
    else:
        vector = np.zeros(2)
        forearm_px = 0.0
    size = config.fallback_px if forearm_px < 8.0 else forearm_px * config.forearm_scale
    size = int(round(np.clip(size, config.minimum_px, config.maximum_px)))
    direction = vector / forearm_px if forearm_px >= 8.0 else np.zeros(2)
    center = wrist + direction * size * config.handward_offset
    x0 = max(0, int(math.floor(center[0] - size / 2)))
    y0 = max(0, int(math.floor(center[1] - size / 2)))
    x1 = min(width, int(math.ceil(center[0] + size / 2)))
    y1 = min(height, int(math.ceil(center[1] + size / 2)))
    if x1 - x0 < 24 or y1 - y0 < 24:
        return None
    return x0, y0, x1 - x0, y1 - y0


def crop_to_full(landmarks_normalized: Sequence[Sequence[float]], roi: Sequence[int]) -> np.ndarray:
    points = np.asarray(landmarks_normalized, dtype=float)
    if points.shape != (21, 2) or not np.isfinite(points).all():
        raise ValueError("expected finite 21x2 normalized crop landmarks")
    x, y, width, height = map(float, roi)
    return points * np.array([width, height]) + np.array([x, y])


def full_to_crop(landmarks_px: Sequence[Sequence[float]], roi: Sequence[int]) -> np.ndarray:
    points = np.asarray(landmarks_px, dtype=float)
    x, y, width, height = map(float, roi)
    if width <= 0 or height <= 0:
        raise ValueError("ROI dimensions must be positive")
    return (points - np.array([x, y])) / np.array([width, height])

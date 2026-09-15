from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class NormalizedHand:
    side: str
    landmarks: np.ndarray
    wrist_world_m: np.ndarray
    rotation_world_from_palm: np.ndarray
    scale_m: float


class PalmNormalizer:
    """Create a continuous, right-handed canonical palm frame."""

    def __init__(self, minimum_scale_m: float = 0.025) -> None:
        self.minimum_scale_m = minimum_scale_m
        self._previous_rotation: dict[str, np.ndarray] = {}

    def reset(self, side: str | None = None) -> None:
        if side is None:
            self._previous_rotation.clear()
        else:
            self._previous_rotation.pop(side, None)

    def normalize(self, points_world: Sequence[Sequence[float]], side: str) -> NormalizedHand:
        points = np.asarray(points_world, dtype=float)
        if points.shape != (21, 3) or not np.isfinite(points).all():
            raise ValueError("expected finite 21x3 world landmarks")
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        wrist = points[0]
        lateral = points[5] - points[17]  # pinky -> index in anatomical space
        forward = points[9] - wrist
        x = lateral / max(np.linalg.norm(lateral), 1e-12)
        y_raw = forward - x * np.dot(forward, x)
        y_norm = float(np.linalg.norm(y_raw))
        width = float(np.linalg.norm(lateral))
        length = float(np.linalg.norm(forward))
        scale = float(np.median([width, length]))
        if y_norm < 1e-6 or scale < self.minimum_scale_m:
            raise ValueError("DEGENERATE_PALM_FRAME")
        y = y_raw / y_norm
        z = np.cross(x, y)
        z /= np.linalg.norm(z)
        # Canonical convention: +X is thumbward for both hands.  Mirroring the
        # left X/Z pair preserves determinant +1 and makes shapes comparable.
        if side == "left":
            x = -x
            z = -z
        rotation = np.column_stack((x, y, z))
        previous = self._previous_rotation.get(side)
        if previous is not None:
            flipped = rotation.copy()
            flipped[:, 0] *= -1.0
            flipped[:, 2] *= -1.0
            if np.trace(previous.T @ flipped) > np.trace(previous.T @ rotation):
                rotation = flipped
        if np.linalg.det(rotation) < 0.999:
            raise ValueError("INVALID_PALM_HANDEDNESS")
        normalized = (rotation.T @ (points - wrist).T).T / scale
        self._previous_rotation[side] = rotation.copy()
        return NormalizedHand(side, normalized, wrist.copy(), rotation, scale)

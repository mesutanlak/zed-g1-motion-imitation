from __future__ import annotations

from dataclasses import dataclass
import math
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


def _project_rotation(value: np.ndarray) -> np.ndarray:
    u, _singular, vh = np.linalg.svd(np.asarray(value, dtype=float))
    rotation = u @ vh
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vh
    return rotation


def _interpolate_so3(previous: np.ndarray, current: np.ndarray, alpha: float) -> np.ndarray:
    """Geodesic interpolation on SO(3), without a SciPy runtime dependency."""
    delta = previous.T @ current
    cosine = float(np.clip((np.trace(delta) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cosine)
    if angle < 1.0e-7:
        return _project_rotation((1.0 - alpha) * previous + alpha * current)
    axis = np.asarray(
        [delta[2, 1] - delta[1, 2], delta[0, 2] - delta[2, 0], delta[1, 0] - delta[0, 1]],
        dtype=float,
    ) / (2.0 * math.sin(angle))
    x, y, z = axis
    skew = np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    step = angle * float(np.clip(alpha, 0.0, 1.0))
    incremental = np.eye(3) + math.sin(step) * skew + (1.0 - math.cos(step)) * (skew @ skew)
    return _project_rotation(previous @ incremental)


class PalmSE3Filter:
    """Low latency translation/SO(3)/scale and canonical-shape filter.

    MediaPipe world landmarks are hand-local, but their palm frame can still
    flip or jitter between independent ROI detections.  Filtering the rigid
    palm transform on SE(3) before the canonical shape reaches the hand IK
    keeps the retarget target continuous without mixing left/right frames.
    """

    def __init__(
        self,
        *,
        pose_cutoff_hz: float = 3.0,
        shape_cutoff_hz: float = 2.0,
        reset_gap_s: float = 0.75,
    ) -> None:
        self.pose_cutoff_hz = float(pose_cutoff_hz)
        self.shape_cutoff_hz = float(shape_cutoff_hz)
        self.reset_gap_s = float(reset_gap_s)
        self._state: dict[str, tuple[int, NormalizedHand]] = {}

    def reset(self, side: str | None = None) -> None:
        if side is None:
            self._state.clear()
        else:
            self._state.pop(side, None)

    @staticmethod
    def _alpha(cutoff_hz: float, dt_s: float) -> float:
        return float(1.0 - math.exp(-2.0 * math.pi * max(cutoff_hz, 0.01) * dt_s))

    def update(self, value: NormalizedHand, timestamp_ns: int) -> NormalizedHand:
        previous_item = self._state.get(value.side)
        if previous_item is None:
            result = value
        else:
            previous_ns, previous = previous_item
            dt_s = max(0.0, (int(timestamp_ns) - previous_ns) / 1e9)
            if dt_s <= 0.0 or dt_s > self.reset_gap_s:
                result = value
            else:
                pose_alpha = self._alpha(self.pose_cutoff_hz, dt_s)
                shape_alpha = self._alpha(self.shape_cutoff_hz, dt_s)
                result = NormalizedHand(
                    side=value.side,
                    landmarks=(
                        previous.landmarks * (1.0 - shape_alpha)
                        + value.landmarks * shape_alpha
                    ),
                    wrist_world_m=(
                        previous.wrist_world_m * (1.0 - pose_alpha)
                        + value.wrist_world_m * pose_alpha
                    ),
                    rotation_world_from_palm=_interpolate_so3(
                        previous.rotation_world_from_palm,
                        value.rotation_world_from_palm,
                        pose_alpha,
                    ),
                    scale_m=float(
                        previous.scale_m * (1.0 - pose_alpha)
                        + value.scale_m * pose_alpha
                    ),
                )
        self._state[value.side] = (int(timestamp_ns), result)
        return result

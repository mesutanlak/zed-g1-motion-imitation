"""Operator anthropometry and pelvis-local coordinate calibration."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

import numpy as np


CALIBRATION_SEGMENTS = {
    "shoulder_width_m": ("LEFT_SHOULDER", "RIGHT_SHOULDER"),
    "pelvis_width_m": ("LEFT_HIP", "RIGHT_HIP"),
    "left_upper_arm_m": ("LEFT_SHOULDER", "LEFT_ELBOW"),
    "right_upper_arm_m": ("RIGHT_SHOULDER", "RIGHT_ELBOW"),
    "left_forearm_m": ("LEFT_ELBOW", "LEFT_WRIST"),
    "right_forearm_m": ("RIGHT_ELBOW", "RIGHT_WRIST"),
    "left_thigh_m": ("LEFT_HIP", "LEFT_KNEE"),
    "right_thigh_m": ("RIGHT_HIP", "RIGHT_KNEE"),
    "left_shank_m": ("LEFT_KNEE", "LEFT_ANKLE"),
    "right_shank_m": ("RIGHT_KNEE", "RIGHT_ANKLE"),
    "foot_separation_m": ("LEFT_ANKLE", "RIGHT_ANKLE"),
}


@dataclass(frozen=True)
class CalibrationResult:
    state: str
    progress: float
    elapsed_s: float
    sample_count: int
    profile: dict[str, object] | None
    reason: str


def _unit(vector: np.ndarray) -> np.ndarray | None:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-6 or not math.isfinite(norm):
        return None
    return vector / norm


def pelvis_frame(points: np.ndarray, index: Mapping[str, int]) -> tuple[np.ndarray, np.ndarray]:
    """Return pelvis origin and camera-to-pelvis rotation [forward,left,up]."""
    xyz = np.asarray(points, dtype=np.float64)
    pelvis = xyz[index["PELVIS"]]
    left_hip = xyz[index["LEFT_HIP"]]
    right_hip = xyz[index["RIGHT_HIP"]]
    left_shoulder = xyz[index["LEFT_SHOULDER"]]
    right_shoulder = xyz[index["RIGHT_SHOULDER"]]
    if not np.isfinite(np.vstack((pelvis, left_hip, right_hip, left_shoulder, right_shoulder))).all():
        raise ValueError("pelvis_frame_missing_keypoint")

    left = _unit(left_hip - right_hip)
    shoulder_center = 0.5 * (left_shoulder + right_shoulder)
    if left is None:
        raise ValueError("pelvis_width_degenerate")
    up_raw = shoulder_center - pelvis
    up = _unit(up_raw - left * float(np.dot(up_raw, left)))
    if up is None:
        raise ValueError("torso_axis_degenerate")
    forward = _unit(np.cross(left, up))
    if forward is None:
        raise ValueError("pelvis_forward_degenerate")
    # Re-orthogonalize so noisy keypoints cannot shear local coordinates.
    left = _unit(np.cross(up, forward))
    assert left is not None
    rotation = np.column_stack((forward, left, up))
    return pelvis, rotation


def to_pelvis_local(points: np.ndarray, index: Mapping[str, int]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    origin, rotation = pelvis_frame(points, index)
    local = (np.asarray(points, dtype=np.float64) - origin) @ rotation
    return local, origin, rotation


def stabilize_pelvis_rotation(
    previous: np.ndarray | None,
    measured: np.ndarray,
    *,
    static_alpha: float = 0.25,
    moving_alpha: float = 0.85,
    moving_angle_deg: float = 15.0,
) -> np.ndarray:
    """Causally suppress BODY_38 pelvis-frame jitter on SO(3).

    Hip/shoulder depth noise rotates an otherwise static pelvis frame several
    degrees per image and makes both arms appear to move together.  Matrix
    blending followed by polar projection keeps the result a proper rotation.
    The blend becomes responsive as the measured rotation step grows, so a
    genuine torso turn is not treated like static-camera noise.
    """
    current = np.asarray(measured, dtype=np.float64)
    if current.shape != (3, 3) or not np.isfinite(current).all():
        raise ValueError("invalid_pelvis_rotation")
    if previous is None:
        return current.copy()
    prior = np.asarray(previous, dtype=np.float64)
    if prior.shape != (3, 3) or not np.isfinite(prior).all():
        return current.copy()
    relative = prior.T @ current
    angle_deg = float(np.degrees(np.arccos(np.clip(
        (float(np.trace(relative)) - 1.0) * 0.5, -1.0, 1.0,
    ))))
    fraction = float(np.clip(angle_deg / max(float(moving_angle_deg), 1.0), 0.0, 1.0))
    alpha = float(np.clip(
        static_alpha + (moving_alpha - static_alpha) * fraction,
        0.0, 1.0,
    ))
    blended = (1.0 - alpha) * prior + alpha * current
    u, _, vt = np.linalg.svd(blended)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    return rotation


class CalibrationManager:
    """Collect robust neutral anthropometry over a fixed 3–5 second window."""

    def __init__(self, duration_s: float = 4.0, min_valid_ratio: float = 0.85) -> None:
        self.duration_s = float(np.clip(duration_s, 3.0, 5.0))
        self.min_valid_ratio = float(np.clip(min_valid_ratio, 0.5, 1.0))
        self.reset()

    def reset(self) -> None:
        self.started_s: float | None = None
        self.samples: list[dict[str, float]] = []
        self.rotations: list[np.ndarray] = []
        self.profile: dict[str, object] | None = None
        self.operator_id: int | None = None

    def update(
        self,
        *,
        operator_id: int,
        timestamp_s: float,
        points: np.ndarray,
        confidence: Sequence[float],
        index: Mapping[str, int],
        confidence_threshold: float,
    ) -> CalibrationResult:
        if self.operator_id is not None and operator_id != self.operator_id:
            self.reset()
        self.operator_id = int(operator_id)
        if self.profile is not None:
            return CalibrationResult("READY", 1.0, self.duration_s, len(self.samples), self.profile, "calibration_ready")
        if self.started_s is None:
            self.started_s = float(timestamp_s)

        xyz = np.asarray(points, dtype=np.float64)
        conf = np.asarray(confidence, dtype=np.float64)
        required_names = sorted({n for pair in CALIBRATION_SEGMENTS.values() for n in pair} | {"PELVIS", "SPINE_3", "LEFT_HEEL", "RIGHT_HEEL"})
        valid = all(
            name in index and np.isfinite(xyz[index[name]]).all()
            and conf[index[name]] >= confidence_threshold
            for name in required_names
        )
        reason = "collecting_neutral_pose"
        if valid:
            sample: dict[str, float] = {}
            for label, (first, second) in CALIBRATION_SEGMENTS.items():
                sample[label] = float(np.linalg.norm(xyz[index[first]] - xyz[index[second]]))
            shoulder_center = 0.5 * (xyz[index["LEFT_SHOULDER"]] + xyz[index["RIGHT_SHOULDER"]])
            sample["torso_length_m"] = float(np.linalg.norm(shoulder_center - xyz[index["PELVIS"]]))
            sample["floor_height_m"] = float(np.median([
                xyz[index["LEFT_HEEL"], 2], xyz[index["RIGHT_HEEL"], 2],
                xyz[index["LEFT_ANKLE"], 2], xyz[index["RIGHT_ANKLE"], 2],
            ]))
            try:
                _, rotation = pelvis_frame(xyz, index)
                self.rotations.append(rotation)
                self.samples.append(sample)
            except ValueError:
                reason = "neutral_orientation_invalid"
        else:
            reason = "neutral_pose_keypoints_missing"

        elapsed = max(0.0, float(timestamp_s) - float(self.started_s))
        progress = min(1.0, elapsed / self.duration_s)
        if elapsed < self.duration_s:
            return CalibrationResult("COLLECTING", progress, elapsed, len(self.samples), None, reason)

        expected = max(1, int(round(self.duration_s * 15.0)))
        if len(self.samples) < expected * self.min_valid_ratio:
            return CalibrationResult("FAILED", progress, elapsed, len(self.samples), None, "insufficient_valid_neutral_samples")

        labels = tuple(self.samples[0])
        profile: dict[str, object] = {}
        coefficients: dict[str, float] = {}
        for label in labels:
            values = np.asarray([sample[label] for sample in self.samples], dtype=np.float64)
            median = float(np.median(values))
            profile[label] = median
            coefficients[label] = float(np.std(values) / max(abs(np.mean(values)), 1e-6))
        mean_rotation = np.mean(np.stack(self.rotations), axis=0)
        u, _, vt = np.linalg.svd(mean_rotation)
        neutral_rotation = u @ vt
        if np.linalg.det(neutral_rotation) < 0:
            u[:, -1] *= -1
            neutral_rotation = u @ vt
        profile.update({
            "operator_id": self.operator_id,
            "duration_s": elapsed,
            "sample_count": len(self.samples),
            "neutral_pelvis_rotation_matrix": neutral_rotation.tolist(),
            "neutral_shoulder_rotation_matrix": neutral_rotation.tolist(),
            "bone_length_cv": coefficients,
            "median_bone_length_cv": float(np.median(list(coefficients.values()))),
        })
        self.profile = profile
        return CalibrationResult("READY", 1.0, elapsed, len(self.samples), profile, "calibration_complete")

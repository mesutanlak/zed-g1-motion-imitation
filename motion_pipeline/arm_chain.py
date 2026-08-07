"""Torso-overlap detection and coherent 3D arm-chain recovery."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class ArmChainResult:
    points: np.ndarray
    overlap: dict[str, bool]
    recovered: dict[str, bool]
    reasons: tuple[str, ...]


def _inside_convex(point: np.ndarray, polygon: np.ndarray) -> bool:
    if not np.isfinite(point).all() or not np.isfinite(polygon).all():
        return False
    signs = []
    for index in range(len(polygon)):
        first = polygon[index]
        second = polygon[(index + 1) % len(polygon)]
        edge = second - first
        relative = point - first
        signs.append(float(edge[0] * relative[1] - edge[1] * relative[0]))
    return all(value >= 0 for value in signs) or all(value <= 0 for value in signs)


def _inside_expanded_convex(
    point: np.ndarray, polygon: np.ndarray, scale: float = 1.15
) -> bool:
    """Boundary-tolerant polygon test without accepting distant keypoints."""
    if not np.isfinite(polygon).all():
        return False
    center = np.mean(polygon, axis=0)
    expanded = center + float(scale) * (polygon - center)
    return _inside_convex(point, expanded)


def _two_bone_chain(shoulder: np.ndarray, target: np.ndarray, previous_elbow: np.ndarray, upper: float, fore: float) -> tuple[np.ndarray, np.ndarray]:
    direction = target - shoulder
    distance = float(np.linalg.norm(direction))
    if distance < 1e-6:
        return previous_elbow.copy(), target.copy()
    axis = direction / distance
    reach = float(np.clip(distance, abs(upper - fore) + 1e-4, upper + fore - 1e-4))
    wrist = shoulder + axis * reach
    along = (upper * upper - fore * fore + reach * reach) / (2.0 * reach)
    height = float(np.sqrt(max(upper * upper - along * along, 0.0)))
    bend = previous_elbow - (shoulder + axis * float(np.dot(previous_elbow - shoulder, axis)))
    if np.linalg.norm(bend) < 1e-5:
        bend = np.cross(axis, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(bend) < 1e-5:
        bend = np.cross(axis, np.array([0.0, 1.0, 0.0]))
    bend /= max(float(np.linalg.norm(bend)), 1e-6)
    elbow = shoulder + axis * along + bend * height
    return elbow, wrist


class ArmChainOptimizer:
    """Recover an occluded elbow/wrist as one length-constrained chain."""

    def __init__(self, hold_s: float = 0.25) -> None:
        self.hold_s = float(hold_s)
        self.previous: dict[str, tuple[float, np.ndarray, np.ndarray]] = {}
        self.overlap_until: dict[str, float] = {}

    def reset(self) -> None:
        self.previous.clear()
        self.overlap_until.clear()

    @staticmethod
    def _inside_torso_projection_3d(
        points: np.ndarray,
        index: Mapping[str, int],
        side: str,
    ) -> bool:
        """Detect optical-axis arm/torso overlap in ZED's Y-Z plane.

        A frontal arm can have valid high-confidence depth points while its
        elbow and wrist are visually superimposed on the torso.  The narrow
        2D polygon test occasionally misses those boundary frames.  This
        pelvis-independent Y-Z projection matches the analysis panel and is
        deliberately used only as an occlusion gate, never as a new target.
        """

        required = (
            "PELVIS",
            "LEFT_SHOULDER",
            "RIGHT_SHOULDER",
            f"{side}_WRIST",
        )
        if any(name not in index for name in required):
            return False
        try:
            pelvis = points[index["PELVIS"]]
            left_shoulder = points[index["LEFT_SHOULDER"]]
            right_shoulder = points[index["RIGHT_SHOULDER"]]
            # Keep this fallback deliberately narrow.  An elbow may be close
            # to the torso in many valid poses; the wrist entering the torso
            # projection is the reliable signature of the frontal overlap
            # failure seen in the recording.
            arm_points = (points[index[f"{side}_WRIST"]],)
        except IndexError:
            return False
        if not all(
            value.shape == (3,) and np.isfinite(value).all()
            for value in (pelvis, left_shoulder, right_shoulder, *arm_points)
        ):
            return False
        center = 0.5 * (left_shoulder + right_shoulder)
        shoulder_axis = left_shoulder - right_shoulder
        width = float(np.linalg.norm(shoulder_axis))
        if width < 0.10:
            return False
        lateral = shoulder_axis / width
        for point in arm_points:
            lateral_position = float(np.dot(point - center, lateral))
            if (
                abs(lateral_position) < 0.55 * width
                and float(pelvis[2]) - 0.05
                <= float(point[2])
                <= float(center[2]) + 0.20
            ):
                return True
        return False

    def update(
        self, *, timestamp_s: float, points_3d: np.ndarray, points_2d: np.ndarray,
        confidence: Sequence[float], index: Mapping[str, int], threshold: float,
        calibration: Mapping[str, object] | None,
    ) -> ArmChainResult:
        output = np.asarray(points_3d, dtype=np.float64).copy()
        pixel = np.asarray(points_2d, dtype=np.float64)
        conf = np.asarray(confidence, dtype=np.float64)
        torso = pixel[[index["LEFT_SHOULDER"], index["RIGHT_SHOULDER"], index["RIGHT_HIP"], index["LEFT_HIP"]]]
        overlap: dict[str, bool] = {}
        recovered: dict[str, bool] = {}
        reasons: list[str] = []
        for side in ("LEFT", "RIGHT"):
            shoulder_i, elbow_i, wrist_i = (index[f"{side}_SHOULDER"], index[f"{side}_ELBOW"], index[f"{side}_WRIST"])
            elbow_overlap = _inside_convex(pixel[elbow_i], torso)
            wrist_overlap = _inside_convex(pixel[wrist_i], torso)
            # The 3D projection is only a confirmation for a keypoint close
            # to the image-space torso boundary.  Using it alone would mark
            # valid side poses as occluded and would change normal IK.
            depth_overlap = self._inside_torso_projection_3d(output, index, side)
            boundary_overlap = depth_overlap and (
                _inside_expanded_convex(pixel[elbow_i], torso)
                or _inside_expanded_convex(pixel[wrist_i], torso)
            )
            detected_overlap = elbow_overlap or wrist_overlap or boundary_overlap
            if detected_overlap:
                self.overlap_until[side] = timestamp_s + self.hold_s
            overlap_active = detected_overlap or timestamp_s <= self.overlap_until.get(side, -np.inf)
            overlap[side.lower()] = overlap_active
            low_quality = overlap_active or conf[elbow_i] < threshold or conf[wrist_i] < threshold
            previous = self.previous.get(side)
            did_recover = False
            if low_quality and previous is not None and timestamp_s - previous[0] <= self.hold_s:
                prev_elbow, prev_wrist = previous[1], previous[2]
                shoulder = output[shoulder_i]
                measured_wrist = output[wrist_i]
                target = measured_wrist if np.isfinite(measured_wrist).all() and conf[wrist_i] >= threshold else prev_wrist
                if overlap_active and np.isfinite(target).all() and np.isfinite(shoulder).all():
                    # During torso overlap ZED often keeps a high confidence
                    # score although wrist depth is ambiguous.  Preserve the
                    # last reliable shoulder-wrist reach (therefore elbow
                    # flexion) and follow only the measured arm direction.
                    # Once the wrist leaves the torso, blend the measured
                    # reach back over the hysteresis window.
                    measured_vector = target - shoulder
                    measured_reach = float(np.linalg.norm(measured_vector))
                    previous_reach = float(np.linalg.norm(prev_wrist - shoulder))
                    if measured_reach > 1e-6 and previous_reach > 1e-6:
                        if detected_overlap:
                            measured_weight = 0.0
                        else:
                            remaining = max(
                                self.overlap_until.get(side, timestamp_s) - timestamp_s,
                                0.0,
                            )
                            measured_weight = float(
                                np.clip(1.0 - remaining / max(self.hold_s, 1e-6), 0.0, 1.0)
                            )
                        reach = (
                            (1.0 - measured_weight) * previous_reach
                            + measured_weight * measured_reach
                        )
                        target = shoulder + measured_vector / measured_reach * reach
                upper = float((calibration or {}).get(f"{side.lower()}_upper_arm_m", np.linalg.norm(prev_elbow - shoulder)))
                fore = float((calibration or {}).get(f"{side.lower()}_forearm_m", np.linalg.norm(prev_wrist - prev_elbow)))
                if np.isfinite(shoulder).all() and upper > 0.05 and fore > 0.05:
                    output[elbow_i], output[wrist_i] = _two_bone_chain(shoulder, target, prev_elbow, upper, fore)
                    did_recover = True
                    reasons.append(f"{side.lower()}_arm_chain_occlusion_recovery")
            recovered[side.lower()] = did_recover
            if not low_quality and np.isfinite(output[[elbow_i, wrist_i]]).all():
                self.previous[side] = (timestamp_s, output[elbow_i].copy(), output[wrist_i].copy())
            elif did_recover:
                self.previous[side] = (timestamp_s, output[elbow_i].copy(), output[wrist_i].copy())
        return ArmChainResult(output, overlap, recovered, tuple(reasons))

"""Confidence-aware, timestamped BODY_38 multi-view state estimation.

The estimator deliberately operates after both calibrated camera skeletons
have been transformed into one world frame.  It never averages a complete
skeleton and it never assumes that camera-local body IDs are shared.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


CORE_NAMES = (
    "PELVIS", "SPINE_1", "SPINE_2", "SPINE_3", "NECK",
    "LEFT_CLAVICLE", "RIGHT_CLAVICLE", "LEFT_SHOULDER",
    "RIGHT_SHOULDER", "LEFT_HIP", "RIGHT_HIP",
)
ARM_CHAINS = {
    "left": ("LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST"),
    "right": ("RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST"),
}
SEGMENT_GROUPS = {
    "pelvis": ("PELVIS", "SPINE_1", "LEFT_HIP", "RIGHT_HIP"),
    "torso": ("SPINE_2", "SPINE_3", "NECK", "LEFT_SHOULDER", "RIGHT_SHOULDER"),
    "left_arm": ("LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST"),
    "right_arm": ("RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST"),
    "left_leg": ("LEFT_HIP", "LEFT_KNEE", "LEFT_ANKLE"),
    "right_leg": ("RIGHT_HIP", "RIGHT_KNEE", "RIGHT_ANKLE"),
}


def load_human_state_config(path: Path | str) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if value.get("schema") != "zed_g1_dual_teleoperation/v1":
        raise ValueError(f"Unsupported dual teleoperation config: {value.get('schema')!r}")
    return value


def quaternion_continuous(previous: Sequence[float] | None, current: Sequence[float]) -> np.ndarray:
    """Return an XYZW quaternion on the same hemisphere as ``previous``."""
    value = np.asarray(current, dtype=np.float64).copy()
    norm = float(np.linalg.norm(value))
    if value.shape != (4,) or not np.isfinite(value).all() or norm < 1.0e-12:
        return np.full(4, np.nan)
    value /= norm
    if previous is not None:
        old = np.asarray(previous, dtype=np.float64)
        if old.shape == (4,) and np.isfinite(old).all() and float(np.dot(old, value)) < 0.0:
            value *= -1.0
    return value


@dataclass
class HumanStateResult:
    points: np.ndarray
    confidence: np.ndarray
    joint_quality: np.ndarray
    joint_source: tuple[str, ...]
    joint_state: tuple[str, ...]
    segment_quality: dict[str, float]
    failure_codes: tuple[str, ...]
    prediction_age_ms: np.ndarray
    elbow_state: dict[str, str]
    camera_weights: dict[str, dict[str, float]]
    core_disagreement_m: float | None

    def as_dict(self, names: Sequence[str]) -> dict[str, Any]:
        return {
            "schema": "clean_human_state/v1",
            "joint_quality": {name: float(self.joint_quality[i]) for i, name in enumerate(names)},
            "joint_source": {name: self.joint_source[i] for i, name in enumerate(names)},
            "joint_state": {name: self.joint_state[i] for i, name in enumerate(names)},
            "prediction_age_ms": {name: float(self.prediction_age_ms[i]) for i, name in enumerate(names)},
            "segment_quality": self.segment_quality,
            "failure_codes": list(self.failure_codes),
            "elbow_state": self.elbow_state,
            "camera_weights": self.camera_weights,
            "core_disagreement_m": self.core_disagreement_m,
        }


class ConfidenceAwareHumanStateEstimator:
    """Latest-valid reference estimator with per-joint multi-view fusion."""

    def __init__(self, names: Sequence[str], edges: Sequence[tuple[str, str]], config: Mapping[str, Any]):
        self.names = tuple(names)
        self.index = {name: i for i, name in enumerate(self.names)}
        self.edges = tuple((self.index[a], self.index[b]) for a, b in edges if a in self.index and b in self.index)
        self.config = dict(config)
        self.fusion = dict(config["fusion"])
        self.anatomy = dict(config["anatomy"])
        self.previous_points = np.full((len(self.names), 3), np.nan)
        self.velocity = np.zeros((len(self.names), 3), dtype=np.float64)
        self.previous_quality = np.zeros(len(self.names), dtype=np.float64)
        self.previous_timestamp_ns = 0
        self.last_valid_timestamp_ns = np.zeros(len(self.names), dtype=np.int64)
        self.camera_history: dict[int, tuple[int, np.ndarray]] = {}
        self.bone_lengths: dict[tuple[int, int], float] = {}
        self.elbow_normals: dict[str, np.ndarray] = {}
        self.elbow_recovery: dict[str, int] = {"left": 0, "right": 0}
        self.swap_votes: dict[int, int] = {}

    def reset(self) -> None:
        self.__init__(self.names, [(self.names[a], self.names[b]) for a, b in self.edges], self.config)

    def _category(self, name: str) -> str:
        if "WRIST" in name:
            return "wrist"
        if "HAND" in name or "THUMB" in name or "INDEX" in name or "MIDDLE" in name or "PINKY" in name:
            return "hand"
        if any(label in name for label in ("SHOULDER", "ELBOW", "CLAVICLE")):
            return "arm"
        if any(label in name for label in ("HIP", "KNEE", "ANKLE", "TOE", "HEEL")):
            return "leg"
        if any(label in name for label in ("NOSE", "EYE", "EAR", "NECK")):
            return "head"
        return "core"

    def _swap_guard(self, serial: int, points: np.ndarray) -> tuple[np.ndarray, bool]:
        if not np.isfinite(self.previous_points).any():
            return points, False
        pairs = [(self.index[f"LEFT_{part}"], self.index[f"RIGHT_{part}"]) for part in ("SHOULDER", "ELBOW", "WRIST")]
        normal = swapped = 0.0
        count = 0
        for left, right in pairs:
            if np.isfinite(points[[left, right]]).all() and np.isfinite(self.previous_points[[left, right]]).all():
                normal += float(np.linalg.norm(points[left] - self.previous_points[left]) + np.linalg.norm(points[right] - self.previous_points[right]))
                swapped += float(np.linalg.norm(points[left] - self.previous_points[right]) + np.linalg.norm(points[right] - self.previous_points[left]))
                count += 1
        risk = count >= 2 and swapped + 0.10 < 0.70 * normal
        votes = self.swap_votes.get(serial, 0)
        votes = min(votes + 1, 10) if risk else max(votes - 1, 0)
        self.swap_votes[serial] = votes
        if votes < int(self.anatomy["left_right_confirm_frames"]):
            return points, risk
        result = points.copy()
        for left, right in pairs:
            result[[left, right]] = result[[right, left]]
        return result, True

    def _camera_velocity(self, serial: int, timestamp_ns: int, points: np.ndarray) -> np.ndarray:
        velocity = np.zeros_like(points)
        previous = self.camera_history.get(serial)
        if previous is not None and timestamp_ns > previous[0]:
            dt = (timestamp_ns - previous[0]) / 1.0e9
            if 0.005 <= dt <= 0.20:
                old = previous[1]
                valid = np.isfinite(points).all(axis=1) & np.isfinite(old).all(axis=1)
                velocity[valid] = (points[valid] - old[valid]) / dt
                speed = np.linalg.norm(velocity, axis=1)
                limit = float(self.fusion["prediction"]["max_velocity_m_s"])
                scale = np.minimum(1.0, limit / np.maximum(speed, 1.0e-9))
                velocity *= scale[:, None]
        self.camera_history[serial] = (timestamp_ns, points.copy())
        return velocity

    def _view_geometry(self, joint: int, points: np.ndarray, camera_position: np.ndarray | None) -> float:
        if camera_position is None or not np.isfinite(camera_position).all() or not np.isfinite(points[joint]).all():
            return 0.65
        ray = points[joint] - camera_position
        ray_norm = float(np.linalg.norm(ray))
        if ray_norm < 1.0e-6:
            return 0.2
        ray /= ray_norm
        segment_scores = []
        for a, b in self.edges:
            if joint not in (a, b) or not np.isfinite(points[[a, b]]).all():
                continue
            direction = points[b] - points[a]
            length = float(np.linalg.norm(direction))
            if length > 1.0e-6:
                segment_scores.append(float(np.linalg.norm(np.cross(direction / length, ray))))
        return float(np.clip(np.mean(segment_scores) if segment_scores else 0.65, 0.15, 1.0))

    def _bone_quality(self, joint: int, points: np.ndarray) -> float:
        scores = []
        tolerance = float(self.anatomy["bone_length_tolerance"])
        for edge in self.edges:
            if joint not in edge or edge not in self.bone_lengths or not np.isfinite(points[list(edge)]).all():
                continue
            length = float(np.linalg.norm(points[edge[1]] - points[edge[0]]))
            reference = self.bone_lengths[edge]
            relative = abs(length - reference) / max(reference, 0.05)
            scores.append(math.exp(-0.5 * (relative / max(tolerance, 1.0e-3)) ** 2))
        return float(np.mean(scores)) if scores else 0.75

    def _quality(self, body: Any, points: np.ndarray, joint: int, camera_position: np.ndarray | None, predicted: np.ndarray) -> float:
        confidence = np.asarray(body.keypoint_confidence, dtype=np.float64)
        keypoint = float(np.clip(confidence[joint] / 100.0, 0.0, 1.0))
        body_confidence = float(np.clip(float(getattr(body, "confidence", 0.0)) / 100.0, 0.0, 1.0))
        if np.isfinite(predicted).all():
            residual = float(np.linalg.norm(points[joint] - predicted))
            temporal = math.exp(-0.5 * (residual / 0.30) ** 2)
        else:
            temporal = 0.75
        view = self._view_geometry(joint, points, camera_position)
        distance = float(np.linalg.norm(points[joint] - camera_position)) if camera_position is not None else 3.0
        distance_score = math.exp(-0.5 * ((distance - 3.0) / 2.2) ** 2)
        bone = self._bone_quality(joint, points)
        weights = self.fusion["quality_weights"]
        return float(np.clip(
            weights["keypoint_confidence"] * keypoint
            + weights["body_confidence"] * body_confidence
            + weights["temporal_continuity"] * temporal
            + weights["view_geometry"] * view
            + weights["distance"] * distance_score
            + weights["bone_consistency"] * bone,
            0.0, 1.0,
        ))

    def _constrain_bones(self, points: np.ndarray, quality: np.ndarray) -> np.ndarray:
        result = points.copy()
        blend = float(self.anatomy["projection_blend"])
        # Learn slowly only from mutually reliable observations.
        for edge in self.edges:
            a, b = edge
            if not np.isfinite(result[[a, b]]).all() or min(quality[a], quality[b]) < 0.60:
                continue
            length = float(np.linalg.norm(result[b] - result[a]))
            if not 0.03 < length < 1.0:
                continue
            old = self.bone_lengths.get(edge)
            self.bone_lengths[edge] = length if old is None else 0.995 * old + 0.005 * length
        # Strongest anatomical projection is reserved for the two arm chains.
        for side in ("left", "right"):
            names = ARM_CHAINS[side]
            for parent_name, child_name in zip(names, names[1:]):
                edge = (self.index[parent_name], self.index[child_name])
                reference = self.bone_lengths.get(edge)
                parent, child = edge
                if reference is None or not np.isfinite(result[[parent, child]]).all():
                    continue
                delta = result[child] - result[parent]
                length = float(np.linalg.norm(delta))
                if length > 1.0e-6:
                    projected = result[parent] + delta * (reference / length)
                    result[child] = (1.0 - blend) * result[child] + blend * projected
        return result

    def _elbow_guard(self, points: np.ndarray, quality: np.ndarray, failure: set[str]) -> dict[str, str]:
        states: dict[str, str] = {}
        singular_limit = float(self.anatomy["elbow_singular_sin"])
        for side, chain in ARM_CHAINS.items():
            s, e, w = (self.index[name] for name in chain)
            if not np.isfinite(points[[s, e, w]]).all() or min(quality[[s, e, w]]) < 0.20:
                states[side] = "ELBOW_OCCLUDED"
                continue
            upper = points[e] - points[s]
            fore = points[w] - points[e]
            upper /= max(float(np.linalg.norm(upper)), 1.0e-9)
            fore /= max(float(np.linalg.norm(fore)), 1.0e-9)
            cross = np.cross(upper, fore)
            sine = float(np.linalg.norm(cross))
            _flexion = math.atan2(sine, float(np.clip(np.dot(upper, fore), -1.0, 1.0)))
            if sine < singular_limit:
                states[side] = "ELBOW_NEAR_SINGULAR"
                failure.add("ELBOW_SINGULARITY")
                continue
            normal = cross / sine
            old = self.elbow_normals.get(side)
            if old is not None and float(np.dot(old, normal)) < -0.25:
                self.elbow_recovery[side] += 1
                if self.elbow_recovery[side] < int(self.anatomy["elbow_flip_confirm_frames"]):
                    states[side] = "ELBOW_RECOVERING"
                    failure.add("ELBOW_PLANE_FLIP_REJECTED")
                    if np.isfinite(self.previous_points[e]).all():
                        points[e] = self.previous_points[e]
                    continue
            self.elbow_recovery[side] = 0
            self.elbow_normals[side] = normal
            states[side] = "ELBOW_PLANE_VALID"
        return states

    def update(
        self,
        observations: Mapping[int, Any],
        capture_timestamps_ns: Mapping[int, int],
        camera_positions: Mapping[int, Sequence[float]] | None = None,
        target_timestamp_ns: int | None = None,
    ) -> HumanStateResult:
        if not observations:
            raise ValueError("At least one calibrated BODY_38 observation is required")
        camera_positions = camera_positions or {}
        timestamp_ns = int(target_timestamp_ns or max(capture_timestamps_ns.get(s, 0) for s in observations) or 0)
        if timestamp_ns <= 0:
            raise ValueError("A positive capture timestamp is required")
        dt_state = (timestamp_ns - self.previous_timestamp_ns) / 1.0e9 if self.previous_timestamp_ns else 0.0
        predicted_state = self.previous_points.copy()
        if 0.0 < dt_state <= 0.15:
            valid = np.isfinite(predicted_state).all(axis=1)
            predicted_state[valid] += self.velocity[valid] * dt_state

        prepared: dict[int, tuple[Any, np.ndarray, np.ndarray]] = {}
        timing_quality: dict[int, float] = {}
        failure: set[str] = set()
        for serial, body in observations.items():
            points = np.asarray(body.keypoint, dtype=np.float64).copy()
            points, swap_risk = self._swap_guard(int(serial), points)
            if swap_risk:
                failure.add("LEFT_RIGHT_SWAP_RISK")
            source_time = int(capture_timestamps_ns.get(serial, timestamp_ns))
            velocity = self._camera_velocity(int(serial), source_time, points)
            offset_s = max(0.0, (timestamp_ns - source_time) / 1.0e9)
            if offset_s * 1000.0 > float(self.fusion["reject_sync_ms"]):
                failure.add("CAMERA_TIME_MISMATCH")
                continue
            offset_ms = offset_s * 1000.0
            timing_quality[int(serial)] = (
                1.0
                if offset_ms <= float(self.fusion["preferred_sync_ms"])
                else 0.88
                if offset_ms <= float(self.fusion["acceptable_sync_ms"])
                else 0.68
            )
            valid = np.isfinite(points).all(axis=1)
            points[valid] += velocity[valid] * offset_s
            prepared[int(serial)] = (body, points, velocity)
        if not prepared:
            raise ValueError("All camera observations failed the synchronization gate")

        count = len(self.names)
        output = np.full((count, 3), np.nan)
        quality = np.zeros(count)
        confidence = np.zeros(count)
        prediction_age = np.full(count, math.inf)
        sources: list[str] = ["INVALID"] * count
        states: list[str] = ["INVALID"] * count
        weights_by_joint: dict[str, dict[str, float]] = {}
        confidence_threshold = float(self.fusion["confidence_threshold"])
        quality_gap = float(self.fusion["quality_difference_select"])
        disagreement_cfg = self.fusion["joint_disagreement_m"]

        for joint, name in enumerate(self.names):
            candidates = []
            joint_weights: dict[str, float] = {}
            for serial, (body, points, _velocity) in prepared.items():
                conf = float(np.asarray(body.keypoint_confidence)[joint])
                if conf < confidence_threshold or not np.isfinite(points[joint]).all():
                    joint_weights[str(serial)] = 0.0
                    continue
                camera_position = np.asarray(camera_positions.get(serial), dtype=np.float64) if serial in camera_positions else None
                score = self._quality(body, points, joint, camera_position, predicted_state[joint])
                score *= timing_quality.get(serial, 1.0)
                joint_weights[str(serial)] = score
                candidates.append((score, serial, points[joint], conf))
            weights_by_joint[name] = joint_weights
            candidates.sort(reverse=True, key=lambda item: item[0])
            category = self._category(name)
            threshold = float(disagreement_cfg[category])
            if len(candidates) >= 2:
                first, second = candidates[:2]
                disagreement = float(np.linalg.norm(first[2] - second[2]))
                if disagreement <= threshold:
                    total = first[0] + second[0]
                    output[joint] = (first[0] * first[2] + second[0] * second[2]) / max(total, 1.0e-9)
                    quality[joint] = min(1.0, 0.5 * total)
                    confidence[joint] = (first[0] * first[3] + second[0] * second[3]) / max(total, 1.0e-9)
                    sources[joint] = "FUSED"
                    states[joint] = "MEASURED"
                elif first[0] - second[0] >= quality_gap:
                    output[joint] = first[2]
                    quality[joint] = first[0]
                    confidence[joint] = first[3]
                    sources[joint] = f"CAM_{first[1]}"
                    states[joint] = "OUTLIER_GATED"
                    failure.add("JOINT_OUTLIER_REJECTED")
                elif np.isfinite(predicted_state[joint]).all():
                    output[joint] = predicted_state[joint]
                    quality[joint] = 0.80 * self.previous_quality[joint]
                    confidence[joint] = 100.0 * quality[joint]
                    sources[joint] = "TEMPORAL"
                    states[joint] = "UNCERTAIN"
                    failure.add("JOINT_OUTLIER_REJECTED")
                else:
                    output[joint] = first[2]
                    quality[joint] = 0.65 * first[0]
                    confidence[joint] = first[3]
                    sources[joint] = f"CAM_{first[1]}"
                    states[joint] = "UNCERTAIN"
            elif candidates:
                best = candidates[0]
                output[joint], quality[joint], confidence[joint] = best[2], best[0], best[3]
                sources[joint] = f"CAM_{best[1]}"
                states[joint] = "MEASURED_SINGLE"
            elif np.isfinite(predicted_state[joint]).all() and self.last_valid_timestamp_ns[joint] > 0:
                age_ms = (timestamp_ns - int(self.last_valid_timestamp_ns[joint])) / 1.0e6
                prediction_age[joint] = age_ms
                limits = self.fusion["prediction"]
                if age_ms <= float(limits["safe_hold_ms"]):
                    output[joint] = predicted_state[joint]
                    decay = math.exp(-max(0.0, age_ms - float(limits["velocity_ms"])) / max(float(limits["confidence_decay_ms"]), 1.0))
                    quality[joint] = self.previous_quality[joint] * decay
                    confidence[joint] = 100.0 * quality[joint]
                    sources[joint] = "PREDICTED" if age_ms <= float(limits["velocity_ms"]) else "HOLD"
                    states[joint] = sources[joint]
                    failure.add("PREDICTED_JOINT")
            if states[joint].startswith("MEASURED") or states[joint] == "OUTLIER_GATED":
                self.last_valid_timestamp_ns[joint] = timestamp_ns
                prediction_age[joint] = 0.0

        output = self._constrain_bones(output, quality)
        elbow_state = self._elbow_guard(output, quality, failure)
        if self.previous_timestamp_ns and 0.005 <= dt_state <= 0.15:
            # Alpha-beta-like latest-reference correction. Prediction supplies
            # continuity; high-confidence measurements remain responsive.
            # At 30 Hz this adds only a small fraction of one frame of lag.
            correction_valid = (
                np.isfinite(output).all(axis=1)
                & np.isfinite(predicted_state).all(axis=1)
            )
            alpha_min = float(self.fusion["temporal_alpha_min"])
            alpha_max = float(self.fusion["temporal_alpha_max"])
            alpha = alpha_min + (alpha_max - alpha_min) * quality
            output[correction_valid] = (
                predicted_state[correction_valid]
                + alpha[correction_valid, None]
                * (output[correction_valid] - predicted_state[correction_valid])
            )
            valid = np.isfinite(output).all(axis=1) & np.isfinite(self.previous_points).all(axis=1)
            measured_velocity = np.zeros_like(self.velocity)
            measured_velocity[valid] = (output[valid] - self.previous_points[valid]) / dt_state
            self.velocity[valid] = 0.65 * self.velocity[valid] + 0.35 * measured_velocity[valid]
        self.previous_points = output.copy()
        self.previous_quality = quality.copy()
        self.previous_timestamp_ns = timestamp_ns

        core_ids = [self.index[name] for name in CORE_NAMES if name in self.index]
        core_errors = []
        if len(prepared) >= 2:
            first, second = list(prepared.values())[:2]
            a, b = first[1], second[1]
            for joint in core_ids:
                if np.isfinite(a[joint]).all() and np.isfinite(b[joint]).all():
                    core_errors.append(float(np.linalg.norm(a[joint] - b[joint])))
        core_disagreement = float(np.median(core_errors)) if core_errors else None
        segment_quality = {
            group: float(np.mean([quality[self.index[name]] for name in members if name in self.index]))
            for group, members in SEGMENT_GROUPS.items()
        }
        return HumanStateResult(
            points=output,
            confidence=confidence,
            joint_quality=quality,
            joint_source=tuple(sources),
            joint_state=tuple(states),
            segment_quality=segment_quality,
            failure_codes=tuple(sorted(failure)),
            prediction_age_ms=prediction_age,
            elbow_state=elbow_state,
            camera_weights=weights_by_joint,
            core_disagreement_m=core_disagreement,
        )

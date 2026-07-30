"""BODY_38 conditioning and per-joint measurements for the Rerun viewer."""

from __future__ import annotations

import copy
import math
import threading
from dataclasses import asdict, dataclass, field
from typing import Any


EDGES = (
    ("PELVIS", "SPINE_1"),
    ("SPINE_1", "SPINE_2"),
    ("SPINE_2", "SPINE_3"),
    ("SPINE_3", "NECK"),
    ("NECK", "NOSE"),
    ("SPINE_3", "LEFT_CLAVICLE"),
    ("LEFT_CLAVICLE", "LEFT_SHOULDER"),
    ("LEFT_SHOULDER", "LEFT_ELBOW"),
    ("LEFT_ELBOW", "LEFT_WRIST"),
    ("SPINE_3", "RIGHT_CLAVICLE"),
    ("RIGHT_CLAVICLE", "RIGHT_SHOULDER"),
    ("RIGHT_SHOULDER", "RIGHT_ELBOW"),
    ("RIGHT_ELBOW", "RIGHT_WRIST"),
    ("PELVIS", "LEFT_HIP"),
    ("LEFT_HIP", "LEFT_KNEE"),
    ("LEFT_KNEE", "LEFT_ANKLE"),
    ("LEFT_ANKLE", "LEFT_HEEL"),
    ("LEFT_ANKLE", "LEFT_BIG_TOE"),
    ("LEFT_BIG_TOE", "LEFT_SMALL_TOE"),
    ("PELVIS", "RIGHT_HIP"),
    ("RIGHT_HIP", "RIGHT_KNEE"),
    ("RIGHT_KNEE", "RIGHT_ANKLE"),
    ("RIGHT_ANKLE", "RIGHT_HEEL"),
    ("RIGHT_ANKLE", "RIGHT_BIG_TOE"),
    ("RIGHT_BIG_TOE", "RIGHT_SMALL_TOE"),
    ("LEFT_WRIST", "LEFT_HAND_THUMB_4"),
    ("LEFT_WRIST", "LEFT_HAND_INDEX_1"),
    ("LEFT_WRIST", "LEFT_HAND_MIDDLE_4"),
    ("LEFT_WRIST", "LEFT_HAND_PINKY_1"),
    ("RIGHT_WRIST", "RIGHT_HAND_THUMB_4"),
    ("RIGHT_WRIST", "RIGHT_HAND_INDEX_1"),
    ("RIGHT_WRIST", "RIGHT_HAND_MIDDLE_4"),
    ("RIGHT_WRIST", "RIGHT_HAND_PINKY_1"),
)


@dataclass
class AnalysisConfig:
    confidence_threshold: float = 35.0
    smoothing_alpha: float = 0.55
    scale: float = 1.0
    offset_x_m: float = 0.0
    offset_y_m: float = 0.0
    offset_z_m: float = 0.0
    max_joint_speed_m_s: float = 6.0
    occlusion_hold_frames: int = 8
    playback_rate: float = 1.0
    paused: bool = False
    capture_enabled: bool = True
    selected_joint: str = "PELVIS"
    joint_offsets_m: dict[str, list[float]] = field(default_factory=dict)


class ConfigStore:
    def __init__(self, initial: AnalysisConfig | None = None) -> None:
        self._value = initial or AnalysisConfig()
        self._lock = threading.Lock()

    def snapshot(self) -> AnalysisConfig:
        with self._lock:
            return copy.deepcopy(self._value)

    def update(self, **values: Any) -> None:
        with self._lock:
            for name, value in values.items():
                if not hasattr(self._value, name):
                    raise AttributeError(name)
                setattr(self._value, name, value)

    def set_joint_offset(self, joint: str, xyz: list[float]) -> None:
        with self._lock:
            self._value.joint_offsets_m[str(joint)] = [float(v) for v in xyz]

    def as_dict(self) -> dict[str, Any]:
        return asdict(self.snapshot())


def finite_point(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    if not all(isinstance(v, (int, float)) and math.isfinite(float(v)) for v in value):
        return None
    return tuple(float(v) for v in value)


class SkeletonConditioner:
    """Deterministic, editable conditioning before kinematic analysis."""

    def __init__(self) -> None:
        self.previous: dict[str, tuple[float, float, float]] = {}
        self.missing_frames: dict[str, int] = {}
        self.previous_timestamp_ns: int | None = None

    def reset(self) -> None:
        self.previous.clear()
        self.missing_frames.clear()
        self.previous_timestamp_ns = None

    def process(
        self,
        source_packet: dict[str, Any],
        config: AnalysisConfig,
    ) -> tuple[dict[str, Any], dict[str, str]]:
        packet = copy.deepcopy(source_packet)
        names = [str(name) for name in packet.get("keypoint_names", [])]
        candidates = (
            packet.get("keypoints_3d_filtered_m")
            or packet.get("keypoints_3d_m")
            or packet.get("keypoints_3d_raw_m")
            or []
        )
        confidences = packet.get("keypoint_confidence", [])
        timestamp_ns = int(packet.get("timestamp_ns", 0) or 0)
        dt_s = None
        if (
            self.previous_timestamp_ns is not None
            and timestamp_ns > self.previous_timestamp_ns
        ):
            candidate_dt = (timestamp_ns - self.previous_timestamp_ns) / 1e9
            if 0.005 <= candidate_dt <= 0.5:
                dt_s = candidate_dt

        pelvis = None
        if "PELVIS" in names:
            pelvis_index = names.index("PELVIS")
            if pelvis_index < len(candidates):
                pelvis = finite_point(candidates[pelvis_index])

        output: list[list[float | None]] = []
        states: dict[str, str] = {}
        alpha = min(1.0, max(0.01, float(config.smoothing_alpha)))
        global_offset = (
            float(config.offset_x_m),
            float(config.offset_y_m),
            float(config.offset_z_m),
        )
        for index, name in enumerate(names):
            point = finite_point(candidates[index]) if index < len(candidates) else None
            confidence = (
                float(confidences[index])
                if index < len(confidences)
                and isinstance(confidences[index], (int, float))
                and math.isfinite(float(confidences[index]))
                else 0.0
            )
            accepted = point is not None and confidence >= config.confidence_threshold
            target = None
            if accepted:
                anchor = pelvis or (0.0, 0.0, 0.0)
                joint_offset = config.joint_offsets_m.get(name, [0.0, 0.0, 0.0])
                target = tuple(
                    anchor[axis]
                    + (point[axis] - anchor[axis]) * float(config.scale)
                    + global_offset[axis]
                    + float(joint_offset[axis])
                    for axis in range(3)
                )
                previous = self.previous.get(name)
                if previous is not None and dt_s is not None:
                    speed = math.dist(target, previous) / dt_s
                    if speed > float(config.max_joint_speed_m_s):
                        accepted = False
                        target = None
                        states[name] = "velocity_rejected"

            if accepted and target is not None:
                previous = self.previous.get(name)
                filtered = (
                    target
                    if previous is None
                    else tuple(
                        alpha * target[axis] + (1.0 - alpha) * previous[axis]
                        for axis in range(3)
                    )
                )
                self.previous[name] = filtered
                self.missing_frames[name] = 0
                output.append(list(filtered))
                states[name] = "tracked"
                continue

            misses = self.missing_frames.get(name, 0) + 1
            self.missing_frames[name] = misses
            if name in self.previous and misses <= int(config.occlusion_hold_frames):
                output.append(list(self.previous[name]))
                states.setdefault(name, "held")
            else:
                output.append([None, None, None])
                states.setdefault(name, "missing")

        packet["keypoints_3d_m"] = output
        packet["analysis_conditioning"] = {
            "confidence_threshold": config.confidence_threshold,
            "smoothing_alpha": alpha,
            "scale": config.scale,
            "global_offset_m": list(global_offset),
            "max_joint_speed_m_s": config.max_joint_speed_m_s,
            "occlusion_hold_frames": config.occlusion_hold_frames,
            "joint_offsets_m": copy.deepcopy(config.joint_offsets_m),
            "tracking_source": states,
        }
        self.previous_timestamp_ns = timestamp_ns or self.previous_timestamp_ns
        return packet, states


def joint_angle_names(joint_name: str, angles: dict[str, Any]) -> list[str]:
    token = joint_name.lower()
    short = token.replace("left_", "").replace("right_", "")
    matches = []
    for name in angles:
        lowered = name.lower()
        if token in lowered or short in lowered:
            if joint_name.startswith("LEFT_") and "right_" in lowered:
                continue
            if joint_name.startswith("RIGHT_") and "left_" in lowered:
                continue
            matches.append(name)
    if joint_name in {"SPINE_1", "SPINE_2", "SPINE_3", "PELVIS"}:
        matches.extend(
            name for name in angles if name.startswith("trunk_") and name not in matches
        )
    if joint_name in {"NECK", "NOSE"}:
        matches.extend(
            name
            for name in angles
            if (name.startswith("head_") or name.startswith("neck_"))
            and name not in matches
        )
    return sorted(set(matches))

"""Convert Stereolabs ZED BODY_38 frames to GMR's SMPL-X-like input.

This module is deliberately perception-only.  It produces global Cartesian
targets for GMR and never sends commands to a robot.  ZED capture data in this
project already uses X-forward, Y-left, Z-up coordinates.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


GMR_BODY38_MAP = {
    "pelvis": "PELVIS",
    "spine3": "SPINE_3",
    "left_hip": "LEFT_HIP",
    "right_hip": "RIGHT_HIP",
    "left_knee": "LEFT_KNEE",
    "right_knee": "RIGHT_KNEE",
    "left_foot": "LEFT_ANKLE",
    "right_foot": "RIGHT_ANKLE",
    "left_shoulder": "LEFT_SHOULDER",
    "right_shoulder": "RIGHT_SHOULDER",
    "left_elbow": "LEFT_ELBOW",
    "right_elbow": "RIGHT_ELBOW",
    "left_wrist": "LEFT_WRIST",
    "right_wrist": "RIGHT_WRIST",
}

IDENTITY_WXYZ = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


@dataclass(frozen=True)
class AdaptedFrame:
    human_data: dict[str, tuple[np.ndarray, np.ndarray]]
    valid_targets: int
    used_memory_targets: int
    raw_fallback_targets: int
    occlusion_held_targets: int
    pelvis_height_m: float
    ground_z_m: float


class Body38ToGMR:
    """Stateful BODY_38 adapter with short per-landmark dropout memory."""

    def __init__(
        self,
        *,
        confidence_threshold: float = 35.0,
        memory_seconds: float = 0.20,
        nominal_fps: float = 30.0,
        max_joint_speed_m_s: float = 5.0,
        max_pelvis_speed_m_s: float = 2.5,
        max_raw_fallback_frames: int = 2,
        reacquire_frames: int = 3,
        persistent_memory_targets: tuple[str, ...] = (),
        max_segment_turn_deg: float = 45.0,
        occluded_segment_turn_deg: float = 30.0,
    ) -> None:
        self.confidence_threshold = float(confidence_threshold)
        self.max_memory_frames = max(0, int(round(memory_seconds * nominal_fps)))
        self.nominal_fps = float(nominal_fps)
        self.max_joint_step_m = float(max_joint_speed_m_s) / self.nominal_fps
        self.max_pelvis_step_m = float(max_pelvis_speed_m_s) / self.nominal_fps
        self.max_raw_fallback_frames = max(0, int(max_raw_fallback_frames))
        self.reacquire_frames = max(1, int(reacquire_frames))
        self.persistent_memory_targets = frozenset(persistent_memory_targets)
        self.max_segment_turn_deg = float(max_segment_turn_deg)
        self.occluded_segment_turn_deg = float(occluded_segment_turn_deg)
        self._memory: dict[str, tuple[np.ndarray, int]] = {}
        self._raw_fallback_streak: dict[str, int] = {}
        self._reacquire: dict[str, tuple[np.ndarray, int]] = {}
        self._just_reacquired: set[str] = set()
        self._frame_no = 0
        self.rejected_outliers = 0
        self.last_rejection_reason = "none"

    @staticmethod
    def _is_finite_xyz(value: Any) -> bool:
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            return False
        return bool(np.all(np.isfinite(np.asarray(value, dtype=np.float64))))

    def _extract_points(
        self, frame: dict[str, Any]
    ) -> tuple[dict[str, np.ndarray], int, int, int]:
        names = frame.get("keypoint_names", [])
        points = frame.get("keypoints_3d_m")
        if points is None:
            points = frame.get("keypoints_3d_filtered_m", [])
        raw_points = frame.get("keypoints_3d_raw_m", [])
        confidence = frame.get("keypoint_confidence", [])
        index = {str(name): i for i, name in enumerate(names)}
        self._just_reacquired.clear()

        result: dict[str, np.ndarray] = {}
        direct = 0
        remembered = 0
        raw_fallback = 0
        for gmr_name, zed_name in GMR_BODY38_MAP.items():
            idx = index.get(zed_name)
            point = points[idx] if idx is not None and idx < len(points) else None
            conf = confidence[idx] if idx is not None and idx < len(confidence) else None
            confidence_ok = conf is None or (
                np.isfinite(float(conf)) and float(conf) >= self.confidence_threshold
            )
            if self._is_finite_xyz(point) and confidence_ok:
                xyz = np.asarray(point, dtype=np.float64)
                cached = self._memory.get(gmr_name)
                persistent = gmr_name in self.persistent_memory_targets
                max_step = (
                    self.max_pelvis_step_m
                    if gmr_name == "pelvis"
                    else self.max_joint_step_m
                )
                cache_is_stale = (
                    cached is not None
                    and not persistent
                    and self._frame_no - cached[1] > self.max_memory_frames
                )
                coherent = cached is None or np.linalg.norm(xyz - cached[0]) <= max_step
                candidate = self._reacquire.get(gmr_name)
                if candidate is not None:
                    if np.linalg.norm(xyz - candidate[0]) > max_step:
                        count = 1
                    else:
                        count = candidate[1] + 1
                    self._reacquire[gmr_name] = (xyz.copy(), count)
                    if count >= self.reacquire_frames:
                        result[gmr_name] = xyz
                        self._memory[gmr_name] = (xyz.copy(), self._frame_no)
                        self._raw_fallback_streak[gmr_name] = 0
                        self._reacquire.pop(gmr_name, None)
                        self._just_reacquired.add(gmr_name)
                        direct += 1
                        continue
                    self.rejected_outliers += 1
                elif coherent and not cache_is_stale:
                    result[gmr_name] = xyz
                    self._memory[gmr_name] = (xyz.copy(), self._frame_no)
                    self._raw_fallback_streak[gmr_name] = 0
                    direct += 1
                    continue
                else:
                    self._reacquire[gmr_name] = (xyz.copy(), 1)
                    self.rejected_outliers += 1

            cached = self._memory.get(gmr_name)
            persistent = gmr_name in self.persistent_memory_targets
            # ZED can temporarily mark a filtered landmark invalid even though
            # the raw stereo point is coherent.  Accept raw data only when it
            # is continuous with a recent trusted point; a raw point never
            # bootstraps a track and still passes the whole-body geometry gate.
            raw_point = (
                raw_points[idx]
                if idx is not None and idx < len(raw_points)
                else None
            )
            if (
                cached is not None
                and (
                    persistent
                    or self._frame_no - cached[1] <= self.max_memory_frames
                )
                and self._is_finite_xyz(raw_point)
                and self._raw_fallback_streak.get(gmr_name, 0)
                < self.max_raw_fallback_frames
            ):
                raw_xyz = np.asarray(raw_point, dtype=np.float64)
                max_step = (
                    self.max_pelvis_step_m
                    if gmr_name == "pelvis"
                    else self.max_joint_step_m
                )
                if np.linalg.norm(raw_xyz - cached[0]) <= max_step:
                    result[gmr_name] = raw_xyz
                    # Raw low-confidence output is never promoted into trusted
                    # memory.  Otherwise a predicted/overlapped limb can keep
                    # itself alive indefinitely and contaminate later frames.
                    self._raw_fallback_streak[gmr_name] = (
                        self._raw_fallback_streak.get(gmr_name, 0) + 1
                    )
                    raw_fallback += 1
                    continue
            if cached is not None and (
                persistent
                or self._frame_no - cached[1] <= self.max_memory_frames
            ):
                result[gmr_name] = cached[0].copy()
                remembered += 1
        return result, direct, remembered, raw_fallback

    @staticmethod
    def _direction_angle_deg(first: np.ndarray, second: np.ndarray) -> float:
        first_norm = float(np.linalg.norm(first))
        second_norm = float(np.linalg.norm(second))
        if first_norm < 1e-6 or second_norm < 1e-6:
            return 180.0
        cosine = float(
            np.clip(
                np.dot(first / first_norm, second / second_norm),
                -1.0,
                1.0,
            )
        )
        return float(np.degrees(np.arccos(cosine)))

    def _stabilize_arm_chains(
        self,
        points: dict[str, np.ndarray],
        previous: dict[str, tuple[np.ndarray, int]],
    ) -> int:
        """Hold only an implausible arm segment during self-occlusion.

        BODY_38 keeps human bone lengths nearly constant, but a wrist can be
        associated with the wrong visible limb when both arms overlap the
        torso. Absolute point-speed gating is insufficient because such a swap
        can remain within one forearm length. Direction continuity catches the
        identity flip while preserving the other arm and the live torso.
        """

        torso_names = (
            "pelvis",
            "left_shoulder",
            "right_shoulder",
        )
        if any(name not in points for name in torso_names):
            return 0
        left_shoulder = points["left_shoulder"]
        right_shoulder = points["right_shoulder"]
        shoulder_center = 0.5 * (left_shoulder + right_shoulder)
        shoulder_axis = left_shoulder - right_shoulder
        shoulder_width = float(np.linalg.norm(shoulder_axis))
        if shoulder_width < 0.10:
            return 0
        lateral = shoulder_axis / shoulder_width
        shoulder_z = float(shoulder_center[2])
        pelvis_z = float(points["pelvis"][2])

        held: set[str] = set()
        for side, own_sign in (("left", 1.0), ("right", -1.0)):
            shoulder = f"{side}_shoulder"
            elbow = f"{side}_elbow"
            wrist = f"{side}_wrist"
            chain = (shoulder, elbow, wrist)
            if any(name not in points or name not in previous for name in chain):
                continue
            # Do not repeatedly judge a segment that already came from memory.
            elbow_is_new = self._memory.get(elbow, (None, -1))[1] == self._frame_no
            wrist_is_new = self._memory.get(wrist, (None, -1))[1] == self._frame_no
            if not elbow_is_new and not wrist_is_new:
                continue
            # Re-acquisition has already required multiple mutually coherent
            # observations. Comparing it once more with the old held pose
            # would create a permanent hold/re-acquire loop during real motion.
            if elbow in self._just_reacquired or wrist in self._just_reacquired:
                continue

            wrist_lateral = float(np.dot(points[wrist] - shoulder_center, lateral))
            inside_torso_projection = (
                abs(wrist_lateral) < 0.55 * shoulder_width
                and pelvis_z - 0.05
                <= float(points[wrist][2])
                <= shoulder_z + 0.20
            )
            crossed_centerline = (
                wrist_lateral * own_sign < -0.15 * shoulder_width
            )
            turn_limit = (
                self.occluded_segment_turn_deg
                if inside_torso_projection or crossed_centerline
                else self.max_segment_turn_deg
            )

            previous_shoulder = previous[shoulder][0]
            previous_elbow = previous[elbow][0]
            previous_wrist = previous[wrist][0]
            upper_turn = self._direction_angle_deg(
                points[elbow] - points[shoulder],
                previous_elbow - previous_shoulder,
            )
            forearm_turn = self._direction_angle_deg(
                points[wrist] - points[elbow],
                previous_wrist - previous_elbow,
            )

            names_to_hold: tuple[str, ...] = ()
            if elbow_is_new and upper_turn > turn_limit:
                names_to_hold = (elbow, wrist)
            elif wrist_is_new and forearm_turn > turn_limit:
                names_to_hold = (wrist,)
            for name in names_to_hold:
                candidate = points[name].copy()
                points[name] = previous[name][0].copy()
                self._memory[name] = (
                    previous[name][0].copy(),
                    previous[name][1],
                )
                self._reacquire[name] = (candidate, 1)
                held.add(name)

        self.rejected_outliers += len(held)
        return len(held)

    @staticmethod
    def _geometry_is_plausible(points: dict[str, np.ndarray]) -> bool:
        """Reject torn/fragmented frames before they can reach the IK solver."""

        def length(a: str, b: str) -> float | None:
            if a not in points or b not in points:
                return None
            return float(np.linalg.norm(points[a] - points[b]))

        checks = (
            ("pelvis", "spine3", 0.12, 0.75),
            ("left_hip", "right_hip", 0.10, 0.60),
            ("left_shoulder", "right_shoulder", 0.18, 0.85),
            ("left_shoulder", "left_elbow", 0.12, 0.65),
            ("left_elbow", "left_wrist", 0.12, 0.65),
            ("right_shoulder", "right_elbow", 0.12, 0.65),
            ("right_elbow", "right_wrist", 0.12, 0.65),
            ("left_hip", "left_knee", 0.18, 0.85),
            ("left_knee", "left_foot", 0.18, 0.85),
            ("right_hip", "right_knee", 0.18, 0.85),
            ("right_knee", "right_foot", 0.18, 0.85),
        )
        evaluated = 0
        for first, second, minimum, maximum in checks:
            segment = length(first, second)
            if segment is None:
                continue
            evaluated += 1
            if not minimum <= segment <= maximum:
                return False
        return evaluated >= 5

    @staticmethod
    def _ground_height(points: dict[str, np.ndarray]) -> float:
        candidates = [
            points[name][2]
            for name in ("left_foot", "right_foot")
            if name in points
        ]
        if candidates:
            return float(np.median(candidates))
        pelvis = points.get("pelvis")
        return float(pelvis[2] - 0.90) if pelvis is not None else 0.0

    def adapt(self, frame: dict[str, Any]) -> AdaptedFrame | None:
        """Return a GMR frame, or ``None`` when the essential torso is missing."""
        self._frame_no += 1
        # Point extraction is speculative. Commit new trusted landmarks only
        # after the complete skeleton passes all gates.
        memory_before = {
            name: (value.copy(), frame_no)
            for name, (value, frame_no) in self._memory.items()
        }
        points, direct, remembered, raw_fallback = self._extract_points(frame)
        occlusion_held = self._stabilize_arm_chains(points, memory_before)
        direct = max(0, direct - occlusion_held)
        remembered += occlusion_held
        if "pelvis" not in points or "spine3" not in points:
            self._memory = memory_before
            self.last_rejection_reason = "missing_torso"
            return None
        if direct + raw_fallback < 8:
            self._memory = memory_before
            self.last_rejection_reason = "too_few_direct_targets"
            return None
        if not self._geometry_is_plausible(points):
            self._memory = memory_before
            self.last_rejection_reason = "implausible_geometry"
            return None
        self.last_rejection_reason = "none"

        # Remove camera translation and keep the subject upright on z=0.  This
        # prevents camera distance from becoming a robot base displacement.
        pelvis = points["pelvis"].copy()
        ground_z = self._ground_height(points)
        origin = np.array([pelvis[0], pelvis[1], ground_z], dtype=np.float64)

        human_data: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for name, xyz in points.items():
            human_data[name] = (xyz - origin, IDENTITY_WXYZ.copy())

        return AdaptedFrame(
            human_data=human_data,
            valid_targets=direct,
            used_memory_targets=remembered,
            raw_fallback_targets=raw_fallback,
            occlusion_held_targets=occlusion_held,
            pelvis_height_m=float(pelvis[2] - ground_z),
            ground_z_m=ground_z,
        )

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

# Official BODY_38 kinematic parent graph.  Stereolabs exposes every joint
# orientation relative to its parent; a retargeter must compose the complete
# path from PELVIS before using it as a global orientation target.
BODY38_PARENT = {
    "SPINE_1": "PELVIS",
    "SPINE_2": "SPINE_1",
    "SPINE_3": "SPINE_2",
    "NECK": "SPINE_3",
    "NOSE": "NECK",
    "LEFT_EYE": "NOSE",
    "RIGHT_EYE": "NOSE",
    "LEFT_EAR": "LEFT_EYE",
    "RIGHT_EAR": "RIGHT_EYE",
    "LEFT_CLAVICLE": "SPINE_3",
    "RIGHT_CLAVICLE": "SPINE_3",
    "LEFT_SHOULDER": "LEFT_CLAVICLE",
    "RIGHT_SHOULDER": "RIGHT_CLAVICLE",
    "LEFT_ELBOW": "LEFT_SHOULDER",
    "RIGHT_ELBOW": "RIGHT_SHOULDER",
    "LEFT_WRIST": "LEFT_ELBOW",
    "RIGHT_WRIST": "RIGHT_ELBOW",
    "LEFT_HIP": "PELVIS",
    "RIGHT_HIP": "PELVIS",
    "LEFT_KNEE": "LEFT_HIP",
    "RIGHT_KNEE": "RIGHT_HIP",
    "LEFT_ANKLE": "LEFT_KNEE",
    "RIGHT_ANKLE": "RIGHT_KNEE",
    "LEFT_BIG_TOE": "LEFT_ANKLE",
    "RIGHT_BIG_TOE": "RIGHT_ANKLE",
    "LEFT_SMALL_TOE": "LEFT_ANKLE",
    "RIGHT_SMALL_TOE": "RIGHT_ANKLE",
    "LEFT_HEEL": "LEFT_ANKLE",
    "RIGHT_HEEL": "RIGHT_ANKLE",
    "LEFT_HAND_THUMB_4": "LEFT_WRIST",
    "RIGHT_HAND_THUMB_4": "RIGHT_WRIST",
    "LEFT_HAND_INDEX_1": "LEFT_WRIST",
    "RIGHT_HAND_INDEX_1": "RIGHT_WRIST",
    "LEFT_HAND_MIDDLE_4": "LEFT_WRIST",
    "RIGHT_HAND_MIDDLE_4": "RIGHT_WRIST",
    "LEFT_HAND_PINKY_1": "LEFT_WRIST",
    "RIGHT_HAND_PINKY_1": "RIGHT_WRIST",
}

IDENTITY_WXYZ = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def _quat_xyzw_to_matrix(value: Any) -> np.ndarray | None:
    try:
        q = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if q.shape != (4,) or not np.isfinite(q).all():
        return None
    norm = float(np.linalg.norm(q))
    if norm < 1e-8:
        return None
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _matrix_to_quat_wxyz(matrix: np.ndarray) -> np.ndarray:
    """Convert a proper 3x3 rotation to a normalized scalar-first quaternion."""
    m = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    else:
        axis = int(np.argmax(np.diag(m)))
        if axis == 0:
            s = np.sqrt(max(1e-12, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) * 2.0
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif axis == 1:
            s = np.sqrt(max(1e-12, 1.0 + m[1, 1] - m[0, 0] - m[2, 2])) * 2.0
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = np.sqrt(max(1e-12, 1.0 + m[2, 2] - m[0, 0] - m[1, 1])) * 2.0
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    return q / max(float(np.linalg.norm(q)), 1e-12)


@dataclass(frozen=True)
class AdaptedFrame:
    human_data: dict[str, tuple[np.ndarray, np.ndarray]]
    valid_targets: int
    used_memory_targets: int
    raw_fallback_targets: int
    occlusion_held_targets: int
    pelvis_height_m: float
    ground_z_m: float
    workspace_projection_count: int
    operator_calibrated: bool


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
        fixed_stance: bool = False,
        anatomical_branch_continuity: bool = True,
        branch_confirm_frames: int = 4,
        simple_arm_mapping: bool = True,
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
        self.fixed_stance = bool(fixed_stance)
        self.anatomical_branch_continuity = bool(anatomical_branch_continuity)
        self.branch_confirm_frames = int(np.clip(branch_confirm_frames, 3, 5))
        self.simple_arm_mapping = bool(simple_arm_mapping)
        self._memory: dict[str, tuple[np.ndarray, int]] = {}
        self._velocity_memory: dict[str, np.ndarray] = {}
        self._raw_fallback_streak: dict[str, int] = {}
        self._reacquire: dict[str, tuple[np.ndarray, int]] = {}
        self._just_reacquired: set[str] = set()
        # Unit pole vectors define the arm-bend plane.  Keeping them as state
        # removes the mirrored two-bone IK branch that appears when a nearly
        # straight elbow or torso overlap makes the measured pole ill-defined.
        self._arm_pole_memory: dict[str, np.ndarray] = {}
        self._arm_plane_normal_memory: dict[str, np.ndarray] = {}
        self._arm_pending_branch: dict[str, tuple[int, int]] = {}
        self.last_arm_branch_sign: dict[str, int] = {"left": 1, "right": 1}
        self.last_arm_pole_source: dict[str, str] = {
            "left": "uninitialized", "right": "uninitialized"
        }
        self._frame_no = 0
        self.rejected_outliers = 0
        self.last_rejection_reason = "none"

    def reset(self) -> None:
        """Forget every landmark and bend-plane from the previous operator.

        Operator re-selection/calibration defines a new control session.  A
        short dropout may use the memories below, but carrying them across an
        explicit reset can pin one arm to the preceding session's last pose.
        """
        self._memory.clear()
        self._velocity_memory.clear()
        self._raw_fallback_streak.clear()
        self._reacquire.clear()
        self._just_reacquired.clear()
        self._arm_pole_memory.clear()
        self._arm_plane_normal_memory.clear()
        self._arm_pending_branch.clear()
        self.last_arm_branch_sign = {"left": 1, "right": 1}
        self.last_arm_pole_source = {
            "left": "uninitialized", "right": "uninitialized"
        }
        self._frame_no = 0
        self.rejected_outliers = 0
        self.last_rejection_reason = "control_session_reset"

    @property
    def branch_change_pending(self) -> bool:
        return any(count > 0 for _, count in self._arm_pending_branch.values())

    @staticmethod
    def _is_finite_xyz(value: Any) -> bool:
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            return False
        return bool(np.all(np.isfinite(np.asarray(value, dtype=np.float64))))

    def _extract_points(
        self, frame: dict[str, Any]
    ) -> tuple[dict[str, np.ndarray], int, int, int]:
        names = frame.get("keypoint_names", [])
        pelvis_frame = frame.get("pelvis_frame") or {}
        points = pelvis_frame.get("keypoints_m")
        using_pelvis_frame = points is not None
        if points is None:
            points = frame.get("keypoints_3d_m")
        if points is None:
            points = frame.get("keypoints_3d_filtered_m", [])
        # Camera-space raw points must never be mixed with pelvis-local points.
        raw_points = [] if using_pelvis_frame else frame.get("keypoints_3d_raw_m", [])
        raw_elbows_local: dict[str, list[float]] = {}
        if using_pelvis_frame:
            # Preserve the ZED SDK's measured elbow position instead of
            # deriving an elbow from independently filtered XYZ landmarks.
            # Convert the raw camera-space elbow into the same pelvis-local
            # frame first; shoulder/wrist targets remain the stable filtered
            # landmarks and the arm-chain projection below restores fixed G1
            # link lengths. This is deliberately limited to the two elbows.
            raw_camera = frame.get("keypoints_3d_raw_m") or []
            origin = pelvis_frame.get("origin_camera_m")
            rotation = pelvis_frame.get("rotation_camera_from_pelvis")
            try:
                origin_np = np.asarray(origin, dtype=np.float64)
                rotation_np = np.asarray(rotation, dtype=np.float64)
                if origin_np.shape == (3,) and rotation_np.shape == (3, 3):
                    name_to_index = {
                        str(name): idx for idx, name in enumerate(names)
                    }
                    for gmr_name, zed_name in (
                        ("left_elbow", "LEFT_ELBOW"),
                        ("right_elbow", "RIGHT_ELBOW"),
                    ):
                        raw_index = name_to_index.get(zed_name)
                        if raw_index is None or raw_index >= len(raw_camera):
                            continue
                        raw_value = raw_camera[raw_index]
                        if self._is_finite_xyz(raw_value):
                            raw_elbows_local[gmr_name] = (
                                rotation_np.T
                                @ (np.asarray(raw_value, dtype=np.float64) - origin_np)
                            ).tolist()
            except (TypeError, ValueError):
                raw_elbows_local = {}
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
            if gmr_name in raw_elbows_local and confidence_ok:
                point = raw_elbows_local[gmr_name]
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
                    if cached is not None:
                        elapsed = max(1, self._frame_no - cached[1])
                        measured_velocity = (
                            (xyz - cached[0]) * self.nominal_fps / elapsed
                        )
                        speed = float(np.linalg.norm(measured_velocity))
                        limit = (
                            2.5 if gmr_name == "pelvis" else 5.0
                        )
                        if speed > limit:
                            measured_velocity *= limit / max(speed, 1e-9)
                        previous_velocity = self._velocity_memory.get(
                            gmr_name, np.zeros(3, dtype=np.float64)
                        )
                        self._velocity_memory[gmr_name] = (
                            0.65 * previous_velocity + 0.35 * measured_velocity
                        )
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
                age = max(0, self._frame_no - cached[1])
                velocity = self._velocity_memory.get(gmr_name)
                if (
                    gmr_name in {
                        "left_elbow", "left_wrist",
                        "right_elbow", "right_wrist",
                    }
                    and velocity is not None
                    and 0 < age <= 2
                ):
                    # Two-frame constant-velocity prediction bridges a brief
                    # BODY_38 dropout without leaving the arm in mid-air.
                    # Longer gaps still use the conservative held pose.
                    displacement = velocity * (age / self.nominal_fps)
                    maximum = self.max_joint_step_m * age
                    norm = float(np.linalg.norm(displacement))
                    if norm > maximum:
                        displacement *= maximum / max(norm, 1e-9)
                    result[gmr_name] = cached[0].copy() + displacement
                else:
                    result[gmr_name] = cached[0].copy()
                remembered += 1
        return result, direct, remembered, raw_fallback

    @staticmethod
    def _global_orientations(
        frame: dict[str, Any],
    ) -> dict[str, np.ndarray]:
        """Reconstruct pelvis-local global BODY_38 rotations as WXYZ.

        ZED local rotations are relative to the BODY_38 parent and identity is
        the fitted T-pose.  They therefore cannot be sent directly to GMR.
        The camera/world heading is removed with the same pelvis frame used by
        the position targets, keeping motion independent of camera placement.
        """
        names = [str(name) for name in frame.get("keypoint_names", [])]
        local_values = frame.get("local_orientation_per_joint_xyzw") or []
        index = {name: idx for idx, name in enumerate(names)}
        root = _quat_xyzw_to_matrix(frame.get("global_root_orientation_xyzw"))
        if root is None or "PELVIS" not in index:
            return {}

        local: dict[str, np.ndarray] = {}
        for name, idx in index.items():
            if idx < len(local_values):
                rotation = _quat_xyzw_to_matrix(local_values[idx])
                if rotation is not None:
                    local[name] = rotation

        global_camera: dict[str, np.ndarray] = {"PELVIS": root}

        def resolve(name: str) -> np.ndarray | None:
            if name in global_camera:
                return global_camera[name]
            parent_name = BODY38_PARENT.get(name)
            relative = local.get(name)
            if parent_name is None or relative is None:
                return None
            parent = resolve(parent_name)
            if parent is None:
                return None
            global_camera[name] = parent @ relative
            return global_camera[name]

        for name in names:
            resolve(name)

        pelvis_frame = frame.get("pelvis_frame") or {}
        camera_from_pelvis = np.asarray(
            pelvis_frame.get("rotation_camera_from_pelvis", root),
            dtype=np.float64,
        )
        if (
            camera_from_pelvis.shape != (3, 3)
            or not np.isfinite(camera_from_pelvis).all()
        ):
            camera_from_pelvis = root
        pelvis_from_camera = camera_from_pelvis.T
        output: dict[str, np.ndarray] = {}
        for gmr_name, zed_name in GMR_BODY38_MAP.items():
            rotation = global_camera.get(zed_name)
            if rotation is not None:
                output[gmr_name] = _matrix_to_quat_wxyz(
                    pelvis_from_camera @ rotation
                )
        return output

    def _map_operator_to_g1_workspace(
        self, points: dict[str, np.ndarray], frame: dict[str, Any]
    ) -> tuple[dict[str, np.ndarray], int, bool]:
        """Normalize calibrated human limbs into G1's reachable task space.

        This is a task-space mapping, not a joint command. GMR still solves the
        official G1 model afterwards and the feasibility layer remains the
        final command boundary.
        """
        profile = ((frame.get("calibration") or {}).get("profile") or {})
        calibrated = (frame.get("calibration") or {}).get("state") == "READY"
        if not calibrated or not profile:
            return points, 0, False
        mapped = {name: value.copy() for name, value in points.items()}
        required = {
            "pelvis", "spine3", "left_shoulder", "right_shoulder",
            "left_elbow", "right_elbow", "left_wrist", "right_wrist",
        }
        if not required.issubset(mapped):
            return points, 0, True

        pelvis = mapped["pelvis"]
        human_torso = float(profile.get("torso_length_m", 0.48))
        # Nominal link centers measured from the official G1 MuJoCo model.
        # GMR tasks target link origins, not anatomical segment endpoints.
        mapped["spine3"] = pelvis + np.array([0.0, 0.0, 0.044])
        lateral = mapped["left_shoulder"] - mapped["right_shoulder"]
        lateral_norm = float(np.linalg.norm(lateral))
        # Anatomical arm origin is the official shoulder-pitch body, before
        # all three shoulder rotations. Targeting shoulder_yaw_link instead
        # over-constrains pitch/roll because that body's origin itself moves.
        shoulder_center = pelvis + np.array([0.0, 0.0, 0.29178])
        if lateral_norm > 1e-6:
            lateral /= lateral_norm
            mapped["left_shoulder"] = shoulder_center + lateral * 0.10022
            mapped["right_shoulder"] = shoulder_center - lateral * 0.10022

        projection_count = 0
        overlap = (frame.get("occlusion_analysis") or {}).get(
            "arm_torso_overlap", {}
        )
        for side in ("left", "right"):
            shoulder_name = f"{side}_shoulder"
            elbow_name = f"{side}_elbow"
            wrist_name = f"{side}_wrist"
            human_shoulder = points[shoulder_name]
            upper_vec = points[elbow_name] - human_shoulder
            fore_vec = points[wrist_name] - points[elbow_name]
            upper_norm = float(np.linalg.norm(upper_vec))
            fore_norm = float(np.linalg.norm(fore_vec))
            if upper_norm < 1e-6 or fore_norm < 1e-6:
                continue
            # Effective G1 shoulder-to-elbow and elbow-to-wrist lengths from
            # the official 23-DOF kinematic chain, with a small reach margin.
            # Official 23-DOF link origins: shoulder_pitch->elbow ~= 0.193 m,
            # elbow->wrist_roll_rubber_hand ~= 0.101 m.  The old 0.185 m
            # value belonged to the 29-DOF model's extra pitch/yaw wrist chain
            # and made GMR solve a robot different from Isaac's articulation.
            g1_upper, g1_fore = 0.1925, 0.101
            shoulder = mapped[shoulder_name]
            elbow_hint = shoulder + upper_vec / upper_norm * g1_upper
            wrist = elbow_hint + fore_vec / fore_norm * g1_fore

            # The upper-body command path consumes limb directions.  These
            # two independently scaled vectors already form an anatomical,
            # reachable two-link target.  Reconstructing the elbow a second
            # time from a projected wrist introduces a competing bend-plane
            # state machine and was the source of visible branch flips in the
            # 2026-08-19 recording.  Retain the legacy reconstruction below
            # for controlled experiments, but keep live imitation direct.
            if self.simple_arm_mapping:
                mapped[elbow_name] = elbow_hint
                mapped[wrist_name] = wrist
                self.last_arm_pole_source[side] = "direct_limb_directions"
                self._arm_pending_branch.pop(side, None)
                continue

            # A single frontal stereo view is least reliable in depth when an
            # arm is nearly vertical. Do not allow that noisy depth component
            # to send a hand far behind G1's shoulder. A small backward reach
            # remains available for natural lateral/upward motions.
            minimum_wrist_x = float(shoulder[0] - 0.060)
            if wrist[0] < minimum_wrist_x:
                wrist[0] = minimum_wrist_x
                projection_count += 1

            shoulder_to_wrist = wrist - shoulder
            requested = float(np.linalg.norm(shoulder_to_wrist))
            minimum_reach = abs(g1_fore - g1_upper) + 1.0e-4
            maximum_reach = g1_fore + g1_upper - 0.002
            distance = float(np.clip(requested, minimum_reach, maximum_reach))
            if abs(distance - requested) > 1.0e-8:
                projection_count += 1
            if requested < 1.0e-8:
                direction = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            else:
                direction = shoulder_to_wrist / requested
            wrist = shoulder + direction * distance

            # Classical analytic two-bone reconstruction: preserve both G1
            # link lengths while selecting the bend plane closest to the
            # observed human elbow. This avoids independent XYZ clamping that
            # would change bone lengths or flip the elbow behind the torso.
            along = (
                g1_upper * g1_upper
                - g1_fore * g1_fore
                + distance * distance
            ) / (2.0 * distance)
            height = float(np.sqrt(max(0.0, g1_upper * g1_upper - along * along)))
            center = shoulder + direction * along
            observed_pole = elbow_hint - center
            observed_pole -= direction * float(np.dot(observed_pole, direction))
            observed_norm = float(np.linalg.norm(observed_pole))

            # Anatomical neutral: elbows prefer the operator's own lateral
            # side with a small downward component.  It is only a fallback;
            # a coherent ZED elbow remains the primary measurement.
            side_sign = 1.0 if side == "left" else -1.0
            neutral_pole = np.array([0.0, side_sign, -0.25], dtype=np.float64)
            neutral_pole -= direction * float(np.dot(neutral_pole, direction))
            neutral_norm = float(np.linalg.norm(neutral_pole))
            if neutral_norm < 1.0e-8:
                neutral_pole = np.cross(direction, np.array([1.0, 0.0, 0.0]))
                neutral_norm = float(np.linalg.norm(neutral_pole))
            neutral_pole /= max(neutral_norm, 1.0e-8)

            previous_pole = self._arm_pole_memory.get(side)
            if previous_pole is not None:
                previous_pole = previous_pole - direction * float(
                    np.dot(previous_pole, direction)
                )
                previous_norm = float(np.linalg.norm(previous_pole))
                previous_pole = (
                    previous_pole / previous_norm
                    if previous_norm > 1.0e-8
                    else None
                )

            ambiguous = bool(overlap.get(side)) or height < 0.025
            if observed_norm > 1.0e-8:
                observed_pole /= observed_norm
                # The directed shoulder-elbow-wrist plane distinguishes the
                # two mirrored elbow branches. A genuine branch transition
                # must remain coherent for several frames; a single ZED depth
                # swap is never allowed to reverse the robot elbow.
                plane_normal = np.cross(upper_vec, fore_vec)
                plane_norm = float(np.linalg.norm(plane_normal))
                if plane_norm > 1.0e-8:
                    plane_normal /= plane_norm
                else:
                    plane_normal = self._arm_plane_normal_memory.get(side)
                previous_normal = self._arm_plane_normal_memory.get(side)
                if (
                    self.anatomical_branch_continuity
                    and plane_normal is not None
                    and previous_normal is not None
                ):
                    candidate_sign = (
                        1
                        if float(np.dot(plane_normal, previous_normal)) >= 0.0
                        else -1
                    )
                    if ambiguous:
                        # Torso overlap and near-full extension are not
                        # reliable evidence of a genuine elbow branch change.
                        # Hold the last trustworthy bend plane, but do not
                        # leave a perpetual pending event that would invoke
                        # MIRROR on every frame.
                        self._arm_pending_branch.pop(side, None)
                        plane_normal = previous_normal.copy()
                        if previous_pole is not None:
                            observed_pole = previous_pole.copy()
                        self.last_arm_pole_source[side] = "ambiguous_plane_hold"
                    elif candidate_sign < 0:
                        pending_sign, pending_count = self._arm_pending_branch.get(
                            side, (0, 0)
                        )
                        pending_count = pending_count + 1 if pending_sign == candidate_sign else 1
                        self._arm_pending_branch[side] = (
                            candidate_sign, pending_count
                        )
                        if pending_count < self.branch_confirm_frames:
                            observed_pole *= -1.0
                            plane_normal *= -1.0
                            self.last_arm_pole_source[side] = "branch_guard"
                        else:
                            self.last_arm_branch_sign[side] *= -1
                            self._arm_pending_branch.pop(side, None)
                    else:
                        self._arm_pending_branch.pop(side, None)
                    # Close to full extension, the elbow plane is singular;
                    # keep the last reliable normal regardless of the current
                    # noisy cross-product.
                    if height < 0.025:
                        plane_normal = previous_normal.copy()
                if plane_normal is not None and height >= 0.025:
                    self._arm_plane_normal_memory[side] = plane_normal.copy()
                if previous_pole is None:
                    alpha = 0.20 if ambiguous else 0.95
                    pole = alpha * observed_pole + (1.0 - alpha) * neutral_pole
                    source = "observed" if not ambiguous else "neutral_blend"
                else:
                    agreement = float(np.dot(observed_pole, previous_pole))
                    # A sign reversal in one frame is the mirrored IK branch,
                    # not a physically possible elbow motion.  Let a genuine
                    # bend-plane change arrive over several coherent frames.
                    alpha = 0.12 if ambiguous or agreement < -0.20 else 0.90
                    pole = alpha * observed_pole + (1.0 - alpha) * previous_pole
                    source = "continuity" if alpha < 0.5 else "observed"
            elif previous_pole is not None:
                pole = previous_pole
                source = "memory"
            else:
                pole = neutral_pole
                source = "neutral"

            pole -= direction * float(np.dot(pole, direction))
            pole_norm = float(np.linalg.norm(pole))
            if pole_norm < 1.0e-8:
                pole = neutral_pole
                pole_norm = 1.0
            pole /= pole_norm
            self._arm_pole_memory[side] = pole.copy()
            self.last_arm_pole_source[side] = source
            elbow = center + pole * height
            mapped[elbow_name] = elbow
            mapped[wrist_name] = wrist

        # The calibrated ratio is kept as telemetry; use it to guard against
        # implausible profiles without allowing arbitrary task amplification.
        if not 0.20 <= human_torso <= 0.90:
            return points, 0, False
        return mapped, projection_count, True

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

    @staticmethod
    def _apply_g1_fixed_stance(points: dict[str, np.ndarray]) -> None:
        """Replace lower-body targets with the official G1 neutral geometry.

        In upper-body imitation the leg joints are intentionally fixed. Human
        thigh/shank lengths must not remain active IK tasks because the G1
        kinematic chain cannot satisfy them and the residual contaminates arm
        optimization. Values are body-origin offsets measured from the
        upstream GMR Unitree G1 model at its neutral qpos.
        """
        pelvis = points.get("pelvis")
        if pelvis is None:
            return
        offsets = {
            "left_hip": [0.0, 0.116452, -0.133165],
            "left_knee": [-0.0000023309, 0.1186009, -0.4392957524],
            "left_foot": [-0.0000023309, 0.118506455, -0.7568637524],
            "right_hip": [0.0, -0.116452, -0.133165],
            "right_knee": [-0.0000023309, -0.1186009, -0.4392957524],
            "right_foot": [-0.0000023309, -0.118506455, -0.7568637524],
        }
        for name, offset in offsets.items():
            if name in points:
                points[name] = pelvis + np.asarray(offset, dtype=np.float64)

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
        points, workspace_projection_count, operator_calibrated = (
            self._map_operator_to_g1_workspace(points, frame)
        )
        if self.fixed_stance:
            self._apply_g1_fixed_stance(points)
        orientations = self._global_orientations(frame)
        self.last_rejection_reason = "none"

        # Remove camera translation and keep the subject upright on z=0.  This
        # prevents camera distance from becoming a robot base displacement.
        pelvis = points["pelvis"].copy()
        ground_z = (
            float(pelvis[2] - 0.793)
            if self.fixed_stance
            else self._ground_height(points)
        )
        origin = np.array([pelvis[0], pelvis[1], ground_z], dtype=np.float64)

        human_data: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for name, xyz in points.items():
            human_data[name] = (
                xyz - origin,
                orientations.get(name, IDENTITY_WXYZ).copy(),
            )

        return AdaptedFrame(
            human_data=human_data,
            valid_targets=direct,
            used_memory_targets=remembered,
            raw_fallback_targets=raw_fallback,
            occlusion_held_targets=occlusion_held,
            pelvis_height_m=float(pelvis[2] - ground_z),
            ground_z_m=ground_z,
            workspace_projection_count=workspace_projection_count,
            operator_calibrated=operator_calibrated,
        )

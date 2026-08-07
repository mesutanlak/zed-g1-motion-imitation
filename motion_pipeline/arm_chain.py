"""Torso-overlap detection and coherent 3D arm-chain recovery."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class ArmChainResult:
    points: np.ndarray
    overlap: dict[str, bool]
    recovered: dict[str, bool]
    reasons: tuple[str, ...]
    candidate_confidence: dict[str, float] = field(default_factory=dict)
    candidate_cost: dict[str, float] = field(default_factory=dict)
    branch_sign: dict[str, int] = field(default_factory=dict)


@dataclass
class _ArmState:
    timestamp_s: float
    shoulder: np.ndarray
    elbow: np.ndarray
    wrist: np.ndarray
    torso_center: np.ndarray
    pole: np.ndarray
    plane_normal: np.ndarray
    branch_sign: int = 1
    pending_branch_sign: int = 0
    pending_branch_frames: int = 0


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


def _unit(vector: np.ndarray) -> np.ndarray | None:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-8 or not np.isfinite(norm):
        return None
    return np.asarray(vector, dtype=np.float64) / norm


def _two_bone_candidates(
    shoulder: np.ndarray,
    target: np.ndarray,
    pole: np.ndarray,
    upper: float,
    fore: float,
) -> tuple[tuple[np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray], float]:
    """Return the two exact elbow branches for one shoulder candidate."""
    direction = target - shoulder
    distance = float(np.linalg.norm(direction))
    if distance < 1e-6:
        elbow = shoulder + np.array([0.0, 0.0, -upper], dtype=np.float64)
        return (elbow, target.copy()), (elbow.copy(), target.copy()), 0.0
    axis = direction / distance
    reach = float(np.clip(distance, abs(upper - fore) + 1e-4, upper + fore - 1e-4))
    wrist = shoulder + axis * reach
    along = (upper * upper - fore * fore + reach * reach) / (2.0 * reach)
    height = float(np.sqrt(max(upper * upper - along * along, 0.0)))
    bend = pole - axis * float(np.dot(pole, axis))
    bend_unit = _unit(bend)
    if bend_unit is None:
        bend_unit = _unit(np.cross(axis, np.array([0.0, 0.0, 1.0])))
    if bend_unit is None:
        bend_unit = _unit(np.cross(axis, np.array([0.0, 1.0, 0.0])))
    assert bend_unit is not None
    center = shoulder + axis * along
    return (
        (center + bend_unit * height, wrist.copy()),
        (center - bend_unit * height, wrist.copy()),
        height,
    )


def _directed_plane_normal(
    shoulder: np.ndarray, elbow: np.ndarray, wrist: np.ndarray
) -> np.ndarray | None:
    """Directed shoulder-elbow-wrist normal used to reject mirror branches."""
    return _unit(np.cross(elbow - shoulder, wrist - elbow))


class ArmChainOptimizer:
    """Conditional AKC-style recovery for an occluded human arm chain.

    Normal high-confidence BODY_38 output is passed through unchanged.  Only
    torso overlap, low-confidence landmarks, or calibrated bone-length
    violations activate four candidate solutions (two shoulder hypotheses x
    two elbow branches).  Branch changes require several coherent frames so a
    single depth outlier cannot mirror the elbow.
    """

    def __init__(
        self,
        hold_s: float = 0.25,
        *,
        recovery_mode: str = "akc",
        branch_confirm_frames: int = 4,
        bone_tolerance: float = 0.22,
        recovery_timeout_s: float = 0.80,
    ) -> None:
        self.hold_s = float(hold_s)
        if recovery_mode not in {"legacy", "akc"}:
            raise ValueError("recovery_mode must be 'legacy' or 'akc'")
        self.recovery_mode = recovery_mode
        self.branch_confirm_frames = int(np.clip(branch_confirm_frames, 3, 5))
        self.bone_tolerance = float(np.clip(bone_tolerance, 0.10, 0.50))
        self.recovery_timeout_s = float(max(hold_s, recovery_timeout_s))
        self.previous: dict[str, _ArmState] = {}
        self.overlap_until: dict[str, float] = {}

    def reset(self) -> None:
        self.previous.clear()
        self.overlap_until.clear()

    @staticmethod
    def _neutral_pole(
        side: str, shoulder: np.ndarray, wrist: np.ndarray
    ) -> np.ndarray:
        axis = _unit(wrist - shoulder)
        if axis is None:
            return np.array([0.0, 1.0 if side == "LEFT" else -1.0, -0.25])
        pole = np.array(
            [0.0, 1.0 if side == "LEFT" else -1.0, -0.25],
            dtype=np.float64,
        )
        pole -= axis * float(np.dot(pole, axis))
        result = _unit(pole)
        if result is None:
            result = _unit(np.cross(axis, np.array([1.0, 0.0, 0.0])))
        return result if result is not None else np.array([0.0, 0.0, -1.0])

    def _bone_violation(
        self,
        side: str,
        shoulder: np.ndarray,
        elbow: np.ndarray,
        wrist: np.ndarray,
        calibration: Mapping[str, object] | None,
    ) -> bool:
        if not calibration:
            return False
        prefix = side.lower()
        expected_upper = float(calibration.get(f"{prefix}_upper_arm_m", 0.0))
        expected_fore = float(calibration.get(f"{prefix}_forearm_m", 0.0))
        if expected_upper <= 0.05 or expected_fore <= 0.05:
            return False
        observed = (
            float(np.linalg.norm(elbow - shoulder)),
            float(np.linalg.norm(wrist - elbow)),
        )
        return bool(
            abs(observed[0] / expected_upper - 1.0) > self.bone_tolerance
            or abs(observed[1] / expected_fore - 1.0) > self.bone_tolerance
        )

    @staticmethod
    def _front_depth_ambiguity(
        points_3d: np.ndarray,
        points_2d: np.ndarray,
        index: Mapping[str, int],
        side: str,
    ) -> bool:
        """Detect a foreshortened arm aimed along the camera depth axis.

        In this pose BODY_38 can report high confidence although shoulder and
        elbow nearly coincide in the image and the inferred elbow plane is
        under-constrained.  This is only a recovery gate: normal lateral arm
        poses continue to pass through unchanged.
        """
        required = (
            "LEFT_SHOULDER", "RIGHT_SHOULDER",
            f"{side}_SHOULDER", f"{side}_ELBOW", f"{side}_WRIST",
        )
        if any(name not in index for name in required):
            return False
        shoulder_i = index[f"{side}_SHOULDER"]
        elbow_i = index[f"{side}_ELBOW"]
        wrist_i = index[f"{side}_WRIST"]
        chain_3d = points_3d[[shoulder_i, elbow_i, wrist_i]]
        chain_2d = points_2d[[shoulder_i, elbow_i, wrist_i]]
        if not np.isfinite(chain_3d).all() or not np.isfinite(chain_2d).all():
            return True
        shoulder_span_px = float(
            np.linalg.norm(
                points_2d[index["LEFT_SHOULDER"]]
                - points_2d[index["RIGHT_SHOULDER"]]
            )
        )
        if shoulder_span_px < 20.0:
            return False
        upper_3d = chain_3d[1] - chain_3d[0]
        fore_3d = chain_3d[2] - chain_3d[1]
        upper_length = float(np.linalg.norm(upper_3d))
        fore_length = float(np.linalg.norm(fore_3d))
        if upper_length < 1e-5 or fore_length < 1e-5:
            return True
        upper_px = float(np.linalg.norm(chain_2d[1] - chain_2d[0]))
        fore_px = float(np.linalg.norm(chain_2d[2] - chain_2d[1]))
        upper_depth_ratio = abs(float(upper_3d[0])) / upper_length
        fore_depth_ratio = abs(float(fore_3d[0])) / fore_length
        projected_collapse = (
            upper_px < 0.24 * shoulder_span_px
            or fore_px < 0.20 * shoulder_span_px
        )
        depth_dominant = max(upper_depth_ratio, fore_depth_ratio) > 0.72
        return bool(projected_collapse and depth_dominant)

    def _recover_akc(
        self,
        *,
        side: str,
        timestamp_s: float,
        shoulder: np.ndarray,
        elbow: np.ndarray,
        wrist: np.ndarray,
        torso_center: np.ndarray,
        confidence_elbow: float,
        confidence_wrist: float,
        overlap: bool,
        upper: float,
        fore: float,
        state: _ArmState,
    ) -> tuple[np.ndarray, np.ndarray, float, int]:
        """Score four AKC candidates without altering reliable BODY_38 frames."""
        torso_delta = torso_center - state.torso_center
        propagated_shoulder = state.shoulder + torso_delta
        shoulder_candidates = (shoulder, propagated_shoulder)

        measured_vector = wrist - shoulder
        measured_direction = _unit(measured_vector)
        previous_reach = float(np.linalg.norm(state.wrist - state.shoulder))
        if measured_direction is None:
            target = state.wrist + torso_delta
        elif overlap:
            target = shoulder + measured_direction * previous_reach
        else:
            target = wrist

        observed_pole = elbow - shoulder
        axis = _unit(target - shoulder)
        if axis is not None:
            observed_pole -= axis * float(np.dot(observed_pole, axis))
        pole = _unit(observed_pole)
        if pole is None or overlap:
            pole = state.pole.copy()
        if _unit(pole) is None:
            pole = self._neutral_pole(side, shoulder, target)

        candidates: list[tuple[float, np.ndarray, np.ndarray, np.ndarray, int]] = []
        elbow_weight = 0.05 if overlap else np.clip(confidence_elbow / 100.0, 0.1, 1.0)
        wrist_weight = 0.20 if overlap else np.clip(confidence_wrist / 100.0, 0.1, 1.0)
        for shoulder_index, shoulder_candidate in enumerate(shoulder_candidates):
            shifted_target = target + (shoulder_candidate - shoulder)
            positive, negative, height = _two_bone_candidates(
                shoulder_candidate, shifted_target, pole, upper, fore
            )
            for candidate_index, (candidate_elbow, candidate_wrist) in enumerate(
                (positive, negative)
            ):
                candidate_axis = _unit(candidate_wrist - shoulder_candidate)
                if candidate_axis is None:
                    candidate_axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
                candidate_pole = _unit(
                    candidate_elbow
                    - shoulder_candidate
                    - candidate_axis
                    * float(
                        np.dot(
                            candidate_elbow - shoulder_candidate,
                            candidate_axis,
                        )
                    )
                )
                if candidate_pole is None:
                    candidate_pole = state.pole.copy()
                branch_sign = 1 if float(np.dot(candidate_pole, state.pole)) >= 0.0 else -1
                measurement = (
                    elbow_weight * np.linalg.norm(candidate_elbow - elbow) / max(upper, 1e-6)
                    + wrist_weight * np.linalg.norm(candidate_wrist - wrist) / max(fore, 1e-6)
                )
                temporal = 0.45 * (
                    np.linalg.norm(candidate_elbow - (state.elbow + torso_delta))
                    / max(upper, 1e-6)
                )
                pole_cost = 0.65 * (1.0 - float(np.clip(np.dot(candidate_pole, state.pole), -1.0, 1.0)))
                shoulder_cost = 0.15 * shoulder_index
                branch_cost = 0.30 if branch_sign != state.branch_sign else 0.0
                straight_cost = 0.50 if height < 0.025 and branch_sign != state.branch_sign else 0.0
                cost = float(measurement + temporal + pole_cost + shoulder_cost + branch_cost + straight_cost)
                candidates.append(
                    (cost, candidate_elbow, candidate_wrist, candidate_pole, branch_sign)
                )

        candidates.sort(key=lambda value: value[0])
        chosen = candidates[0]
        if chosen[4] != state.branch_sign:
            if state.pending_branch_sign == chosen[4]:
                state.pending_branch_frames += 1
            else:
                state.pending_branch_sign = chosen[4]
                state.pending_branch_frames = 1
            if state.pending_branch_frames < self.branch_confirm_frames:
                same_branch = [value for value in candidates if value[4] == state.branch_sign]
                if same_branch:
                    chosen = same_branch[0]
        else:
            state.pending_branch_sign = 0
            state.pending_branch_frames = 0

        if chosen[4] != state.branch_sign and state.pending_branch_frames >= self.branch_confirm_frames:
            state.branch_sign = chosen[4]
            state.pending_branch_sign = 0
            state.pending_branch_frames = 0
        confidence = float(np.exp(-min(chosen[0], 8.0)))
        return chosen[1], chosen[2], confidence, state.branch_sign

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
        candidate_confidence: dict[str, float] = {}
        candidate_cost: dict[str, float] = {}
        branch_sign: dict[str, int] = {}
        torso_center = 0.5 * (
            output[index["LEFT_SHOULDER"]] + output[index["RIGHT_SHOULDER"]]
        )
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
            bone_violation = self._bone_violation(
                side,
                output[shoulder_i],
                output[elbow_i],
                output[wrist_i],
                calibration,
            )
            front_depth_ambiguity = self._front_depth_ambiguity(
                output, pixel, index, side
            )
            low_quality = (
                overlap_active
                or conf[elbow_i] < threshold
                or conf[wrist_i] < threshold
                or bone_violation
                or front_depth_ambiguity
            )
            if bone_violation:
                reasons.append(f"{side.lower()}_bone_length_violation")
            if front_depth_ambiguity:
                reasons.append(f"{side.lower()}_front_depth_ambiguity")
            previous = self.previous.get(side)
            did_recover = False
            score = 1.0
            if (
                low_quality
                and previous is not None
                and timestamp_s - previous.timestamp_s <= self.recovery_timeout_s
            ):
                prev_elbow, prev_wrist = previous.elbow, previous.wrist
                shoulder = output[shoulder_i]
                measured_wrist = output[wrist_i]
                target = measured_wrist if np.isfinite(measured_wrist).all() and conf[wrist_i] >= threshold else prev_wrist
                # A partially occluded BODY_38 chain can contain a missing
                # elbow while the wrist remains usable.  Never feed that NaN
                # into the candidate cost: continue from the last trusted
                # elbow, translated with the torso, until a valid measurement
                # is available again.
                recovery_elbow = output[elbow_i]
                if not np.isfinite(recovery_elbow).all():
                    recovery_elbow = prev_elbow + (torso_center - previous.torso_center)
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
                upper = float((calibration or {}).get(f"{side.lower()}_upper_arm_m", np.linalg.norm(previous.elbow - previous.shoulder)))
                fore = float((calibration or {}).get(f"{side.lower()}_forearm_m", np.linalg.norm(previous.wrist - previous.elbow)))
                if np.isfinite(shoulder).all() and upper > 0.05 and fore > 0.05:
                    if self.recovery_mode == "akc":
                        output[elbow_i], output[wrist_i], score, selected_branch = self._recover_akc(
                            side=side,
                            timestamp_s=timestamp_s,
                            shoulder=shoulder,
                            elbow=recovery_elbow,
                            wrist=target,
                            torso_center=torso_center,
                            confidence_elbow=float(conf[elbow_i]),
                            confidence_wrist=float(conf[wrist_i]),
                            overlap=overlap_active or front_depth_ambiguity,
                            upper=upper,
                            fore=fore,
                            state=previous,
                        )
                        reasons.append(f"{side.lower()}_akc_4candidate_recovery")
                        branch_sign[side.lower()] = selected_branch
                    else:
                        positive, negative, _ = _two_bone_candidates(
                            shoulder, target, prev_elbow, upper, fore
                        )
                        output[elbow_i], output[wrist_i] = min(
                            (positive, negative),
                            key=lambda value: float(np.linalg.norm(value[0] - prev_elbow)),
                        )
                        branch_sign[side.lower()] = previous.branch_sign
                    did_recover = True
                    reasons.append(f"{side.lower()}_arm_chain_occlusion_recovery")
            recovered[side.lower()] = did_recover
            score = float(score) if np.isfinite(score) else 0.0
            candidate_confidence[side.lower()] = score
            candidate_cost[side.lower()] = float(-np.log(max(score, 1e-9)))
            if (not low_quality or did_recover) and np.isfinite(output[[shoulder_i, elbow_i, wrist_i]]).all():
                pole = output[elbow_i] - output[shoulder_i]
                axis = _unit(output[wrist_i] - output[shoulder_i])
                if axis is not None:
                    pole -= axis * float(np.dot(pole, axis))
                pole = _unit(pole)
                normal = _directed_plane_normal(
                    output[shoulder_i], output[elbow_i], output[wrist_i]
                )
                if pole is None and previous is not None:
                    pole = previous.pole.copy()
                if normal is None and previous is not None:
                    normal = previous.plane_normal.copy()
                if pole is not None and normal is not None:
                    current_branch = (
                        branch_sign.get(side.lower())
                        or (previous.branch_sign if previous is not None else 1)
                    )
                    pending_sign = previous.pending_branch_sign if previous is not None else 0
                    pending_frames = previous.pending_branch_frames if previous is not None else 0
                    self.previous[side] = _ArmState(
                        timestamp_s=timestamp_s,
                        shoulder=output[shoulder_i].copy(),
                        elbow=output[elbow_i].copy(),
                        wrist=output[wrist_i].copy(),
                        torso_center=torso_center.copy(),
                        pole=pole.copy(),
                        plane_normal=normal.copy(),
                        branch_sign=current_branch,
                        pending_branch_sign=pending_sign,
                        pending_branch_frames=pending_frames,
                    )
                    branch_sign[side.lower()] = current_branch
        return ArmChainResult(
            output,
            overlap,
            recovered,
            tuple(dict.fromkeys(reasons)),
            candidate_confidence,
            candidate_cost,
            branch_sign,
        )

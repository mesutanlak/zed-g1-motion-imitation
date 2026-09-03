"""Continuous upper-body capsule distances for G1 imitation safety.

This module is deliberately independent of ZED capture and the main IK
solver.  It augments MuJoCo's binary contact result with a smooth distance
signal suitable for a reference governor and an event-triggered rescue path.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Mapping, Sequence

import numpy as np


# These values are shared by deterministic IK safety and policy residual
# safety.  The torso radius intentionally includes a small shell around the
# visual mesh: a live command must stop before a physical forearm/hand reaches
# the trunk, not after MuJoCo has already reported penetration.
G1_TORSO_CAPSULE_RADIUS_M = 0.110
G1_UPPER_ARM_CAPSULE_RADIUS_M = 0.038
G1_FOREARM_CAPSULE_RADIUS_M = 0.038
G1_HAND_CAPSULE_RADIUS_M = 0.045
# One circular torso capsule intentionally overbounds the official G1 trunk
# meshes. Without this calibration the neutral forearms have only 0.014 mm
# apparent clearance and valid front-corner motion is rejected. Exact MuJoCo
# contacts remain a separate hard boundary.
G1_TORSO_CAPSULE_MODEL_ALLOWANCE_M = 0.040


@dataclass(frozen=True)
class Capsule:
    start: np.ndarray
    end: np.ndarray
    radius: float
    arm: str | None = None


@dataclass(frozen=True)
class CollisionDistanceReport:
    # Hard pairs are allowed to govern the command.  Soft pairs are still
    # measured and logged, but are reserved for links that are physically
    # attached and therefore overlap in the coarse capsule approximation.
    minimum_margin_m: float
    pair_margins_m: dict[str, float]
    arm_minimum_margin_m: dict[str, float]
    risk_pairs: tuple[str, ...]
    soft_pair_margins_m: dict[str, float] = field(default_factory=dict)
    soft_risk_pairs: tuple[str, ...] = ()
    hard_pair_margins_m: dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class ConfigurationBarrierProjection:
    joint_position: np.ndarray
    alpha: float
    applied: bool
    minimum_margin_m: float
    self_contact_count: int


def _point(value: Sequence[float]) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError("invalid_capsule_point")
    return result


def segment_segment_distance(
    first_start: Sequence[float],
    first_end: Sequence[float],
    second_start: Sequence[float],
    second_end: Sequence[float],
) -> float:
    """Shortest distance between two finite 3D segments."""
    p1, q1, p2, q2 = map(_point, (first_start, first_end, second_start, second_end))
    d1 = q1 - p1
    d2 = q2 - p2
    relative = p1 - p2
    a = float(np.dot(d1, d1))
    e = float(np.dot(d2, d2))
    epsilon = 1.0e-12
    if a <= epsilon and e <= epsilon:
        return float(np.linalg.norm(p1 - p2))
    if a <= epsilon:
        first_t = 0.0
        second_t = float(np.clip(np.dot(d2, relative) / e, 0.0, 1.0))
    else:
        c = float(np.dot(d1, relative))
        if e <= epsilon:
            second_t = 0.0
            first_t = float(np.clip(-c / a, 0.0, 1.0))
        else:
            b = float(np.dot(d1, d2))
            denominator = a * e - b * b
            first_t = (
                float(np.clip((b * float(np.dot(d2, relative)) - c * e) / denominator, 0.0, 1.0))
                if abs(denominator) > epsilon
                else 0.0
            )
            second_t = (b * first_t + float(np.dot(d2, relative))) / e
            if second_t < 0.0:
                second_t = 0.0
                first_t = float(np.clip(-c / a, 0.0, 1.0))
            elif second_t > 1.0:
                second_t = 1.0
                first_t = float(np.clip((b - c) / a, 0.0, 1.0))
    closest_first = p1 + d1 * first_t
    closest_second = p2 + d2 * second_t
    return float(np.linalg.norm(closest_first - closest_second))


def capsule_margin(first: Capsule, second: Capsule) -> float:
    return segment_segment_distance(
        first.start, first.end, second.start, second.end
    ) - first.radius - second.radius


def project_configuration_along_safe_path(
    previous_safe: Sequence[float],
    candidate: Sequence[float],
    evaluator: Callable[[np.ndarray], tuple[CollisionDistanceReport, int]],
    *,
    clearance_m: float = 0.005,
    coarse_steps: int = 12,
    bisection_steps: int = 10,
) -> ConfigurationBarrierProjection:
    """Keep the largest collision-free prefix of a joint-space command path.

    The ordinary feasibility governor predicts collision from the final IK
    target.  This last boundary also evaluates the actual command produced
    after rate/acceleration limiting.  Scanning outward from the prior safe
    pose avoids jumping across a narrow colliding interval.
    """

    previous = np.asarray(previous_safe, dtype=np.float64)
    proposed = np.asarray(candidate, dtype=np.float64)
    if previous.shape != proposed.shape or previous.ndim != 1:
        raise ValueError("robot body projection expects matching 1-D joint arrays")

    def evaluate(alpha: float):
        q = previous + float(alpha) * (proposed - previous)
        report, contacts = evaluator(q)
        safe = int(contacts) == 0 and float(report.minimum_margin_m) >= float(clearance_m)
        return q, report, int(contacts), safe

    q_candidate, report_candidate, contacts_candidate, candidate_safe = evaluate(1.0)
    if candidate_safe:
        return ConfigurationBarrierProjection(
            q_candidate,
            1.0,
            False,
            float(report_candidate.minimum_margin_m),
            contacts_candidate,
        )

    q_previous, report_previous, contacts_previous, previous_is_safe = evaluate(0.0)
    if not previous_is_safe:
        # Fail closed. The caller should provide its known nominal command as
        # previous_safe when its last command is no longer valid.
        return ConfigurationBarrierProjection(
            q_previous,
            0.0,
            True,
            float(report_previous.minimum_margin_m),
            contacts_previous,
        )

    low = 0.0
    low_q, low_report, low_contacts = q_previous, report_previous, contacts_previous
    high = 1.0
    for index in range(1, max(2, int(coarse_steps)) + 1):
        alpha = index / max(2, int(coarse_steps))
        q, report, contacts, safe = evaluate(alpha)
        if safe:
            low, low_q, low_report, low_contacts = alpha, q, report, contacts
            continue
        high = alpha
        break
    for _ in range(max(0, int(bisection_steps))):
        middle = 0.5 * (low + high)
        q, report, contacts, safe = evaluate(middle)
        if safe:
            low, low_q, low_report, low_contacts = middle, q, report, contacts
        else:
            high = middle
    return ConfigurationBarrierProjection(
        low_q,
        float(low),
        True,
        float(low_report.minimum_margin_m),
        int(low_contacts),
    )


def _trim_start(start: np.ndarray, end: np.ndarray, fraction: float) -> np.ndarray:
    return start + float(fraction) * (end - start)


def upper_body_capsule_report(
    positions: Mapping[str, Sequence[float]],
    *,
    warning_margin_m: float = 0.005,
) -> CollisionDistanceReport:
    """Approximate relevant G1 upper-body separation with smooth capsules.

    Proximal upper-arm sections are trimmed because the shoulder joint is
    intentionally attached to the torso. Forearms and hands remain hard
    pairs. Torso pairs include a calibrated allowance because a circular
    trunk capsule overestimates the official mesh at its front corners;
    exact model contacts are still authoritative at the caller.
    """
    required = (
        "pelvis",
        "torso_link",
        "left_shoulder_pitch_link",
        "left_elbow_link",
        "left_wrist_roll_rubber_hand",
        "right_shoulder_pitch_link",
        "right_elbow_link",
        "right_wrist_roll_rubber_hand",
    )
    if not all(name in positions for name in required):
        return CollisionDistanceReport(float("inf"), {}, {"left": float("inf"), "right": float("inf")}, ())
    xyz = {name: _point(positions[name]) for name in required}
    torso_axis = xyz["torso_link"] - xyz["pelvis"]
    torso = Capsule(
        xyz["pelvis"] + 0.10 * torso_axis,
        xyz["torso_link"] + np.array([0.0, 0.0, 0.08]),
        G1_TORSO_CAPSULE_RADIUS_M,
    )
    head_center = xyz["torso_link"] + np.array([0.0, 0.0, 0.285])
    head = Capsule(head_center, head_center, 0.105)
    capsules: dict[str, Capsule] = {"torso": torso, "head": head}
    for side in ("left", "right"):
        shoulder = xyz[f"{side}_shoulder_pitch_link"]
        elbow = xyz[f"{side}_elbow_link"]
        wrist = xyz[f"{side}_wrist_roll_rubber_hand"]
        fore_direction = wrist - elbow
        fore_norm = float(np.linalg.norm(fore_direction))
        # New bridge packets expose the official rubber-hand endpoint.  Keep
        # the legacy 5.5 cm estimate for old recordings/tests that contain only
        # body origins, but use the complete hand geometry whenever available.
        endpoint_name = f"{side}_hand_endpoint"
        hand_end = (
            _point(positions[endpoint_name])
            if endpoint_name in positions
            else wrist + 0.055 * fore_direction / fore_norm
            if fore_norm > 1.0e-8
            else wrist
        )
        capsules[f"{side}_upper"] = Capsule(
            _trim_start(shoulder, elbow, 0.32),
            elbow,
            G1_UPPER_ARM_CAPSULE_RADIUS_M,
            side,
        )
        capsules[f"{side}_fore"] = Capsule(
            elbow, wrist, G1_FOREARM_CAPSULE_RADIUS_M, side
        )
        capsules[f"{side}_hand"] = Capsule(
            wrist, hand_end, G1_HAND_CAPSULE_RADIUS_M, side
        )

    # Only the proximal upper arm is soft because it is attached beside the
    # torso and the coarse capsules overlap in ordinary poses.  A forearm or
    # hand penetrating the torso is never an intentional robot configuration.
    pairs = (
        # The upper arm is physically attached beside the torso, so the two
        # coarse capsules overlap in ordinary arms-down and chest-reaching
        # poses. Treat this approximation as measured telemetry; inter-arm
        # and hand/head contacts below remain hard safety events.
        ("left_upper", "torso", True, 0.0),
        ("left_fore", "torso", False, G1_TORSO_CAPSULE_MODEL_ALLOWANCE_M),
        ("left_hand", "torso", False, G1_TORSO_CAPSULE_MODEL_ALLOWANCE_M),
        ("left_hand", "head", False, 0.0),
        ("right_upper", "torso", True, 0.0),
        ("right_fore", "torso", False, G1_TORSO_CAPSULE_MODEL_ALLOWANCE_M),
        ("right_hand", "torso", False, G1_TORSO_CAPSULE_MODEL_ALLOWANCE_M),
        ("right_hand", "head", False, 0.0),
        ("left_upper", "right_upper", False, 0.0),
        ("left_upper", "right_fore", False, 0.0),
        ("left_fore", "right_upper", False, 0.0),
        ("left_fore", "right_fore", False, 0.0),
        ("left_hand", "right_hand", False, 0.0),
    )
    margins: dict[str, float] = {}
    hard_margins: dict[str, float] = {}
    soft_margins: dict[str, float] = {}
    arm_margins = {"left": float("inf"), "right": float("inf")}
    for first_name, second_name, is_soft, model_allowance_m in pairs:
        first = capsules[first_name]
        second = capsules[second_name]
        margin = capsule_margin(first, second)
        label = f"{first_name}__{second_name}"
        margins[label] = margin
        if is_soft:
            soft_margins[label] = margin
            continue
        effective_margin = margin + model_allowance_m
        hard_margins[label] = effective_margin
        affected = {value.arm for value in (first, second) if value.arm is not None}
        for side in affected:
            arm_margins[side] = min(arm_margins[side], effective_margin)
    minimum = min(hard_margins.values(), default=float("inf"))
    risks = tuple(
        name for name, margin in hard_margins.items()
        if margin < warning_margin_m
    )
    soft_risks = tuple(
        name for name, margin in soft_margins.items() if margin < 0.0
    )
    return CollisionDistanceReport(
        minimum,
        margins,
        arm_margins,
        risks,
        soft_margins,
        soft_risks,
        hard_margins,
    )

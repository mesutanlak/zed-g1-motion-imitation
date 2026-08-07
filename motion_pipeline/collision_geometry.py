"""Continuous upper-body capsule distances for G1 imitation safety.

This module is deliberately independent of ZED capture and the main IK
solver.  It augments MuJoCo's binary contact result with a smooth distance
signal suitable for a reference governor and an event-triggered rescue path.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class Capsule:
    start: np.ndarray
    end: np.ndarray
    radius: float
    arm: str | None = None


@dataclass(frozen=True)
class CollisionDistanceReport:
    # Hard pairs are allowed to govern the command.  Soft pairs are still
    # measured and logged, but represent intentional manipulation contact
    # such as a hand/forearm crossing the front of the torso.
    minimum_margin_m: float
    pair_margins_m: dict[str, float]
    arm_minimum_margin_m: dict[str, float]
    risk_pairs: tuple[str, ...]
    soft_pair_margins_m: dict[str, float] = field(default_factory=dict)
    soft_risk_pairs: tuple[str, ...] = ()
    hard_pair_margins_m: dict[str, float] = field(default_factory=dict)


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


def _trim_start(start: np.ndarray, end: np.ndarray, fraction: float) -> np.ndarray:
    return start + float(fraction) * (end - start)


def upper_body_capsule_report(
    positions: Mapping[str, Sequence[float]],
    *,
    warning_margin_m: float = 0.005,
) -> CollisionDistanceReport:
    """Approximate relevant G1 upper-body separation with smooth capsules.

    Proximal upper-arm sections are trimmed because the shoulder joint is
    intentionally attached to the torso.  Hands touching the abdomen are not
    treated as an immediate hard stop; the continuous margin lets the caller
    slow/project the target instead of freezing the whole robot.
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
        0.085,
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
        hand_end = (
            wrist + 0.055 * fore_direction / fore_norm
            if fore_norm > 1.0e-8
            else wrist
        )
        capsules[f"{side}_upper"] = Capsule(
            _trim_start(shoulder, elbow, 0.32), elbow, 0.038, side
        )
        capsules[f"{side}_fore"] = Capsule(elbow, wrist, 0.038, side)
        capsules[f"{side}_hand"] = Capsule(wrist, hand_end, 0.045, side)

    # A forearm or hand in front of the chest is a common, valid imitation
    # target.  Capsule overlap there is not equivalent to an inter-arm or
    # hand/head collision, especially because the simplified torso capsule
    # does not model the chest surface accurately.  Keep those distances as
    # soft telemetry rather than letting them freeze the deterministic IK.
    pairs = (
        # The 12 mm mounting allowance compensates only for the simplified
        # capsule near the physical shoulder attachment. Deep penetration is
        # still a hard event.
        ("left_upper", "torso", False, 0.012),
        ("left_fore", "torso", True, 0.0),
        ("left_hand", "torso", True, 0.0),
        ("left_hand", "head", False, 0.0),
        ("right_upper", "torso", False, 0.012),
        ("right_fore", "torso", True, 0.0),
        ("right_hand", "torso", True, 0.0),
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

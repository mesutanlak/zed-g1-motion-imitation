"""Shared deployment contract for the G1 upper-body residual policy.

The learned action is deliberately *not* a replacement for BODY_38/GMR IK.
It is a bounded correction around the current feasible IK reference.  Keeping
this contract free of Isaac imports lets training, live inference and tests use
the exact same joint/body order and observation scaling.
"""

from __future__ import annotations

from typing import Iterable

import numpy as np

from .collision_geometry import (
    G1_FOREARM_CAPSULE_RADIUS_M,
    G1_HAND_CAPSULE_RADIUS_M,
    G1_TORSO_CAPSULE_RADIUS_M,
    segment_segment_distance,
)


SCHEMA = "g1_upper_body_residual_policy/v1"
INFERENCE_WRAPPER_VERSION = "g1_residual_inference/v5_robot_body_barrier"
POLICY_STEP_DT = 0.02  # 50 Hz policy over the 200 Hz Isaac control loop.
# Retained for old metadata/checkpoint compatibility.  New policies use the
# joint-specific vector below.
RESIDUAL_SCALE_RAD = 0.15
ACTION_CLIP = 1.0
ACTION_SLEW_PER_STEP = 0.25
FULL_TRACKING_CONFIDENCE = 0.70
HOLD_BELOW_CONFIDENCE = 0.25
SUPPORT_ERROR_DEADBAND_RAD = 0.015
SUPPORT_FULL_ERROR_RAD = 0.12
SUPPORT_MAX_LEAD_RATIO = 0.50
SUPPORT_MAX_BIAS_RAD = 0.008

# A fixed-root, fixed-double-support training scene does not have meaningful
# force-sensor contacts.  It historically produced (0, 0), while live sent
# (1, 1).  The resulting 100-sigma OOD input dominated every policy action.
# Keep the established zero encoding on both sides until the policy is moved
# to a dynamic/contact-aware task and the observation schema is versioned.
FIXED_DOUBLE_SUPPORT_CONTACT_STATE = (0.0, 0.0)

# Waist is part of upper-body imitation.  Legs/ankles are intentionally absent:
# they stay at the official nominal double-support pose in training and live.
UPPER_POLICY_JOINTS = (
    "waist_yaw_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
)

# These joints are outside the learned action space.  The upper-body task must
# nevertheless command them every physics step; leaving an actuated joint with
# no current position target lets gravity/contact transients move the legs in
# the GUI even though the actor has no lower-body outputs.
LOWER_POLICY_JOINTS = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
)

# Residuals are deliberately smaller on waist/yaw/wrist axes.  The IK remains
# the primary controller; policy authority is only large enough to correct
# repeatable tracking bias without inventing a new pose.
RESIDUAL_SCALE_RAD_BY_JOINT = {
    "waist_yaw_joint": 0.060,
    "left_shoulder_pitch_joint": 0.110,
    "left_shoulder_roll_joint": 0.100,
    "left_shoulder_yaw_joint": 0.080,
    "left_elbow_joint": 0.090,
    "left_wrist_roll_joint": 0.055,
    "right_shoulder_pitch_joint": 0.110,
    "right_shoulder_roll_joint": 0.100,
    "right_shoulder_yaw_joint": 0.080,
    "right_elbow_joint": 0.090,
    "right_wrist_roll_joint": 0.055,
}

# Upper-body task frames present in both the offline clips and the live GMR
# g1_skeleton.safe_positions_m packet.
TRACKED_BODIES = (
    "pelvis",
    "torso_link",
    "left_shoulder_pitch_link",
    "left_elbow_link",
    "left_wrist_roll_rubber_hand",
    "right_shoulder_pitch_link",
    "right_elbow_link",
    "right_wrist_roll_rubber_hand",
)

OBSERVATION_SCALES = {
    "projected_gravity": 1.0,
    "base_ang_vel": 0.25,
    "q_minus_q_default": 1.0,
    "qd": 0.05,
    "q_ref_minus_q": 1.0,
    "qd_ref_minus_qd": 0.05,
    "body_reference_error": 4.0,
    "foot_contact_state": 1.0,
    "previous_action": 1.0,
    "reference_confidence": 1.0,
}

ACTION_DIM = len(UPPER_POLICY_JOINTS)
OBSERVATION_DIM = (
    3  # projected gravity
    + 3  # base angular velocity
    + 4 * ACTION_DIM  # q, qd, q_ref-q, qd_ref-qd
    + 3 * len(TRACKED_BODIES)
    + 2  # fixed double-support contact state
    + ACTION_DIM  # previous residual action
    + 1  # reference confidence
)

# Link-origin clearance gates for inter-arm separation. Torso separation uses
# full capsules below; checking only wrist/elbow origins can miss a forearm
# segment passing directly through the trunk.
POLICY_COLLISION_CONSTRAINTS = (
    {
        "name": "wrist_wrist",
        "first": "left_wrist_roll_rubber_hand",
        "second": "right_wrist_roll_rubber_hand",
        "hard_margin_m": 0.12,
        "release_buffer_m": 0.08,
        "arms": ("left", "right"),
    },
    {
        "name": "elbow_elbow",
        "first": "left_elbow_link",
        "second": "right_elbow_link",
        "hard_margin_m": 0.16,
        "release_buffer_m": 0.08,
        "arms": ("left", "right"),
    },
)

# Surface-to-surface torso clearance for the residual-policy guard.  The same
# radii are used by the normal IK feasibility governor.  The 5 mm hard shell
# is followed by a 40 mm release band, so authority fades continuously before
# contact instead of switching after penetration.
POLICY_TORSO_CAPSULE_CONSTRAINTS = tuple(
    {
        "name": f"{side}_{part}_torso_capsule",
        "side": side,
        "part": part,
        "hard_margin_m": 0.005,
        "release_buffer_m": 0.040,
        "arms": (side,),
    }
    for side in ("left", "right")
    for part in ("forearm", "hand")
)
POLICY_TORSO_END_OFFSET_Z_M = 0.08
POLICY_HAND_EXTENSION_M = 0.055

POLICY_ARM_ACTION_INDICES = {
    "left": tuple(range(1, 6)),
    "right": tuple(range(6, 11)),
}


def _clearance_gain(distance_m, hard_margin_m: float, release_buffer_m: float):
    return np.clip(
        (distance_m - float(hard_margin_m)) / max(float(release_buffer_m), 1.0e-6),
        0.0,
        1.0,
    )


def _torso_and_arm_capsules_numpy(
    positions: dict[str, np.ndarray], side: str, part: str
) -> tuple[np.ndarray, np.ndarray, float, np.ndarray, np.ndarray, float] | None:
    required = (
        "pelvis",
        "torso_link",
        f"{side}_elbow_link",
        f"{side}_wrist_roll_rubber_hand",
    )
    if any(positions.get(name, np.empty(0)).shape != (3,) for name in required):
        return None
    pelvis = positions["pelvis"]
    torso = positions["torso_link"]
    elbow = positions[f"{side}_elbow_link"]
    wrist = positions[f"{side}_wrist_roll_rubber_hand"]
    torso_start = pelvis + 0.10 * (torso - pelvis)
    torso_end = torso + np.asarray(
        [0.0, 0.0, POLICY_TORSO_END_OFFSET_Z_M], dtype=np.float32
    )
    if part == "forearm":
        arm_start, arm_end, arm_radius = (
            elbow,
            wrist,
            G1_FOREARM_CAPSULE_RADIUS_M,
        )
    elif part == "hand":
        direction = wrist - elbow
        norm = float(np.linalg.norm(direction))
        hand_end = (
            wrist + POLICY_HAND_EXTENSION_M * direction / norm
            if norm > 1.0e-8
            else wrist
        )
        arm_start, arm_end, arm_radius = wrist, hand_end, G1_HAND_CAPSULE_RADIUS_M
    else:
        raise ValueError(f"unknown arm capsule part: {part}")
    return (
        arm_start,
        arm_end,
        float(arm_radius),
        torso_start,
        torso_end,
        G1_TORSO_CAPSULE_RADIUS_M,
    )


def _capsule_surface_margin_numpy(
    positions: dict[str, np.ndarray], side: str, part: str
) -> float | None:
    capsules = _torso_and_arm_capsules_numpy(positions, side, part)
    if capsules is None:
        return None
    arm_start, arm_end, arm_radius, torso_start, torso_end, torso_radius = capsules
    return float(
        segment_segment_distance(arm_start, arm_end, torso_start, torso_end)
        - arm_radius
        - torso_radius
    )


def _batched_segment_distance_torch(first_start, first_end, second_start, second_end):
    """Torch equivalent of the finite-segment distance used by normal IK."""

    import torch

    d1 = first_end - first_start
    d2 = second_end - second_start
    relative = first_start - second_start
    a = torch.sum(d1 * d1, dim=-1)
    e = torch.sum(d2 * d2, dim=-1)
    b = torch.sum(d1 * d2, dim=-1)
    c = torch.sum(d1 * relative, dim=-1)
    f = torch.sum(d2 * relative, dim=-1)
    epsilon = 1.0e-12
    safe_a = torch.clamp(a, min=epsilon)
    safe_e = torch.clamp(e, min=epsilon)
    denominator = a * e - b * b
    general_s = torch.where(
        torch.abs(denominator) > epsilon,
        torch.clamp((b * f - c * e) / torch.clamp(denominator, min=epsilon), 0.0, 1.0),
        torch.zeros_like(a),
    )
    general_t = (b * general_s + f) / safe_e
    below = general_t < 0.0
    above = general_t > 1.0
    general_s = torch.where(below, torch.clamp(-c / safe_a, 0.0, 1.0), general_s)
    general_s = torch.where(above, torch.clamp((b - c) / safe_a, 0.0, 1.0), general_s)
    general_t = torch.where(below, torch.zeros_like(general_t), general_t)
    general_t = torch.where(above, torch.ones_like(general_t), general_t)

    first_point = a <= epsilon
    second_point = e <= epsilon
    s = torch.where(second_point & ~first_point, torch.clamp(-c / safe_a, 0.0, 1.0), general_s)
    s = torch.where(first_point, torch.zeros_like(s), s)
    t = torch.where(first_point & ~second_point, torch.clamp(f / safe_e, 0.0, 1.0), general_t)
    t = torch.where(second_point, torch.zeros_like(t), t)
    closest_first = first_start + s[..., None] * d1
    closest_second = second_start + t[..., None] * d2
    return torch.linalg.vector_norm(closest_first - closest_second, dim=-1)


def _capsule_surface_margin_torch(positions, body_names, side: str, part: str):
    import torch

    pelvis = positions[:, body_names.index("pelvis")]
    torso = positions[:, body_names.index("torso_link")]
    elbow = positions[:, body_names.index(f"{side}_elbow_link")]
    wrist = positions[:, body_names.index(f"{side}_wrist_roll_rubber_hand")]
    torso_start = pelvis + 0.10 * (torso - pelvis)
    offset = positions.new_tensor([0.0, 0.0, POLICY_TORSO_END_OFFSET_Z_M])
    torso_end = torso + offset
    if part == "forearm":
        arm_start, arm_end, arm_radius = elbow, wrist, G1_FOREARM_CAPSULE_RADIUS_M
    elif part == "hand":
        direction = wrist - elbow
        direction = direction / torch.clamp(
            torch.linalg.vector_norm(direction, dim=-1, keepdim=True), min=1.0e-8
        )
        arm_start = wrist
        arm_end = wrist + POLICY_HAND_EXTENSION_M * direction
        arm_radius = G1_HAND_CAPSULE_RADIUS_M
    else:
        raise ValueError(f"unknown arm capsule part: {part}")
    return (
        _batched_segment_distance_torch(arm_start, arm_end, torso_start, torso_end)
        - float(arm_radius)
        - G1_TORSO_CAPSULE_RADIUS_M
    )


def policy_collision_action_scale_numpy(
    safe_positions_m: dict[str, Iterable[float]],
    actual_positions_m: dict[str, Iterable[float]] | None = None,
) -> dict:
    """Return per-action residual authority from safe/actual link clearances.

    Distances are frame-invariant, so the GMR packet and Isaac world positions
    may each use their native origin. Missing optional measurements never
    disable a valid residual for optional inter-arm checks.  Torso geometry is
    fail-closed because complete live/training contracts always provide its
    required frames and allowing a residual without that barrier is unsafe.
    """

    safe = {
        name: np.asarray(value, dtype=np.float32).reshape(-1)
        for name, value in (safe_positions_m or {}).items()
    }
    actual = {
        name: np.asarray(value, dtype=np.float32).reshape(-1)
        for name, value in (actual_positions_m or {}).items()
    }
    arm_scale = {"left": 1.0, "right": 1.0}
    clearances: dict[str, dict[str, float | None]] = {}
    reasons: list[str] = []
    for constraint in POLICY_COLLISION_CONSTRAINTS:
        first = str(constraint["first"])
        second = str(constraint["second"])
        safe_distance = None
        actual_distance = None
        factors: list[float] = []
        if safe.get(first, np.empty(0)).shape == (3,) and safe.get(
            second, np.empty(0)
        ).shape == (3,):
            safe_distance = float(np.linalg.norm(safe[first] - safe[second]))
            factors.append(
                float(
                    _clearance_gain(
                        safe_distance,
                        float(constraint["hard_margin_m"]),
                        float(constraint["release_buffer_m"]),
                    )
                )
            )
        if actual.get(first, np.empty(0)).shape == (3,) and actual.get(
            second, np.empty(0)
        ).shape == (3,):
            actual_distance = float(np.linalg.norm(actual[first] - actual[second]))
            factors.append(
                float(
                    _clearance_gain(
                        actual_distance,
                        float(constraint["hard_margin_m"]),
                        float(constraint["release_buffer_m"]),
                    )
                )
            )
        factor = min(factors, default=1.0)
        clearances[str(constraint["name"])] = {
            "safe_m": safe_distance,
            "actual_m": actual_distance,
            "residual_gain": factor,
        }
        if factor < 0.999:
            reasons.append(str(constraint["name"]))
        for arm in constraint["arms"]:
            arm_scale[str(arm)] = min(arm_scale[str(arm)], factor)

    require_actual = actual_positions_m is not None
    for constraint in POLICY_TORSO_CAPSULE_CONSTRAINTS:
        side = str(constraint["side"])
        part = str(constraint["part"])
        safe_margin = _capsule_surface_margin_numpy(safe, side, part)
        actual_margin = _capsule_surface_margin_numpy(actual, side, part)
        factors: list[float] = []
        if safe_margin is None:
            factors.append(0.0)
        else:
            factors.append(float(_clearance_gain(
                safe_margin,
                float(constraint["hard_margin_m"]),
                float(constraint["release_buffer_m"]),
            )))
        if require_actual:
            if actual_margin is None:
                factors.append(0.0)
            else:
                factors.append(float(_clearance_gain(
                    actual_margin,
                    float(constraint["hard_margin_m"]),
                    float(constraint["release_buffer_m"]),
                )))
        factor = min(factors, default=0.0)
        name = str(constraint["name"])
        clearances[name] = {
            "safe_m": safe_margin,
            "actual_m": actual_margin,
            "residual_gain": factor,
        }
        if factor < 0.999:
            reasons.append(name)
        arm_scale[side] = min(arm_scale[side], factor)

    action_scale = np.ones(ACTION_DIM, dtype=np.float32)
    for arm, indices in POLICY_ARM_ACTION_INDICES.items():
        action_scale[list(indices)] = float(arm_scale[arm])
    return {
        "action_scale": action_scale,
        "left_arm_scale": float(arm_scale["left"]),
        "right_arm_scale": float(arm_scale["right"]),
        "active": bool(reasons),
        "reasons": reasons,
        "clearances": clearances,
    }


def policy_collision_action_scale_torch(
    safe_positions,
    actual_positions,
    body_names: list[str] | tuple[str, ...],
):
    """Batched Torch equivalent used by the Isaac Lab training action term."""

    import torch

    if safe_positions.shape != actual_positions.shape or safe_positions.ndim != 3:
        raise ValueError("collision guard expects matching [env, body, xyz] tensors")
    scales = torch.ones(
        (safe_positions.shape[0], ACTION_DIM),
        dtype=safe_positions.dtype,
        device=safe_positions.device,
    )
    for constraint in POLICY_COLLISION_CONSTRAINTS:
        first_id = body_names.index(str(constraint["first"]))
        second_id = body_names.index(str(constraint["second"]))
        safe_distance = torch.linalg.vector_norm(
            safe_positions[:, first_id] - safe_positions[:, second_id], dim=-1
        )
        actual_distance = torch.linalg.vector_norm(
            actual_positions[:, first_id] - actual_positions[:, second_id], dim=-1
        )
        hard = float(constraint["hard_margin_m"])
        buffer = max(float(constraint["release_buffer_m"]), 1.0e-6)
        factor = torch.minimum(
            torch.clamp((safe_distance - hard) / buffer, min=0.0, max=1.0),
            torch.clamp((actual_distance - hard) / buffer, min=0.0, max=1.0),
        )
        for arm in constraint["arms"]:
            indices = list(POLICY_ARM_ACTION_INDICES[str(arm)])
            scales[:, indices] = torch.minimum(scales[:, indices], factor[:, None])
    for constraint in POLICY_TORSO_CAPSULE_CONSTRAINTS:
        safe_margin = _capsule_surface_margin_torch(
            safe_positions,
            body_names,
            str(constraint["side"]),
            str(constraint["part"]),
        )
        actual_margin = _capsule_surface_margin_torch(
            actual_positions,
            body_names,
            str(constraint["side"]),
            str(constraint["part"]),
        )
        hard = float(constraint["hard_margin_m"])
        buffer = max(float(constraint["release_buffer_m"]), 1.0e-6)
        factor = torch.minimum(
            torch.clamp((safe_margin - hard) / buffer, min=0.0, max=1.0),
            torch.clamp((actual_margin - hard) / buffer, min=0.0, max=1.0),
        )
        for arm in constraint["arms"]:
            indices = list(POLICY_ARM_ACTION_INDICES[str(arm)])
            scales[:, indices] = torch.minimum(scales[:, indices], factor[:, None])
    return scales


def residual_scale_vector() -> np.ndarray:
    """Return policy correction authority in canonical action order."""

    return np.asarray(
        [RESIDUAL_SCALE_RAD_BY_JOINT[name] for name in UPPER_POLICY_JOINTS],
        dtype=np.float32,
    )


def supportive_residual_numpy(
    correction_rad: np.ndarray,
    q_ref_minus_q_rad: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Keep only a bounded lead correction that reduces current IK lag."""

    correction = np.asarray(correction_rad, dtype=np.float32)
    error = np.asarray(q_ref_minus_q_rad, dtype=np.float32)
    if correction.shape != error.shape:
        raise ValueError("supportive residual expects matching correction/error shapes")
    gain = np.clip(
        (np.abs(error) - SUPPORT_ERROR_DEADBAND_RAD)
        / max(SUPPORT_FULL_ERROR_RAD - SUPPORT_ERROR_DEADBAND_RAD, 1.0e-6),
        0.0,
        1.0,
    )
    aligned = np.where(correction * error > 0.0, correction, 0.0)
    cap = SUPPORT_MAX_LEAD_RATIO * np.abs(error) + SUPPORT_MAX_BIAS_RAD
    result = np.sign(error) * np.minimum(np.abs(aligned), cap) * gain
    return result.astype(np.float32, copy=False), gain.astype(np.float32, copy=False)


def supportive_residual_torch(correction_rad, q_ref_minus_q_rad):
    """Torch equivalent of :func:`supportive_residual_numpy`."""

    import torch

    gain = torch.clamp(
        (torch.abs(q_ref_minus_q_rad) - SUPPORT_ERROR_DEADBAND_RAD)
        / max(SUPPORT_FULL_ERROR_RAD - SUPPORT_ERROR_DEADBAND_RAD, 1.0e-6),
        min=0.0,
        max=1.0,
    )
    aligned = torch.where(
        correction_rad * q_ref_minus_q_rad > 0.0,
        correction_rad,
        torch.zeros_like(correction_rad),
    )
    cap = SUPPORT_MAX_LEAD_RATIO * torch.abs(q_ref_minus_q_rad) + SUPPORT_MAX_BIAS_RAD
    result = torch.sign(q_ref_minus_q_rad) * torch.minimum(torch.abs(aligned), cap) * gain
    return result, gain


def postprocess_action_numpy(
    raw_action: np.ndarray,
    previous_slewed_action: np.ndarray,
    reference_confidence: float | np.ndarray,
    *,
    action_clip: float = ACTION_CLIP,
    slew_per_step: float = ACTION_SLEW_PER_STEP,
    full_tracking_confidence: float = FULL_TRACKING_CONFIDENCE,
    hold_below_confidence: float = HOLD_BELOW_CONFIDENCE,
    residual_scale_rad: np.ndarray | None = None,
    safety_scale: np.ndarray | None = None,
    support_error_rad: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    """Exact deployment wrapper shared by live inference and validation.

    The returned ``raw`` output is never hidden by clipping.  ``slewed`` is the
    state carried to the next policy step, while ``applied`` also includes the
    confidence gate and is what belongs in the next observation.
    """

    raw = np.asarray(raw_action, dtype=np.float32)
    previous = np.asarray(previous_slewed_action, dtype=np.float32)
    if raw.shape != previous.shape or raw.shape[-1] != ACTION_DIM:
        raise ValueError(
            f"action shapes must match and end in {ACTION_DIM}: "
            f"raw={raw.shape}, previous={previous.shape}"
        )
    if not np.isfinite(raw).all() or not np.isfinite(previous).all():
        raise ValueError("policy action contains a non-finite value")
    clipped = np.clip(raw, -float(action_clip), float(action_clip))
    pre_safety_slewed = previous + np.clip(
        clipped - previous, -float(slew_per_step), float(slew_per_step)
    )
    if safety_scale is None:
        guard = np.ones_like(pre_safety_slewed, dtype=np.float32)
    else:
        guard = np.broadcast_to(
            np.asarray(safety_scale, dtype=np.float32), pre_safety_slewed.shape
        )
        if not np.isfinite(guard).all():
            raise ValueError("policy safety scale contains a non-finite value")
        guard = np.clip(guard, 0.0, 1.0)
    # Guard the carried slew state as well as the applied correction.  This
    # prevents a stored colliding action from snapping back on release.
    slewed = pre_safety_slewed * guard
    confidence = np.asarray(reference_confidence, dtype=np.float32)
    gain = np.clip(
        (confidence - float(hold_below_confidence))
        / max(float(full_tracking_confidence) - float(hold_below_confidence), 1.0e-6),
        0.0,
        1.0,
    )
    while gain.ndim < slewed.ndim:
        gain = np.expand_dims(gain, axis=-1)
    applied = slewed * gain
    scale = residual_scale_vector() if residual_scale_rad is None else np.asarray(
        residual_scale_rad, dtype=np.float32
    )
    correction = applied * scale
    support_gain = np.ones_like(applied, dtype=np.float32)
    if support_error_rad is not None:
        correction, support_gain = supportive_residual_numpy(
            correction, np.broadcast_to(
                np.asarray(support_error_rad, dtype=np.float32), correction.shape
            )
        )
        applied = np.divide(
            correction,
            scale,
            out=np.zeros_like(correction),
            where=np.abs(scale) > 1.0e-9,
        )
        # Do not retain a hidden action that the supportive projection rejected.
        slewed = applied.copy()
    return {
        "raw": raw,
        "clipped": clipped.astype(np.float32, copy=False),
        "slewed": slewed.astype(np.float32, copy=False),
        "pre_safety_slewed": pre_safety_slewed.astype(np.float32, copy=False),
        "safety_scale": guard.astype(np.float32, copy=False),
        "applied": applied.astype(np.float32, copy=False),
        "correction_rad": correction.astype(np.float32, copy=False),
        "support_gain": support_gain.astype(np.float32, copy=False),
        "clip_delta": (raw - clipped).astype(np.float32, copy=False),
        "saturation_mask": (np.abs(raw) > float(action_clip)),
    }


def postprocess_action_torch(
    raw_action,
    previous_slewed_action,
    reference_confidence,
    *,
    action_clip: float = ACTION_CLIP,
    slew_per_step: float = ACTION_SLEW_PER_STEP,
    full_tracking_confidence: float = FULL_TRACKING_CONFIDENCE,
    hold_below_confidence: float = HOLD_BELOW_CONFIDENCE,
    safety_scale=None,
    residual_scale_rad=None,
    support_error_rad=None,
):
    """Torch equivalent of :func:`postprocess_action_numpy` for Isaac Lab."""

    import torch

    clipped = torch.clamp(raw_action, -float(action_clip), float(action_clip))
    pre_safety_slewed = previous_slewed_action + torch.clamp(
        clipped - previous_slewed_action,
        -float(slew_per_step),
        float(slew_per_step),
    )
    if safety_scale is None:
        guard = torch.ones_like(pre_safety_slewed)
    else:
        guard = torch.broadcast_to(
            safety_scale.to(dtype=pre_safety_slewed.dtype, device=pre_safety_slewed.device),
            pre_safety_slewed.shape,
        ).clamp(0.0, 1.0)
    slewed = pre_safety_slewed * guard
    gain = torch.clamp(
        (reference_confidence - float(hold_below_confidence))
        / max(float(full_tracking_confidence) - float(hold_below_confidence), 1.0e-6),
        min=0.0,
        max=1.0,
    )
    while gain.ndim < slewed.ndim:
        gain = gain.unsqueeze(-1)
    applied = slewed * gain
    support_gain = torch.ones_like(applied)
    correction = None
    if residual_scale_rad is not None:
        scale = torch.broadcast_to(
            residual_scale_rad.to(dtype=applied.dtype, device=applied.device),
            applied.shape,
        )
        correction = applied * scale
        if support_error_rad is not None:
            correction, support_gain = supportive_residual_torch(
                correction,
                torch.broadcast_to(support_error_rad, correction.shape),
            )
            applied = torch.where(
                torch.abs(scale) > 1.0e-9,
                correction / scale,
                torch.zeros_like(correction),
            )
            slewed = applied
    return {
        "raw": raw_action,
        "clipped": clipped,
        "slewed": slewed,
        "pre_safety_slewed": pre_safety_slewed,
        "safety_scale": guard,
        "applied": applied,
        "correction_rad": correction,
        "support_gain": support_gain,
        "clip_delta": raw_action - clipped,
        "saturation_mask": torch.abs(raw_action) > float(action_clip),
    }


def _vector(name: str, value: Iterable[float], size: int) -> np.ndarray:
    result = np.asarray(value, dtype=np.float32).reshape(-1)
    if result.shape != (size,):
        raise ValueError(f"{name} must contain {size} values, got {result.shape}")
    if not np.isfinite(result).all():
        raise ValueError(f"{name} contains a non-finite value")
    return result


def assemble_observation(
    *,
    projected_gravity: Iterable[float],
    base_ang_vel: Iterable[float],
    q_minus_q_default: Iterable[float],
    qd: Iterable[float],
    q_ref_minus_q: Iterable[float],
    qd_ref_minus_qd: Iterable[float],
    body_reference_error: Iterable[float],
    foot_contact_state: Iterable[float],
    previous_action: Iterable[float],
    reference_confidence: float,
) -> np.ndarray:
    """Assemble the normalized 1-D actor observation used by ONNX.

    Inputs are raw SI values.  Scale factors mirror ``tracking_env_cfg.py``.
    The returned vector never silently pads, truncates, or accepts NaN values.
    """

    confidence = float(reference_confidence)
    if not np.isfinite(confidence):
        raise ValueError("reference_confidence must be finite")
    confidence = float(np.clip(confidence, 0.0, 1.0))
    parts = (
        _vector("projected_gravity", projected_gravity, 3)
        * OBSERVATION_SCALES["projected_gravity"],
        _vector("base_ang_vel", base_ang_vel, 3)
        * OBSERVATION_SCALES["base_ang_vel"],
        _vector("q_minus_q_default", q_minus_q_default, ACTION_DIM)
        * OBSERVATION_SCALES["q_minus_q_default"],
        _vector("qd", qd, ACTION_DIM) * OBSERVATION_SCALES["qd"],
        _vector("q_ref_minus_q", q_ref_minus_q, ACTION_DIM)
        * OBSERVATION_SCALES["q_ref_minus_q"],
        _vector("qd_ref_minus_qd", qd_ref_minus_qd, ACTION_DIM)
        * OBSERVATION_SCALES["qd_ref_minus_qd"],
        _vector(
            "body_reference_error",
            body_reference_error,
            3 * len(TRACKED_BODIES),
        )
        * OBSERVATION_SCALES["body_reference_error"],
        _vector("foot_contact_state", foot_contact_state, 2)
        * OBSERVATION_SCALES["foot_contact_state"],
        _vector("previous_action", previous_action, ACTION_DIM)
        * OBSERVATION_SCALES["previous_action"],
        np.asarray(
            [confidence * OBSERVATION_SCALES["reference_confidence"]],
            dtype=np.float32,
        ),
    )
    observation = np.concatenate(parts).astype(np.float32, copy=False)
    if observation.shape != (OBSERVATION_DIM,):
        raise RuntimeError(
            f"internal observation contract error: {observation.shape} != "
            f"({OBSERVATION_DIM},)"
        )
    return observation


def policy_metadata() -> dict:
    """Serializable metadata written next to the exported ONNX policy."""

    return {
        "schema": SCHEMA,
        "mode": "ik_residual_upper_body_fixed_double_support",
        "observation_dim": OBSERVATION_DIM,
        "action_dim": ACTION_DIM,
        "joint_names": list(UPPER_POLICY_JOINTS),
        "fixed_lower_body_joint_names": list(LOWER_POLICY_JOINTS),
        "tracked_body_names": list(TRACKED_BODIES),
        "observation_scales": dict(OBSERVATION_SCALES),
        "residual_scale_rad": RESIDUAL_SCALE_RAD,
        "residual_scale_rad_by_joint": dict(RESIDUAL_SCALE_RAD_BY_JOINT),
        "action_clip": ACTION_CLIP,
        "action_slew_per_step": ACTION_SLEW_PER_STEP,
        "full_tracking_confidence": FULL_TRACKING_CONFIDENCE,
        "hold_below_confidence": HOLD_BELOW_CONFIDENCE,
        "support_error_deadband_rad": SUPPORT_ERROR_DEADBAND_RAD,
        "support_full_error_rad": SUPPORT_FULL_ERROR_RAD,
        "support_max_lead_ratio": SUPPORT_MAX_LEAD_RATIO,
        "support_max_bias_rad": SUPPORT_MAX_BIAS_RAD,
        "fixed_double_support_contact_state": list(FIXED_DOUBLE_SUPPORT_CONTACT_STATE),
        "inference_wrapper_version": INFERENCE_WRAPPER_VERSION,
        "policy_collision_constraints": [dict(item) for item in POLICY_COLLISION_CONSTRAINTS],
        "policy_torso_capsule_constraints": [
            dict(item) for item in POLICY_TORSO_CAPSULE_CONSTRAINTS
        ],
        "policy_step_dt": POLICY_STEP_DT,
        "lower_body_control": "nominal_fixed_double_support",
        "base_control": "fixed_root_training",
    }

"""Fail-closed feasibility projection for the official G1 23-DOF order."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Iterable

import numpy as np


class SafetyLevel(IntEnum):
    GREEN = 0
    YELLOW = 1
    ORANGE = 2
    RED = 3


G1_23_LIMITS_RAD = np.asarray([
    [-2.5307, 2.8798], [-0.5236, 2.9671], [-2.7576, 2.7576], [-0.087267, 2.8798], [-0.87267, 0.5236], [-0.2618, 0.2618],
    [-2.5307, 2.8798], [-2.9671, 0.5236], [-2.7576, 2.7576], [-0.087267, 2.8798], [-0.87267, 0.5236], [-0.2618, 0.2618],
    [-2.618, 2.618], [-3.0892, 2.6704], [-1.5882, 2.2515], [-2.618, 2.618], [-1.0472, 2.0944], [-1.97222, 1.97222],
    [-3.0892, 2.6704], [-2.2515, 1.5882], [-2.618, 2.618], [-1.0472, 2.0944], [-1.97222, 1.97222],
], dtype=np.float64)

# The official actuator model allows a small negative elbow angle, but that
# is the non-human mirrored IK branch.  GMR already solves both elbows with a
# non-negative lower bound; use the same boundary for the final safe command
# so velocity/acceleration state cannot carry an otherwise-valid solution
# through anatomical hyper-extension.
G1_23_ANATOMICAL_LIMITS_RAD = G1_23_LIMITS_RAD.copy()
G1_23_ANATOMICAL_LIMITS_RAD[[16, 21], 0] = 0.0

# Conservative command-boundary limits, deliberately below model maxima.
G1_23_VELOCITY_RAD_S = np.asarray([3.0] * 12 + [3.5] + [6.0, 6.0, 6.0, 7.0, 5.0] * 2)
G1_23_ACCELERATION_RAD_S2 = np.asarray([12.0] * 12 + [15.0] + [30.0] * 10)


@dataclass(frozen=True)
class FeasibilityResult:
    raw_q: np.ndarray
    safe_q: np.ndarray
    level: SafetyLevel
    reasons: tuple[str, ...]
    blend: float
    joint_limit_saturation_count: int
    velocity_limit_count: int
    acceleration_limit_count: int
    arm_blend: dict[str, float] = field(default_factory=dict)
    minimum_collision_margin_m: float = float("inf")


class G1FeasibilityFilter:
    def __init__(
        self,
        nominal_q: Iterable[float] | None = None,
        soft_margin_rad: float = 0.04,
        *,
        residual_warn: float = 0.10,
        residual_severe: float = 0.45,
        yellow_blend: float = 0.55,
        orange_return_after_s: float | None = None,
        orange_return_tau_s: float = 0.80,
    ) -> None:
        self.nominal = np.zeros(23) if nominal_q is None else np.asarray(list(nominal_q), dtype=np.float64)
        if self.nominal.shape != (23,):
            raise ValueError("nominal_q must contain 23 joints")
        self.margin = float(max(0.0, soft_margin_rad))
        self.residual_warn = float(max(0.0, residual_warn))
        self.residual_severe = float(max(self.residual_warn, residual_severe))
        self.yellow_blend = float(np.clip(yellow_blend, 0.0, 1.0))
        self.orange_return_after_s = (
            None
            if orange_return_after_s is None
            else float(max(0.0, orange_return_after_s))
        )
        self.orange_return_tau_s = float(max(0.05, orange_return_tau_s))
        self.safe_q: np.ndarray | None = None
        self.velocity = np.zeros(23, dtype=np.float64)
        self.last_reliable: np.ndarray | None = None
        self.orange_elapsed_s = 0.0
        self.arm_collision_elapsed_s = {"left": 0.0, "right": 0.0}

    def reset(self) -> None:
        self.safe_q = None
        self.velocity.fill(0.0)
        self.last_reliable = None
        self.orange_elapsed_s = 0.0
        self.arm_collision_elapsed_s = {"left": 0.0, "right": 0.0}

    @staticmethod
    def _collision_blend(margin_m: float) -> float:
        """Continuous reference-governor blend from signed capsule margin."""
        if not np.isfinite(margin_m) or margin_m >= 0.030:
            return 1.0
        if margin_m >= 0.0:
            return float(0.35 + 0.65 * margin_m / 0.030)
        if margin_m >= -0.020:
            return float(0.05 + 0.30 * (margin_m + 0.020) / 0.020)
        return 0.0

    def update(
        self,
        raw_q: Iterable[float],
        dt: float,
        *,
        reasons: Iterable[str] = (),
        gmr_residual: float = 0.0,
        self_collision: bool = False,
        stale: bool = False,
        collision_margin_m: float = float("inf"),
        arm_collision_margins: dict[str, float] | None = None,
        arm_reasons: dict[str, Iterable[str]] | None = None,
        arm_quality_blend: dict[str, float] | None = None,
    ) -> FeasibilityResult:
        raw = np.asarray(list(raw_q), dtype=np.float64)
        external = list(dict.fromkeys(str(reason) for reason in reasons))
        per_arm_reasons = {
            side: list(dict.fromkeys(str(reason) for reason in values))
            for side, values in (arm_reasons or {}).items()
            if side in {"left", "right"}
        }
        if raw.shape != (23,) or not np.isfinite(raw).all():
            target = self.last_reliable if self.last_reliable is not None else self.nominal
            self.safe_q = target.copy()
            return FeasibilityResult(raw, target.copy(), SafetyLevel.RED, tuple(external + ["non_finite_target"]), 0.0, 0, 0, 0, {"left": 0.0, "right": 0.0}, float(collision_margin_m))

        low = G1_23_ANATOMICAL_LIMITS_RAD[:, 0] + self.margin
        high = G1_23_ANATOMICAL_LIMITS_RAD[:, 1] - self.margin
        projected = np.clip(raw, low, high)
        joint_count = int(np.count_nonzero(np.abs(projected - raw) > 1e-9))
        if joint_count:
            external.append("joint_limit_saturation")
        if gmr_residual > self.residual_warn:
            external.append("gmr_high_residual")
        arm_margins = dict(arm_collision_margins or {})
        continuous_collision_available = bool(arm_margins)
        if continuous_collision_available:
            for side in ("left", "right"):
                side_margin = float(arm_margins.get(side, float("inf")))
                if side_margin < 0.005:
                    per_arm_reasons.setdefault(side, []).append(
                        "self_collision_proximity"
                    )
                if side_margin < 0.0:
                    per_arm_reasons.setdefault(side, []).append(
                        "self_collision_risk"
                    )
        elif collision_margin_m < 0.005:
            external.append("self_collision_proximity")
            if collision_margin_m < 0.0:
                external.append("self_collision_risk")
        if self_collision and not continuous_collision_available:
            external.append("self_collision_risk")
        if stale:
            external.append("stale_packet")

        bilateral_penetration = all(
            float(arm_margins.get(side, float("inf"))) < -0.020
            for side in ("left", "right")
        )
        severe = (
            stale
            or gmr_residual > self.residual_severe
            or joint_count >= 4
            or bilateral_penetration
            or (self_collision and not continuous_collision_available)
        )
        arm_severe = any(
            float(arm_margins.get(side, float("inf"))) < -0.020
            for side in ("left", "right")
        )
        degraded = bool(external) or gmr_residual > self.residual_warn
        arm_degraded = any(per_arm_reasons.values())
        level = SafetyLevel.ORANGE if (severe or arm_severe) else (
            SafetyLevel.YELLOW if (degraded or arm_degraded) else SafetyLevel.GREEN
        )
        dt = float(np.clip(dt, 1.0 / 240.0, 0.1))
        motion_level = SafetyLevel.ORANGE if severe else (
            SafetyLevel.YELLOW if degraded else SafetyLevel.GREEN
        )
        blend = {
            SafetyLevel.GREEN: 1.0,
            SafetyLevel.YELLOW: self.yellow_blend,
            SafetyLevel.ORANGE: 0.0,
        }[motion_level]
        base = self.safe_q.copy() if self.safe_q is not None else self.nominal.copy()
        if motion_level == SafetyLevel.ORANGE:
            self.orange_elapsed_s += dt
            target = (
                self.last_reliable.copy()
                if self.last_reliable is not None
                else base.copy()
            )
            # A brief ambiguity should hold the last reliable command.  A
            # persistent ambiguity must not freeze the humanoid forever in an
            # awkward pose: after a grace interval, return continuously toward
            # the official neutral command.  This is opt-in so existing
            # whole-body/physical-robot behavior is unchanged.
            if (
                self.orange_return_after_s is not None
                and self.orange_elapsed_s > self.orange_return_after_s
            ):
                alpha = 1.0 - np.exp(-dt / self.orange_return_tau_s)
                target = base + alpha * (self.nominal - base)
                external.append("orange_safe_return")
        else:
            self.orange_elapsed_s = 0.0
            target = base + blend * (projected - base)

        arm_blend: dict[str, float] = {}
        for side, indices in (
            ("left", slice(13, 18)),
            ("right", slice(18, 23)),
        ):
            side_margin = float(arm_margins.get(side, float("inf")))
            collision_blend = self._collision_blend(side_margin)
            quality_blend = float(np.clip(
                (arm_quality_blend or {}).get(side, 1.0), 0.0, 1.0
            ))
            side_blend = min(collision_blend, quality_blend)
            arm_blend[side] = side_blend
            if side_blend < 1.0 and motion_level != SafetyLevel.ORANGE:
                target[indices] = base[indices] + side_blend * (
                    projected[indices] - base[indices]
                )
                self.arm_collision_elapsed_s[side] += dt
                # Never hold one arm forever. A persistent unsafe target
                # returns only that arm toward neutral while the other arm and
                # torso remain live.
                if (
                    side_blend <= 0.05
                    and self.arm_collision_elapsed_s[side] > 0.35
                ):
                    alpha = 1.0 - np.exp(-dt / self.orange_return_tau_s)
                    target[indices] = base[indices] + alpha * (
                        self.nominal[indices] - base[indices]
                    )
                    external.append(f"{side}_arm_safe_return")
            else:
                self.arm_collision_elapsed_s[side] = 0.0

        desired_velocity = (target - base) / dt
        limited_velocity = np.clip(desired_velocity, -G1_23_VELOCITY_RAD_S, G1_23_VELOCITY_RAD_S)
        velocity_count = int(np.count_nonzero(np.abs(limited_velocity - desired_velocity) > 1e-9))
        max_dv = G1_23_ACCELERATION_RAD_S2 * dt
        accel_velocity = self.velocity + np.clip(limited_velocity - self.velocity, -max_dv, max_dv)
        acceleration_count = int(np.count_nonzero(np.abs(accel_velocity - limited_velocity) > 1e-9))
        unconstrained = base + accel_velocity * dt
        safe = np.clip(unconstrained, low, high)
        # Anti-windup at a hard limit: discard only velocity that points
        # farther through the boundary.  Without this, a rapidly extending
        # elbow can remain negative for several frames even though every raw
        # GMR target is anatomically valid.
        outward = ((unconstrained < low) & (accel_velocity < 0.0)) | (
            (unconstrained > high) & (accel_velocity > 0.0)
        )
        accel_velocity[outward] = 0.0
        self.safe_q = safe
        self.velocity = accel_velocity
        # A local arm governor must not prevent the remaining valid command
        # from becoming the next reliable state.
        if motion_level <= SafetyLevel.YELLOW:
            self.last_reliable = safe.copy()
        combined_reasons = external[:]
        for side in ("left", "right"):
            combined_reasons.extend(per_arm_reasons.get(side, ()))
        return FeasibilityResult(
            raw,
            safe.copy(),
            level,
            tuple(dict.fromkeys(combined_reasons)),
            blend,
            joint_count,
            velocity_count,
            acceleration_count,
            arm_blend,
            float(collision_margin_m),
        )

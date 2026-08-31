"""Bounded reference-motion timing for simulation and policy datasets."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ReferenceMotionSample:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray


class BoundedReferenceMotion:
    """Critically damped position tracking with explicit qd/qdd/jerk limits."""

    def __init__(
        self,
        initial_position: np.ndarray,
        *,
        response_hz: float = 1.2,
        max_velocity: float = 1.5,
        max_acceleration: float = 5.0,
        max_jerk: float = 45.0,
    ) -> None:
        self.position = np.asarray(initial_position, dtype=np.float64).copy()
        self.velocity = np.zeros_like(self.position)
        self.acceleration = np.zeros_like(self.position)
        self.response_hz = float(max(0.1, response_hz))
        self.max_velocity = float(max(0.1, max_velocity))
        self.max_acceleration = float(max(0.1, max_acceleration))
        self.max_jerk = float(max(0.1, max_jerk))

    def reset(self, position: np.ndarray) -> None:
        value = np.asarray(position, dtype=np.float64)
        if value.shape != self.position.shape:
            raise ValueError("Reference position shape changed")
        self.position = value.copy()
        self.velocity.fill(0.0)
        self.acceleration.fill(0.0)

    def update(
        self,
        target: np.ndarray,
        dt: float,
        *,
        lower: np.ndarray | None = None,
        upper: np.ndarray | None = None,
    ) -> ReferenceMotionSample:
        target_value = np.asarray(target, dtype=np.float64)
        if target_value.shape != self.position.shape or not np.isfinite(target_value).all():
            raise ValueError("Reference target must be finite and shape-stable")
        dt = float(np.clip(dt, 1.0e-4, 0.05))
        omega = 2.0 * np.pi * self.response_hz
        requested_acceleration = (
            omega * omega * (target_value - self.position)
            - 2.0 * omega * self.velocity
        )
        previous_acceleration = self.acceleration.copy()
        self.acceleration += np.clip(
            requested_acceleration - self.acceleration,
            -self.max_jerk * dt,
            self.max_jerk * dt,
        )
        self.acceleration = np.clip(
            self.acceleration, -self.max_acceleration, self.max_acceleration
        )
        self.velocity = np.clip(
            self.velocity + self.acceleration * dt,
            -self.max_velocity, self.max_velocity,
        )
        self.position = self.position + self.velocity * dt
        if lower is not None and upper is not None:
            clipped = np.clip(self.position, lower, upper)
            hit = np.abs(clipped - self.position) > 1.0e-12
            self.position = clipped
            self.velocity[hit] = 0.0
            self.acceleration[hit] = 0.0
        jerk = (self.acceleration - previous_acceleration) / dt
        return ReferenceMotionSample(
            self.position.copy(), self.velocity.copy(),
            self.acceleration.copy(), jerk,
        )


class LowLatencyReferenceMotion:
    """Rate-bounded tracking with velocity feed-forward.

    The upstream GMR feasibility filter already enforces G1 joint limits,
    velocity and acceleration.  This final simulator stage therefore avoids
    another low-bandwidth position filter: it follows the resampled target
    velocity and uses only the position error needed to catch up.  A braking
    envelope prevents step targets from overshooting materially.
    """

    def __init__(
        self,
        initial_position: np.ndarray,
        *,
        response_hz: float = 4.0,
        max_velocity: float = 5.0,
        max_acceleration: float = 30.0,
        max_jerk: float | None = None,
    ) -> None:
        self.position = np.asarray(initial_position, dtype=np.float64).copy()
        self.velocity = np.zeros_like(self.position)
        self.acceleration = np.zeros_like(self.position)
        self.response_hz = float(max(0.1, response_hz))
        self.max_velocity = float(max(0.1, max_velocity))
        self.max_acceleration = float(max(0.1, max_acceleration))
        self.max_jerk = (
            float("inf")
            if max_jerk is None
            else float(max(0.1, max_jerk))
        )

    def reset(self, position: np.ndarray) -> None:
        value = np.asarray(position, dtype=np.float64)
        if value.shape != self.position.shape:
            raise ValueError("Reference position shape changed")
        self.position = value.copy()
        self.velocity.fill(0.0)
        self.acceleration.fill(0.0)

    def update(
        self,
        target: np.ndarray,
        target_velocity: np.ndarray,
        dt: float,
        *,
        lower: np.ndarray | None = None,
        upper: np.ndarray | None = None,
    ) -> ReferenceMotionSample:
        target_value = np.asarray(target, dtype=np.float64)
        target_velocity_value = np.asarray(target_velocity, dtype=np.float64)
        if (
            target_value.shape != self.position.shape
            or target_velocity_value.shape != self.position.shape
            or not np.isfinite(target_value).all()
            or not np.isfinite(target_velocity_value).all()
        ):
            raise ValueError("Reference target and velocity must be finite and shape-stable")
        dt = float(np.clip(dt, 1.0e-4, 0.05))
        error = target_value - self.position
        tau = 1.0 / (2.0 * np.pi * self.response_hz)
        requested_velocity = target_velocity_value + error / tau
        # Relative braking speed sqrt(2*a*distance) makes a static step stop
        # at its target instead of ringing around it.
        braking_velocity = np.sqrt(
            2.0 * self.max_acceleration * np.abs(error)
        )
        relative_velocity = np.clip(
            requested_velocity - target_velocity_value,
            -braking_velocity,
            braking_velocity,
        )
        requested_velocity = np.clip(
            target_velocity_value + relative_velocity,
            -self.max_velocity,
            self.max_velocity,
        )
        previous_velocity = self.velocity.copy()
        previous_acceleration = self.acceleration.copy()
        requested_acceleration = np.clip(
            (requested_velocity - self.velocity) / dt,
            -self.max_acceleration,
            self.max_acceleration,
        )
        if np.isfinite(self.max_jerk):
            self.acceleration += np.clip(
                requested_acceleration - self.acceleration,
                -self.max_jerk * dt,
                self.max_jerk * dt,
            )
            # Keep the new acceleration inside the jerk-braking viability
            # envelope. For positive acceleration, for example,
            #   a*dt + a^2/(2*j) <= vmax-v
            # guarantees that future max-jerk braking can reach zero
            # acceleration without ever crossing the velocity limit. This
            # avoids turning a hard velocity clip into a hidden jerk spike.
            jerk_dt = self.max_jerk * dt
            positive_headroom = np.maximum(
                0.0, self.max_velocity - self.velocity
            )
            negative_headroom = np.maximum(
                0.0, self.max_velocity + self.velocity
            )
            positive_acceleration_limit = (
                -jerk_dt
                + np.sqrt(
                    jerk_dt * jerk_dt
                    + 2.0 * self.max_jerk * positive_headroom
                )
            )
            negative_acceleration_limit = (
                -jerk_dt
                + np.sqrt(
                    jerk_dt * jerk_dt
                    + 2.0 * self.max_jerk * negative_headroom
                )
            )
            self.acceleration = np.minimum(
                self.acceleration, positive_acceleration_limit
            )
            self.acceleration = np.maximum(
                self.acceleration, -negative_acceleration_limit
            )
        else:
            self.acceleration = requested_acceleration
        self.acceleration = np.clip(
            self.acceleration, -self.max_acceleration, self.max_acceleration
        )
        self.velocity += self.acceleration * dt
        self.velocity = np.clip(
            self.velocity, -self.max_velocity, self.max_velocity
        )
        self.acceleration = (self.velocity - previous_velocity) / dt
        step = self.velocity * dt
        # The braking envelope normally stops before the target.  Protect the
        # final sub-step as well so a coarse GUI wall-clock update cannot
        # cross and ring around a static pose.
        crosses_target = (
            (np.abs(step) > np.abs(error))
            & (np.sign(step) == np.sign(error))
        )
        self.position = self.position + step
        if np.any(crosses_target):
            self.position[crosses_target] = target_value[crosses_target]
        if lower is not None and upper is not None:
            clipped = np.clip(self.position, lower, upper)
            hit = np.abs(clipped - self.position) > 1.0e-12
            self.position = clipped
            self.velocity[hit] = 0.0
            self.acceleration[hit] = 0.0
        jerk = (self.acceleration - previous_acceleration) / dt
        return ReferenceMotionSample(
            self.position.copy(), self.velocity.copy(),
            self.acceleration.copy(), jerk,
        )

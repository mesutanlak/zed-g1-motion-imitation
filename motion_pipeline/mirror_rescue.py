"""Event-triggered, MIRROR-inspired continuation rescue for G1-23 arms.

The upstream deterministic GMR solution remains nominal.  This module only
searches nearby arm-only corrections when the nominal solution is suspicious.
It is intentionally not a reimplementation of the full research MIRROR GPU
stack; it preserves the same integration contract (parallel continuations,
progress check, collision-aware candidate selection) without replacing GMR.
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from time import perf_counter
from typing import Callable, Mapping, Sequence

import numpy as np

from .collision_geometry import CollisionDistanceReport


LEFT_ARM = np.asarray([13, 14, 15, 16, 17], dtype=np.int64)
RIGHT_ARM = np.asarray([18, 19, 20, 21, 22], dtype=np.int64)
LEFT_POSITION_ARM = np.asarray([13, 14, 15, 16], dtype=np.int64)
RIGHT_POSITION_ARM = np.asarray([18, 19, 20, 21], dtype=np.int64)


@dataclass(frozen=True)
class KinematicEvaluation:
    positions: Mapping[str, Sequence[float]]
    collision: CollisionDistanceReport


@dataclass(frozen=True)
class MirrorRescueResult:
    q: np.ndarray
    triggered: bool
    applied: bool
    trigger_reasons: tuple[str, ...]
    candidate_count: int
    selected_index: int
    nominal_cost: float
    selected_cost: float
    nominal_task_error_m: float
    selected_task_error_m: float
    nominal_collision_margin_m: float
    selected_collision_margin_m: float
    solve_ms: float


class MirrorContinuationRescue:
    """Nearby multi-start arm IK used only as a recovery path."""

    def __init__(
        self,
        *,
        enabled: bool = True,
        workers: int = 4,
        residual_trigger_m: float = 0.12,
        collision_trigger_m: float = 0.025,
        joint_margin_trigger_rad: float = 0.08,
        nominal_step_trigger_rad: float = 0.55,
        damping: float = 2.0e-2,
        finite_difference_rad: float = 2.0e-3,
        maximum_correction_rad: float = 0.16,
        minimum_interval_s: float = 0.15,
    ) -> None:
        self.enabled = bool(enabled)
        self.workers = max(1, int(workers))
        self.residual_trigger_m = float(max(0.0, residual_trigger_m))
        self.collision_trigger_m = float(collision_trigger_m)
        self.joint_margin_trigger_rad = float(max(0.0, joint_margin_trigger_rad))
        self.nominal_step_trigger_rad = float(max(0.0, nominal_step_trigger_rad))
        self.damping = float(max(1.0e-6, damping))
        self.finite_difference_rad = float(max(1.0e-5, finite_difference_rad))
        self.maximum_correction_rad = float(max(0.01, maximum_correction_rad))
        self.minimum_interval_s = float(max(0.0, minimum_interval_s))
        self.last_run_s = -float("inf")
        self.last_nominal_q: np.ndarray | None = None
        self.continuation = (0.45, 0.72, 1.0)

    def reset(self) -> None:
        self.last_run_s = -float("inf")
        self.last_nominal_q = None

    @staticmethod
    def _side_task_error(
        positions: Mapping[str, Sequence[float]],
        targets: Mapping[str, Sequence[float]],
        side: str,
    ) -> tuple[np.ndarray, float]:
        components: list[np.ndarray] = []
        for body_name, target_name, weight in (
            (f"{side}_elbow_link", f"{side}_elbow", 0.65),
            (f"{side}_wrist_roll_rubber_hand", f"{side}_wrist", 1.0),
        ):
            if body_name not in positions or target_name not in targets:
                continue
            current = np.asarray(positions[body_name], dtype=np.float64)
            target = np.asarray(targets[target_name], dtype=np.float64)
            if current.shape == (3,) and target.shape == (3,) and np.isfinite(current).all() and np.isfinite(target).all():
                components.append(weight * (target - current))
        if not components:
            return np.empty(0, dtype=np.float64), 0.0
        shoulder_body = f"{side}_shoulder_pitch_link"
        shoulder_target = f"{side}_shoulder"
        elbow_body = f"{side}_elbow_link"
        elbow_target = f"{side}_elbow"
        wrist_body = f"{side}_wrist_roll_rubber_hand"
        wrist_target = f"{side}_wrist"
        if all(
            name in positions
            for name in (shoulder_body, elbow_body, wrist_body)
        ) and all(
            name in targets
            for name in (shoulder_target, elbow_target, wrist_target)
        ):
            robot = [
                np.asarray(positions[name], dtype=np.float64)
                for name in (shoulder_body, elbow_body, wrist_body)
            ]
            human = [
                np.asarray(targets[name], dtype=np.float64)
                for name in (shoulder_target, elbow_target, wrist_target)
            ]
            for first, second in ((0, 1), (1, 2)):
                robot_vector = robot[second] - robot[first]
                human_vector = human[second] - human[first]
                robot_norm = float(np.linalg.norm(robot_vector))
                human_norm = float(np.linalg.norm(human_vector))
                if robot_norm > 1.0e-8 and human_norm > 1.0e-8:
                    # Metre-equivalent directional residual. It participates
                    # in both the finite-difference correction and acceptance
                    # score, preventing a rescue that improves wrist XYZ but
                    # turns the elbow/forearm onto a mirrored branch.
                    components.append(
                        0.035 * (
                            human_vector / human_norm
                            - robot_vector / robot_norm
                        )
                    )
        vector = np.concatenate(components)
        return vector, float(np.sqrt(np.mean(np.square(vector))))

    def _linearized_arm_correction(
        self,
        nominal: np.ndarray,
        targets: Mapping[str, Sequence[float]],
        evaluate: Callable[[np.ndarray], KinematicEvaluation],
        limits: np.ndarray,
        active_sides: tuple[str, ...],
    ) -> np.ndarray:
        """One bounded arm-only DLS/QP direction shared by all continuations."""
        correction_all = np.zeros_like(nominal)
        nominal_state = evaluate(nominal)
        for side in active_sides:
            # Wrist roll is an orientation-only soft task and cannot improve
            # elbow/wrist positions in the official 5-DOF arm chain.
            indices = LEFT_POSITION_ARM if side == "left" else RIGHT_POSITION_ARM
            error, _ = self._side_task_error(
                nominal_state.positions, targets, side
            )
            if error.size == 0:
                continue
            jacobian = np.zeros((error.size, len(indices)), dtype=np.float64)
            for column, joint_index in enumerate(indices):
                perturbed = nominal.copy()
                perturbed[joint_index] = float(
                    np.clip(
                        perturbed[joint_index] + self.finite_difference_rad,
                        limits[joint_index, 0],
                        limits[joint_index, 1],
                    )
                )
                perturbed_state = evaluate(perturbed)
                perturbed_error, _ = self._side_task_error(
                    perturbed_state.positions, targets, side
                )
                if perturbed_error.shape == error.shape:
                    # error = target-current, therefore d(current)/dq is the
                    # negative derivative of the error vector.
                    jacobian[:, column] = -(
                        perturbed_error - error
                    ) / self.finite_difference_rad
            lhs = jacobian.T @ jacobian + self.damping * np.eye(len(indices))
            rhs = jacobian.T @ error
            try:
                correction = np.linalg.solve(lhs, rhs)
            except np.linalg.LinAlgError:
                correction = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
            correction = np.clip(
                correction,
                -self.maximum_correction_rad,
                self.maximum_correction_rad,
            )
            correction_all[indices] = correction
        return correction_all

    def _score(
        self,
        q: np.ndarray,
        nominal_q: np.ndarray,
        previous_q: np.ndarray,
        targets: Mapping[str, Sequence[float]],
        evaluate: Callable[[np.ndarray], KinematicEvaluation],
        active_sides: tuple[str, ...],
    ) -> tuple[float, float, float]:
        state = evaluate(q)
        errors = [
            self._side_task_error(state.positions, targets, side)[1]
            for side in active_sides
        ]
        task_error = float(max(errors, default=0.0))
        margin = float(state.collision.minimum_margin_m)
        continuity = float(np.linalg.norm(q - previous_q))
        nominal_distance = float(np.linalg.norm(q - nominal_q))
        collision_penalty = 80.0 * max(0.0, self.collision_trigger_m - margin) ** 2
        cost = task_error + 0.018 * continuity + 0.010 * nominal_distance + collision_penalty
        return float(cost), task_error, margin

    def update(
        self,
        *,
        nominal_q: Sequence[float],
        previous_safe_q: Sequence[float] | None,
        targets: Mapping[str, Sequence[float]],
        evaluate: Callable[[np.ndarray], KinematicEvaluation],
        joint_limits: np.ndarray,
        residual_m: float,
        branch_change_pending: bool = False,
    ) -> MirrorRescueResult:
        started = perf_counter()
        nominal = np.asarray(nominal_q, dtype=np.float64)
        previous = (
            nominal.copy()
            if previous_safe_q is None
            else np.asarray(previous_safe_q, dtype=np.float64).copy()
        )
        previous_nominal = (
            nominal.copy()
            if self.last_nominal_q is None
            else self.last_nominal_q.copy()
        )
        self.last_nominal_q = nominal.copy()
        nominal_state = evaluate(nominal)
        side_task_errors = {
            side: self._side_task_error(
                nominal_state.positions, targets, side
            )[1]
            for side in ("left", "right")
        }
        lower_margin = nominal - joint_limits[:, 0]
        upper_margin = joint_limits[:, 1] - nominal
        # Trigger continuity from q_nom(k)-q_nom(k-1), not from the lagged
        # safe command.  The latter is intentionally rate-limited and would
        # otherwise keep MIRROR armed throughout every fast but valid motion.
        nominal_delta = nominal - previous_nominal
        approaching_limit = (
            ((lower_margin < self.joint_margin_trigger_rad) & (nominal_delta < -0.01))
            | ((upper_margin < self.joint_margin_trigger_rad) & (nominal_delta > 0.01))
        )
        # A straight anatomical elbow normally rests at its non-negative
        # lower bound.  Proximity alone is therefore not an emergency; only a
        # target moving farther into that bound is a continuation trigger.
        side_limit_near = {
            "left": bool(np.any(approaching_limit[LEFT_ARM])),
            "right": bool(np.any(approaching_limit[RIGHT_ARM])),
        }
        arm_step = float(
            max(
                np.linalg.norm(nominal[LEFT_ARM] - previous_nominal[LEFT_ARM]),
                np.linalg.norm(nominal[RIGHT_ARM] - previous_nominal[RIGHT_ARM]),
            )
        )
        reasons: list[str] = []
        if residual_m > self.residual_trigger_m:
            reasons.append("gmr_high_residual")
        if nominal_state.collision.minimum_margin_m < self.collision_trigger_m:
            reasons.append("collision_margin_low")
        if any(side_limit_near.values()):
            reasons.append("joint_limit_near")
        if arm_step > self.nominal_step_trigger_rad:
            reasons.append("nominal_step_large")
        if branch_change_pending:
            reasons.append("elbow_branch_change_pending")

        active_sides = tuple(
            side
            for side in ("left", "right")
            if (
                nominal_state.collision.arm_minimum_margin_m.get(side, float("inf"))
                < self.collision_trigger_m
                or side_task_errors[side] > self.residual_trigger_m
                or branch_change_pending
                or side_limit_near[side]
                or np.linalg.norm(
                    nominal[LEFT_ARM if side == "left" else RIGHT_ARM]
                    - previous_nominal[LEFT_ARM if side == "left" else RIGHT_ARM]
                ) > self.nominal_step_trigger_rad
            )
        )
        if not active_sides:
            active_sides = ("left", "right")
        nominal_cost, nominal_error, nominal_margin = self._score(
            nominal, nominal, previous, targets, evaluate, active_sides
        )
        if not self.enabled or not reasons or previous_safe_q is None:
            return MirrorRescueResult(
                nominal, bool(reasons), False, tuple(reasons), 0, -1,
                nominal_cost, nominal_cost, nominal_error, nominal_error,
                nominal_margin, nominal_margin, (perf_counter() - started) * 1000.0,
            )

        urgent = bool(
            branch_change_pending
            or nominal_state.collision.minimum_margin_m < 0.0
        )
        now_s = perf_counter()
        if not urgent and now_s - self.last_run_s < self.minimum_interval_s:
            return MirrorRescueResult(
                nominal,
                True,
                False,
                tuple([*reasons, "rescue_rate_limited"]),
                0,
                -1,
                nominal_cost,
                nominal_cost,
                nominal_error,
                nominal_error,
                nominal_margin,
                nominal_margin,
                (perf_counter() - started) * 1000.0,
            )
        self.last_run_s = now_s

        correction = self._linearized_arm_correction(
            nominal, targets, evaluate, joint_limits, active_sides
        )
        candidates = []
        for alpha in self.continuation:
            # Continuation remains anchored between the last safe state and
            # q_nom; the shared linearized arm correction never touches the
            # lower body or waist.
            candidate = previous + alpha * (nominal - previous)
            candidate += alpha * correction
            candidate = np.clip(
                candidate, joint_limits[:, 0], joint_limits[:, 1]
            )
            candidate[[16, 21]] = np.maximum(candidate[[16, 21]], 0.0)
            candidates.append(candidate)
        # Candidate scoring is independent and may run concurrently.  The
        # expensive finite-difference Jacobian above is intentionally shared.
        if self.workers > 1:
            with ThreadPoolExecutor(max_workers=min(self.workers, len(candidates))) as executor:
                scored = list(
                    executor.map(
                        lambda candidate: self._score(
                            candidate,
                            nominal,
                            previous,
                            targets,
                            evaluate,
                            active_sides,
                        ),
                        candidates,
                    )
                )
        else:
            scored = [
                self._score(
                    candidate, nominal, previous, targets, evaluate, active_sides
                )
                for candidate in candidates
            ]
        selected_index = min(range(len(candidates)), key=lambda index: scored[index][0])
        selected = candidates[selected_index]
        selected_cost, selected_error, selected_margin = scored[selected_index]
        # Lyapunov-like progress certificate: accept a lower total cost, or a
        # materially safer collision margin with only a small task-error loss.
        accepted = bool(
            (
                selected_cost < 0.98 * nominal_cost
                and selected_error < nominal_error - 0.002
                and selected_margin >= nominal_margin - 0.005
            )
            or (
                selected_margin > nominal_margin + 0.010
                and selected_error <= nominal_error + 0.015
            )
        )
        return MirrorRescueResult(
            selected if accepted else nominal,
            True,
            accepted,
            tuple(reasons),
            len(candidates),
            selected_index,
            nominal_cost,
            selected_cost,
            nominal_error,
            selected_error,
            nominal_margin,
            selected_margin,
            (perf_counter() - started) * 1000.0,
        )

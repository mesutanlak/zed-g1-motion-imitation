"""Name-based G1 29-DOF to physical G1 EDU 23-DOF projection."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from pathlib import Path

import numpy as np
import mujoco
import mink


G1_23DOF_ORDER = (
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

LOCKED_G1_29DOF_JOINTS = (
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)


class AnatomicalElbowRegularizer:
    """Low-priority closed-form elbow-flexion target for Mink/GMR.

    Cartesian elbow and wrist targets determine an arm pose, but close to a
    straight arm their Jacobian is ill-conditioned and the shoulder/elbow
    decomposition may jump between mirrored branches.  This task contributes
    only the two hinge DoFs.  Frame tasks still determine the actual endpoint
    pose, while the law-of-cosines target selects the human-like branch.
    """

    def __init__(
        self,
        retargeter,
        *,
        cost: float = 0.05,
        nominal_fps: float = 15.0,
        max_speed_rad_s: float = 7.0,
    ) -> None:
        self.retargeter = retargeter
        self.model = retargeter.model
        self.nominal_fps = float(max(1.0, nominal_fps))
        self.max_step = float(max_speed_rad_s) / self.nominal_fps
        costs = np.zeros(self.model.nv, dtype=np.float64)
        self.joints: dict[str, tuple[int, int]] = {}
        for side in ("left", "right"):
            name = f"{side}_elbow_joint"
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise KeyError(name)
            qpos_id = int(self.model.jnt_qposadr[joint_id])
            dof_id = int(self.model.jnt_dofadr[joint_id])
            costs[dof_id] = float(max(0.0, cost))
            self.joints[side] = (joint_id, qpos_id)
        self.task = mink.PostureTask(
            self.model, cost=costs, gain=0.8, lm_damping=1.0e-3
        )
        self.previous: dict[str, float] = {}
        self.last_target_rad: dict[str, float] = {}

    def reset(self, qpos: np.ndarray | None = None) -> None:
        """Clear elbow branch history and restore a neutral posture target."""
        self.previous.clear()
        self.last_target_rad.clear()
        target = (
            self.retargeter.configuration.data.qpos.copy()
            if qpos is None
            else np.asarray(qpos, dtype=np.float64).copy()
        )
        self.task.set_target(target)

    @staticmethod
    def _flexion(
        shoulder: np.ndarray, elbow: np.ndarray, wrist: np.ndarray
    ) -> float | None:
        upper = shoulder - elbow
        fore = wrist - elbow
        denominator = float(np.linalg.norm(upper) * np.linalg.norm(fore))
        if denominator < 1.0e-9:
            return None
        interior = float(
            np.arccos(np.clip(np.dot(upper, fore) / denominator, -1.0, 1.0))
        )
        return float(np.pi - interior)

    def update(self, human_data: Mapping[str, tuple[np.ndarray, np.ndarray]]) -> None:
        target = self.retargeter.configuration.data.qpos.copy()
        for side, (joint_id, qpos_id) in self.joints.items():
            names = (
                f"{side}_shoulder", f"{side}_elbow", f"{side}_wrist"
            )
            if not all(name in human_data for name in names):
                continue
            flexion = self._flexion(
                *(np.asarray(human_data[name][0], dtype=np.float64) for name in names)
            )
            if flexion is None or not np.isfinite(flexion):
                continue
            low, high = map(float, self.model.jnt_range[joint_id])
            desired = float(np.clip(flexion, low, high))
            previous = self.previous.get(side, desired)
            desired = previous + float(
                np.clip(desired - previous, -self.max_step, self.max_step)
            )
            self.previous[side] = desired
            self.last_target_rad[side] = desired
            target[qpos_id] = desired
        self.task.set_target(target)


class ArmPositionIKRefiner:
    """Deterministic arm-only IK on the official G1 23-DOF chain.

    Whole-body GMR is a useful initializer, but its coupled Cartesian task
    solve can converge to a wrong shoulder branch even when BODY_38 clearly
    observes a hanging arm.  This refiner solves only the four position DoFs
    that determine each elbow and wrist origin: shoulder pitch/roll/yaw and
    elbow flexion.  Targets are shoulder-relative, so the result is invariant
    to camera placement, pelvis translation, and the fixed-stance controller.

    The previous accepted arm pose is the next frame's initial condition.
    That makes the branch choice causal and deterministic instead of relying
    on an under-constrained whole-body optimizer to rediscover it each frame.
    """

    def __init__(
        self,
        retargeter,
        *,
        iterations: int = 18,
        damping: float = 2.0e-2,
        max_step_rad: float = 0.30,
        max_frame_delta_rad: float = 0.18,
        continuity_weight: float = 4.0e-2,
        straightness_weight: float = 8.0e-2,
        straightness_threshold_deg: float = 18.0,
        elbow_weight: float = 1.0,
        wrist_weight: float = 1.35,
        direction_weight_m_per_rad: float = 2.5e-2,
        candidate_continuity_m_per_rad: float = 3.0e-3,
        candidate_switch_margin_m: float = 7.5e-4,
        straightness_candidate_m_per_rad: float = 4.0e-2,
    ) -> None:
        self.retargeter = retargeter
        self.model = retargeter.model
        self.data = retargeter.configuration.data
        self.iterations = max(1, int(iterations))
        self.damping = float(max(1.0e-6, damping))
        self.max_step_rad = float(max(1.0e-3, max_step_rad))
        self.max_frame_delta_rad = float(max(1.0e-3, max_frame_delta_rad))
        self.continuity_weight = float(max(0.0, continuity_weight))
        self.straightness_weight = float(max(0.0, straightness_weight))
        self.straightness_threshold_deg = float(
            max(1.0, straightness_threshold_deg)
        )
        # Candidate selection deliberately uses metre-equivalent penalties.
        # A Cartesian-only score can prefer a mirrored shoulder branch for a
        # millimetre endpoint improvement even though the limb directions are
        # tens of degrees wrong.  Direction and continuity therefore take
        # part in branch selection, while the DLS solve itself stays the same.
        self.direction_weight_m_per_rad = float(
            max(0.0, direction_weight_m_per_rad)
        )
        self.candidate_continuity_m_per_rad = float(
            max(0.0, candidate_continuity_m_per_rad)
        )
        self.candidate_switch_margin_m = float(
            max(0.0, candidate_switch_margin_m)
        )
        self.straightness_candidate_m_per_rad = float(
            max(0.0, straightness_candidate_m_per_rad)
        )
        self.weights = np.repeat(
            np.asarray([elbow_weight, wrist_weight], dtype=np.float64), 3
        )
        self.chains: dict[str, dict[str, object]] = {}
        self.previous: dict[str, np.ndarray] = {}
        self.last_error_m: dict[str, float] = {"left": 0.0, "right": 0.0}
        self.last_gmr_error_m: dict[str, float] = {
            "left": 0.0, "right": 0.0,
        }
        self.last_used_baseline: dict[str, bool] = {
            "left": False, "right": False,
        }
        self.last_straightness_blend: dict[str, float] = {
            "left": 0.0, "right": 0.0,
        }
        self.last_candidate_count: dict[str, int] = {"left": 1, "right": 1}
        self.last_selected_source: dict[str, str] = {
            "left": "UNINITIALIZED", "right": "UNINITIALIZED",
        }
        self.last_direction_error_deg: dict[str, dict[str, float]] = {
            "left": {"upper": 0.0, "forearm": 0.0},
            "right": {"upper": 0.0, "forearm": 0.0},
        }
        self.last_candidate_objective_m: dict[str, float] = {
            "left": 0.0, "right": 0.0,
        }

        for side in ("left", "right"):
            joint_names = (
                f"{side}_shoulder_pitch_joint",
                f"{side}_shoulder_roll_joint",
                f"{side}_shoulder_yaw_joint",
                f"{side}_elbow_joint",
            )
            joint_ids = np.asarray(
                [
                    mujoco.mj_name2id(
                        self.model, mujoco.mjtObj.mjOBJ_JOINT, name
                    )
                    for name in joint_names
                ],
                dtype=np.int32,
            )
            if np.any(joint_ids < 0):
                missing = [
                    name for name, joint_id in zip(joint_names, joint_ids)
                    if joint_id < 0
                ]
                raise KeyError(f"G1 arm IK joints missing: {missing}")
            body_names = (
                f"{side}_shoulder_pitch_link",
                f"{side}_elbow_link",
                f"{side}_wrist_roll_rubber_hand",
            )
            body_ids = np.asarray(
                [
                    mujoco.mj_name2id(
                        self.model, mujoco.mjtObj.mjOBJ_BODY, name
                    )
                    for name in body_names
                ],
                dtype=np.int32,
            )
            if np.any(body_ids < 0):
                missing = [
                    name for name, body_id in zip(body_names, body_ids)
                    if body_id < 0
                ]
                raise KeyError(f"G1 arm IK bodies missing: {missing}")
            self.chains[side] = {
                "joint_ids": joint_ids,
                "qpos_ids": np.asarray(
                    [self.model.jnt_qposadr[joint_id] for joint_id in joint_ids],
                    dtype=np.int32,
                ),
                "dof_ids": np.asarray(
                    [self.model.jnt_dofadr[joint_id] for joint_id in joint_ids],
                    dtype=np.int32,
                ),
                "body_ids": body_ids,
            }
            # The official G1 elbow joint has non-collinear link offsets:
            # zero radians is visibly bent and maximum reach is near 1.4 rad.
            # Derive that value from the loaded model instead of assuming a
            # conventional zero-angle straight elbow.
            elbow_joint = int(joint_ids[-1])
            low, high = self.model.jnt_range[elbow_joint]
            samples = np.linspace(float(low), float(high), 257)
            reach = []
            saved_qpos = self.data.qpos.copy()
            for value in samples:
                self.data.qpos[int(self.model.jnt_qposadr[elbow_joint])] = value
                mujoco.mj_forward(self.model, self.data)
                reach.append(float(np.linalg.norm(
                    self.data.xpos[int(body_ids[-1])]
                    - self.data.xpos[int(body_ids[0])]
                )))
            self.data.qpos[:] = saved_qpos
            mujoco.mj_forward(self.model, self.data)
            self.chains[side]["straight_elbow_rad"] = float(
                samples[int(np.argmax(reach))]
            )

    def reset(self, qpos: np.ndarray | None = None) -> None:
        self.previous.clear()
        self.last_error_m = {"left": 0.0, "right": 0.0}
        self.last_gmr_error_m = {"left": 0.0, "right": 0.0}
        self.last_used_baseline = {"left": False, "right": False}
        self.last_straightness_blend = {"left": 0.0, "right": 0.0}
        self.last_candidate_count = {"left": 1, "right": 1}
        self.last_selected_source = {
            "left": "UNINITIALIZED", "right": "UNINITIALIZED",
        }
        self.last_direction_error_deg = {
            "left": {"upper": 0.0, "forearm": 0.0},
            "right": {"upper": 0.0, "forearm": 0.0},
        }
        self.last_candidate_objective_m = {"left": 0.0, "right": 0.0}
        if qpos is None:
            return
        values = np.asarray(qpos, dtype=np.float64)
        for side, chain in self.chains.items():
            self.previous[side] = values[chain["qpos_ids"]].copy()

    def _relative_jacobian(self, body_id: int, root_id: int) -> np.ndarray:
        jac_body = np.zeros((3, self.model.nv), dtype=np.float64)
        jac_root = np.zeros((3, self.model.nv), dtype=np.float64)
        mujoco.mj_jacBody(self.model, self.data, jac_body, None, body_id)
        mujoco.mj_jacBody(self.model, self.data, jac_root, None, root_id)
        return jac_body - jac_root

    @staticmethod
    def _direction_error_rad(first: np.ndarray, second: np.ndarray) -> float:
        """Stable unsigned angle for two limb vectors."""
        first_norm = float(np.linalg.norm(first))
        second_norm = float(np.linalg.norm(second))
        if first_norm < 1.0e-8 or second_norm < 1.0e-8:
            return float(np.pi)
        first_unit = first / first_norm
        second_unit = second / second_norm
        return float(np.arctan2(
            np.linalg.norm(np.cross(first_unit, second_unit)),
            np.clip(np.dot(first_unit, second_unit), -1.0, 1.0),
        ))

    def _chain_metrics(
        self,
        chain: Mapping[str, object],
        target: np.ndarray,
        arm_q: np.ndarray,
    ) -> tuple[float, float, float]:
        """Return Cartesian RMS plus upper/forearm direction errors."""
        qpos_ids = chain["qpos_ids"]
        shoulder_body, elbow_body, wrist_body = chain["body_ids"]
        self.data.qpos[qpos_ids] = arm_q
        mujoco.mj_forward(self.model, self.data)
        shoulder_position = self.data.xpos[shoulder_body]
        achieved_elbow = self.data.xpos[elbow_body] - shoulder_position
        achieved_wrist = self.data.xpos[wrist_body] - shoulder_position
        target_elbow = target[:3]
        target_wrist = target[3:]
        achieved = np.concatenate((achieved_elbow, achieved_wrist))
        position_rms = float(np.sqrt(np.mean((target - achieved) ** 2)))
        upper_error = self._direction_error_rad(target_elbow, achieved_elbow)
        forearm_error = self._direction_error_rad(
            target_wrist - target_elbow,
            achieved_wrist - achieved_elbow,
        )
        return position_rms, upper_error, forearm_error

    def _candidate_objective(
        self,
        chain: Mapping[str, object],
        target: np.ndarray,
        arm_q: np.ndarray,
        continuity_reference: np.ndarray,
        straightness_blend: float = 0.0,
    ) -> tuple[float, float, float, float]:
        position_rms, upper_error, forearm_error = self._chain_metrics(
            chain, target, arm_q
        )
        direction_error = 0.45 * upper_error + 0.55 * forearm_error
        objective = (
            position_rms
            + self.direction_weight_m_per_rad * direction_error
            + self.candidate_continuity_m_per_rad
            * float(np.linalg.norm(arm_q - continuity_reference))
            + self.straightness_candidate_m_per_rad
            * float(np.clip(straightness_blend, 0.0, 1.0))
            * abs(float(arm_q[3]) - float(chain["straight_elbow_rad"]))
        )
        return float(objective), position_rms, upper_error, forearm_error

    def _solve_chain_candidate(
        self,
        chain: Mapping[str, object],
        target: np.ndarray,
        start: np.ndarray,
        continuity_reference: np.ndarray,
        straightness_blend: float,
    ) -> tuple[np.ndarray, float]:
        """Solve one deterministic arm branch and return its Cartesian RMS."""
        qpos_ids = chain["qpos_ids"]
        dof_ids = chain["dof_ids"]
        joint_ids = chain["joint_ids"]
        shoulder_body, elbow_body, wrist_body = chain["body_ids"]
        self.data.qpos[qpos_ids] = start
        for _ in range(self.iterations):
            mujoco.mj_forward(self.model, self.data)
            shoulder_position = self.data.xpos[shoulder_body]
            current = np.concatenate((
                self.data.xpos[elbow_body] - shoulder_position,
                self.data.xpos[wrist_body] - shoulder_position,
            ))
            residual = (target - current) * self.weights
            if float(np.linalg.norm(residual)) < 5.0e-4:
                break
            jacobian = np.vstack((
                self._relative_jacobian(
                    elbow_body, shoulder_body
                )[:, dof_ids],
                self._relative_jacobian(
                    wrist_body, shoulder_body
                )[:, dof_ids],
            ))
            jacobian *= self.weights[:, None]
            normal = jacobian.T @ jacobian
            normal.flat[:: normal.shape[0] + 1] += self.damping**2
            rhs = jacobian.T @ residual
            if straightness_blend > 0.0 and self.straightness_weight > 0.0:
                cost = (self.straightness_weight * straightness_blend) ** 2
                normal[3, 3] += cost
                rhs[3] += cost * (
                    float(chain["straight_elbow_rad"])
                    - float(self.data.qpos[qpos_ids[3]])
                )
            if self.continuity_weight > 0.0:
                cost = self.continuity_weight**2
                normal.flat[:: normal.shape[0] + 1] += cost
                rhs += cost * (
                    continuity_reference - self.data.qpos[qpos_ids]
                )
            try:
                step = np.linalg.solve(normal, rhs)
            except np.linalg.LinAlgError:
                step = np.linalg.lstsq(normal, rhs, rcond=None)[0]
            step_norm = float(np.linalg.norm(step))
            if step_norm > self.max_step_rad:
                step *= self.max_step_rad / step_norm
            candidate = self.data.qpos[qpos_ids] + step
            for index, joint_id in enumerate(joint_ids):
                if bool(self.model.jnt_limited[joint_id]):
                    low, high = self.model.jnt_range[joint_id]
                    candidate[index] = np.clip(candidate[index], low, high)
            self.data.qpos[qpos_ids] = candidate
        mujoco.mj_forward(self.model, self.data)
        shoulder_position = self.data.xpos[shoulder_body]
        achieved = np.concatenate((
            self.data.xpos[elbow_body] - shoulder_position,
            self.data.xpos[wrist_body] - shoulder_position,
        ))
        return (
            self.data.qpos[qpos_ids].copy(),
            float(np.sqrt(np.mean((target - achieved) ** 2))),
        )

    def update(
        self,
        qpos: np.ndarray,
        human_data: Mapping[str, tuple[np.ndarray, np.ndarray]],
    ) -> np.ndarray:
        result = np.asarray(qpos, dtype=np.float64).copy()
        self.data.qpos[:] = result

        for side, chain in self.chains.items():
            names = (
                f"{side}_shoulder", f"{side}_elbow", f"{side}_wrist"
            )
            if not all(name in human_data for name in names):
                continue
            shoulder, elbow, wrist = (
                np.asarray(human_data[name][0], dtype=np.float64)
                for name in names
            )
            if not np.isfinite(np.concatenate((shoulder, elbow, wrist))).all():
                continue
            target = np.concatenate((elbow - shoulder, wrist - shoulder))
            upper_vector = elbow - shoulder
            fore_vector = wrist - elbow
            upper_direction = upper_vector / max(
                float(np.linalg.norm(upper_vector)), 1.0e-9
            )
            fore_direction = fore_vector / max(
                float(np.linalg.norm(fore_vector)), 1.0e-9
            )
            arm_turn_deg = float(np.degrees(np.arccos(np.clip(
                np.dot(upper_direction, fore_direction), -1.0, 1.0
            ))))
            # BODY_38 and the workspace mapper retain a small bend even when
            # the operator is visually straight.  Apply a gradual preference
            # only inside this near-straight deadband; ordinary bent-arm IK is
            # unchanged.  The preference is soft, so wrist/elbow tracking can
            # still override it when the measurement is genuinely different.
            straightness_blend = float(np.clip(
                (self.straightness_threshold_deg - arm_turn_deg)
                / max(self.straightness_threshold_deg - 8.0, 1.0),
                0.0, 1.0,
            ))
            self.last_straightness_blend[side] = straightness_blend
            hanging_arm = bool(
                upper_direction[2] < -0.55
                and fore_direction[2] < -0.45
                and wrist[2] < shoulder[2] - 0.16
            )
            qpos_ids = chain["qpos_ids"]
            shoulder_body, elbow_body, wrist_body = chain["body_ids"]

            # Neutral G1 is a stable first-frame branch (arms down). Later
            # frames continue from the preceding direct-IK result.
            continuity_reference = self.previous.get(side)
            if continuity_reference is None:
                continuity_reference = np.zeros(len(qpos_ids), dtype=np.float64)
            # A hanging arm is singular around its long axis. Always solve
            # that case from the official neutral branch, then rate-limit the
            # command from the previous pose below. This lets a bad raised-arm
            # branch recover instead of preserving it forever.
            solve_reference = (
                np.zeros(len(qpos_ids), dtype=np.float64)
                if hanging_arm
                else continuity_reference
            )
            starts: list[tuple[str, np.ndarray]] = [(
                "DIRECT_NEUTRAL" if hanging_arm else "DIRECT_PREVIOUS",
                solve_reference,
            )]
            gmr_reference = result[qpos_ids].copy()
            gmr_metrics = self._chain_metrics(chain, target, gmr_reference)
            previous_metrics = self._chain_metrics(
                chain, target, continuity_reference,
            )
            gmr_error = gmr_metrics[0]
            previous_error = previous_metrics[0]
            self.last_gmr_error_m[side] = gmr_error
            if (
                not hanging_arm
                and float(np.linalg.norm(gmr_reference - solve_reference)) > 0.05
            ):
                # Lateral and overhead poses can be just as branch-ambiguous
                # as a hand in front of the torso.  Always evaluate GMR's
                # whole-body proposal when it represents a distinct start.
                starts.append(("DIRECT_GMR_START", gmr_reference))
            best_baseline_direction = min(
                0.45 * gmr_metrics[1] + 0.55 * gmr_metrics[2],
                0.45 * previous_metrics[1] + 0.55 * previous_metrics[2],
            )
            neutral = np.zeros(len(qpos_ids), dtype=np.float64)
            if (
                not hanging_arm
                and (
                    min(gmr_error, previous_error) > 0.040
                    or best_baseline_direction > np.radians(22.0)
                )
                and all(
                    float(np.linalg.norm(neutral - start)) > 0.05
                    for _, start in starts
                )
            ):
                # Recovery-only neutral seed. It is not evaluated in normal
                # well-tracked motion, so live latency stays low; when both
                # causal and GMR seeds occupy a bad branch it gives DLS a
                # deterministic route back to the anatomical arm solution.
                starts.append(("DIRECT_RECOVERY_NEUTRAL", neutral))
            candidates = []
            for source, start in starts:
                candidate, _ = self._solve_chain_candidate(
                    chain, target, start, continuity_reference,
                    straightness_blend,
                )
                candidates.append((source, candidate))
            self.last_candidate_count[side] = len(candidates)
            # GMR and the preceding accepted pose are protected baselines.
            # Select using endpoint, limb-direction and temporal continuity
            # together; position RMS alone is insufficient near singular or
            # mirrored arm branches.
            protected_candidates = candidates + [
                ("GMR_BASELINE", gmr_reference),
                ("PREVIOUS_HOLD", continuity_reference.copy()),
            ]
            scored_candidates = [
                (
                    source,
                    candidate,
                    *self._candidate_objective(
                        chain, target, candidate, continuity_reference,
                        straightness_blend,
                    ),
                )
                for source, candidate in protected_candidates
            ]
            best = min(scored_candidates, key=lambda item: item[2])
            previous_choice = next(
                item for item in scored_candidates if item[0] == "PREVIOUS_HOLD"
            )
            # A sub-millimetre objective gain is normally measurement noise,
            # not a reason to change shoulder branch while the human is still.
            previous_is_well_tracked = bool(
                previous_choice[3] < 0.020
                and previous_choice[4] < np.radians(12.0)
                and previous_choice[5] < np.radians(15.0)
            )
            if (
                best[0] != "PREVIOUS_HOLD"
                and previous_is_well_tracked
                and best[2] > previous_choice[2] - self.candidate_switch_margin_m
            ):
                best = previous_choice
            selected_source, solved_candidate = best[0], best[1]
            self.last_selected_source[side] = selected_source
            self.last_candidate_objective_m[side] = float(best[2])
            self.last_used_baseline[side] = selected_source in {
                "GMR_BASELINE", "PREVIOUS_HOLD",
            }
            self.data.qpos[qpos_ids] = solved_candidate

            frame_delta = self.data.qpos[qpos_ids] - continuity_reference
            frame_delta_norm = float(np.linalg.norm(frame_delta))
            if frame_delta_norm > self.max_frame_delta_rad:
                self.data.qpos[qpos_ids] = continuity_reference + frame_delta * (
                    self.max_frame_delta_rad / frame_delta_norm
                )

            mujoco.mj_forward(self.model, self.data)
            shoulder_position = self.data.xpos[shoulder_body]
            achieved = np.concatenate((
                self.data.xpos[elbow_body] - shoulder_position,
                self.data.xpos[wrist_body] - shoulder_position,
            ))
            self.last_error_m[side] = float(
                np.sqrt(np.mean((target - achieved) ** 2))
            )
            _, upper_error, forearm_error = self._chain_metrics(
                chain, target, self.data.qpos[qpos_ids].copy()
            )
            self.last_direction_error_deg[side] = {
                "upper": float(np.degrees(upper_error)),
                "forearm": float(np.degrees(forearm_error)),
            }
            solved = self.data.qpos[qpos_ids].copy()
            self.previous[side] = solved
            result[qpos_ids] = solved

        self.data.qpos[:] = result
        mujoco.mj_forward(self.model, self.data)
        return result


class SimpleArmPositionIK(ArmPositionIKRefiner):
    """Single warm-start, limb-direction IK through two Cartesian points.

    BODY_38 human arm lengths must not be sent to the shorter G1 arm as raw
    elbow/wrist positions.  Doing that makes the target unreachable and lets
    the least-squares solve escape through a shoulder joint-limit branch.  We
    instead retain only the observed upper-arm and forearm directions, rebuild
    a reachable target with the link lengths measured from the loaded G1
    model, and run one causal DLS solve.  There is no mirror candidate, branch
    race or frame-wise global reinitialisation.
    """

    def __init__(
        self,
        retargeter,
        *,
        iterations: int = 12,
        damping: float = 2.0e-2,
        max_step_rad: float = 0.30,
        max_frame_delta_rad: float = 0.35,
        continuity_weight: float = 4.0e-2,
        straightness_weight: float = 8.0e-2,
        straightness_threshold_deg: float = 18.0,
        anchor_fixed_base: bool = True,
    ) -> None:
        super().__init__(
            retargeter,
            iterations=iterations,
            damping=damping,
            max_step_rad=max_step_rad,
            max_frame_delta_rad=max_frame_delta_rad,
            continuity_weight=continuity_weight,
            straightness_weight=straightness_weight,
            straightness_threshold_deg=straightness_threshold_deg,
        )
        self.segment_lengths: dict[str, tuple[float, float]] = {}
        self.free_joint_qpos_addresses = [
            int(self.model.jnt_qposadr[joint_id])
            for joint_id in range(self.model.njnt)
            if self.model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE
        ]
        self.anchor_fixed_base = bool(anchor_fixed_base)
        saved_qpos = self.data.qpos.copy()
        for side, chain in self.chains.items():
            shoulder_body, elbow_body, wrist_body = chain["body_ids"]
            mujoco.mj_forward(self.model, self.data)
            upper_length = float(np.linalg.norm(
                self.data.xpos[elbow_body] - self.data.xpos[shoulder_body]
            ))
            forearm_length = float(np.linalg.norm(
                self.data.xpos[wrist_body] - self.data.xpos[elbow_body]
            ))
            self.segment_lengths[side] = (upper_length, forearm_length)
        self.data.qpos[:] = saved_qpos
        mujoco.mj_forward(self.model, self.data)

    def update(
        self,
        qpos: np.ndarray,
        human_data: Mapping[str, tuple[np.ndarray, np.ndarray]],
    ) -> np.ndarray:
        result = np.asarray(qpos, dtype=np.float64).copy()
        # GMR optimizes a floating root, while the upper-body Isaac mode uses
        # a fixed base and transmits only the 23 actuated joints.  Solving the
        # arms under GMR's yawed root and then discarding that root rotates the
        # resulting motor pose a second time in Isaac (the recorded root was
        # about 147 degrees away from identity).  Anchor only the arm-FK copy
        # to the exact fixed-base frame used by Isaac before solving.
        if self.anchor_fixed_base:
            for address in self.free_joint_qpos_addresses:
                result[address + 3:address + 7] = (1.0, 0.0, 0.0, 0.0)
        self.data.qpos[:] = result
        for side, chain in self.chains.items():
            names = (
                f"{side}_shoulder", f"{side}_elbow", f"{side}_wrist"
            )
            if not all(name in human_data for name in names):
                continue
            shoulder, elbow, wrist = (
                np.asarray(human_data[name][0], dtype=np.float64)
                for name in names
            )
            if not np.isfinite(np.concatenate((shoulder, elbow, wrist))).all():
                continue
            upper = elbow - shoulder
            forearm = wrist - elbow
            upper_norm = float(np.linalg.norm(upper))
            forearm_norm = float(np.linalg.norm(forearm))
            if upper_norm < 1.0e-8 or forearm_norm < 1.0e-8:
                continue
            upper_direction = upper / upper_norm
            forearm_direction = forearm / forearm_norm
            upper_length, forearm_length = self.segment_lengths[side]
            target_elbow = upper_direction * upper_length
            target = np.concatenate((
                target_elbow,
                target_elbow + forearm_direction * forearm_length,
            ))
            turn = self._direction_error_rad(upper, forearm)
            straightness = float(np.clip(
                (np.radians(self.straightness_threshold_deg) - turn)
                / np.radians(10.0), 0.0, 1.0
            ))
            hanging_arm = bool(
                upper_direction[2] < -0.55
                and forearm_direction[2] < -0.45
                and wrist[2] < shoulder[2] - 0.16
            )
            self.last_straightness_blend[side] = straightness
            qpos_ids = chain["qpos_ids"]
            previous = self.previous.get(side)
            if previous is None:
                previous = np.zeros(len(qpos_ids), dtype=np.float64)
            gmr_reference = result[qpos_ids].copy()
            self.last_gmr_error_m[side] = self._chain_metrics(
                chain, target, gmr_reference
            )[0]
            # Preserve both causal baselines.  The whole-body GMR proposal is
            # often already the best reachable arm solution; the local
            # refiner must never replace a 2 cm GMR fit with an 8 cm local fit
            # merely because only the previous seed was offered.  A second
            # DLS solve from the GMR proposal is still deterministic and keeps
            # the implementation branch-free.
            candidates: list[tuple[str, np.ndarray]] = [
                ("PREVIOUS_HOLD", previous.copy()),
                ("GMR_BASELINE", gmr_reference.copy()),
            ]
            starts = [("PREVIOUS_SEED", previous)]
            if float(np.linalg.norm(gmr_reference - previous)) > 1.0e-3:
                starts.append(("GMR_SEED", gmr_reference))
            neutral = np.zeros(len(qpos_ids), dtype=np.float64)
            if hanging_arm and all(
                float(np.linalg.norm(start - neutral)) > 1.0e-3
                for _, start in starts
            ):
                # A vertical straight arm is singular around its long axis.
                # One neutral seed gives a deterministic escape from a stale
                # raised-arm shoulder branch without introducing mirrored or
                # frame-wise global candidates.
                starts.append(("HANGING_NEUTRAL_SEED", neutral))
            for source, start in starts:
                solved, _ = self._solve_chain_candidate(
                    chain, target, start, previous, straightness
                )
                candidates.append((source, solved))

            def score(candidate: np.ndarray) -> float:
                objective, _, _, _ = self._candidate_objective(
                    chain, target, candidate, previous, straightness
                )
                return objective

            source, proposed = min(
                candidates, key=lambda item: score(item[1])
            )
            if straightness >= 0.5:
                # At BODY_38 elbow angles of roughly 168 degrees and above,
                # the measured elbow plane is close to singular but the
                # flexion itself is unambiguous: the arm is straight. Make
                # that scalar constraint explicit after branch selection.
                # Shoulder DoFs remain the IK result and the existing full
                # arm frame-delta limit below keeps recovery continuous.
                straight_elbow = float(chain["straight_elbow_rad"])
                proposed = proposed.copy()
                proposed[3] = (
                    (1.0 - straightness) * proposed[3]
                    + straightness * straight_elbow
                )
            delta = proposed - previous
            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > self.max_frame_delta_rad:
                proposed = previous + delta * (
                    self.max_frame_delta_rad / delta_norm
                )
            self.data.qpos[qpos_ids] = proposed
            position_rms, upper_error, forearm_error = self._chain_metrics(
                chain, target, proposed
            )
            self.last_error_m[side] = position_rms
            self.last_used_baseline[side] = source in {
                "PREVIOUS_HOLD", "GMR_BASELINE",
            }
            self.last_candidate_count[side] = len(candidates)
            self.last_selected_source[side] = source
            self.last_direction_error_deg[side] = {
                "upper": float(np.degrees(upper_error)),
                "forearm": float(np.degrees(forearm_error)),
            }
            self.last_candidate_objective_m[side] = position_rms
            self.previous[side] = proposed.copy()
            result[qpos_ids] = proposed
        self.data.qpos[:] = result
        mujoco.mj_forward(self.model, self.data)
        return result


class SimpleArmDirectionIK(ArmPositionIKRefiner):
    """Causal arm IK driven only by BODY_38 limb directions.

    Human and G1 link lengths differ, so elbow/wrist XYZ is not a stable
    objective for upper-body imitation.  This solver tracks the normalized
    shoulder->elbow and elbow->wrist vectors on the official G1 chain.  It has
    one warm start (the preceding accepted pose), one DLS solve and one rate
    limit; there is no branch competition, rescue candidate or pose averaging.
    """

    def __init__(
        self,
        retargeter,
        *,
        iterations: int = 12,
        damping: float = 3.5e-2,
        max_step_rad: float = 0.24,
        max_frame_delta_rad: float = 0.22,
        continuity_weight: float = 1.5e-2,
        upper_weight: float = 1.0,
        forearm_weight: float = 1.15,
    ) -> None:
        super().__init__(
            retargeter,
            iterations=iterations,
            damping=damping,
            max_step_rad=max_step_rad,
            max_frame_delta_rad=max_frame_delta_rad,
            continuity_weight=continuity_weight,
            straightness_weight=0.0,
        )
        self.direction_weights = np.repeat(
            np.asarray([upper_weight, forearm_weight], dtype=np.float64), 3
        )

    @staticmethod
    def _unit_with_jacobian(
        vector: np.ndarray, jacobian: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray] | None:
        norm = float(np.linalg.norm(vector))
        if norm < 1.0e-8:
            return None
        direction = vector / norm
        projector = np.eye(3) - np.outer(direction, direction)
        return direction, (projector @ jacobian) / norm

    def update(
        self,
        qpos: np.ndarray,
        human_data: Mapping[str, tuple[np.ndarray, np.ndarray]],
    ) -> np.ndarray:
        result = np.asarray(qpos, dtype=np.float64).copy()
        self.data.qpos[:] = result

        for side, chain in self.chains.items():
            names = (
                f"{side}_shoulder", f"{side}_elbow", f"{side}_wrist"
            )
            if not all(name in human_data for name in names):
                continue
            shoulder, elbow, wrist = (
                np.asarray(human_data[name][0], dtype=np.float64)
                for name in names
            )
            if not np.isfinite(np.concatenate((shoulder, elbow, wrist))).all():
                continue
            target_upper_vector = elbow - shoulder
            target_forearm_vector = wrist - elbow
            upper_norm = float(np.linalg.norm(target_upper_vector))
            forearm_norm = float(np.linalg.norm(target_forearm_vector))
            if upper_norm < 1.0e-8 or forearm_norm < 1.0e-8:
                continue
            target_upper = target_upper_vector / upper_norm
            target_forearm = target_forearm_vector / forearm_norm
            target_position = np.concatenate((
                target_upper * upper_norm,
                target_upper * upper_norm + target_forearm * forearm_norm,
            ))

            qpos_ids = chain["qpos_ids"]
            dof_ids = chain["dof_ids"]
            joint_ids = chain["joint_ids"]
            shoulder_body, elbow_body, wrist_body = chain["body_ids"]
            previous = self.previous.get(side)
            if previous is None:
                previous = np.zeros(len(qpos_ids), dtype=np.float64)
            gmr_reference = result[qpos_ids].copy()
            self.last_gmr_error_m[side] = self._chain_metrics(
                chain, target_position, gmr_reference
            )[0]
            self.data.qpos[qpos_ids] = previous

            for _ in range(self.iterations):
                mujoco.mj_forward(self.model, self.data)
                shoulder_position = self.data.xpos[shoulder_body]
                elbow_position = self.data.xpos[elbow_body]
                wrist_position = self.data.xpos[wrist_body]
                upper_vector = elbow_position - shoulder_position
                forearm_vector = wrist_position - elbow_position
                upper_jacobian = self._relative_jacobian(
                    elbow_body, shoulder_body
                )[:, dof_ids]
                forearm_jacobian = (
                    self._relative_jacobian(wrist_body, shoulder_body)
                    - self._relative_jacobian(elbow_body, shoulder_body)
                )[:, dof_ids]
                upper_state = self._unit_with_jacobian(
                    upper_vector, upper_jacobian
                )
                forearm_state = self._unit_with_jacobian(
                    forearm_vector, forearm_jacobian
                )
                if upper_state is None or forearm_state is None:
                    break
                current = np.concatenate((upper_state[0], forearm_state[0]))
                desired = np.concatenate((target_upper, target_forearm))
                residual = (desired - current) * self.direction_weights
                if float(np.linalg.norm(residual)) < 2.0e-3:
                    break
                jacobian = np.vstack((upper_state[1], forearm_state[1]))
                jacobian *= self.direction_weights[:, None]
                normal = jacobian.T @ jacobian
                normal.flat[:: normal.shape[0] + 1] += self.damping**2
                rhs = jacobian.T @ residual
                if self.continuity_weight > 0.0:
                    cost = self.continuity_weight**2
                    normal.flat[:: normal.shape[0] + 1] += cost
                    rhs += cost * (previous - self.data.qpos[qpos_ids])
                try:
                    step = np.linalg.solve(normal, rhs)
                except np.linalg.LinAlgError:
                    step = np.linalg.lstsq(normal, rhs, rcond=None)[0]
                step_norm = float(np.linalg.norm(step))
                if step_norm > self.max_step_rad:
                    step *= self.max_step_rad / step_norm
                candidate = self.data.qpos[qpos_ids] + step
                for index, joint_id in enumerate(joint_ids):
                    if bool(self.model.jnt_limited[joint_id]):
                        low, high = self.model.jnt_range[joint_id]
                        candidate[index] = np.clip(candidate[index], low, high)
                self.data.qpos[qpos_ids] = candidate

            proposed = self.data.qpos[qpos_ids].copy()
            delta = proposed - previous
            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > self.max_frame_delta_rad:
                proposed = previous + delta * (
                    self.max_frame_delta_rad / delta_norm
                )
            self.data.qpos[qpos_ids] = proposed
            position_rms, upper_error, forearm_error = self._chain_metrics(
                chain, target_position, proposed
            )
            self.last_error_m[side] = position_rms
            self.last_used_baseline[side] = False
            self.last_candidate_count[side] = 1
            self.last_selected_source[side] = "DIRECTION_WARM_START"
            self.last_direction_error_deg[side] = {
                "upper": float(np.degrees(upper_error)),
                "forearm": float(np.degrees(forearm_error)),
            }
            self.last_candidate_objective_m[side] = float(
                0.025 * (0.45 * upper_error + 0.55 * forearm_error)
            )
            arm_turn = self._direction_error_rad(
                target_upper_vector, target_forearm_vector
            )
            self.last_straightness_blend[side] = float(np.clip(
                (np.radians(18.0) - arm_turn) / np.radians(10.0), 0.0, 1.0
            ))
            self.previous[side] = proposed.copy()
            result[qpos_ids] = proposed

        self.data.qpos[:] = result
        mujoco.mj_forward(self.model, self.data)
        return result

def official_g1_23dof_xml() -> Path:
    """Locate Unitree's unmodified official 23-DOF MuJoCo model.

    The first path is the dedicated official ``unitree_ros`` checkout used by
    the Isaac project.  The second is the official ``unitree_mujoco`` model in
    the ROS workspace.  Keeping this lookup here prevents GMR from silently
    falling back to its 29-DOF mocap model, whose extra wrist/waist links do
    not exist in the Isaac 23-DOF articulation.
    """
    candidates = (
        Path.home()
        / "g1_isaaclab_project/repos/unitree_ros/robots/g1_description/g1_23dof.xml",
        Path.home()
        / "ros2_ws/src/unitree_mujoco/unitree_robots/g1/g1_23dof.xml",
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(
        "Official Unitree G1 23-DOF XML was not found; expected one of: "
        + ", ".join(str(path) for path in candidates)
    )


def constrain_gmr_to_23dof(
    retargeter,
    epsilon: float = 1.0e-6,
    *,
    anatomical_elbows: bool = True,
    lock_waist_yaw: bool = False,
    restrict_backward_arms: bool = False,
    command_margin_rad: float = 0.041,
) -> None:
    """Lock joints absent from the official G1 23-DOF model during IK.

    Projecting a free 29-DOF solution after IK changes the achieved wrist and
    torso poses.  Constraining those six coordinates before solving makes GMR
    optimize the kinematics that Isaac and the physical 23-DOF G1 actually
    possess.
    """
    for name in LOCKED_G1_29DOF_JOINTS:
        joint_id = mujoco.mj_name2id(
            retargeter.model, mujoco.mjtObj.mjOBJ_JOINT, name
        )
        # The exact official 23-DOF model has no such joint, which is already
        # the strongest possible constraint.  This branch is retained only
        # for compatibility with old 29-DOF offline artifacts.
        if joint_id < 0:
            continue
        qpos_address = int(retargeter.model.jnt_qposadr[joint_id])
        retargeter.configuration.data.qpos[qpos_address] = 0.0
        retargeter.model.jnt_limited[joint_id] = True
        retargeter.model.jnt_range[joint_id] = (-float(epsilon), float(epsilon))
    if anatomical_elbows:
        # The official G1 elbow coordinate is offset from the anatomical
        # flexion angle: maximum reach is around +1.3..+1.4 rad, while normal
        # deep flexion uses negative motor angles.  The old lower bound of
        # zero removed roughly half of the useful human elbow workspace and
        # forced the shoulders into a compensating/crossed pose.  Preserve
        # the hardware lower bound and clip only the post-straight branch that
        # would represent anatomical hyperextension.
        for name in ("left_elbow_joint", "right_elbow_joint"):
            joint_id = mujoco.mj_name2id(
                retargeter.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise KeyError(f"GMR model is missing elbow joint: {name}")
            retargeter.model.jnt_limited[joint_id] = True
            retargeter.model.jnt_range[joint_id, 1] = min(
                float(retargeter.model.jnt_range[joint_id, 1]), 1.40
            )
    if lock_waist_yaw:
        joint_id = mujoco.mj_name2id(
            retargeter.model, mujoco.mjtObj.mjOBJ_JOINT, "waist_yaw_joint"
        )
        if joint_id < 0:
            raise KeyError("GMR model is missing waist_yaw_joint")
        qpos_address = int(retargeter.model.jnt_qposadr[joint_id])
        retargeter.configuration.data.qpos[qpos_address] = 0.0
        retargeter.model.jnt_limited[joint_id] = True
        retargeter.model.jnt_range[joint_id] = (-float(epsilon), float(epsilon))
    if restrict_backward_arms:
        # Positive shoulder pitch sends the hand behind a frontal operator.
        # Keep a small natural rear reach while excluding the ambiguous
        # arm-behind-torso IK branch produced by single-camera depth noise.
        for name in ("left_shoulder_pitch_joint", "right_shoulder_pitch_joint"):
            joint_id = mujoco.mj_name2id(
                retargeter.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise KeyError(f"GMR model is missing shoulder joint: {name}")
            retargeter.model.jnt_limited[joint_id] = True
            retargeter.model.jnt_range[joint_id, 1] = min(
                float(retargeter.model.jnt_range[joint_id, 1]), 0.75
            )
    # Put the final command boundary's soft margin into IK itself. Otherwise
    # GMR repeatedly selects an exact mechanical limit and the safety layer
    # has to clip/blend every otherwise-valid frame, causing visible lag.
    margin = float(max(0.0, command_margin_rad))
    for name in G1_23DOF_ORDER:
        joint_id = mujoco.mj_name2id(
            retargeter.model, mujoco.mjtObj.mjOBJ_JOINT, name
        )
        if joint_id < 0 or not bool(retargeter.model.jnt_limited[joint_id]):
            continue
        low, high = map(float, retargeter.model.jnt_range[joint_id])
        if high - low > 2.0 * margin:
            retargeter.model.jnt_range[joint_id] = (
                low + margin, high - margin
            )
    # ConfigurationLimit copies the ranges at construction, so rebuild it
    # after narrowing the six joints. Preserve any optional velocity limits.
    retargeter.ik_limits[0] = mink.ConfigurationLimit(retargeter.model, gain=1.0)
    mujoco.mj_forward(retargeter.model, retargeter.configuration.data)


def named_joint_values(
    joint_names: Sequence[str], joint_positions: Sequence[float]
) -> dict[str, float]:
    if len(joint_names) != len(joint_positions):
        raise ValueError("joint name/value lengths differ")
    return {str(name): float(value) for name, value in zip(joint_names, joint_positions)}


def project_29_to_23(values: Mapping[str, float]) -> np.ndarray:
    missing = [name for name in G1_23DOF_ORDER if name not in values]
    if missing:
        raise KeyError(f"GMR output is missing G1 joints: {missing}")
    return np.asarray([values[name] for name in G1_23DOF_ORDER], dtype=np.float64)


def expand_23_to_29(
    values_23: Sequence[float], target_29_order: Sequence[str]
) -> np.ndarray:
    if len(values_23) != len(G1_23DOF_ORDER):
        raise ValueError(f"expected 23 values, got {len(values_23)}")
    source = dict(zip(G1_23DOF_ORDER, map(float, values_23)))
    return np.asarray([source.get(name, 0.0) for name in target_29_order], dtype=np.float64)

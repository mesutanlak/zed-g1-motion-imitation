"""Live ZED BODY_38 -> upstream GMR -> safe G1 23-DOF UDP bridge.

Input:  ``zed_body38_live/v1`` UDP packets from ``zed_g1_skeleton.py``.
Output: ``zed_gmr_g1_23dof_live/v1`` UDP packets for Isaac Lab only.

No Unitree DDS topics are used, so this process cannot address a physical G1.
"""

from __future__ import annotations

import argparse
from collections import deque
import json
import socket
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from body38_to_gmr import Body38ToGMR, GMR_BODY38_MAP
from g1_dof_projection import (
    AnatomicalElbowRegularizer,
    SimpleArmPositionIK,
    G1_23DOF_ORDER,
    constrain_gmr_to_23dof,
    named_joint_values,
    official_g1_23dof_xml,
    project_29_to_23,
)
from motion_pipeline.safety import (
    G1_23_ANATOMICAL_LIMITS_RAD,
    G1_23_LIMITS_RAD,
    G1FeasibilityFilter,
    SafetyLevel,
)
from motion_pipeline.metrics import PacketMetrics
from motion_pipeline.collision_geometry import (
    project_configuration_along_safe_path,
    upper_body_capsule_report,
)
from motion_pipeline.mirror_rescue import (
    KinematicEvaluation,
    MirrorContinuationRescue,
)


MAX_VELOCITY_RAD_S = np.asarray(
    [8.0] * 12 + [6.0] + [12.0] * 10, dtype=np.float64
)

# Joint-group One Euro ranges. Filtering happens after IK in joint space, so
# rigid human bone lengths are not independently distorted in XYZ.
MIN_CUTOFF_HZ_23 = np.asarray(
    [3, 3, 3, 3, 2, 2] * 2 + [3] + [5, 5, 5, 6, 4] * 2,
    dtype=np.float64,
)
MAX_CUTOFF_HZ_23 = np.asarray(
    [6, 6, 6, 6, 5, 5] * 2 + [5] + [8, 8, 8, 10, 7] * 2,
    dtype=np.float64,
)
UPPER_REQUIRED = {
    "pelvis",
    "spine3",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
}

G1_SKELETON_BODIES = (
    "pelvis", "torso_link",
    "left_hip_roll_link", "left_knee_link", "left_ankle_roll_link",
    "right_hip_roll_link", "right_knee_link", "right_ankle_roll_link",
    "left_shoulder_pitch_link", "left_elbow_link", "left_wrist_roll_rubber_hand",
    "right_shoulder_pitch_link", "right_elbow_link", "right_wrist_roll_rubber_hand",
)
G1_SKELETON_EDGES = (
    ("pelvis", "torso_link"),
    ("pelvis", "left_hip_roll_link"), ("left_hip_roll_link", "left_knee_link"),
    ("left_knee_link", "left_ankle_roll_link"),
    ("pelvis", "right_hip_roll_link"), ("right_hip_roll_link", "right_knee_link"),
    ("right_knee_link", "right_ankle_roll_link"),
    ("torso_link", "left_shoulder_pitch_link"),
    ("left_shoulder_pitch_link", "left_elbow_link"),
    ("left_elbow_link", "left_wrist_roll_rubber_hand"),
    ("torso_link", "right_shoulder_pitch_link"),
    ("right_shoulder_pitch_link", "right_elbow_link"),
    ("right_elbow_link", "right_wrist_roll_rubber_hand"),
)
HUMAN_RETARGET_EDGES = (
    ("pelvis", "spine3"),
    ("pelvis", "left_hip"), ("left_hip", "left_knee"),
    ("left_knee", "left_foot"),
    ("pelvis", "right_hip"), ("right_hip", "right_knee"),
    ("right_knee", "right_foot"),
    ("spine3", "left_shoulder"),
    ("left_shoulder", "left_elbow"), ("left_elbow", "left_wrist"),
    ("spine3", "right_shoulder"),
    ("right_shoulder", "right_elbow"), ("right_elbow", "right_wrist"),
)


def pelvis_left_right_swap_risk(frame: dict) -> bool:
    """Check anatomical shoulder order in the operator pelvis frame.

    Historical captures computed this flag in camera coordinates, which marks
    a correctly labelled person facing the camera as swapped. Recompute it at
    the control boundary so old recordings and new live packets behave alike.
    """
    names = [str(name) for name in frame.get("keypoint_names", [])]
    points = (frame.get("pelvis_frame") or {}).get("keypoints_m")
    if points is None:
        return bool((frame.get("perception_metrics") or {}).get("left_right_swap_risk"))
    try:
        left = np.asarray(points[names.index("LEFT_SHOULDER")], dtype=float)
        right = np.asarray(points[names.index("RIGHT_SHOULDER")], dtype=float)
    except (ValueError, IndexError, TypeError):
        return False
    return bool(
        left.shape == (3,)
        and right.shape == (3,)
        and np.isfinite(left).all()
        and np.isfinite(right).all()
        and left[1] < right[1]
    )


class AdaptiveJointFilter:
    """One-Euro-style causal filter for responsive, low-jitter joint targets.

    Slow/noisy motion uses ``min_cutoff_hz``. Real movement raises the cutoff
    toward ``max_cutoff_hz`` based on filtered joint velocity, reducing the
    lag of a fixed low-pass filter. A final per-joint slew limit preserves the
    existing command-boundary safety invariant.
    """

    def __init__(
        self,
        *,
        min_cutoff_hz: float,
        max_cutoff_hz: float,
        velocity_beta: float,
        derivative_cutoff_hz: float,
    ) -> None:
        self.min_cutoff_hz = np.asarray(min_cutoff_hz, dtype=np.float64)
        self.max_cutoff_hz = np.asarray(max_cutoff_hz, dtype=np.float64)
        self.velocity_beta = float(velocity_beta)
        self.derivative_cutoff_hz = float(derivative_cutoff_hz)
        self.value: np.ndarray | None = None
        self.previous_raw: np.ndarray | None = None
        self.velocity: np.ndarray | None = None
        self.last_cutoff_hz: np.ndarray | None = None

    @staticmethod
    def _alpha(cutoff_hz: np.ndarray | float, dt: float) -> np.ndarray:
        return 1.0 - np.exp(-2.0 * np.pi * np.asarray(cutoff_hz) * dt)

    def reset(self) -> None:
        self.value = None
        self.previous_raw = None
        self.velocity = None
        self.last_cutoff_hz = None

    def update(
        self, raw: np.ndarray, dt: float, velocity_limit: np.ndarray
    ) -> np.ndarray:
        raw = np.asarray(raw, dtype=np.float64)
        if self.value is None:
            self.value = raw.copy()
            self.previous_raw = raw.copy()
            self.velocity = np.zeros_like(raw)
            self.last_cutoff_hz = np.broadcast_to(
                self.min_cutoff_hz, raw.shape
            ).astype(np.float64).copy()
            return self.value.copy()

        raw_velocity = (raw - self.previous_raw) / dt
        derivative_alpha = self._alpha(self.derivative_cutoff_hz, dt)
        self.velocity += derivative_alpha * (raw_velocity - self.velocity)
        cutoff = np.clip(
            self.min_cutoff_hz + self.velocity_beta * np.abs(self.velocity),
            self.min_cutoff_hz,
            self.max_cutoff_hz,
        )
        alpha = self._alpha(cutoff, dt)
        candidate = self.value + alpha * (raw - self.value)
        max_delta = np.asarray(velocity_limit, dtype=np.float64) * dt
        self.value += np.clip(candidate - self.value, -max_delta, max_delta)
        self.previous_raw = raw.copy()
        self.last_cutoff_hz = cutoff
        return self.value.copy()


class RelativeHandRoll:
    """Estimate wrist roll from BODY_38 index/pinky geometry.

    BODY_38 does not directly expose a G1 wrist actuator angle.  The first
    coherent second establishes a neutral palm plane; subsequent rotation
    around the observed forearm becomes the relative G1 wrist-roll target.
    """

    def __init__(
        self,
        calibration_frames: int = 12,
        max_speed_rad_s: float = 2.4,
        nominal_fps: float = 60.0,
        invalid_hold_s: float = 0.25,
        neutral_return_tau_s: float = 0.80,
        smoothing_tau_s: float = 0.10,
        tracking_deadband_rad: float = 0.045,
    ) -> None:
        self.calibration_frames = calibration_frames
        self.max_speed_rad_s = float(max_speed_rad_s)
        self.nominal_dt = 1.0 / max(1.0, float(nominal_fps))
        self.samples: dict[str, list[float]] = {"left": [], "right": []}
        self.baseline: dict[str, float] = {}
        self.last: dict[str, float] = {"left": 0.0, "right": 0.0}
        self.unwrapped: dict[str, float] = {}
        self.last_timestamp_ns: dict[str, int] = {}
        self.invalid_elapsed_s: dict[str, float] = {"left": 0.0, "right": 0.0}
        self.invalid_hold_s = float(max(0.0, invalid_hold_s))
        self.neutral_return_tau_s = float(max(0.05, neutral_return_tau_s))
        self.smoothing_tau_s = float(max(0.01, smoothing_tau_s))
        self.tracking_deadband_rad = float(max(0.0, tracking_deadband_rad))
        self.quality: dict[str, str] = {"left": "CALIBRATING", "right": "CALIBRATING"}

    def reset(self) -> None:
        """Start wrist neutral calibration again for a new operator session."""
        self.samples = {"left": [], "right": []}
        self.baseline.clear()
        self.last = {"left": 0.0, "right": 0.0}
        self.unwrapped.clear()
        self.last_timestamp_ns.clear()
        self.invalid_elapsed_s = {"left": 0.0, "right": 0.0}
        self.quality = {"left": "CALIBRATING", "right": "CALIBRATING"}

    @staticmethod
    def _occluded(frame: dict, side: str) -> bool:
        overlap = (frame.get("occlusion_analysis") or {}).get(
            "arm_torso_overlap", {}
        )
        return bool(overlap.get(side))

    def _dt(self, frame: dict, side: str) -> float:
        timestamp_ns = int(frame.get("timestamp_ns") or 0)
        previous_timestamp_ns = self.last_timestamp_ns.get(side)
        if timestamp_ns > 0 and previous_timestamp_ns is not None:
            dt = float(
                np.clip(
                    (timestamp_ns - previous_timestamp_ns) * 1.0e-9,
                    1.0 / 120.0,
                    0.1,
                )
            )
        else:
            dt = self.nominal_dt
        if timestamp_ns > 0:
            self.last_timestamp_ns[side] = timestamp_ns
        return dt

    @staticmethod
    def _angle(frame: dict, side: str) -> float | None:
        names = frame.get("keypoint_names", [])
        # Live packets use ``keypoints_3d_m`` while lossless recordings keep
        # explicit raw/filtered arrays.  Accept all official project schemas;
        # otherwise wrist roll silently remains at zero during replay.
        points = (
            frame.get("keypoints_3d_m")
            or frame.get("keypoints_3d_filtered_m")
            or frame.get("keypoints_3d_raw_m")
            or []
        )
        lookup = {str(name): index for index, name in enumerate(names)}
        prefix = side.upper()
        required = (
            f"{prefix}_SHOULDER",
            f"{prefix}_ELBOW",
            f"{prefix}_WRIST",
            f"{prefix}_HAND_INDEX_1",
            f"{prefix}_HAND_PINKY_1",
            "LEFT_SHOULDER",
            "RIGHT_SHOULDER",
        )
        if any(name not in lookup for name in required):
            return None
        confidence = frame.get("keypoint_confidence") or []
        if confidence:
            observed = (
                f"{prefix}_ELBOW",
                f"{prefix}_WRIST",
                f"{prefix}_HAND_INDEX_1",
                f"{prefix}_HAND_PINKY_1",
            )
            try:
                if min(float(confidence[lookup[name]]) for name in observed) < 45.0:
                    return None
            except (TypeError, ValueError, IndexError):
                return None
        try:
            xyz = {
                name: np.asarray(points[lookup[name]], dtype=np.float64)
                for name in required
            }
        except (TypeError, ValueError, IndexError):
            return None
        if not all(value.shape == (3,) and np.isfinite(value).all() for value in xyz.values()):
            return None

        axis = xyz[f"{prefix}_WRIST"] - xyz[f"{prefix}_ELBOW"]
        shoulder_axis = xyz["LEFT_SHOULDER"] - xyz["RIGHT_SHOULDER"]
        hand_lateral = (
            xyz[f"{prefix}_HAND_INDEX_1"] - xyz[f"{prefix}_HAND_PINKY_1"]
        )
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm < 0.08:
            return None
        axis /= axis_norm
        reference = shoulder_axis - axis * float(np.dot(shoulder_axis, axis))
        lateral = hand_lateral - axis * float(np.dot(hand_lateral, axis))
        if np.linalg.norm(reference) < 0.03 or np.linalg.norm(lateral) < 0.015:
            return None
        reference /= np.linalg.norm(reference)
        lateral /= np.linalg.norm(lateral)
        return float(
            np.arctan2(
                np.dot(np.cross(reference, lateral), axis),
                np.dot(reference, lateral),
            )
        )

    def update(self, frame: dict) -> tuple[dict[str, float], bool]:
        for side in ("left", "right"):
            dt = self._dt(frame, side)
            angle = None if self._occluded(frame, side) else self._angle(frame, side)
            if angle is None:
                self.invalid_elapsed_s[side] += dt
                self.quality[side] = (
                    "HOLD"
                    if self.invalid_elapsed_s[side] <= self.invalid_hold_s
                    else "NEUTRAL_RETURN"
                )
                if self.invalid_elapsed_s[side] > self.invalid_hold_s:
                    # Orientation is a soft task on the 5-DOF G1 arm.  A
                    # missing/occluded hand must not preserve a stale roll
                    # forever; return gradually without disturbing position IK.
                    alpha = 1.0 - np.exp(-dt / self.neutral_return_tau_s)
                    previous = self.unwrapped.get(side, self.last[side])
                    candidate = previous + alpha * (0.0 - previous)
                    max_step = self.max_speed_rad_s * dt
                    candidate = previous + float(
                        np.clip(candidate - previous, -max_step, max_step)
                    )
                    self.unwrapped[side] = candidate
                    self.last[side] = float(np.clip(candidate, -1.75, 1.75))
                continue
            self.invalid_elapsed_s[side] = 0.0
            self.quality[side] = "TRACKING" if side in self.baseline else "CALIBRATING"
            if side not in self.baseline:
                self.samples[side].append(angle)
                if len(self.samples[side]) >= self.calibration_frames:
                    values = np.asarray(self.samples[side], dtype=np.float64)
                    self.baseline[side] = float(
                        np.arctan2(np.mean(np.sin(values)), np.mean(np.cos(values)))
                    )
                    # The first post-calibration sample must start from the
                    # neutral relative orientation. Without initializing the
                    # unwrap state here, that sample bypasses the rate limiter
                    # and can command an immediate +/-1.75 rad hand flip.
                    self.unwrapped[side] = 0.0
                    self.last[side] = 0.0
                continue
            wrapped = float(
                np.arctan2(
                    np.sin(angle - self.baseline[side]),
                    np.cos(angle - self.baseline[side]),
                )
            )
            # Keep the closest 2*pi-equivalent angle to the preceding sample.
            # Direct clipping of [-pi, pi] caused +1.75 -> -1.75 jumps when a
            # hand crossed the wrap boundary, visually flipping the G1 hand.
            previous_unwrapped = self.unwrapped.get(side, wrapped)
            raw_candidate = previous_unwrapped + float(
                np.arctan2(
                    np.sin(wrapped - previous_unwrapped),
                    np.cos(wrapped - previous_unwrapped),
                )
            )
            error = raw_candidate - previous_unwrapped
            if abs(error) <= self.tracking_deadband_rad:
                candidate = previous_unwrapped
            else:
                # Remove the sensor-noise deadband and low-pass the remaining
                # circular error. BODY_38 finger points are much noisier than
                # elbow/wrist positions and must not rotate a stationary hand.
                error -= np.copysign(self.tracking_deadband_rad, error)
                alpha = 1.0 - np.exp(-dt / self.smoothing_tau_s)
                candidate = previous_unwrapped + alpha * error
            max_step = self.max_speed_rad_s * dt
            candidate = previous_unwrapped + float(
                np.clip(candidate - previous_unwrapped, -max_step, max_step)
            )
            self.unwrapped[side] = candidate
            self.last[side] = float(np.clip(candidate, -1.75, 1.75))
        return self.last.copy(), len(self.baseline) == 2


DEFAULT_GMR_ROOT = (
    Path.home() / "g1_isaaclab_project" / "repos" / "GMR"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=15050)
    parser.add_argument("--output-host", default="127.0.0.1")
    parser.add_argument("--output-port", type=int, default=15051)
    parser.add_argument("--telemetry-host", default=None)
    parser.add_argument("--telemetry-port", type=int, default=15053)
    parser.add_argument(
        "--gmr-root",
        type=Path,
        default=DEFAULT_GMR_ROOT,
    )
    parser.add_argument("--human-height", type=float, default=1.80)
    parser.add_argument("--cutoff-hz", type=float, default=10.0)
    parser.add_argument("--min-cutoff-hz", type=float, default=2.0)
    parser.add_argument("--velocity-beta", type=float, default=1.2)
    parser.add_argument("--derivative-cutoff-hz", type=float, default=1.0)
    parser.add_argument("--input-fps", type=float, default=60.0)
    parser.add_argument(
        "--require-calibration",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--gmr-velocity-limit",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Enable GMR's internal 3*pi rad/s solver limit. Disabled by "
            "default because the downstream bridge already enforces per-joint "
            "velocity limits and double limiting adds visible lag."
        ),
    )
    parser.add_argument("--max-ik-iterations", type=int, default=20)
    parser.add_argument(
        "--elbow-posture-cost",
        type=float,
        default=0.05,
        help="Low-priority anatomical elbow hinge regularization cost.",
    )
    parser.add_argument(
        "--anatomical-branch-continuity",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Require four coherent frames before changing the directed elbow "
            "plane. Disable only to reproduce the protected baseline."
        ),
    )
    parser.add_argument(
        "--mirror-rescue",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Enable event-triggered arm-only continuation recovery after the "
            "nominal deterministic GMR solve. Use --no-mirror-rescue for the "
            "protected baseline behavior."
        ),
    )
    parser.add_argument("--mirror-workers", type=int, default=4)
    parser.add_argument(
        "--mode", choices=("upper_body", "whole_body"), default="upper_body"
    )
    parser.add_argument("--stale-after", type=float, default=0.35)
    return parser.parse_args()


def actuator_joint_names(model: mujoco.MjModel) -> list[str]:
    names = []
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        names.append(
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        )
    return names


def qpos_for_joints(
    model: mujoco.MjModel, qpos: np.ndarray, names: list[str]
) -> np.ndarray:
    values = []
    for name in names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        values.append(float(qpos[int(model.jnt_qposadr[joint_id])]))
    return np.asarray(values, dtype=np.float64)


def close_kinematic_relatives(
    model: mujoco.MjModel, first: int, second: int, max_hops: int = 3
) -> bool:
    """Return true for nearby links whose collision meshes overlap by design."""
    for child, possible_ancestor in ((first, second), (second, first)):
        current = int(child)
        for _ in range(max_hops):
            current = int(model.body_parentid[current])
            if current == possible_ancestor:
                return True
            if current == 0:
                break
    return False


def forward_g1_skeleton(
    model: mujoco.MjModel,
    base_qpos: np.ndarray,
    joint_values: np.ndarray,
) -> tuple[dict[str, list[float]], int]:
    data = mujoco.MjData(model)
    data.qpos[:] = np.asarray(base_qpos, dtype=np.float64)
    for name, value in zip(G1_23DOF_ORDER, joint_values):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id >= 0:
            data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)
    mujoco.mj_forward(model, data)
    positions: dict[str, list[float]] = {}
    for name in G1_SKELETON_BODIES:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id >= 0:
            positions[name] = data.xpos[body_id].astype(float).tolist()
    self_contacts = 0
    for contact in data.contact:
        first = int(model.geom_bodyid[int(contact.geom1)])
        second = int(model.geom_bodyid[int(contact.geom2)])
        if (
            first > 0
            and second > 0
            and first != second
            and not close_kinematic_relatives(model, first, second)
        ):
            self_contacts += 1
    return positions, self_contacts


def retarget_with_iterations(
    gmr,
    human_data: dict,
    offset_to_ground: bool,
    extra_tasks: tuple = (),
) -> tuple[np.ndarray, int]:
    """Run upstream GMR's table-1 loop while exposing its iteration count."""
    import mink

    gmr.update_targets(human_data, offset_to_ground)
    current_error = gmr.error1()
    dt = gmr.configuration.model.opt.timestep
    tasks = [*gmr.tasks1, *extra_tasks]
    velocity = mink.solve_ik(
        gmr.configuration, tasks, dt, gmr.solver, gmr.damping, gmr.ik_limits
    )
    gmr.configuration.integrate_inplace(velocity, dt)
    iterations = 1
    next_error = gmr.error1()
    while current_error - next_error > 0.001 and iterations <= gmr.max_iter:
        current_error = next_error
        velocity = mink.solve_ik(
            gmr.configuration, tasks, dt, gmr.solver, gmr.damping,
            gmr.ik_limits,
        )
        gmr.configuration.integrate_inplace(velocity, dt)
        iterations += 1
        next_error = gmr.error1()
    return gmr.configuration.data.qpos.copy(), iterations


def limb_direction_metrics(
    human_data: dict[str, tuple[np.ndarray, np.ndarray]],
    robot_positions: dict[str, list[float]],
) -> tuple[dict[str, float], dict[str, float]]:
    angle_errors: dict[str, float] = {}
    endpoint_errors: dict[str, float] = {}
    for side in ("left", "right"):
        human_chain = (f"{side}_shoulder", f"{side}_elbow", f"{side}_wrist")
        robot_chain = (
            f"{side}_shoulder_pitch_link", f"{side}_elbow_link",
            f"{side}_wrist_roll_rubber_hand",
        )
        if not all(name in human_data for name in human_chain) or not all(
            name in robot_positions for name in robot_chain
        ):
            continue
        human = [np.asarray(human_data[name][0]) for name in human_chain]
        robot = [np.asarray(robot_positions[name]) for name in robot_chain]
        for segment, first, second in (
            ("upper", 0, 1), ("forearm", 1, 2)
        ):
            h = human[second] - human[first]
            r = robot[second] - robot[first]
            denominator = max(float(np.linalg.norm(h) * np.linalg.norm(r)), 1e-8)
            cosine = float(np.clip(np.dot(h, r) / denominator, -1.0, 1.0))
            angle_errors[f"{side}_{segment}_direction_error_deg"] = float(
                np.degrees(np.arccos(cosine))
            )
        endpoint_errors[f"{side}_wrist_normalized_error"] = float(
            np.linalg.norm(
                (human[2] - human[0]) - (robot[2] - robot[0])
            ) / 0.258
        )
    return angle_errors, endpoint_errors


def main() -> int:
    args = parse_args()
    sys.path.insert(0, str(args.gmr_root.resolve()))
    from general_motion_retargeting import params

    # Keep the complete official G1 kinematic chain constrained inside GMR.
    # Isaac decides which output joints are applied; removing lower-body IK
    # constraints can leave free numerical DoFs and produce non-finite qpos.
    config = Path(__file__).with_name("zed_body38_to_g1_23dof.json").resolve()
    robot_key = "unitree_g1_23dof"
    params.ROBOT_XML_DICT[robot_key] = official_g1_23dof_xml()
    params.IK_CONFIG_DICT.setdefault("zed_body38", {})[robot_key] = config
    from general_motion_retargeting.motion_retarget import GeneralMotionRetargeting

    gmr = GeneralMotionRetargeting(
        src_human="zed_body38",
        tgt_robot=robot_key,
        actual_human_height=args.human_height,
        solver="daqp",
        damping=0.5,
        verbose=False,
        use_velocity_limit=args.gmr_velocity_limit,
    )
    gmr.max_iter = max(1, int(args.max_ik_iterations))
    constrain_gmr_to_23dof(
        gmr,
        lock_waist_yaw=args.mode == "upper_body",
        restrict_backward_arms=args.mode == "upper_body",
    )
    task_names = list(gmr.ik_match_table1.keys())
    elbow_regularizer = AnatomicalElbowRegularizer(
        gmr,
        cost=max(0.0, args.elbow_posture_cost),
        nominal_fps=args.input_fps,
    )
    # Live upper-body imitation uses one causal solve against reachable G1
    # elbow/wrist targets reconstructed from BODY_38 limb directions.  The
    # legacy multi-candidate refiner remains available to offline A/B tools.
    arm_ik_refiner = SimpleArmPositionIK(
        gmr, anchor_fixed_base=args.mode == "upper_body"
    )
    order_29 = actuator_joint_names(gmr.model)
    fixed_lower_targets = (
        "left_hip",
        "right_hip",
        "left_knee",
        "right_knee",
        "left_foot",
        "right_foot",
    )
    adapter = Body38ToGMR(
        nominal_fps=args.input_fps,
        memory_seconds=0.25,
        fixed_stance=args.mode == "upper_body",
        persistent_memory_targets=(
            fixed_lower_targets if args.mode == "upper_body" else ()
        ),
        anatomical_branch_continuity=args.anatomical_branch_continuity,
        branch_confirm_frames=4,
    )
    hand_roll = RelativeHandRoll(nominal_fps=args.input_fps)
    required = set(GMR_BODY38_MAP)
    neutral_gmr_qpos = gmr.configuration.data.qpos.copy()

    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    receiver.bind((args.listen_host, args.listen_port))
    receiver.settimeout(0.1)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = (args.output_host, args.output_port)
    telemetry_destination = (
        (args.telemetry_host, args.telemetry_port)
        if args.telemetry_host
        else None
    )

    nyquist_safe_hz = 0.45 * float(args.input_fps)
    max_cutoff_hz = np.minimum(MAX_CUTOFF_HZ_23, nyquist_safe_hz)
    min_cutoff_hz = np.minimum(MIN_CUTOFF_HZ_23, max_cutoff_hz)
    joint_filter = AdaptiveJointFilter(
        min_cutoff_hz=np.maximum(0.1, min_cutoff_hz),
        max_cutoff_hz=np.maximum(0.1, max_cutoff_hz),
        velocity_beta=max(0.0, float(args.velocity_beta)),
        derivative_cutoff_hz=max(0.1, float(args.derivative_cutoff_hz)),
    )
    # Simulation-only tuning: body-origin Cartesian tasks have a measurable
    # residual even for a useful solution. The former 0.10 m warning reduced
    # nearly every good frame to 55%, producing avoidable live lag. Joint,
    # velocity, collision and watchdog protections remain unchanged.
    feasibility = G1FeasibilityFilter(
        residual_warn=0.24 if args.mode == "upper_body" else 0.10,
        residual_severe=0.40 if args.mode == "upper_body" else 0.45,
        yellow_blend=0.82 if args.mode == "upper_body" else 0.55,
        # In upper-body simulation, do not leave the arms indefinitely at an
        # occluded/self-colliding target. Brief events hold the last reliable
        # pose; persistent ORANGE states return smoothly to neutral. Keep the
        # behavior disabled for whole-body and physical-robot paths.
        orange_return_after_s=0.35 if args.mode == "upper_body" else None,
        orange_return_tau_s=0.80,
    )
    mirror_rescue = MirrorContinuationRescue(
        # The event-driven mirrored branch competed with the deterministic
        # arm solve on 97% of this recording and made forearm tracking worse.
        # Keep it available for whole-body experiments, never in the clean
        # upper-body command path.
        enabled=args.mirror_rescue and args.mode != "upper_body",
        workers=max(1, args.mirror_workers),
        # The measured project baseline has useful upper-body solutions below
        # roughly 12 cm. Rescue is therefore reserved for the tail, not every
        # nominal GMR frame.
        residual_trigger_m=0.12,
        collision_trigger_m=0.005,
    )
    packet_metrics = PacketMetrics()
    stale_watchdog_active = False
    last_update = time.monotonic()
    last_status = last_update
    accepted = 0
    rejected = 0
    superseded = 0
    accepted_times: deque[float] = deque()
    last_solve_ms = 0.0
    last_source_age_ms = 0.0
    last_ik_error = 0.0
    last_ik_position_mean_m = 0.0
    last_ik_position_max_m = 0.0
    last_ik_upper_position_max_m = 0.0
    last_solver_iterations = 0
    rejection_reasons: dict[str, int] = {}
    control_session_active = False
    control_session_id = 0

    def reset_control_session(reason: str) -> None:
        """Reset every causal state after operator/calibration invalidation."""
        nonlocal control_session_active, control_session_id, last_update
        control_session_id += 1
        control_session_active = False
        adapter.reset()
        joint_filter.reset()
        feasibility.reset()
        mirror_rescue.reset()
        hand_roll.reset()
        gmr.configuration.update(neutral_gmr_qpos.copy())
        elbow_regularizer.reset(neutral_gmr_qpos)
        arm_ik_refiner.reset(neutral_gmr_qpos)
        last_update = time.monotonic()
        status_packet = {
            "schema": "zed_gmr_g1_23dof_live/status/v1",
            "status": "CONTROL_SESSION_RESET",
            "reason": reason,
            "control_session_id": control_session_id,
            "timestamp_ns": time.time_ns(),
        }
        encoded = json.dumps(status_packet, separators=(",", ":")).encode()
        sender.sendto(encoded, destination)
        if telemetry_destination is not None:
            sender.sendto(encoded, telemetry_destination)
        print(
            f"CONTROL_SESSION_RESET id={control_session_id} reason={reason}",
            flush=True,
        )
    print(
        f"GMR bridge input={args.listen_host}:{args.listen_port} "
        f"output={args.output_host}:{args.output_port}"
    )
    try:
        while True:
            try:
                payload, _ = receiver.recvfrom(65535)
                receive_timestamp_ns = time.time_ns()
            except socket.timeout:
                if time.monotonic() - last_update > args.stale_after:
                    if not stale_watchdog_active:
                        packet_metrics.watchdog_triggers += 1
                        stale_watchdog_active = True
                    sender.sendto(
                        json.dumps(
                            {
                                "schema": "zed_gmr_g1_23dof_live/status/v1",
                                "status": "STALE",
                                "age_s": time.monotonic() - last_update,
                            },
                            separators=(",", ":"),
                        ).encode(),
                        destination,
                    )
                continue

            # GMR is optimization-based. If capture briefly outruns IK, discard
            # queued historical frames and solve only the newest pose. This
            # bounds latency instead of reproducing old motion seconds later.
            receiver.setblocking(False)
            while True:
                try:
                    payload, _ = receiver.recvfrom(65535)
                    receive_timestamp_ns = time.time_ns()
                    superseded += 1
                except BlockingIOError:
                    break
            receiver.settimeout(0.1)

            frame = json.loads(payload)
            if frame.get("schema") != "zed_body38_live/v1":
                rejected += 1
                reason = str(frame.get("status", "non_body_packet"))
                rejection_reasons[reason] = rejection_reasons.get(reason, 0) + 1
                continue
            stale_watchdog_active = False
            trace_in = frame.get("latency_trace_ns") or {}
            # Do not subtract Windows and WSL wall clocks directly. Their
            # epoch offset can jump after suspend/resume and produced false
            # 150-400 ms "network" latency on this same-host UDP path.
            windows_to_wsl_ms = None
            packet_metrics.observe(
                int(frame.get("sequence", accepted)),
                receive_timestamp_ns,
                windows_to_wsl_ms,
            )
            calibration_ready = (
                (frame.get("calibration") or {}).get("state") == "READY"
            )
            operator_locked = (
                (frame.get("operator_selection") or {}).get("state") == "LOCKED"
            )
            if args.require_calibration and not calibration_ready:
                if control_session_active:
                    reset_control_session("calibration_not_ready")
                rejected += 1
                rejection_reasons["calibration_not_ready"] = (
                    rejection_reasons.get("calibration_not_ready", 0) + 1
                )
                now = time.monotonic()
                if now - last_status >= 1.0:
                    calibration_info = frame.get("calibration") or {}
                    print(
                        "CONTROL_BLOCKED "
                        f"reason=calibration_not_ready "
                        f"state={calibration_info.get('state', 'MISSING')} "
                        f"progress={float(calibration_info.get('progress', 0.0)):.0%}"
                    )
                    last_status = now
                continue
            if not operator_locked:
                if control_session_active:
                    reset_control_session("operator_not_locked")
                rejected += 1
                rejection_reasons["operator_not_locked"] = (
                    rejection_reasons.get("operator_not_locked", 0) + 1
                )
                now = time.monotonic()
                if now - last_status >= 1.0:
                    selection_info = frame.get("operator_selection") or {}
                    print(
                        "CONTROL_BLOCKED "
                        f"reason=operator_not_locked "
                        f"state={selection_info.get('state', 'MISSING')} "
                        f"acquisition_frames={selection_info.get('acquisition_frames', 0)}"
                    )
                    last_status = now
                continue
            adapted = adapter.adapt(frame)
            if adapted is None or not required.issubset(adapted.human_data):
                rejected += 1
                reason = (
                    adapter.last_rejection_reason
                    if adapted is None
                    else "missing_gmr_targets"
                )
                rejection_reasons[reason] = rejection_reasons.get(reason, 0) + 1
                continue

            solve_started_ns = time.time_ns()
            solve_started = time.perf_counter()
            elbow_regularizer.update(adapted.human_data)
            qpos, last_solver_iterations = retarget_with_iterations(
                gmr,
                adapted.human_data,
                offset_to_ground=True,
                extra_tasks=(elbow_regularizer.task,),
            )
            qpos = arm_ik_refiner.update(qpos, adapted.human_data)
            gmr.configuration.update(qpos)
            last_ik_error = float(gmr.error1())
            position_errors = [
                float(np.linalg.norm(task.compute_error(gmr.configuration)[:3]))
                for task in gmr.tasks1
            ]
            last_ik_position_mean_m = float(np.mean(position_errors))
            last_ik_position_max_m = float(np.max(position_errors))
            upper_position_errors = [
                error
                for name, error in zip(task_names, position_errors)
                if any(
                    token in name
                    for token in ("pelvis", "torso", "shoulder", "elbow", "wrist")
                )
            ]
            last_ik_upper_position_max_m = (
                float(np.max(upper_position_errors))
                if upper_position_errors
                else last_ik_position_max_m
            )
            q29 = qpos_for_joints(gmr.model, qpos, order_29)
            raw = project_29_to_23(named_joint_values(order_29, q29))
            if args.mode == "upper_body":
                # Isaac deliberately ignores the first 12 lower-body targets
                # and applies its official fixed double-support controller.
                # Do not let harmless numerical IK motion in those unused DoFs
                # raise a global YELLOW state and slow every upper-body joint.
                raw[:12] = 0.0
            pelvis_yaw = (frame.get("pelvis_frame") or {}).get(
                "relative_neutral_yaw_rad"
            )
            if isinstance(pelvis_yaw, (int, float)) and np.isfinite(pelvis_yaw):
                raw[G1_23DOF_ORDER.index("waist_yaw_joint")] = float(
                    np.clip(pelvis_yaw, -1.2, 1.2)
                )
            wrist_roll, wrist_calibrated = hand_roll.update(frame)
            raw[G1_23DOF_ORDER.index("left_wrist_roll_joint")] = wrist_roll["left"]
            raw[G1_23DOF_ORDER.index("right_wrist_roll_joint")] = wrist_roll["right"]
            if not np.isfinite(raw).all():
                rejected += 1
                continue

            human_targets = {
                name: np.asarray(value[0], dtype=np.float64)
                for name, value in adapted.human_data.items()
            }

            def evaluate_arm_candidate(candidate_q: np.ndarray) -> KinematicEvaluation:
                candidate_skeleton, _ = forward_g1_skeleton(
                    gmr.model, qpos, candidate_q
                )
                return KinematicEvaluation(
                    candidate_skeleton,
                    upper_body_capsule_report(candidate_skeleton),
                )

            mirror_result = mirror_rescue.update(
                nominal_q=raw,
                previous_safe_q=feasibility.safe_q,
                targets=human_targets,
                evaluate=evaluate_arm_candidate,
                joint_limits=np.column_stack((
                    G1_23_ANATOMICAL_LIMITS_RAD[:, 0] + feasibility.margin,
                    G1_23_ANATOMICAL_LIMITS_RAD[:, 1] - feasibility.margin,
                )),
                residual_m=last_ik_upper_position_max_m,
                branch_change_pending=adapter.branch_change_pending,
            )
            raw = mirror_result.q

            now = time.monotonic()
            dt = min(max(now - last_update, 1.0 / 120.0), 0.10)
            filtered = joint_filter.update(raw, dt, MAX_VELOCITY_RAD_S)
            if not np.isfinite(filtered).all():
                # A non-finite target must never cross the simulation/robot
                # command boundary. Reset the perception filter and wait for a
                # new coherent GMR solution.
                joint_filter.reset()
                rejected += 1
                continue
            perception_reasons: list[str] = []
            arm_perception_reasons: dict[str, list[str]] = {
                "left": [], "right": [],
            }
            occlusion_analysis = frame.get("occlusion_analysis") or {}
            overlap = occlusion_analysis.get(
                "arm_torso_overlap", {}
            )
            if overlap.get("right"):
                arm_perception_reasons["right"].append("right_wrist_occluded")
            if overlap.get("left"):
                arm_perception_reasons["left"].append("left_wrist_occluded")
            if pelvis_left_right_swap_risk(frame):
                perception_reasons.append("left_right_swap_risk")
            calibration_profile = (frame.get("calibration") or {}).get("profile") or {}
            if float(calibration_profile.get("median_bone_length_cv", 0.0)) > 0.08:
                perception_reasons.append("bone_length_violation")
            akc_confidence = occlusion_analysis.get("akc_candidate_confidence") or {}
            akc_recovered = occlusion_analysis.get("arm_chain_recovered") or {}
            occlusion_reasons = set(occlusion_analysis.get("reasons") or ())
            bilateral_front = (
                "bilateral_front_arm_occlusion" in occlusion_reasons
                and bool(akc_recovered.get("left"))
                and bool(akc_recovered.get("right"))
            )
            arm_quality_blend: dict[str, float] = {}
            for side in ("left", "right"):
                if not overlap.get(side):
                    arm_quality_blend[side] = 1.0
                    continue
                confidence_value = akc_confidence.get(side, 0.0)
                confidence = (
                    float(confidence_value)
                    if isinstance(confidence_value, (int, float))
                    and np.isfinite(confidence_value)
                    else 0.0
                )
                # A coherent AKC reconstruction is almost fully trusted.  A
                # low-confidence overlap is softened only on the affected arm.
                arm_quality_blend[side] = (
                    0.92 if akc_recovered.get(side) and confidence >= 0.50
                    else 0.72
                )
            # Multi-view confidence is segment-local. A weak wrist no longer
            # disables the complete BODY_38 frame: only the corresponding arm
            # task is softened while its upper/forearm direction remains live.
            segment_quality = (frame.get("human_state") or {}).get(
                "segment_quality", {}
            )
            retargeting_policy = (frame.get("human_state") or {}).get(
                "retargeting_policy", {}
            )
            low_quality_blend = float(
                retargeting_policy.get("low_quality_arm_blend", 0.55)
            )
            full_quality_blend = float(
                retargeting_policy.get("full_quality_arm_blend", 1.0)
            )
            minimum_segment_quality = float(
                retargeting_policy.get("minimum_segment_quality", 0.25)
            )
            for side in ("left", "right"):
                value = segment_quality.get(f"{side}_arm", 1.0)
                value = (
                    float(value)
                    if isinstance(value, (int, float)) and np.isfinite(value)
                    else 0.0
                )
                confidence_blend = float(np.clip(
                    low_quality_blend
                    + (full_quality_blend - low_quality_blend) * value,
                    min(low_quality_blend, full_quality_blend),
                    max(low_quality_blend, full_quality_blend),
                ))
                arm_quality_blend[side] = min(
                    arm_quality_blend.get(side, 1.0), confidence_blend
                )
                if value < minimum_segment_quality:
                    arm_perception_reasons[side].append(
                        f"low_{side}_arm_fusion_confidence"
                    )
            bilateral_continuity_blend = 1.0
            bilateral_confidence = 0.0
            if bilateral_front and feasibility.safe_q is not None:
                confidence_values = []
                for side in ("left", "right"):
                    value = akc_confidence.get(side, 0.0)
                    confidence_values.append(
                        float(value)
                        if isinstance(value, (int, float)) and np.isfinite(value)
                        else 0.0
                    )
                    arm_perception_reasons[side].append(
                        "bilateral_front_continuity"
                    )
                # GMR remains the nominal solver, but simultaneous occlusion
                # is allowed to move both arms only continuously from the
                # previous feasible command. This rejects a single-frame
                # mirrored/both-arms-behind-torso solution without freezing a
                # sustained, valid chest-level gesture.
                bilateral_continuity_blend = (
                    0.52 if min(confidence_values) >= 0.50 else 0.28
                )
                bilateral_confidence = min(confidence_values)
                filtered[13:23] = feasibility.safe_q[13:23] + (
                    bilateral_continuity_blend
                    * (filtered[13:23] - feasibility.safe_q[13:23])
                )
            raw_skeleton, raw_self_collisions = forward_g1_skeleton(
                gmr.model, qpos, raw
            )
            # The bilateral continuity governor is the actual candidate sent
            # across the feasibility boundary. Evaluate collision on that
            # candidate, rather than rejecting it because an intermediate raw
            # GMR solution was deliberately softened.
            collision_skeleton = raw_skeleton
            raw_self_collisions_for_safety = raw_self_collisions
            if bilateral_front:
                collision_skeleton, raw_self_collisions_for_safety = forward_g1_skeleton(
                    gmr.model, qpos, filtered
                )
            raw_collision_report = upper_body_capsule_report(collision_skeleton)
            limb_direction_error, end_effector_error = limb_direction_metrics(
                adapted.human_data, raw_skeleton
            )
            upper_relative_residual_m = (
                max(end_effector_error.values()) * 0.258
                if end_effector_error
                else last_ik_upper_position_max_m
            )
            if bilateral_front and bilateral_confidence >= 0.50:
                # A high-confidence AKC bilateral reconstruction has already
                # been geometrically projected. Keep it in the arm-local
                # YELLOW path instead of forcing a global ORANGE hold due to
                # the ungoverned nominal residual.
                upper_relative_residual_m = min(
                    upper_relative_residual_m,
                    0.85 * feasibility.residual_warn,
                )
            previous_safe_command = (
                feasibility.safe_q.copy()
                if feasibility.safe_q is not None
                else feasibility.nominal.copy()
            )
            feasibility_result = feasibility.update(
                filtered,
                dt,
                reasons=perception_reasons,
                gmr_residual=(
                    upper_relative_residual_m
                    if args.mode == "upper_body"
                    else last_ik_position_max_m
                ),
                self_collision=raw_self_collisions_for_safety > 0,
                stale=False,
                collision_margin_m=raw_collision_report.minimum_margin_m,
                arm_collision_margins=raw_collision_report.arm_minimum_margin_m,
                arm_reasons=arm_perception_reasons,
                arm_quality_blend=arm_quality_blend,
            )
            safe = feasibility_result.safe_q

            def evaluate_body_barrier(joint_position):
                skeleton, contacts = forward_g1_skeleton(
                    gmr.model, qpos, joint_position
                )
                return upper_body_capsule_report(skeleton), contacts

            barrier_projection = project_configuration_along_safe_path(
                previous_safe_command,
                safe,
                evaluate_body_barrier,
                # The official neutral hand/hip arrangement sits almost on
                # the conservative capsule shell.  Require non-penetration at
                # the final projection; the upstream governor already starts
                # fading commands inside the 5 mm warning band.
                clearance_m=0.0,
            )
            safe = barrier_projection.joint_position
            safety_reasons = list(feasibility_result.reasons)
            safety_level = SafetyLevel(feasibility_result.level)
            if barrier_projection.applied:
                safety_reasons.append("robot_body_barrier_projection")
                safety_level = max(safety_level, SafetyLevel.YELLOW)
                # Keep the rate governor's state identical to the command that
                # actually leaves the bridge; otherwise hidden velocity can
                # push through the barrier on the following frame.
                feasibility.safe_q = safe.copy()
                feasibility.velocity = (safe - previous_safe_command) / dt
                feasibility.last_reliable = safe.copy()
            saturation_low = G1_23_LIMITS_RAD[:, 0] + feasibility.margin
            saturation_high = G1_23_LIMITS_RAD[:, 1] - feasibility.margin
            saturation_names = [
                name
                for name, value, low, high in zip(
                    G1_23DOF_ORDER, filtered, saturation_low, saturation_high
                )
                if float(value) < float(low) or float(value) > float(high)
            ]
            safe_skeleton, safe_self_collisions = forward_g1_skeleton(
                gmr.model, qpos, safe
            )
            last_update = now
            accepted += 1
            control_session_active = True
            last_solve_ms = (time.perf_counter() - solve_started) * 1000.0
            source_timestamp_ns = int(frame.get("timestamp_ns", 0))
            trace_in = frame.get("latency_trace_ns") or {}
            try:
                capture_to_windows_send_ms = max(
                    0.0,
                    (
                        int(trace_in["t2_windows_udp_send_ns"])
                        - int(trace_in["t0_capture_ns"])
                    )
                    / 1e6,
                )
            except (KeyError, TypeError, ValueError):
                capture_to_windows_send_ms = 0.0
            # Both terms below are measured inside one clock domain. Their sum
            # is a stable source-age estimate without assuming clock alignment.
            last_source_age_ms = capture_to_windows_send_ms + max(
                0.0, (time.time_ns() - receive_timestamp_ns) / 1e6
            )
            accepted_times.append(now)
            while accepted_times and now - accepted_times[0] > 1.0:
                accepted_times.popleft()

            bridge_send_timestamp_ns = time.time_ns()
            latency_trace = dict(frame.get("latency_trace_ns") or {})
            latency_trace.update(
                {
                    "t3_wsl_receive_ns": receive_timestamp_ns,
                    "t4_gmr_start_ns": solve_started_ns,
                    "t5_gmr_finish_ns": bridge_send_timestamp_ns,
                }
            )
            packet = {
                "schema": "zed_gmr_g1_23dof_live/v1",
                "sequence": int(frame.get("sequence", accepted)),
                "control_session_id": control_session_id,
                "source_timestamp_ns": source_timestamp_ns,
                "bridge_timestamp_ns": time.time_ns(),
                "joint_names": G1_23DOF_ORDER,
                "joint_position_rad": safe.tolist(),
                "raw_joint_position_rad": raw.tolist(),
                "filtered_joint_position_rad": filtered.tolist(),
                "safe_joint_position_rad": safe.tolist(),
                "bilateral_front_continuity_blend": bilateral_continuity_blend,
                "body_confidence": float(frame.get("body_confidence", 0.0)),
                "source_multi_camera": frame.get("multi_camera"),
                "source_perception_metrics": frame.get("perception_metrics"),
                "source_occlusion_analysis": frame.get("occlusion_analysis"),
                "source_human_state": frame.get("human_state"),
                "source_control_mode_request": frame.get(
                    "control_mode_request"
                ),
                "valid_targets": adapted.valid_targets,
                "memory_targets": adapted.used_memory_targets,
                "raw_fallback_targets": adapted.raw_fallback_targets,
                "occlusion_held_targets": adapted.occlusion_held_targets,
                "pelvis_height_m": adapted.pelvis_height_m,
                "physical_robot_output": False,
                "safety": {
                    "level": safety_level.name,
                    "reasons": list(dict.fromkeys(safety_reasons)),
                    "blend": feasibility_result.blend,
                    "joint_limit_saturation": feasibility_result.joint_limit_saturation_count,
                    "joint_limit_saturation_names": saturation_names,
                    "velocity_limit_events": feasibility_result.velocity_limit_count,
                    "acceleration_limit_events": feasibility_result.acceleration_limit_count,
                    "workspace_projection_count": adapted.workspace_projection_count,
                    "operator_calibrated": adapted.operator_calibrated,
                    "raw_self_collision_count": raw_self_collisions,
                    "safe_self_collision_count": safe_self_collisions,
                    "robot_body_barrier_projection_applied": bool(
                        barrier_projection.applied
                    ),
                    "robot_body_barrier_projection_alpha": float(
                        barrier_projection.alpha
                    ),
                    "robot_body_barrier_safe_margin_m": float(
                        barrier_projection.minimum_margin_m
                    ),
                    "minimum_collision_margin_m": raw_collision_report.minimum_margin_m,
                    "arm_collision_margin_m": raw_collision_report.arm_minimum_margin_m,
                    "collision_risk_pairs": list(raw_collision_report.risk_pairs),
                    "hard_collision_margin_by_pair_m": raw_collision_report.hard_pair_margins_m,
                    "soft_contact_pairs": list(raw_collision_report.soft_risk_pairs),
                    "soft_contact_margin_m": raw_collision_report.soft_pair_margins_m,
                    "arm_reference_blend": feasibility_result.arm_blend,
                },
                "g1_skeleton": {
                    "body_names": list(G1_SKELETON_BODIES),
                    "edges": [list(edge) for edge in G1_SKELETON_EDGES],
                    "raw_positions_m": raw_skeleton,
                    "safe_positions_m": safe_skeleton,
                },
                "retarget_comparison": {
                    "human_body_names": list(adapted.human_data),
                    "human_edges": [list(edge) for edge in HUMAN_RETARGET_EDGES],
                    "human_positions_m": {
                        name: np.asarray(value[0], dtype=float).tolist()
                        for name, value in adapted.human_data.items()
                    },
                },
                "latency_trace_ns": latency_trace,
                "bridge_metrics": {
                    "output_hz": len(accepted_times),
                    "solve_ms": last_solve_ms,
                    "source_age_ms": last_source_age_ms,
                    "superseded_frames": superseded,
                    "filter_mode": "adaptive_one_euro",
                    "min_cutoff_hz_by_joint": joint_filter.min_cutoff_hz.tolist(),
                    "max_cutoff_hz_by_joint": joint_filter.max_cutoff_hz.tolist(),
                    "mean_active_cutoff_hz": (
                        float(np.mean(joint_filter.last_cutoff_hz))
                        if joint_filter.last_cutoff_hz is not None
                        else float(np.mean(joint_filter.min_cutoff_hz))
                    ),
                    "velocity_beta": joint_filter.velocity_beta,
                    "input_fps": args.input_fps,
                    "wrist_roll_calibrated": wrist_calibrated,
                    "left_elbow_regularizer_target_rad": (
                        elbow_regularizer.last_target_rad.get("left")
                    ),
                    "right_elbow_regularizer_target_rad": (
                        elbow_regularizer.last_target_rad.get("right")
                    ),
                    "left_direct_arm_ik_rms_m": arm_ik_refiner.last_error_m["left"],
                    "right_direct_arm_ik_rms_m": arm_ik_refiner.last_error_m["right"],
                    "left_gmr_arm_rms_m": arm_ik_refiner.last_gmr_error_m["left"],
                    "right_gmr_arm_rms_m": arm_ik_refiner.last_gmr_error_m["right"],
                    "left_arm_baseline_preserved": arm_ik_refiner.last_used_baseline["left"],
                    "right_arm_baseline_preserved": arm_ik_refiner.last_used_baseline["right"],
                    "left_direct_ik_selected_source": arm_ik_refiner.last_selected_source["left"],
                    "right_direct_ik_selected_source": arm_ik_refiner.last_selected_source["right"],
                    "left_direct_ik_objective_m": arm_ik_refiner.last_candidate_objective_m["left"],
                    "right_direct_ik_objective_m": arm_ik_refiner.last_candidate_objective_m["right"],
                    "left_direct_ik_direction_error_deg": arm_ik_refiner.last_direction_error_deg["left"],
                    "right_direct_ik_direction_error_deg": arm_ik_refiner.last_direction_error_deg["right"],
                    "left_straight_arm_blend": (
                        arm_ik_refiner.last_straightness_blend["left"]
                    ),
                    "right_straight_arm_blend": (
                        arm_ik_refiner.last_straightness_blend["right"]
                    ),
                    "left_direct_ik_candidate_count": (
                        arm_ik_refiner.last_candidate_count["left"]
                    ),
                    "right_direct_ik_candidate_count": (
                        arm_ik_refiner.last_candidate_count["right"]
                    ),
                    "left_arm_pole_source": adapter.last_arm_pole_source["left"],
                    "right_arm_pole_source": adapter.last_arm_pole_source["right"],
                    "left_elbow_branch_sign": adapter.last_arm_branch_sign["left"],
                    "right_elbow_branch_sign": adapter.last_arm_branch_sign["right"],
                    "left_wrist_orientation_quality": hand_roll.quality["left"],
                    "right_wrist_orientation_quality": hand_roll.quality["right"],
                    "mirror_rescue_enabled": mirror_rescue.enabled,
                    "mirror_rescue_triggered": mirror_result.triggered,
                    "mirror_rescue_applied": mirror_result.applied,
                    "mirror_rescue_reasons": list(mirror_result.trigger_reasons),
                    "mirror_rescue_candidates": mirror_result.candidate_count,
                    "mirror_rescue_solve_ms": mirror_result.solve_ms,
                    "mirror_nominal_task_error_m": mirror_result.nominal_task_error_m,
                    "mirror_selected_task_error_m": mirror_result.selected_task_error_m,
                    "mirror_nominal_collision_margin_m": mirror_result.nominal_collision_margin_m,
                    "mirror_selected_collision_margin_m": mirror_result.selected_collision_margin_m,
                    "left_wrist_roll_rad": wrist_roll["left"],
                    "right_wrist_roll_rad": wrist_roll["right"],
                    "ik_error_norm": last_ik_error,
                    "ik_position_mean_m": last_ik_position_mean_m,
                    "ik_position_max_m": last_ik_position_max_m,
                    "ik_upper_position_max_m": last_ik_upper_position_max_m,
                    "ik_upper_relative_residual_m": upper_relative_residual_m,
                    "solver_iterations": last_solver_iterations,
                    "solver_iteration_limit": gmr.max_iter + 1,
                    "limb_direction_error_deg": limb_direction_error,
                    "end_effector_normalized_error": end_effector_error,
                    "gmr_velocity_limit": args.gmr_velocity_limit,
                    "system_transport": packet_metrics.snapshot(),
                },
            }
            sender.sendto(
                json.dumps(packet, separators=(",", ":")).encode(), destination
            )
            if telemetry_destination is not None:
                sender.sendto(
                    json.dumps(packet, separators=(",", ":")).encode(),
                    telemetry_destination,
                )
            if now - last_status >= 1.0:
                print(
                    f"accepted={accepted} rejected={rejected} "
                    f"superseded={superseded} hz={len(accepted_times)} "
                    f"solve={last_solve_ms:.1f}ms age={last_source_age_ms:.1f}ms "
                    f"targets={adapted.valid_targets}+{adapted.raw_fallback_targets}"
                    f"+{adapted.used_memory_targets} "
                    f"occlusion_hold={adapted.occlusion_held_targets} "
                    f"ik_error={last_ik_error:.4f} "
                    f"ik_pos={last_ik_position_mean_m:.3f}/"
                    f"{last_ik_position_max_m:.3f}m "
                    f"confidence={packet['body_confidence']:.0f} "
                    f"reject_reasons={rejection_reasons}"
                )
                last_status = now
    except KeyboardInterrupt:
        return 0
    finally:
        receiver.close()
        sender.close()


if __name__ == "__main__":
    raise SystemExit(main())

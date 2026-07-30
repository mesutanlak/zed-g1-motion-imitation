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

from body38_to_gmr import Body38ToGMR, GMR_BODY38_MAP
from g1_dof_projection import (
    G1_23DOF_ORDER,
    constrain_gmr_to_23dof,
    named_joint_values,
    project_29_to_23,
)


MAX_VELOCITY_RAD_S = np.asarray(
    [8.0] * 12 + [6.0] + [12.0] * 10, dtype=np.float64
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
        self.min_cutoff_hz = float(min_cutoff_hz)
        self.max_cutoff_hz = float(max_cutoff_hz)
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
            self.last_cutoff_hz = np.full_like(raw, self.min_cutoff_hz)
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

    def __init__(self, calibration_frames: int = 12) -> None:
        self.calibration_frames = calibration_frames
        self.samples: dict[str, list[float]] = {"left": [], "right": []}
        self.baseline: dict[str, float] = {}
        self.last: dict[str, float] = {"left": 0.0, "right": 0.0}

    @staticmethod
    def _angle(frame: dict, side: str) -> float | None:
        names = frame.get("keypoint_names", [])
        points = frame.get("keypoints_3d_m", [])
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
        if np.linalg.norm(reference) < 0.03 or np.linalg.norm(lateral) < 0.01:
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
            angle = self._angle(frame, side)
            if angle is None:
                continue
            if side not in self.baseline:
                self.samples[side].append(angle)
                if len(self.samples[side]) >= self.calibration_frames:
                    values = np.asarray(self.samples[side], dtype=np.float64)
                    self.baseline[side] = float(
                        np.arctan2(np.mean(np.sin(values)), np.mean(np.cos(values)))
                    )
                continue
            delta = float(
                np.arctan2(
                    np.sin(angle - self.baseline[side]),
                    np.cos(angle - self.baseline[side]),
                )
            )
            self.last[side] = float(np.clip(delta, -1.75, 1.75))
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
    parser.add_argument("--input-fps", type=float, default=15.0)
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


def main() -> int:
    args = parse_args()
    sys.path.insert(0, str(args.gmr_root.resolve()))
    from general_motion_retargeting import params

    # Keep the complete official G1 kinematic chain constrained inside GMR.
    # Isaac decides which output joints are applied; removing lower-body IK
    # constraints can leave free numerical DoFs and produce non-finite qpos.
    config = Path(__file__).with_name("zed_body38_to_g1.json").resolve()
    params.IK_CONFIG_DICT.setdefault("zed_body38", {})["unitree_g1"] = config
    from general_motion_retargeting.motion_retarget import GeneralMotionRetargeting

    gmr = GeneralMotionRetargeting(
        src_human="zed_body38",
        tgt_robot="unitree_g1",
        actual_human_height=args.human_height,
        solver="daqp",
        damping=0.5,
        verbose=False,
        use_velocity_limit=args.gmr_velocity_limit,
    )
    gmr.max_iter = max(1, int(args.max_ik_iterations))
    constrain_gmr_to_23dof(gmr)
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
        persistent_memory_targets=(
            fixed_lower_targets if args.mode == "upper_body" else ()
        ),
    )
    hand_roll = RelativeHandRoll()
    required = set(GMR_BODY38_MAP)

    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind((args.listen_host, args.listen_port))
    receiver.settimeout(0.1)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destination = (args.output_host, args.output_port)

    max_cutoff_hz = min(float(args.cutoff_hz), 0.45 * float(args.input_fps))
    min_cutoff_hz = min(float(args.min_cutoff_hz), max_cutoff_hz)
    joint_filter = AdaptiveJointFilter(
        min_cutoff_hz=max(0.1, min_cutoff_hz),
        max_cutoff_hz=max(0.1, max_cutoff_hz),
        velocity_beta=max(0.0, float(args.velocity_beta)),
        derivative_cutoff_hz=max(0.1, float(args.derivative_cutoff_hz)),
    )
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
    rejection_reasons: dict[str, int] = {}
    print(
        f"GMR bridge input={args.listen_host}:{args.listen_port} "
        f"output={args.output_host}:{args.output_port}"
    )
    try:
        while True:
            try:
                payload, _ = receiver.recvfrom(65535)
            except socket.timeout:
                if time.monotonic() - last_update > args.stale_after:
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

            solve_started = time.perf_counter()
            qpos = gmr.retarget(adapted.human_data, offset_to_ground=True)
            last_ik_error = float(gmr.error1())
            position_errors = [
                float(np.linalg.norm(task.compute_error(gmr.configuration)[:3]))
                for task in gmr.tasks1
            ]
            last_ik_position_mean_m = float(np.mean(position_errors))
            last_ik_position_max_m = float(np.max(position_errors))
            q29 = qpos_for_joints(gmr.model, qpos, order_29)
            raw = project_29_to_23(named_joint_values(order_29, q29))
            wrist_roll, wrist_calibrated = hand_roll.update(frame)
            raw[G1_23DOF_ORDER.index("left_wrist_roll_joint")] = wrist_roll["left"]
            raw[G1_23DOF_ORDER.index("right_wrist_roll_joint")] = wrist_roll["right"]
            if not np.isfinite(raw).all():
                rejected += 1
                continue

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
            last_update = now
            accepted += 1
            last_solve_ms = (time.perf_counter() - solve_started) * 1000.0
            source_timestamp_ns = int(frame.get("timestamp_ns", 0))
            last_source_age_ms = (
                max(0.0, (time.time_ns() - source_timestamp_ns) / 1e6)
                if source_timestamp_ns > 0
                else 0.0
            )
            accepted_times.append(now)
            while accepted_times and now - accepted_times[0] > 1.0:
                accepted_times.popleft()

            packet = {
                "schema": "zed_gmr_g1_23dof_live/v1",
                "sequence": int(frame.get("sequence", accepted)),
                "source_timestamp_ns": source_timestamp_ns,
                "bridge_timestamp_ns": time.time_ns(),
                "joint_names": G1_23DOF_ORDER,
                "joint_position_rad": filtered.tolist(),
                "body_confidence": float(frame.get("body_confidence", 0.0)),
                "valid_targets": adapted.valid_targets,
                "memory_targets": adapted.used_memory_targets,
                "raw_fallback_targets": adapted.raw_fallback_targets,
                "occlusion_held_targets": adapted.occlusion_held_targets,
                "pelvis_height_m": adapted.pelvis_height_m,
                "physical_robot_output": False,
                "bridge_metrics": {
                    "output_hz": len(accepted_times),
                    "solve_ms": last_solve_ms,
                    "source_age_ms": last_source_age_ms,
                    "superseded_frames": superseded,
                    "filter_mode": "adaptive_one_euro",
                    "min_cutoff_hz": joint_filter.min_cutoff_hz,
                    "max_cutoff_hz": joint_filter.max_cutoff_hz,
                    "mean_active_cutoff_hz": (
                        float(np.mean(joint_filter.last_cutoff_hz))
                        if joint_filter.last_cutoff_hz is not None
                        else joint_filter.min_cutoff_hz
                    ),
                    "velocity_beta": joint_filter.velocity_beta,
                    "input_fps": args.input_fps,
                    "wrist_roll_calibrated": wrist_calibrated,
                    "left_wrist_roll_rad": wrist_roll["left"],
                    "right_wrist_roll_rad": wrist_roll["right"],
                    "ik_error_norm": last_ik_error,
                    "ik_position_mean_m": last_ik_position_mean_m,
                    "ik_position_max_m": last_ik_position_max_m,
                    "gmr_velocity_limit": args.gmr_velocity_limit,
                },
            }
            sender.sendto(
                json.dumps(packet, separators=(",", ":")).encode(), destination
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

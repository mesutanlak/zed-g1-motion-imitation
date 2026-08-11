#!/usr/bin/env python3
"""Simulation-only ZED BODY_38 -> Unitree G1 23-DOF retargeter.

The official Unitree MuJoCo model is used without DDS or motor commands.  The
default mode advances MuJoCo physics with gravity, contacts, torque-limited PD
control and an explicit soft safety harness.  Kinematic mode is retained only
as a retargeting/debug view.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import time
from pathlib import Path
from typing import Any, Iterator

import mujoco
import numpy as np


G1_23_JOINTS = [
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
]

# Unitree's official g1_23dof FixStand target and gains.  The source contains
# 29 values; the six locked 23-DOF joints are intentionally omitted here.
FIX_STAND = {
    "left_hip_pitch_joint": -0.1,
    "left_hip_roll_joint": 0.0,
    "left_hip_yaw_joint": 0.0,
    "left_knee_joint": 0.3,
    "left_ankle_pitch_joint": -0.2,
    "left_ankle_roll_joint": 0.0,
    "right_hip_pitch_joint": -0.1,
    "right_hip_roll_joint": 0.0,
    "right_hip_yaw_joint": 0.0,
    "right_knee_joint": 0.3,
    "right_ankle_pitch_joint": -0.2,
    "right_ankle_roll_joint": 0.0,
    "waist_yaw_joint": 0.0,
    "left_shoulder_pitch_joint": 0.35,
    "left_shoulder_roll_joint": 0.18,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 0.87,
    "left_wrist_roll_joint": 0.0,
    "right_shoulder_pitch_joint": 0.35,
    "right_shoulder_roll_joint": -0.18,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 0.87,
    "right_wrist_roll_joint": 0.0,
}

LOWER_BODY_JOINTS = {
    name
    for name in G1_23_JOINTS
    if any(part in name for part in ("hip", "knee", "ankle"))
}

# The official XML contains small negative ranges for controller tolerance.
# Human retargeting must not use those ranges as elbow/knee hyper-extension.
RETARGET_LIMITS = {
    "left_elbow_joint": (0.0, 2.50),
    "right_elbow_joint": (0.0, 2.50),
    "left_knee_joint": (0.0, 2.60),
    "right_knee_joint": (0.0, 2.60),
}

# Position-only IK cannot observe rotation about the terminal wrist/ankle axis.
# These joints stay near neutral until orientation retargeting is validated.
IK_JOINTS = [
    name
    for name in G1_23_JOINTS
    if name
    not in {
        "waist_yaw_joint",
        "left_wrist_roll_joint",
        "right_wrist_roll_joint",
        "left_ankle_roll_joint",
        "right_ankle_roll_joint",
    }
]

SIDE_CONFIG = {
    "left": {
        "human": {
            "shoulder": "LEFT_SHOULDER",
            "elbow": "LEFT_ELBOW",
            "wrist": "LEFT_WRIST",
            "hip": "LEFT_HIP",
            "knee": "LEFT_KNEE",
            "ankle": "LEFT_ANKLE",
        },
        "robot": {
            "shoulder": "left_shoulder_pitch_link",
            "elbow": "left_elbow_link",
            "wrist": "left_wrist_roll_rubber_hand",
            "hip": "left_hip_pitch_link",
            "knee": "left_knee_link",
            "ankle": "left_ankle_roll_link",
        },
    },
    "right": {
        "human": {
            "shoulder": "RIGHT_SHOULDER",
            "elbow": "RIGHT_ELBOW",
            "wrist": "RIGHT_WRIST",
            "hip": "RIGHT_HIP",
            "knee": "RIGHT_KNEE",
            "ankle": "RIGHT_ANKLE",
        },
        "robot": {
            "shoulder": "right_shoulder_pitch_link",
            "elbow": "right_elbow_link",
            "wrist": "right_wrist_roll_rubber_hand",
            "hip": "right_hip_pitch_link",
            "knee": "right_knee_link",
            "ankle": "right_ankle_roll_link",
        },
    },
}


def normalize(vector: np.ndarray) -> np.ndarray | None:
    norm = float(np.linalg.norm(vector))
    if not np.isfinite(norm) or norm < 1e-7:
        return None
    return vector / norm


def signed_angle(first: np.ndarray, second: np.ndarray, axis: np.ndarray) -> float:
    first_n = normalize(first)
    second_n = normalize(second)
    axis_n = normalize(axis)
    if first_n is None or second_n is None or axis_n is None:
        return 0.0
    return math.atan2(
        float(np.dot(axis_n, np.cross(first_n, second_n))),
        float(np.clip(np.dot(first_n, second_n), -1.0, 1.0)),
    )


def compact_packet(record: dict[str, Any]) -> dict[str, Any]:
    if record.get("schema") == "zed_body38_live/v1":
        return record
    return {
        "schema": "zed_body38_live/v1",
        "sequence": record.get("frame_index"),
        "timestamp_ns": record.get("timestamp_ns"),
        "coordinate_system": record.get("coordinate_system"),
        "units": record.get("units"),
        "body_id": record.get("body_id"),
        "body_confidence": record.get("body_confidence"),
        "root_position_m": record.get("root_position_m"),
        "global_root_orientation_xyzw": record.get(
            "global_root_orientation_xyzw"
        ),
        "keypoint_names": record.get("keypoint_names"),
        "keypoints_3d_m": record.get("keypoints_3d_filtered_m"),
        "keypoint_confidence": record.get("keypoint_confidence"),
        "local_orientation_per_joint_xyzw": record.get(
            "local_orientation_per_joint_xyzw"
        ),
    }


def replay_packets(path: Path, start_packet: int = 0) -> Iterator[dict[str, Any]]:
    packet_index = 0
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            item = json.loads(line)
            if item.get("schema") == "zed_body38_g1_reference/v1":
                if packet_index < start_packet:
                    packet_index += 1
                    continue
                packet_index += 1
                yield compact_packet(item)


class G1Retargeter:
    def __init__(
        self,
        model_path: Path,
        confidence_threshold: float,
        damping: float,
        iterations: int,
        command_tau: float,
        speed_scale: float,
        arm_kp_scale: float,
        regularization: float,
        segment_memory: float,
    ) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        # IK works on a separate state.  It must never teleport the state used
        # by the physics engine.
        self.ik_data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        mujoco.mj_forward(self.model, self.ik_data)
        self.confidence_threshold = confidence_threshold
        self.damping = damping
        self.iterations = iterations
        self.command_tau = command_tau
        self.speed_scale = speed_scale
        self.arm_kp_scale = arm_kp_scale
        self.regularization = regularization
        self.segment_memory = segment_memory
        self.neutral_qpos = self.data.qpos.copy()
        self.command_qpos = self.data.qpos.copy()
        self.base_qpos = self.data.qpos[:7].copy()

        self.qpos_index: dict[str, int] = {}
        self.dof_index: dict[str, int] = {}
        self.actuator_index: dict[str, int] = {}
        self.joint_limits: dict[str, tuple[float, float]] = {}
        for name in G1_23_JOINTS:
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise ValueError(f"Resmî modelde eklem bulunamadı: {name}")
            self.qpos_index[name] = int(self.model.jnt_qposadr[joint_id])
            self.dof_index[name] = int(self.model.jnt_dofadr[joint_id])
            low, high = self.model.jnt_range[joint_id]
            self.joint_limits[name] = (float(low), float(high))
            matching_actuators = np.flatnonzero(
                self.model.actuator_trnid[:, 0] == joint_id
            )
            if matching_actuators.size != 1:
                raise ValueError(f"Eklem için tek aktüatör bulunamadı: {name}")
            self.actuator_index[name] = int(matching_actuators[0])

        self.ik_names = list(IK_JOINTS)
        self.ik_qpos = np.asarray(
            [self.qpos_index[name] for name in self.ik_names], dtype=int
        )
        self.ik_dofs = np.asarray(
            [self.dof_index[name] for name in self.ik_names], dtype=int
        )
        self.body_ids: dict[str, int] = {}
        for side in SIDE_CONFIG.values():
            for name in side["robot"].values():
                self.body_ids[name] = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_BODY, name
                )
                if self.body_ids[name] < 0:
                    raise ValueError(f"Resmî modelde body bulunamadı: {name}")

        self.lengths: dict[str, dict[str, float]] = {}
        for side_name, side in SIDE_CONFIG.items():
            robot = side["robot"]
            position = {
                key: self.ik_data.xpos[self.body_ids[value]].copy()
                for key, value in robot.items()
            }
            self.lengths[side_name] = {
                "upper_arm": float(
                    np.linalg.norm(position["elbow"] - position["shoulder"])
                ),
                "forearm": float(
                    np.linalg.norm(position["wrist"] - position["elbow"])
                ),
                "thigh": float(
                    np.linalg.norm(position["knee"] - position["hip"])
                ),
                "shank": float(
                    np.linalg.norm(position["ankle"] - position["knee"])
                ),
            }

        self.speed_limits = np.asarray(
            [self._speed_limit(name) for name in G1_23_JOINTS], dtype=float
        ) * self.speed_scale
        nominal_data = mujoco.MjData(self.model)
        nominal_data.qpos[:7] = self.base_qpos
        for name, value in FIX_STAND.items():
            nominal_data.qpos[self.qpos_index[name]] = value
        mujoco.mj_forward(self.model, nominal_data)
        self.nominal_ankle_positions = {
            side: nominal_data.xpos[
                self.body_ids[f"{side}_ankle_roll_link"]
            ].copy()
            for side in ("left", "right")
        }
        self.swing_side: str | None = None
        self.segment_directions: dict[str, dict[str, np.ndarray | None]] = {
            side: {
                segment: None
                for segment in ("upper_arm", "forearm", "thigh", "shank")
            }
            for side in ("left", "right")
        }
        self.segment_ages: dict[str, dict[str, float]] = {
            side: {
                segment: math.inf
                for segment in ("upper_arm", "forearm", "thigh", "shank")
            }
            for side in ("left", "right")
        }
        self.cached_segment_count = 0
        self.last_error_m = math.nan
        self.last_target_count = 0
        self.limit_events = 0
        self.physics_resets = 0
        self.pelvis_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis"
        )
        self.foot_body_ids = {
            side: self.body_ids[f"{side}_ankle_roll_link"] for side in ("left", "right")
        }
        self.floor_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor"
        )
        self.total_mass = float(self.model.body_subtreemass[0])
        self.harness_xy = self.base_qpos[:2].copy()
        self._physics_target = self.command_qpos.copy()
        self.reset_physics()

    @staticmethod
    def _speed_limit(name: str) -> float:
        if "waist" in name:
            return 0.8
        if "ankle" in name:
            return 1.0
        if "hip" in name or "knee" in name:
            return 1.4
        if "wrist" in name:
            return 1.2
        return 1.8

    def _clip_active(
        self, qpos: np.ndarray, count_events: bool = True
    ) -> np.ndarray:
        for name in G1_23_JOINTS:
            index = self.qpos_index[name]
            low, high = self.joint_limits[name]
            if name in RETARGET_LIMITS:
                retarget_low, retarget_high = RETARGET_LIMITS[name]
                low = max(low, retarget_low)
                high = min(high, retarget_high)
            before = float(qpos[index])
            qpos[index] = np.clip(qpos[index], low, high)
            if count_events:
                self.limit_events += int(float(qpos[index]) != before)
        return qpos

    def _human_data(
        self, packet: dict[str, Any]
    ) -> tuple[dict[str, np.ndarray], dict[str, float], np.ndarray] | None:
        if (
            packet.get("schema") != "zed_body38_live/v1"
            or packet.get("coordinate_system") != "RIGHT_HANDED_Z_UP_X_FWD"
            or packet.get("units") != "meter"
        ):
            return None
        try:
            if float(packet.get("body_confidence", 0.0)) < self.confidence_threshold:
                return None
        except (TypeError, ValueError):
            return None
        names = packet.get("keypoint_names")
        points_raw = packet.get("keypoints_3d_m")
        confidence_raw = packet.get("keypoint_confidence")
        if not (
            isinstance(names, list)
            and isinstance(points_raw, list)
            and isinstance(confidence_raw, list)
            and len(names) == len(points_raw) == len(confidence_raw) == 38
        ):
            return None
        points_array = np.asarray(
            [
                value if isinstance(value, list) and len(value) >= 3 else [math.nan] * 3
                for value in points_raw
            ],
            dtype=float,
        )[:, :3]
        confidence_array = np.asarray(confidence_raw, dtype=float)
        points = {name: points_array[index] for index, name in enumerate(names)}
        confidence = {
            name: float(confidence_array[index]) for index, name in enumerate(names)
        }
        required = [
            "PELVIS",
            "SPINE_3",
            "LEFT_SHOULDER",
            "RIGHT_SHOULDER",
            "LEFT_HIP",
            "RIGHT_HIP",
        ]
        if any(
            name not in points
            or not np.isfinite(points[name]).all()
            or confidence[name] < self.confidence_threshold
            for name in required
        ):
            return None

        left = normalize(
            (points["LEFT_SHOULDER"] - points["RIGHT_SHOULDER"])
            + (points["LEFT_HIP"] - points["RIGHT_HIP"])
        )
        up = normalize(points["SPINE_3"] - points["PELVIS"])
        if left is None or up is None:
            return None
        forward = normalize(np.cross(left, up))
        if forward is None:
            return None
        left = normalize(np.cross(up, forward))
        if left is None:
            return None
        basis = np.column_stack((forward, left, up))
        return points, confidence, basis

    def _direction(
        self,
        points: dict[str, np.ndarray],
        confidence: dict[str, float],
        basis: np.ndarray,
        first: str,
        second: str,
    ) -> np.ndarray | None:
        if (
            confidence.get(first, 0.0) < self.confidence_threshold
            or confidence.get(second, 0.0) < self.confidence_threshold
        ):
            return None
        if not (
            np.isfinite(points.get(first, np.full(3, np.nan))).all()
            and np.isfinite(points.get(second, np.full(3, np.nan))).all()
        ):
            return None
        return normalize(basis.T @ (points[second] - points[first]))

    def _remember_direction(
        self,
        side: str,
        segment: str,
        direction: np.ndarray | None,
        dt: float,
    ) -> np.ndarray | None:
        if direction is not None:
            self.segment_directions[side][segment] = direction
            self.segment_ages[side][segment] = 0.0
            return direction
        self.segment_ages[side][segment] += dt
        cached = self.segment_directions[side][segment]
        if cached is not None and self.segment_ages[side][segment] <= self.segment_memory:
            self.cached_segment_count += 1
            return cached
        return None

    def _waist_yaw(
        self,
        points: dict[str, np.ndarray],
        confidence: dict[str, float],
        basis: np.ndarray,
    ) -> float:
        names = ["LEFT_SHOULDER", "RIGHT_SHOULDER", "LEFT_HIP", "RIGHT_HIP"]
        if any(confidence.get(name, 0.0) < self.confidence_threshold for name in names):
            return 0.0
        shoulder_line = basis.T @ (
            points["LEFT_SHOULDER"] - points["RIGHT_SHOULDER"]
        )
        hip_line = basis.T @ (points["LEFT_HIP"] - points["RIGHT_HIP"])
        angle = signed_angle(hip_line, shoulder_line, np.array([0.0, 0.0, 1.0]))
        return float(np.clip(angle, -0.6, 0.6))

    def solve(
        self, packet: dict[str, Any], dt: float, lower_body: str = "grounded"
    ) -> bool:
        parsed = self._human_data(packet)
        if parsed is None:
            return False
        points, confidence, basis = parsed
        qpos = self.command_qpos.copy()
        qpos[:7] = self.base_qpos
        waist_index = self.qpos_index["waist_yaw_joint"]
        qpos[waist_index] = self._waist_yaw(points, confidence, basis)
        active_ik_names = [
            name
            for name in self.ik_names
            if lower_body != "hold" or name not in LOWER_BODY_JOINTS
        ]
        active_ik_qpos = np.asarray(
            [self.qpos_index[name] for name in active_ik_names], dtype=int
        )
        active_ik_dofs = np.asarray(
            [self.dof_index[name] for name in active_ik_names], dtype=int
        )

        limb_directions: dict[str, dict[str, np.ndarray | None]] = {}
        self.cached_segment_count = 0
        for side_name, side in SIDE_CONFIG.items():
            human = side["human"]
            raw_directions = {
                "upper_arm": self._direction(
                    points, confidence, basis, human["shoulder"], human["elbow"]
                ),
                "forearm": self._direction(
                    points, confidence, basis, human["elbow"], human["wrist"]
                ),
                "thigh": self._direction(
                    points, confidence, basis, human["hip"], human["knee"]
                ),
                "shank": self._direction(
                    points, confidence, basis, human["knee"], human["ankle"]
                ),
            }
            limb_directions[side_name] = {
                segment: self._remember_direction(
                    side_name, segment, direction, dt
                )
                for segment, direction in raw_directions.items()
            }

        if lower_body == "grounded":
            ankle_heights: dict[str, float] = {}
            for side_name, side in SIDE_CONFIG.items():
                human = side["human"]
                ankle = human["ankle"]
                if (
                    confidence.get(ankle, 0.0) >= self.confidence_threshold
                    and np.isfinite(points[ankle]).all()
                ):
                    ankle_heights[side_name] = float(
                        (basis.T @ (points[ankle] - points["PELVIS"]))[2]
                    )
            if len(ankle_heights) == 2:
                height_delta = ankle_heights["left"] - ankle_heights["right"]
                if abs(height_delta) >= 0.035:
                    self.swing_side = "left" if height_delta > 0.0 else "right"
                elif abs(height_delta) <= 0.015:
                    self.swing_side = None
        elif lower_body == "hold":
            self.swing_side = None

        target_count = 0
        final_errors: list[float] = []
        for _ in range(self.iterations):
            self.ik_data.qpos[:] = qpos
            self.ik_data.qvel[:] = 0.0
            mujoco.mj_forward(self.model, self.ik_data)
            residuals: list[np.ndarray] = []
            jacobians: list[np.ndarray] = []
            final_errors = []
            target_count = 0

            for side_name, side in SIDE_CONFIG.items():
                robot = side["robot"]
                directions = limb_directions[side_name]
                lengths = self.lengths[side_name]

                shoulder_position = self.ik_data.xpos[
                    self.body_ids[robot["shoulder"]]
                ].copy()
                if (
                    directions["upper_arm"] is not None
                    and directions["forearm"] is not None
                ):
                    elbow_target = (
                        shoulder_position
                        + directions["upper_arm"] * lengths["upper_arm"]
                    )
                    wrist_target = (
                        elbow_target + directions["forearm"] * lengths["forearm"]
                    )
                    target_specs = [
                        (robot["elbow"], elbow_target, 1.35),
                        (robot["wrist"], wrist_target, 1.80),
                    ]
                else:
                    target_specs = []

                hip_position = self.ik_data.xpos[
                    self.body_ids[robot["hip"]]
                ].copy()
                if lower_body != "hold" and (
                    directions["thigh"] is not None
                    and directions["shank"] is not None
                ):
                    knee_target = hip_position + directions["thigh"] * lengths["thigh"]
                    ankle_target = knee_target + directions["shank"] * lengths["shank"]
                    if lower_body == "grounded":
                        nominal_ankle = self.nominal_ankle_positions[side_name]
                        is_swing = self.swing_side == side_name
                        horizontal_limit = 0.32 if is_swing else 0.07
                        ankle_target[:2] = np.clip(
                            ankle_target[:2],
                            nominal_ankle[:2] - horizontal_limit,
                            nominal_ankle[:2] + horizontal_limit,
                        )
                        if is_swing:
                            other_side = "right" if side_name == "left" else "left"
                            lift = max(
                                0.0,
                                ankle_heights.get(side_name, -math.inf)
                                - ankle_heights.get(other_side, math.inf)
                                - 0.03,
                            )
                            ankle_target[2] = nominal_ankle[2] + min(
                                0.32, 0.95 * lift
                            )
                        else:
                            # At least one support foot remains on the floor.
                            ankle_target[2] = nominal_ankle[2]
                    target_specs.extend(
                        [
                            (robot["knee"], knee_target, 1.0),
                            (robot["ankle"], ankle_target, 1.45),
                        ]
                    )

                for body_name, target, weight in target_specs:
                    body_id = self.body_ids[body_name]
                    error = target - self.ik_data.xpos[body_id]
                    jac_position = np.zeros((3, self.model.nv))
                    jac_rotation = np.zeros((3, self.model.nv))
                    mujoco.mj_jacBody(
                        self.model,
                        self.ik_data,
                        jac_position,
                        jac_rotation,
                        body_id,
                    )
                    residuals.append(weight * error)
                    jacobians.append(weight * jac_position[:, active_ik_dofs])
                    final_errors.append(float(np.linalg.norm(error)))
                    target_count += 1

            if not residuals:
                return False
            jacobian = np.vstack(jacobians)
            error_vector = np.concatenate(residuals)
            regularization = self.regularization
            augmented_jacobian = np.vstack(
                (
                    jacobian,
                    math.sqrt(regularization) * np.eye(len(active_ik_names)),
                )
            )
            neutral_error = (
                self.neutral_qpos[active_ik_qpos] - qpos[active_ik_qpos]
            )
            augmented_error = np.concatenate(
                (error_vector, math.sqrt(regularization) * neutral_error)
            )
            normal = (
                augmented_jacobian.T @ augmented_jacobian
                + (self.damping**2) * np.eye(len(active_ik_names))
            )
            delta = np.linalg.solve(
                normal, augmented_jacobian.T @ augmented_error
            )
            delta = np.clip(delta, -0.22, 0.22)
            qpos[active_ik_qpos] += delta
            self._clip_active(qpos, count_events=False)
            if float(np.linalg.norm(error_vector)) < 0.005:
                break

        active_qpos = np.asarray(
            [self.qpos_index[name] for name in G1_23_JOINTS], dtype=int
        )
        previous = self.command_qpos[active_qpos]
        desired = qpos[active_qpos]
        maximum_delta = self.speed_limits * max(1.0 / 120.0, min(dt, 0.1))
        velocity_limited = np.clip(desired, previous - maximum_delta, previous + maximum_delta)
        smoothing = (
            1.0
            if self.command_tau <= 0.0
            else 1.0 - math.exp(-max(dt, 1.0 / 240.0) / self.command_tau)
        )
        self.command_qpos[active_qpos] = (
            previous + smoothing * (velocity_limited - previous)
        )
        self.command_qpos[:7] = self.base_qpos
        self._clip_active(self.command_qpos)
        self.last_error_m = float(np.mean(final_errors)) if final_errors else math.nan
        self.last_target_count = target_count
        return True

    def move_to_neutral(self, dt: float) -> None:
        indexes = np.asarray(
            [self.qpos_index[name] for name in G1_23_JOINTS], dtype=int
        )
        current = self.command_qpos[indexes]
        target = np.asarray([FIX_STAND[name] for name in G1_23_JOINTS])
        delta = np.clip(target - current, -0.5 * dt, 0.5 * dt)
        self.command_qpos[indexes] = current + delta
        self.command_qpos[:7] = self.base_qpos
        self.last_target_count = 0
        self.last_error_m = math.nan

    def apply_kinematic(self) -> None:
        """Debug-only pose display; this deliberately does not step physics."""
        self.data.qpos[:] = self.command_qpos
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def target_qpos(self, lower_body: str) -> np.ndarray:
        target = self.command_qpos.copy()
        if lower_body == "hold":
            for name in LOWER_BODY_JOINTS:
                target[self.qpos_index[name]] = FIX_STAND[name]
        # Keep the two locked waist and four locked wrist joints at zero.
        return self._clip_active(target, count_events=False)

    def _pd_gain(self, name: str) -> tuple[float, float]:
        # Exact gains from Unitree's official g1_23dof FixStand configuration.
        if "knee" in name:
            return 150.0, 4.0
        if "ankle" in name:
            return 40.0, 2.0
        if "hip" in name:
            return 100.0, 2.0
        if "waist" in name:
            return 200.0, 5.0
        return 40.0 * self.arm_kp_scale, 10.0

    def reset_physics(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        for name, value in FIX_STAND.items():
            self.data.qpos[self.qpos_index[name]] = value
            self.command_qpos[self.qpos_index[name]] = value
        # Start only slightly above nominal so gravity establishes floor contact.
        self.data.qpos[2] = max(float(self.base_qpos[2]), 0.80)
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _foot_contacts(self) -> tuple[bool, bool]:
        result = {"left": False, "right": False}
        if self.floor_geom_id < 0:
            return False, False
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            if contact.geom1 == self.floor_geom_id:
                other_geom = contact.geom2
            elif contact.geom2 == self.floor_geom_id:
                other_geom = contact.geom1
            else:
                continue
            body_id = int(self.model.geom_bodyid[other_geom])
            for side, foot_id in self.foot_body_ids.items():
                if body_id == foot_id:
                    result[side] = True
        return result["left"], result["right"]

    def physics_status(self) -> tuple[float, float, float, bool, bool]:
        left_contact, right_contact = self._foot_contacts()
        return (
            float(self.data.xpos[self.pelvis_id, 2]),
            float(self.data.xpos[self.foot_body_ids["left"], 2]),
            float(self.data.xpos[self.foot_body_ids["right"], 2]),
            left_contact,
            right_contact,
        )

    def step_physics(
        self,
        target: np.ndarray,
        elapsed: float,
        harness_support: float,
        harness_xy_kp: float,
        harness_upright_kp: float,
    ) -> None:
        """Advance free-base MuJoCo dynamics using forces, never qpos teleporting."""
        elapsed = float(np.clip(elapsed, self.model.opt.timestep, 0.05))
        substeps = max(1, int(round(elapsed / self.model.opt.timestep)))
        gravity = abs(float(self.model.opt.gravity[2]))
        target = target.copy()
        for _ in range(substeps):
            for name in G1_23_JOINTS:
                actuator = self.actuator_index[name]
                q_index = self.qpos_index[name]
                v_index = self.dof_index[name]
                kp, kd = self._pd_gain(name)
                torque = (
                    kp * (float(target[q_index]) - float(self.data.qpos[q_index]))
                    - kd * float(self.data.qvel[v_index])
                )
                low, high = self.model.actuator_ctrlrange[actuator]
                self.data.ctrl[actuator] = np.clip(torque, low, high)

            velocity = np.zeros(6)
            mujoco.mj_objectVelocity(
                self.model,
                self.data,
                mujoco.mjtObj.mjOBJ_BODY,
                self.pelvis_id,
                velocity,
                0,
            )
            pelvis_z_axis = self.data.xmat[self.pelvis_id].reshape(3, 3)[:, 2]
            upright_error = np.cross(pelvis_z_axis, np.array([0.0, 0.0, 1.0]))
            force = np.array(
                [
                    harness_xy_kp
                    * (self.harness_xy[0] - self.data.xpos[self.pelvis_id, 0])
                    - 18.0 * velocity[3],
                    harness_xy_kp
                    * (self.harness_xy[1] - self.data.xpos[self.pelvis_id, 1])
                    - 18.0 * velocity[4],
                    harness_support * self.total_mass * gravity,
                ]
            )
            torque = np.array(
                [
                    harness_upright_kp * upright_error[0] - 12.0 * velocity[0],
                    harness_upright_kp * upright_error[1] - 12.0 * velocity[1],
                    -3.0 * velocity[2],
                ]
            )
            self.data.xfrc_applied[:] = 0.0
            self.data.xfrc_applied[self.pelvis_id, :3] = force
            self.data.xfrc_applied[self.pelvis_id, 3:] = torque
            mujoco.mj_step(self.model, self.data)

        pelvis_z = float(self.data.xpos[self.pelvis_id, 2])
        up_dot = float(
            np.dot(
                self.data.xmat[self.pelvis_id].reshape(3, 3)[:, 2],
                np.array([0.0, 0.0, 1.0]),
            )
        )
        if pelvis_z < 0.48 or up_dot < 0.45 or not np.isfinite(self.data.qpos).all():
            self.physics_resets += 1
            self.reset_physics()


def receive_packet(sock: socket.socket) -> dict[str, Any] | None:
    latest = None
    while True:
        try:
            payload, _ = sock.recvfrom(65535)
            latest = json.loads(payload.decode("utf-8"))
        except BlockingIOError:
            return latest
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ZED BODY_38 ile resmî G1-23DOF MuJoCo kinematik taklidi."
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=(
            Path.home() / "ros2_ws" / "src" / "unitree_mujoco"
            / "unitree_robots" / "g1" / "scene_23dof.xml"
        ),
    )
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=15050)
    parser.add_argument("--confidence", type=float, default=50.0)
    parser.add_argument("--stale-timeout", type=float, default=0.5)
    parser.add_argument("--ik-iterations", type=int, default=8)
    parser.add_argument("--ik-damping", type=float, default=0.04)
    parser.add_argument(
        "--ik-regularization",
        type=float,
        default=0.008,
        help="Nötr poza çekim ağırlığı; düşük değer uç hedeflerini daha iyi izler",
    )
    parser.add_argument(
        "--segment-memory",
        type=float,
        default=0.18,
        help="Tek kamera örtülmesinde son geçerli uzuv yönünü koruma süresi",
    )
    parser.add_argument("--replay-jsonl", type=Path, default=None)
    parser.add_argument("--replay-fps", type=float, default=30.0)
    parser.add_argument(
        "--replay-start",
        type=int,
        default=0,
        help="Kayıt testinde atlanacak geçerli iskelet paketi sayısı",
    )
    parser.add_argument(
        "--mode",
        choices=("physics", "kinematic"),
        default="physics",
        help="physics: yerçekimi/temas/tork; kinematic: yalnızca IK hata ayıklama",
    )
    parser.add_argument(
        "--lower-body",
        choices=("hold", "grounded", "mimic"),
        default="grounded",
        help=(
            "hold FixStand; grounded temas korumalı yerinde tam-vücut; "
            "mimic serbest deneysel IK"
        ),
    )
    parser.add_argument(
        "--command-tau",
        type=float,
        default=0.025,
        help="IK komut filtresi zaman sabiti, saniye (0=filtresiz)",
    )
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=1.6,
        help="Yalnız simülasyondaki eklem hız sınırı çarpanı",
    )
    parser.add_argument(
        "--arm-kp-scale",
        type=float,
        default=1.6,
        help="Yalnız simülasyondaki kol PD Kp çarpanı",
    )
    parser.add_argument(
        "--harness-support",
        type=float,
        default=0.55,
        help="Ağırlığın askı tarafından taşınan oranı (0..0.9)",
    )
    parser.add_argument("--harness-xy-kp", type=float, default=140.0)
    parser.add_argument("--harness-upright-kp", type=float, default=90.0)
    parser.add_argument(
        "--ros-sensors",
        action="store_true",
        help="Mesh, joint_states, TF, Mid-360 ve depth verilerini ROS 2'ye yayınla",
    )
    parser.add_argument(
        "--ros-state-rate",
        type=float,
        default=20.0,
        help="Mesh, joint_states ve TF yayın hızı",
    )
    parser.add_argument(
        "--ros-sensor-rate",
        type=float,
        default=5.0,
        help="Pahalı LiDAR/depth ray-cast yayın hızı",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument(
        "--max-seconds",
        type=float,
        default=0.0,
        help="Test için belirtilen duvar süresinden sonra çık",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.model.is_file():
        print(f"MuJoCo modeli bulunamadı: {args.model}")
        return 2
    if not 0 <= args.confidence <= 100 or not 1 <= args.port <= 65535:
        print("Confidence veya port geçersiz.")
        return 2
    if not 0.0 <= args.harness_support <= 0.9:
        print("--harness-support 0 ile 0.9 arasında olmalı.")
        return 2
    if (
        args.command_tau < 0.0
        or args.speed_scale <= 0.0
        or args.arm_kp_scale <= 0.0
        or args.ik_regularization < 0.0
        or args.segment_memory < 0.0
    ):
        print("Komut filtresi/hız/kol kazancı parametreleri geçersiz.")
        return 2

    retargeter = G1Retargeter(
        args.model,
        args.confidence,
        args.ik_damping,
        args.ik_iterations,
        args.command_tau,
        args.speed_scale,
        args.arm_kp_scale,
        args.ik_regularization,
        args.segment_memory,
    )
    ros_bridge = None
    if args.ros_sensors:
        bridge_directory = (
            Path.home() / "ros2_ws" / "src" / "unitree_mujoco"
            / "simulate_python"
        )
        sys.path.insert(0, str(bridge_directory))
        try:
            from ros_sensor_bridge import MujocoRosSensors

            ros_bridge = MujocoRosSensors(retargeter.model, retargeter.data)
            print(
                "ROS 2: /g1/robot_mesh, /joint_states, /livox/lidar ve "
                "/camera/camera/depth/* yayınlanıyor."
            )
        except Exception as error:
            print(f"ROS 2 sensör köprüsü başlatılamadı: {error}")
            return 2
    packet_iterator = (
        replay_packets(args.replay_jsonl, args.replay_start)
        if args.replay_jsonl is not None
        else None
    )
    sock = None
    if packet_iterator is None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((args.bind, args.port))
        sock.setblocking(False)
        print(f"ZED UDP bekleniyor: {args.bind}:{args.port}")
    else:
        print(f"Kayıt oynatılıyor: {args.replay_jsonl}")

    viewer = None
    if not args.headless:
        import mujoco.viewer

        viewer = mujoco.viewer.launch_passive(retargeter.model, retargeter.data)
        viewer.cam.lookat[:] = np.array([0.0, 0.0, 0.8])
        viewer.cam.distance = 2.4
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -15

    processed = 0
    accepted = 0
    previous_report_processed = 0
    previous_report_accepted = 0
    ik_error_history: list[float] = []
    last_transport_time = time.monotonic()
    last_valid_packet_time = last_transport_time
    last_step_time = last_transport_time
    last_report = last_transport_time
    started_at = last_transport_time
    replay_period = 1.0 / max(args.replay_fps, 1.0)
    ros_state_period = 1.0 / max(args.ros_state_rate, 0.5)
    ros_sensor_period = 1.0 / max(args.ros_sensor_rate, 0.2)
    last_ros_state_publish = started_at - ros_state_period
    last_ros_sensor_publish = started_at - ros_sensor_period
    if args.mode == "physics":
        print(
            "Fizik modu: serbest taban + yerçekimi + temas + tork sınırlı PD; "
            f"lower_body={args.lower_body}, harness={args.harness_support:.2f}"
        )
    else:
        print("UYARI: kinematic modu fizik uygulamaz; yalnızca IK görsel testidir.")
    try:
        while viewer is None or viewer.is_running():
            loop_started = time.monotonic()
            dt = max(1.0 / 120.0, min(loop_started - last_step_time, 0.1))
            last_step_time = loop_started
            packet = None
            if packet_iterator is not None:
                try:
                    packet = next(packet_iterator)
                except StopIteration:
                    break
            elif sock is not None:
                packet = receive_packet(sock)

            if packet is not None:
                processed += 1
                last_transport_time = loop_started
                if (
                    packet.get("schema") == "zed_body38_live/v1"
                    and retargeter.solve(packet, dt, args.lower_body)
                ):
                    accepted += 1
                    if math.isfinite(retargeter.last_error_m):
                        ik_error_history.append(retargeter.last_error_m)
                    last_valid_packet_time = loop_started
            if loop_started - last_valid_packet_time > args.stale_timeout:
                retargeter.move_to_neutral(dt)

            if args.mode == "physics":
                retargeter.step_physics(
                    retargeter.target_qpos(args.lower_body),
                    dt,
                    args.harness_support,
                    args.harness_xy_kp,
                    args.harness_upright_kp,
                )
            else:
                retargeter.apply_kinematic()

            if ros_bridge is not None:
                sensor_due = (
                    loop_started - last_ros_sensor_publish >= ros_sensor_period
                )
                state_due = (
                    loop_started - last_ros_state_publish >= ros_state_period
                )
                if sensor_due or state_due:
                    ros_bridge.publish(include_sensors=sensor_due)
                    last_ros_state_publish = loop_started
                    if sensor_due:
                        last_ros_sensor_publish = loop_started

            if viewer is not None:
                viewer.sync()
            if loop_started - last_report >= 1.0:
                report_period = max(loop_started - last_report, 1e-6)
                rx_rate = (processed - previous_report_processed) / report_period
                accepted_rate = (
                    accepted - previous_report_accepted
                ) / report_period
                transport_age = loop_started - last_transport_time
                valid_age = loop_started - last_valid_packet_time
                if processed == 0:
                    input_state = "WAITING"
                elif transport_age > args.stale_timeout:
                    input_state = "STALE"
                elif valid_age > args.stale_timeout:
                    input_state = "NO_BODY"
                else:
                    input_state = "LIVE"
                pelvis_z, left_z, right_z, left_contact, right_contact = (
                    retargeter.physics_status()
                )
                maximum_targets = 4 if args.lower_body == "hold" else 8
                print(
                    f"input={input_state} transport_age={transport_age:.2f}s "
                    f"valid_age={valid_age:.2f}s "
                    f"rx={processed} ({rx_rate:.1f}/s) "
                    f"accepted={accepted} ({accepted_rate:.1f}/s) "
                    f"targets={retargeter.last_target_count}/{maximum_targets} "
                    f"mean_ik_error={retargeter.last_error_m:.4f}m "
                    f"swing={retargeter.swing_side or '-'} "
                    f"cached={retargeter.cached_segment_count} "
                    f"limit_events={retargeter.limit_events} "
                    f"pelvis_z={pelvis_z:.3f} "
                    f"ankle_z=({left_z:.3f},{right_z:.3f}) "
                    f"foot_contact=({int(left_contact)},{int(right_contact)}) "
                    f"resets={retargeter.physics_resets}"
                )
                previous_report_processed = processed
                previous_report_accepted = accepted
                last_report = loop_started
            if args.max_frames and processed >= args.max_frames:
                break
            if args.max_seconds > 0 and loop_started - started_at >= args.max_seconds:
                break
            if packet_iterator is not None:
                time.sleep(max(0.0, replay_period - (time.monotonic() - loop_started)))
            else:
                time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        if sock is not None:
            sock.close()
        if ros_bridge is not None:
            ros_bridge.close()

    mean_error = (
        float(np.mean(ik_error_history)) if ik_error_history else math.nan
    )
    p95_error = (
        float(np.percentile(ik_error_history, 95))
        if ik_error_history
        else math.nan
    )
    print(
        f"Tamamlandı: alınan={processed}, kabul={accepted} "
        f"(%{100.0 * accepted / processed if processed else 0.0:.1f}), "
        f"IK ortalama={mean_error:.4f} m, p95={p95_error:.4f} m, "
        f"limit_events={retargeter.limit_events}"
    )
    print("SIMULATION ONLY: DDS veya fiziksel robota komut gönderilmedi.")
    result = 0 if accepted > 0 else 3
    if viewer is not None:
        # MuJoCo's GLFW teardown can trigger an MIT-SHM BadDrawable fault on
        # WSLg after the window has already closed. The process owns no robot
        # or file resources, so terminate after flushing instead of re-entering
        # the faulty X11 teardown path.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(result)
    return result


if __name__ == "__main__":
    raise SystemExit(main())

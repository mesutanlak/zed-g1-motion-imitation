#!/usr/bin/env python3
"""
ZED 2i BODY_38 skeleton extractor prepared for Unitree G1 23-DOF retargeting.

This program is perception-only. It never connects to or commands a robot.

Official references:
  https://www.stereolabs.com/docs/development/zed-sdk/modules/body-tracking
  https://www.stereolabs.com/docs/development/zed-sdk/modules/body-tracking/using-the-api
  https://github.com/stereolabs/zed-sdk/tree/master/tutorials/tutorial%208%20-%20body%20tracking/python
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import socket
import sys
import time
from typing import Any

try:
    import msvcrt
except ImportError:  # Windows console hotkeys are optional on other platforms.
    msvcrt = None

from motion_pipeline.arm_chain import ArmChainOptimizer
from motion_pipeline.calibration import CalibrationManager, to_pelvis_local
from motion_pipeline.metrics import PerceptionMetrics
from motion_pipeline.operator_selector import OperatorSelector, OperatorState


def _configure_windows_dll_search() -> list[Any]:
    """Keep os.add_dll_directory handles alive for the complete process."""
    handles: list[Any] = []
    if sys.platform != "win32" or not hasattr(os, "add_dll_directory"):
        return handles

    candidates = [
        os.environ.get("ZED_SDK_ROOT_DIR", r"C:\Program Files (x86)\ZED SDK")
        + r"\bin",
        os.environ.get(
            "CUDA_PATH", r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.3"
        )
        + r"\bin\x64",
        os.environ.get(
            "CUDA_PATH", r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.3"
        )
        + r"\bin",
    ]
    for directory in candidates:
        if os.path.isdir(directory):
            handles.append(os.add_dll_directory(directory))
    return handles


_DLL_HANDLES = _configure_windows_dll_search()

try:
    import cv2
    import numpy as np
    import pyzed.sl as sl
except (ImportError, OSError) as exc:
    print(
        "ZED Python bağımlılıkları yüklenemedi.\n"
        "ZED SDK Python API, numpy ve opencv-python kurulmuş olmalıdır.\n"
        f"Ayrıntı: {exc}",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


# Official BODY_38 order from sl.BODY_38_PARTS in ZED SDK 5.4.
BODY38_NAMES = [
    "PELVIS",
    "SPINE_1",
    "SPINE_2",
    "SPINE_3",
    "NECK",
    "NOSE",
    "LEFT_EYE",
    "RIGHT_EYE",
    "LEFT_EAR",
    "RIGHT_EAR",
    "LEFT_CLAVICLE",
    "RIGHT_CLAVICLE",
    "LEFT_SHOULDER",
    "RIGHT_SHOULDER",
    "LEFT_ELBOW",
    "RIGHT_ELBOW",
    "LEFT_WRIST",
    "RIGHT_WRIST",
    "LEFT_HIP",
    "RIGHT_HIP",
    "LEFT_KNEE",
    "RIGHT_KNEE",
    "LEFT_ANKLE",
    "RIGHT_ANKLE",
    "LEFT_BIG_TOE",
    "RIGHT_BIG_TOE",
    "LEFT_SMALL_TOE",
    "RIGHT_SMALL_TOE",
    "LEFT_HEEL",
    "RIGHT_HEEL",
    "LEFT_HAND_THUMB_4",
    "RIGHT_HAND_THUMB_4",
    "LEFT_HAND_INDEX_1",
    "RIGHT_HAND_INDEX_1",
    "LEFT_HAND_MIDDLE_4",
    "RIGHT_HAND_MIDDLE_4",
    "LEFT_HAND_PINKY_1",
    "RIGHT_HAND_PINKY_1",
]
IDX = {name: index for index, name in enumerate(BODY38_NAMES)}

# Display-only links. Retargeting uses the numeric data, not these lines.
BODY38_EDGES = [
    ("PELVIS", "SPINE_1"),
    ("SPINE_1", "SPINE_2"),
    ("SPINE_2", "SPINE_3"),
    ("SPINE_3", "NECK"),
    ("NECK", "NOSE"),
    ("NOSE", "LEFT_EYE"),
    ("LEFT_EYE", "LEFT_EAR"),
    ("NOSE", "RIGHT_EYE"),
    ("RIGHT_EYE", "RIGHT_EAR"),
    ("SPINE_3", "LEFT_CLAVICLE"),
    ("LEFT_CLAVICLE", "LEFT_SHOULDER"),
    ("LEFT_SHOULDER", "LEFT_ELBOW"),
    ("LEFT_ELBOW", "LEFT_WRIST"),
    ("SPINE_3", "RIGHT_CLAVICLE"),
    ("RIGHT_CLAVICLE", "RIGHT_SHOULDER"),
    ("RIGHT_SHOULDER", "RIGHT_ELBOW"),
    ("RIGHT_ELBOW", "RIGHT_WRIST"),
    ("PELVIS", "LEFT_HIP"),
    ("LEFT_HIP", "LEFT_KNEE"),
    ("LEFT_KNEE", "LEFT_ANKLE"),
    ("LEFT_ANKLE", "LEFT_BIG_TOE"),
    ("LEFT_ANKLE", "LEFT_SMALL_TOE"),
    ("LEFT_ANKLE", "LEFT_HEEL"),
    ("PELVIS", "RIGHT_HIP"),
    ("RIGHT_HIP", "RIGHT_KNEE"),
    ("RIGHT_KNEE", "RIGHT_ANKLE"),
    ("RIGHT_ANKLE", "RIGHT_BIG_TOE"),
    ("RIGHT_ANKLE", "RIGHT_SMALL_TOE"),
    ("RIGHT_ANKLE", "RIGHT_HEEL"),
    ("LEFT_WRIST", "LEFT_HAND_THUMB_4"),
    ("LEFT_WRIST", "LEFT_HAND_INDEX_1"),
    ("LEFT_WRIST", "LEFT_HAND_MIDDLE_4"),
    ("LEFT_WRIST", "LEFT_HAND_PINKY_1"),
    ("RIGHT_WRIST", "RIGHT_HAND_THUMB_4"),
    ("RIGHT_WRIST", "RIGHT_HAND_INDEX_1"),
    ("RIGHT_WRIST", "RIGHT_HAND_MIDDLE_4"),
    ("RIGHT_WRIST", "RIGHT_HAND_PINKY_1"),
]

# Exact order verified from Unitree's official g1_23dof.xml.
G1_23DOF_JOINT_ORDER = [
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

DEX3_JOINT_ORDER = [
    "thumb_0",
    "thumb_1",
    "thumb_2",
    "middle_0",
    "middle_1",
    "index_0",
    "index_1",
]

# Unitree SDK2 g1_dex3_example.cpp limits. They are metadata for the future
# retargeter; this perception program does not publish them as commands.
DEX3_LIMITS_RAD = {
    "left": {
        "min": [-1.05, -0.724, 0.0, -1.57, -1.75, -1.57, -1.75],
        "max": [1.05, 1.05, 1.75, 0.0, 0.0, 0.0, 0.0],
    },
    "right": {
        "min": [-1.05, -1.05, -1.75, 0.0, 0.0, 0.0, 0.0],
        "max": [1.05, 0.742, 0.0, 1.57, 1.75, 1.57, 1.75],
    },
}

CRITICAL_GROUPS = {
    "torso": ["PELVIS", "SPINE_2", "SPINE_3", "LEFT_HIP", "RIGHT_HIP"],
    "left_arm": ["LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST"],
    "right_arm": ["RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST"],
    "left_leg": ["LEFT_HIP", "LEFT_KNEE", "LEFT_ANKLE", "LEFT_HEEL", "LEFT_BIG_TOE"],
    "right_leg": [
        "RIGHT_HIP",
        "RIGHT_KNEE",
        "RIGHT_ANKLE",
        "RIGHT_HEEL",
        "RIGHT_BIG_TOE",
    ],
}


class ConfidenceAwareLowPass:
    """Low-pass valid keypoints without inventing values for missing detections."""

    def __init__(self, time_constant_s: float) -> None:
        self.time_constant_s = max(float(time_constant_s), 0.0)
        self._states: dict[int, np.ndarray] = {}
        self._last_t: dict[int, float] = {}

    def reset(self, body_id: int | None = None) -> None:
        if body_id is None:
            self._states.clear()
            self._last_t.clear()
        else:
            self._states.pop(body_id, None)
            self._last_t.pop(body_id, None)

    def update(
        self,
        body_id: int,
        values: np.ndarray,
        confidence: np.ndarray,
        min_confidence: float,
        timestamp_s: float,
    ) -> np.ndarray:
        raw = np.asarray(values, dtype=np.float64)
        output = raw.copy()
        valid = np.isfinite(raw).all(axis=1) & (confidence >= min_confidence)

        previous = self._states.get(body_id)
        last_t = self._last_t.get(body_id)
        if previous is None or last_t is None or previous.shape != raw.shape:
            state = raw.copy()
        else:
            dt = max(timestamp_s - last_t, 1e-4)
            alpha = 1.0 if self.time_constant_s == 0.0 else dt / (
                self.time_constant_s + dt
            )
            state = previous.copy()
            can_blend = valid & np.isfinite(previous).all(axis=1)
            state[can_blend] = (
                previous[can_blend]
                + alpha * (raw[can_blend] - previous[can_blend])
            )
            state[valid & ~np.isfinite(previous).all(axis=1)] = raw[
                valid & ~np.isfinite(previous).all(axis=1)
            ]

        # Missing points remain missing. Previous positions are not replayed as live data.
        output[valid] = state[valid]
        output[~valid] = np.nan
        state[~valid] = np.nan
        self._states[body_id] = state
        self._last_t[body_id] = timestamp_s
        return output


def finite_vector(value: Any, expected: int | None = None) -> np.ndarray | None:
    vector = np.asarray(value, dtype=np.float64).reshape(-1)
    if expected is not None and vector.size != expected:
        return None
    if vector.size == 0 or not np.isfinite(vector).all():
        return None
    return vector


def quaternion_xyzw_to_matrix(quaternion: Any) -> np.ndarray:
    q = finite_vector(quaternion, 4)
    if q is None:
        return np.eye(3, dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if norm < 1e-8:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def joint_angle_deg(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float | None:
    if not (
        np.isfinite(a).all() and np.isfinite(b).all() and np.isfinite(c).all()
    ):
        return None
    ba = a - b
    bc = c - b
    denominator = float(np.linalg.norm(ba) * np.linalg.norm(bc))
    if denominator < 1e-8:
        return None
    cosine = float(np.clip(np.dot(ba, bc) / denominator, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def distance_between(points: np.ndarray, first: str, second: str) -> float | None:
    a = points[IDX[first]]
    b = points[IDX[second]]
    if not (np.isfinite(a).all() and np.isfinite(b).all()):
        return None
    return float(np.linalg.norm(a - b))


def group_validity(
    points: np.ndarray, confidence: np.ndarray, threshold: float
) -> dict[str, bool]:
    result: dict[str, bool] = {}
    for group_name, joint_names in CRITICAL_GROUPS.items():
        indexes = [IDX[name] for name in joint_names]
        result[group_name] = bool(
            np.isfinite(points[indexes]).all()
            and np.all(confidence[indexes] >= threshold)
        )
    return result


def build_g1_features(
    filtered_points: np.ndarray,
    confidence: np.ndarray,
    threshold: float,
) -> dict[str, Any]:
    validity = group_validity(filtered_points, confidence, threshold)
    shoulder_width = distance_between(
        filtered_points, "LEFT_SHOULDER", "RIGHT_SHOULDER"
    )
    hip_width = distance_between(filtered_points, "LEFT_HIP", "RIGHT_HIP")

    angles = {
        "left_elbow_interior_deg": joint_angle_deg(
            filtered_points[IDX["LEFT_SHOULDER"]],
            filtered_points[IDX["LEFT_ELBOW"]],
            filtered_points[IDX["LEFT_WRIST"]],
        ),
        "right_elbow_interior_deg": joint_angle_deg(
            filtered_points[IDX["RIGHT_SHOULDER"]],
            filtered_points[IDX["RIGHT_ELBOW"]],
            filtered_points[IDX["RIGHT_WRIST"]],
        ),
        "left_knee_interior_deg": joint_angle_deg(
            filtered_points[IDX["LEFT_HIP"]],
            filtered_points[IDX["LEFT_KNEE"]],
            filtered_points[IDX["LEFT_ANKLE"]],
        ),
        "right_knee_interior_deg": joint_angle_deg(
            filtered_points[IDX["RIGHT_HIP"]],
            filtered_points[IDX["RIGHT_KNEE"]],
            filtered_points[IDX["RIGHT_ANKLE"]],
        ),
    }

    return {
        "valid_groups": validity,
        "upper_body_reference_ready": bool(
            validity["torso"] and validity["left_arm"] and validity["right_arm"]
        ),
        "whole_body_reference_ready": bool(all(validity.values())),
        "anthropometry_m": {
            "shoulder_width": shoulder_width,
            "hip_width": hip_width,
            "left_upper_arm": distance_between(
                filtered_points, "LEFT_SHOULDER", "LEFT_ELBOW"
            ),
            "left_forearm": distance_between(
                filtered_points, "LEFT_ELBOW", "LEFT_WRIST"
            ),
            "right_upper_arm": distance_between(
                filtered_points, "RIGHT_SHOULDER", "RIGHT_ELBOW"
            ),
            "right_forearm": distance_between(
                filtered_points, "RIGHT_ELBOW", "RIGHT_WRIST"
            ),
            "left_thigh": distance_between(
                filtered_points, "LEFT_HIP", "LEFT_KNEE"
            ),
            "left_shank": distance_between(
                filtered_points, "LEFT_KNEE", "LEFT_ANKLE"
            ),
            "right_thigh": distance_between(
                filtered_points, "RIGHT_HIP", "RIGHT_KNEE"
            ),
            "right_shank": distance_between(
                filtered_points, "RIGHT_KNEE", "RIGHT_ANKLE"
            ),
        },
        "geometric_angles": angles,
        "note": (
            "These are perception features, not G1 motor commands. "
            "Balance-aware retargeting is required before robot control."
        ),
    }


def sanitize_for_json(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return sanitize_for_json(value.tolist())
    if isinstance(value, np.generic):
        return sanitize_for_json(value.item())
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): sanitize_for_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [sanitize_for_json(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if hasattr(value, "width") and hasattr(value, "height"):
        return {
            "width": int(value.width),
            "height": int(value.height),
        }
    return value


def zed_value(value: Any) -> Any:
    """Convert a ZED SDK vector/matrix wrapper without depending on its version."""
    try:
        if hasattr(value, "get"):
            value = value.get()
        if hasattr(value, "width") and hasattr(value, "height"):
            return {
                "width": int(value.width),
                "height": int(value.height),
            }
        converted = sanitize_for_json(np.asarray(value).tolist())
        json.dumps(converted)
        return converted
    except Exception:
        return None


def camera_metadata(zed: Any) -> dict[str, Any]:
    """Capture calibration needed to reproduce pixel/depth measurements."""
    try:
        info = zed.get_camera_information()
        configuration = info.camera_configuration
        calibration = configuration.calibration_parameters

        def intrinsics(parameters: Any) -> dict[str, Any]:
            return {
                "fx": float(parameters.fx),
                "fy": float(parameters.fy),
                "cx": float(parameters.cx),
                "cy": float(parameters.cy),
                "distortion": zed_value(parameters.disto),
                "horizontal_fov_deg": float(parameters.h_fov),
                "vertical_fov_deg": float(parameters.v_fov),
                "image_size": zed_value(parameters.image_size),
            }

        return {
            "serial_number": int(info.serial_number),
            "camera_model": str(info.camera_model),
            "camera_firmware": int(configuration.firmware_version),
            "fps": float(configuration.fps),
            "resolution": zed_value(configuration.resolution),
            "left_intrinsics": intrinsics(calibration.left_cam),
            "right_intrinsics": intrinsics(calibration.right_cam),
            "stereo_translation_m": zed_value(
                calibration.stereo_transform.get_translation()
            ),
            "stereo_orientation_xyzw": zed_value(
                calibration.stereo_transform.get_orientation()
            ),
        }
    except Exception as exc:
        return {"metadata_error": str(exc)}


def read_imu(zed: Any, sensors: Any) -> dict[str, Any] | None:
    """Read image-time IMU data when supported by the connected ZED model."""
    try:
        status = zed.get_sensors_data(sensors, sl.TIME_REFERENCE.IMAGE)
        if status != sl.ERROR_CODE.SUCCESS:
            return None
        imu = sensors.get_imu_data()
        return sanitize_for_json(
            {
                "timestamp_ns": int(imu.timestamp.get_nanoseconds()),
                "orientation_xyzw": zed_value(imu.get_pose().get_orientation()),
                "angular_velocity_deg_s": zed_value(imu.get_angular_velocity()),
                "linear_acceleration_m_s2": zed_value(imu.get_linear_acceleration()),
                "orientation_covariance": zed_value(
                    imu.get_pose_covariance()
                ),
                "angular_velocity_covariance": zed_value(
                    imu.get_angular_velocity_covariance()
                ),
                "linear_acceleration_covariance": zed_value(
                    imu.get_linear_acceleration_covariance()
                ),
            }
        )
    except Exception:
        return None


def build_record(
    body: Any,
    filtered_points: np.ndarray,
    timestamp_ns: int,
    frame_index: int,
    confidence_threshold: float,
    imu: dict[str, Any] | None = None,
) -> dict[str, Any]:
    raw_points = np.asarray(body.keypoint, dtype=np.float64)
    keypoint_2d = np.asarray(body.keypoint_2d, dtype=np.float64)
    confidence = np.asarray(body.keypoint_confidence, dtype=np.float64)
    root_q = np.asarray(body.global_root_orientation, dtype=np.float64)
    local_positions = np.asarray(body.local_position_per_joint, dtype=np.float64)
    local_orientations = np.asarray(
        body.local_orientation_per_joint, dtype=np.float64
    )

    pelvis = filtered_points[IDX["PELVIS"]]
    root_rotation = quaternion_xyzw_to_matrix(root_q)
    if np.isfinite(pelvis).all():
        root_relative = (root_rotation.T @ (filtered_points - pelvis).T).T
    else:
        root_relative = np.full_like(filtered_points, np.nan)

    shoulder_width = distance_between(
        filtered_points, "LEFT_SHOULDER", "RIGHT_SHOULDER"
    )
    if shoulder_width is not None and shoulder_width > 0.05:
        shoulder_normalized = root_relative / shoulder_width
    else:
        shoulder_normalized = np.full_like(root_relative, np.nan)

    position = finite_vector(body.position, 3)
    if position is None:
        position = np.full(3, np.nan)

    return sanitize_for_json(
        {
            "schema": "zed_body38_g1_reference/v1",
            "timestamp_ns": int(timestamp_ns),
            "frame_index": int(frame_index),
            "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
            "units": "meter",
            "body_id": int(body.id),
            "unique_object_id": str(body.unique_object_id),
            "tracking_state": str(body.tracking_state),
            "action_state": str(body.action_state),
            "body_confidence": float(body.confidence),
            "root_position_m": position,
            "forward_distance_m": float(position[0]),
            "euclidean_distance_m": float(np.linalg.norm(position)),
            "global_root_orientation_xyzw": root_q,
            "keypoint_names": BODY38_NAMES,
            "keypoints_2d_px": keypoint_2d,
            "keypoints_3d_raw_m": raw_points,
            "keypoints_3d_filtered_m": filtered_points,
            "keypoint_confidence": confidence,
            "local_position_per_joint_m": local_positions,
            "local_orientation_per_joint_xyzw": local_orientations,
            "root_relative_keypoints_m": root_relative,
            "shoulder_width_normalized_keypoints": shoulder_normalized,
            "imu": imu,
            "g1_reference_features": build_g1_features(
                filtered_points, confidence, confidence_threshold
            ),
        }
    )


def valid_pixel(point: np.ndarray, width: int, height: int) -> bool:
    return bool(
        point.size >= 2
        and np.isfinite(point[:2]).all()
        and 0 <= point[0] < width
        and 0 <= point[1] < height
    )


def detect_torn_frame(frame: np.ndarray) -> tuple[bool, int, float]:
    """Detect UVC frames assembled from incompatible horizontal bands.

    ZED USB corruption typically presents as several full-width discontinuities
    containing pieces of older frames.  Natural horizontal edges may create one
    or two strong row transitions, but not many across the complete image.
    """
    if frame.ndim != 3 or frame.shape[0] < 100 or frame.shape[1] < 100:
        return True, 0, 0.0
    if frame.shape[2] == 4:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
    else:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, (320, 180), interpolation=cv2.INTER_AREA)
    row_delta = np.mean(
        np.abs(np.diff(gray.astype(np.float32), axis=0)),
        axis=1,
    )
    baseline = float(np.median(row_delta))
    threshold = max(18.0, baseline * 4.0)
    strong_boundaries = int(np.count_nonzero(row_delta > threshold))
    return strong_boundaries >= 5, strong_boundaries, float(np.max(row_delta))


def draw_skeleton(
    frame: np.ndarray,
    body: Any,
    selected: bool,
    confidence_threshold: float,
) -> None:
    points = np.asarray(body.keypoint_2d, dtype=np.float64)
    confidence = np.asarray(body.keypoint_confidence, dtype=np.float64)
    height, width = frame.shape[:2]
    line_color = (0, 220, 0) if selected else (100, 100, 100)

    for first, second in BODY38_EDGES:
        i, j = IDX[first], IDX[second]
        if (
            confidence[i] >= confidence_threshold
            and confidence[j] >= confidence_threshold
            and valid_pixel(points[i], width, height)
            and valid_pixel(points[j], width, height)
        ):
            cv2.line(
                frame,
                tuple(points[i, :2].astype(int)),
                tuple(points[j, :2].astype(int)),
                line_color,
                2 if selected else 1,
                cv2.LINE_AA,
            )

    for index, point in enumerate(points):
        if (
            confidence[index] >= confidence_threshold
            and valid_pixel(point, width, height)
        ):
            color = (0, 255, 255) if selected else (130, 130, 130)
            cv2.circle(frame, tuple(point[:2].astype(int)), 4, color, -1, cv2.LINE_AA)

    valid_points = [
        point
        for index, point in enumerate(points)
        if confidence[index] >= confidence_threshold
        and valid_pixel(point, width, height)
    ]
    if valid_points:
        anchor = np.min(np.asarray(valid_points), axis=0).astype(int)
        root = finite_vector(body.position, 3)
        distance_text = "?"
        if root is not None:
            distance_text = f"{float(np.linalg.norm(root)):.2f} m"
        cv2.putText(
            frame,
            f"ID {int(body.id)} | {distance_text}",
            (max(8, int(anchor[0])), max(25, int(anchor[1]) - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            line_color,
            2,
            cv2.LINE_AA,
        )


def number_or_none(value: Any) -> float | None:
    try:
        converted = float(value)
        return converted if math.isfinite(converted) else None
    except (TypeError, ValueError):
        return None


def format_xyz(value: Any) -> str:
    vector_value = finite_vector(value, 3)
    if vector_value is None:
        return "n/a"
    return f"{vector_value[0]:+.2f} {vector_value[1]:+.2f} {vector_value[2]:+.2f}"


def draw_diagnostics_panel(
    frame: np.ndarray,
    record: dict[str, Any] | None,
    confidence_threshold: float,
    recording: bool,
    recorded_frames: int,
    svo_enabled: bool,
    distance_min_m: float,
    distance_max_m: float,
    stream_target: str | None = None,
    streamed_frames: int = 0,
) -> None:
    """Show capture-critical raw values without covering the whole image."""
    panel_width = min(430, max(330, frame.shape[1] // 3))
    panel_height = min(frame.shape[0] - 85, 410)
    overlay = frame.copy()
    cv2.rectangle(
        overlay, (8, 82), (8 + panel_width, 82 + panel_height), (10, 10, 10), -1
    )
    cv2.addWeighted(overlay, 0.78, frame, 0.22, 0, frame)

    lines: list[tuple[str, tuple[int, int, int]]] = [
        ("RAW / CAPTURE DIAGNOSTICS", (0, 255, 255)),
        (
            f"REC: {'ON' if recording else 'OFF'}  frames={recorded_frames}  "
            f"SVO2={'ON' if svo_enabled else 'OFF'}",
            (255, 255, 255),
        ),
    ]
    if stream_target:
        lines.append(
            (f"UDP: {stream_target}  sent={streamed_frames}", (255, 220, 160))
        )
    if record is None:
        lines.append(("No valid tracked person", (0, 165, 255)))
    else:
        distance_m = number_or_none(record.get("euclidean_distance_m"))
        forward_m = number_or_none(record.get("forward_distance_m"))
        confidence = np.asarray(record.get("keypoint_confidence", []), dtype=float)
        valid_count = int(
            np.count_nonzero(
                np.isfinite(confidence) & (confidence >= confidence_threshold)
            )
        )
        distance_ok = bool(
            distance_m is not None
            and distance_min_m <= distance_m <= distance_max_m
        )
        lines.extend(
            [
                (
                    f"Distance 3D: "
                    f"{distance_m:.2f} m  [{'IDEAL' if distance_ok else 'CHECK'}]"
                    if distance_m is not None
                    else "Distance 3D: n/a",
                    (0, 255, 0) if distance_ok else (0, 165, 255),
                ),
                (
                    f"Forward X: "
                    f"{forward_m:.2f} m  Root XYZ: "
                    f"{format_xyz(record.get('root_position_m'))}"
                    if forward_m is not None
                    else "Forward X: n/a",
                    (255, 255, 255),
                ),
                (
                    f"Body: id={record.get('body_id')} "
                    f"conf={float(record.get('body_confidence', 0.0)):.0f}/100  "
                    f"valid KP={valid_count}/38",
                    (255, 255, 255),
                ),
            ]
        )
        names = record.get("keypoint_names", [])
        points = record.get("keypoints_3d_raw_m", [])
        point_index = {name: index for index, name in enumerate(names)}
        for label, left_name, right_name in (
            ("Wrist", "LEFT_WRIST", "RIGHT_WRIST"),
            ("Ankle", "LEFT_ANKLE", "RIGHT_ANKLE"),
        ):
            left = points[point_index[left_name]] if left_name in point_index else None
            right = points[point_index[right_name]] if right_name in point_index else None
            lines.append((f"{label} raw L: {format_xyz(left)}", (200, 230, 255)))
            lines.append((f"{label} raw R: {format_xyz(right)}", (200, 230, 255)))
        groups = record.get("g1_reference_features", {}).get("valid_groups", {})
        lines.append(
            (
                "Groups T/LA/RA/LL/RL: "
                + "/".join(
                    "1" if groups.get(name) else "0"
                    for name in (
                        "torso",
                        "left_arm",
                        "right_arm",
                        "left_leg",
                        "right_leg",
                    )
                ),
                (255, 255, 255),
            )
        )
        imu = record.get("imu")
        if isinstance(imu, dict):
            lines.append(
                (
                    f"IMU gyro deg/s: "
                    f"{format_xyz(imu.get('angular_velocity_deg_s'))}",
                    (180, 255, 180),
                )
            )
            lines.append(
                (
                    f"IMU accel m/s2: "
                    f"{format_xyz(imu.get('linear_acceleration_m_s2'))}",
                    (180, 255, 180),
                )
            )
        else:
            lines.append(("IMU: unavailable", (128, 128, 255)))
    lines.append(("Axes: X forward, Y left, Z up | D: panel", (170, 170, 170)))

    y = 106
    for text, color in lines:
        cv2.putText(
            frame,
            text,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.47,
            color,
            1,
            cv2.LINE_AA,
        )
        y += 28


def select_body(
    body_list: list[Any],
    locked_id: int | None,
    minimum_body_confidence: float,
) -> tuple[Any | None, int | None]:
    candidates = [
        body
        for body in body_list
        if body.tracking_state == sl.OBJECT_TRACKING_STATE.OK
        and float(body.confidence) >= minimum_body_confidence
    ]
    if locked_id is not None:
        for body in candidates:
            if int(body.id) == locked_id:
                return body, locked_id
        return None, locked_id

    if not candidates:
        return None, None

    def distance(body: Any) -> float:
        position = finite_vector(body.position, 3)
        return float(np.linalg.norm(position)) if position is not None else math.inf

    selected = min(candidates, key=distance)
    return selected, int(selected.id)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "ZED 2i BODY_38 iskeletini G1 23-DOF retargeting için çıkarır. "
            "Bu program robota komut göndermez."
        )
    )
    parser.add_argument(
        "--model",
        choices=("fast", "medium", "accurate"),
        default="medium",
        help="ZED body tracking modeli (varsayılan: medium)",
    )
    parser.add_argument("--fps", type=int, choices=(15, 30, 60), default=30)
    parser.add_argument(
        "--svo-input",
        type=Path,
        default=None,
        help="Canli kamera yerine mevcut SVO2 dosyasini offline BODY_38 olarak yeniden isle",
    )
    parser.add_argument(
        "--record-stem",
        default=None,
        help="Offline donusumde deterministik JSONL dosya adi (uzantisiz)",
    )
    parser.add_argument("--operator-acquire-frames", type=int, default=10)
    parser.add_argument("--calibration-seconds", type=float, default=4.0)
    parser.add_argument(
        "--depth-mode",
        choices=("neural-light", "neural", "performance"),
        default="neural",
        help="Derinlik hız/kalite profili (varsayılan: neural)",
    )
    parser.add_argument(
        "--confidence",
        type=float,
        default=50.0,
        help="İnsan ve keypoint güven eşiği, 0-100 (varsayılan: 50)",
    )
    parser.add_argument(
        "--filter-tau",
        type=float,
        default=0.08,
        help="3B keypoint low-pass zaman sabiti, saniye (varsayılan: 0.08)",
    )
    parser.add_argument(
        "--prediction-timeout",
        type=float,
        default=0.10,
        help="ZED kayıp insan tahmin süresi, saniye (varsayılan: 0.10)",
    )
    parser.add_argument(
        "--arm-recovery-mode",
        choices=("legacy", "akc"),
        default="akc",
        help=(
            "BODY_38 sonrasinda yalnizca ortulu/dusuk guvenli kol zincirlerinde "
            "kullanilan kurtarma yontemi. 'legacy' guvenli geri donus secenegidir."
        ),
    )
    parser.add_argument(
        "--skeleton-smoothing",
        type=float,
        default=0.15,
        help="ZED iskelet yumuşatma miktarı, 0-1 (varsayılan: 0.15)",
    )
    parser.add_argument(
        "--reduced-precision",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Daha akıcı AI çıkarımı için düşük hassasiyetli inference kullan",
    )
    parser.add_argument(
        "--distance-min",
        type=float,
        default=2.0,
        help="Ekrandaki önerilen en yakın 3B insan mesafesi, metre",
    )
    parser.add_argument(
        "--distance-max",
        type=float,
        default=4.0,
        help="Ekrandaki önerilen en uzak 3B insan mesafesi, metre",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "recordings",
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help="Açılışta JSONL kaydını başlat (arayüzde S ile açılıp kapatılabilir)",
    )
    parser.add_argument(
        "--record-svo2",
        action="store_true",
        help=(
            "JSONL ile birlikte ZED'in yerel SVO2 görüntü/sensör kaydını oluştur. "
            "İyi parmak retargeting'i için önerilir."
        ),
    )
    parser.add_argument(
        "--svo-compression",
        choices=("h265", "h264", "lossless"),
        default="h265",
        help="SVO2 sıkıştırması (varsayılan: h265)",
    )
    parser.add_argument(
        "--stream-host",
        default=None,
        help="BODY_38 canlı UDP hedefi; örnek: WSL IP adresi",
    )
    parser.add_argument(
        "--stream-port",
        type=int,
        default=15050,
        help="BODY_38 UDP hedef portu (varsayılan: 15050)",
    )
    parser.add_argument(
        "--monitor-host",
        default=None,
        help=(
            "İkinci BODY_38 UDP kopyasının hedefi. Canlı analiz paneli için "
            "WSL IP adresi verilebilir."
        ),
    )
    parser.add_argument(
        "--monitor-port",
        type=int,
        default=15052,
        help="Canlı analiz paneli UDP portu (varsayılan: 15052)",
    )
    parser.add_argument(
        "--ros-host",
        default=None,
        help="Üçüncü BODY_38 UDP kopyası; ROS 2 köprüsü için WSL IP adresi",
    )
    parser.add_argument(
        "--ros-port",
        type=int,
        default=15054,
        help="BODY_38 ROS 2 köprüsü UDP portu (varsayılan: 15054)",
    )
    parser.add_argument(
        "--stream-max-hz",
        type=float,
        default=30.0,
        help="Maksimum UDP yayın hızı (varsayılan: 30 Hz)",
    )
    parser.add_argument(
        "--monitor-max-hz",
        type=float,
        default=30.0,
        help="Rerun/analiz UDP kopyasi azami hizi (varsayilan: 30 Hz)",
    )
    parser.add_argument(
        "--ros-max-hz",
        type=float,
        default=30.0,
        help="ROS 2 UDP kopyasi azami hizi (varsayilan: 30 Hz)",
    )
    parser.add_argument(
        "--camera-timeout",
        type=float,
        default=3.0,
        help=(
            "Başarılı kamera karesi gelmezse kaydı güvenli kapatma süresi, "
            "saniye (varsayılan: 3)"
        ),
    )
    parser.add_argument(
        "--max-corrupt-consecutive",
        type=int,
        default=30,
        help=(
            "Bu sayida ardisik yirtilmis USB karesinden sonra kamerayi yeniden "
            "baslatmak icin uygulamadan hata koduyla cik (varsayilan: 30)"
        ),
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="Belirtilirse bu süreden sonra otomatik kapanır",
    )
    parser.add_argument("--list-devices", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def run_self_test() -> int:
    assert len(BODY38_NAMES) == 38
    assert len(G1_23DOF_JOINT_ORDER) == 23
    assert np.allclose(quaternion_xyzw_to_matrix([0, 0, 0, 1]), np.eye(3))
    assert abs(joint_angle_deg(
        np.array([1.0, 0.0, 0.0]),
        np.zeros(3),
        np.array([0.0, 1.0, 0.0]),
    ) - 90.0) < 1e-8
    resolution = sl.Resolution(1280, 720)
    assert zed_value(resolution) == {"width": 1280, "height": 720}
    json.dumps(
        sanitize_for_json(
            {
                "camera": {"resolution": resolution},
                "path": Path("test.svo2"),
                "pelvis_frame": {
                    "origin_camera_m": np.asarray([1.0, 2.0, 3.0]),
                    "rotation_camera_from_pelvis": np.eye(3),
                    "relative_neutral_yaw_rad": np.float64(0.25),
                },
            }
        )
    )
    test_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    draw_diagnostics_panel(
        test_frame,
        {
            "euclidean_distance_m": 3.0,
            "forward_distance_m": 2.9,
            "root_position_m": [2.9, 0.1, 1.0],
            "body_id": 1,
            "body_confidence": 95.0,
            "keypoint_confidence": [90.0] * 38,
            "keypoint_names": BODY38_NAMES,
            "keypoints_3d_raw_m": [[0.0, 0.0, 0.0]] * 38,
            "g1_reference_features": {
                "valid_groups": {name: True for name in CRITICAL_GROUPS}
            },
            "imu": {
                "angular_velocity_deg_s": [0.0, 0.0, 0.0],
                "linear_acceleration_m_s2": [0.0, 0.0, 9.81],
            },
        },
        50.0,
        True,
        30,
        True,
        2.0,
        4.0,
    )
    assert np.count_nonzero(test_frame) > 0
    print(f"ZED SDK: {sl.Camera.get_sdk_version()}")
    print(
        "Self-test başarılı: BODY_38, G1 23-DOF, JSON metadata "
        "ve tanı paneli doğrulandı."
    )
    return 0


def list_devices() -> int:
    devices = sl.Camera.get_device_list()
    if not devices:
        print("ZED kamera bulunamadı.")
        return 1
    for device in devices:
        print(
            f"id={device.id} serial={device.serial_number} "
            f"state={device.camera_state}"
        )
    return 0


def main() -> int:
    args = parse_args()
    if not 0 <= args.confidence <= 100:
        print("--confidence 0 ile 100 arasında olmalıdır.", file=sys.stderr)
        return 2
    if args.max_corrupt_consecutive < 1:
        print("--max-corrupt-consecutive en az 1 olmalidir.", file=sys.stderr)
        return 2
    if args.distance_min < 0 or args.distance_max <= args.distance_min:
        print("--distance-min/--distance-max aralığı geçersiz.", file=sys.stderr)
        return 2
    if (
        not 1 <= args.stream_port <= 65535
        or not 1 <= args.monitor_port <= 65535
        or args.stream_max_hz <= 0
        or args.monitor_max_hz <= 0
        or args.ros_max_hz <= 0
    ):
        print(
            "--stream-port, --monitor-port veya --stream-max-hz geçersiz.",
            file=sys.stderr,
        )
        return 2
    if args.operator_acquire_frames < 1:
        print("--operator-acquire-frames en az 1 olmalı.", file=sys.stderr)
        return 2
    if not 3.0 <= args.calibration_seconds <= 5.0:
        print("--calibration-seconds 3 ile 5 arasında olmalı.", file=sys.stderr)
        return 2
    if args.camera_timeout <= 0:
        print("--camera-timeout sıfırdan büyük olmalı.", file=sys.stderr)
        return 2
    if args.svo_input is not None and args.record_svo2:
        print("--svo-input ile --record-svo2 birlikte kullanilamaz.", file=sys.stderr)
        return 2
    if args.svo_input is not None and not args.svo_input.is_file():
        print(f"SVO2 dosyasi bulunamadi: {args.svo_input}", file=sys.stderr)
        return 2
    if not 0.0 <= args.skeleton_smoothing <= 1.0:
        print("--skeleton-smoothing 0 ile 1 arasında olmalıdır.", file=sys.stderr)
        return 2
    if args.self_test:
        return run_self_test()
    if args.list_devices:
        return list_devices()

    devices = sl.Camera.get_device_list()
    if args.svo_input is None and not devices:
        print(
            "ZED kamera bulunamadı. ZED Depth Viewer'ı kapatın, kamerayı doğrudan "
            "USB 3.x porta bağlayın ve --list-devices ile tekrar deneyin.",
            file=sys.stderr,
        )
        return 3

    zed = sl.Camera()
    if args.svo_input is not None:
        input_type = sl.InputType()
        input_type.set_from_svo_file(str(args.svo_input.resolve()))
        # Keep capture timing during replay: operator/calibration gates use elapsed
        # time and must see the same cadence as the original recording.
        init = sl.InitParameters(input_t=input_type, svo_real_time_mode=True)
    else:
        init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = args.fps
    init.depth_mode = {
        "neural-light": sl.DEPTH_MODE.NEURAL_LIGHT,
        "neural": sl.DEPTH_MODE.NEURAL,
        "performance": sl.DEPTH_MODE.PERFORMANCE,
    }[args.depth_mode]
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init.depth_maximum_distance = 8.0
    init.sdk_verbose = 1
    # Let the SDK recover a temporarily interrupted USB/UVC stream in the
    # background.  ZED SDK 4+ exposes this option; keep compatibility with
    # older Python bindings used on secondary machines.
    if hasattr(init, "async_grab_camera_recovery"):
        init.async_grab_camera_recovery = True

    open_result = zed.open(init)
    if open_result != sl.ERROR_CODE.SUCCESS:
        print(f"ZED açılamadı: {open_result}", file=sys.stderr)
        return 4

    positional = sl.PositionalTrackingParameters()
    positional.set_as_static = True
    positional.set_floor_as_origin = True
    tracking_result = zed.enable_positional_tracking(positional)
    if tracking_result != sl.ERROR_CODE.SUCCESS:
        print(f"Positional tracking başlatılamadı: {tracking_result}", file=sys.stderr)
        zed.close()
        return 5

    model_map = {
        "fast": sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST,
        "medium": sl.BODY_TRACKING_MODEL.HUMAN_BODY_MEDIUM,
        "accurate": sl.BODY_TRACKING_MODEL.HUMAN_BODY_ACCURATE,
    }
    body_parameters = sl.BodyTrackingParameters()
    body_parameters.detection_model = model_map[args.model]
    body_parameters.body_format = sl.BODY_FORMAT.BODY_38
    body_parameters.body_selection = sl.BODY_KEYPOINTS_SELECTION.FULL
    body_parameters.enable_tracking = True
    body_parameters.enable_body_fitting = True
    body_parameters.enable_segmentation = False
    body_parameters.max_range = 8.0
    body_parameters.allow_reduced_precision_inference = bool(args.reduced_precision)
    body_parameters.prediction_timeout_s = max(
        0.0, min(float(args.prediction_timeout), 1.0)
    )

    print("BODY_38 modeli yükleniyor...")
    enable_result = zed.enable_body_tracking(body_parameters)
    if enable_result != sl.ERROR_CODE.SUCCESS:
        print(f"Body tracking başlatılamadı: {enable_result}", file=sys.stderr)
        zed.disable_positional_tracking()
        zed.close()
        return 6

    runtime = sl.BodyTrackingRuntimeParameters()
    runtime.detection_confidence_threshold = float(args.confidence)
    runtime.minimum_keypoints_threshold = 8
    runtime.skeleton_smoothing = float(args.skeleton_smoothing)
    bodies = sl.Bodies()
    left_image = sl.Mat()
    sensors = sl.SensorsData()
    low_pass = ConfidenceAwareLowPass(args.filter_tau)
    operator_selector = OperatorSelector(args.operator_acquire_frames)
    calibration_manager = CalibrationManager(args.calibration_seconds)
    arm_optimizer = ArmChainOptimizer(
        hold_s=max(0.15, args.prediction_timeout + 0.10),
        recovery_mode=args.arm_recovery_mode,
        branch_confirm_frames=4,
    )
    perception_metrics = PerceptionMetrics()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    record_stem = args.record_stem or time.strftime("zed_body38_%Y%m%d_%H%M%S")
    record_path = args.output_dir / f"{record_stem}.jsonl"
    svo_path = args.output_dir / f"{record_stem}.svo2"
    record_file = None
    recording = bool(args.record)
    svo_enabled = False
    recorded_frames = 0
    recording_started_at = None
    locked_id: int | None = None
    frame_index = 0
    started_at = time.monotonic()
    fps_started_at = started_at
    fps_frames = 0
    measured_fps = 0.0
    diagnostics_visible = True
    policy_powered_requested = False
    control_mode_revision = 0
    control_mode_session_ns = time.time_ns()
    stream_targets = []
    stream_target_rates: dict[tuple[str, int], float] = {}
    stream_target_roles: dict[tuple[str, int], str] = {}
    if args.stream_host:
        target = (args.stream_host, args.stream_port)
        stream_targets.append(target)
        stream_target_rates[target] = args.stream_max_hz
        stream_target_roles[target] = "GMR"
    if args.monitor_host:
        target = (args.monitor_host, args.monitor_port)
        stream_targets.append(target)
        stream_target_rates[target] = args.monitor_max_hz
        stream_target_roles[target] = "analysis"
    if args.ros_host:
        target = (args.ros_host, args.ros_port)
        stream_targets.append(target)
        stream_target_rates[target] = args.ros_max_hz
        stream_target_roles[target] = "ROS2"
    stream_targets = list(dict.fromkeys(stream_targets))
    stream_socket = (
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if stream_targets
        else None
    )
    last_target_stream_time = {target: 0.0 for target in stream_targets}
    last_stream_source_timestamp_ns: int | None = None
    # When the requested UDP rate equals the camera rate, an elapsed-time
    # comparison at exactly 1/fps can reject every other frame because normal
    # scheduler jitter makes a 66.67 ms interval microscopically shorter than
    # the threshold. In that common live mode every captured BODY_38 frame
    # should be published; lower requested rates still use the limiter.
    maximum_stream_hz = max(stream_target_rates.values(), default=0.0)
    stream_every_capture = maximum_stream_hz >= 0.95 * float(args.fps)
    streamed_frames = 0
    target_roles = ("GMR", "analiz", "ROS2")
    for target_index, stream_target in enumerate(stream_targets):
        role = (
            target_roles[target_index]
            if target_index < len(target_roles)
            else f"kopya-{target_index}"
        )
        print(
            f"Canlı BODY_38 UDP {role} hedefi: "
            f"{stream_target[0]}:{stream_target[1]}"
        )

    def set_recording(enabled: bool) -> None:
        nonlocal recording, record_file, recording_started_at, svo_enabled
        if enabled and record_file is None:
            # Line buffering exposes completed JSONL lines while capture is active
            # and minimizes data loss after an unexpected exit.
            record_file = record_path.open(
                "w", encoding="utf-8", buffering=1
            )
            metadata = {
                "schema": "zed_body38_g1_reference/metadata/v1",
                "created_unix_ns": time.time_ns(),
                "zed_sdk_version": sl.Camera.get_sdk_version(),
                "body_format": "BODY_38",
                "keypoint_names": BODY38_NAMES,
                "g1_23dof_joint_order": G1_23DOF_JOINT_ORDER,
                "dex3_1": {
                    "fingers": ["thumb", "index", "middle"],
                    "joint_order_per_hand": DEX3_JOINT_ORDER,
                    "joint_limits_rad": DEX3_LIMITS_RAD,
                    "body38_note": (
                        "BODY_38 fingertip proxies are insufficient to uniquely "
                        "recover all seven Dex3-1 actuator angles. Preserve SVO2 "
                        "for a dedicated hand-landmark retargeter."
                    ),
                },
                "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
                "units": "meter",
                "recommended_capture_distance_m": [
                    args.distance_min,
                    args.distance_max,
                ],
                "camera": camera_metadata(zed),
                "native_svo2_path": str(svo_path) if args.record_svo2 else None,
                "safety": "Perception only; contains no robot motor commands.",
            }
            record_file.write(
                json.dumps(sanitize_for_json(metadata), ensure_ascii=False) + "\n"
            )
            record_file.flush()
            recording_started_at = time.monotonic()
            if args.record_svo2:
                compression = {
                    "h265": sl.SVO_COMPRESSION_MODE.H265,
                    "h264": sl.SVO_COMPRESSION_MODE.H264,
                    "lossless": sl.SVO_COMPRESSION_MODE.LOSSLESS,
                }[args.svo_compression]
                parameters = sl.RecordingParameters(str(svo_path), compression)
                result = zed.enable_recording(parameters)
                if result == sl.ERROR_CODE.SUCCESS:
                    svo_enabled = True
                    print(f"Yerel ZED SVO2 kaydı AÇIK: {svo_path}")
                else:
                    print(
                        f"UYARI: SVO2 başlatılamadı: {result}; JSONL devam ediyor.",
                        file=sys.stderr,
                    )
        elif enabled and svo_enabled:
            zed.pause_recording(False)
        elif not enabled and record_file is not None:
            record_file.flush()
            if svo_enabled:
                zed.pause_recording(True)
        recording = enabled
        print(
            (
                f"Kayıt AÇIK: {record_path}"
                if recording
                else f"Kayıt KAPALI ({recorded_frames} BODY_38 karesi)"
            )
        )

    def poll_control_key() -> int:
        """Read controls from the OpenCV window or the PowerShell console."""
        key = -1
        if not args.headless:
            key = cv2.waitKey(1) & 0xFF
        if msvcrt is not None and msvcrt.kbhit():
            character = msvcrt.getwch().lower()
            if character:
                key = ord(character[0])
        return key

    def handle_control_key(key: int) -> bool:
        nonlocal locked_id, diagnostics_visible
        nonlocal policy_powered_requested, control_mode_revision
        if key in (ord("q"), 27):
            return True
        if key == ord("r"):
            operator_selector.reset()
            calibration_manager.reset()
            arm_optimizer.reset()
            locked_id = None
            low_pass.reset()
            print("R: Kisi kilidi, kalibrasyon ve kol hafizasi sifirlandi.")
        elif key == ord("s"):
            set_recording(not recording)
        elif key == ord("p"):
            policy_powered_requested = not policy_powered_requested
            control_mode_revision += 1
            print(
                "P: "
                + (
                    "POLICY POWERED istendi (BODY_38/GMR IK + residual)."
                    if policy_powered_requested
                    else "NORMAL IK istendi."
                ),
                flush=True,
            )
        elif key == ord("d"):
            diagnostics_visible = not diagnostics_visible
            print(f"D: Tani paneli {'ACIK' if diagnostics_visible else 'KAPALI'}.")
        return False

    if recording:
        set_recording(True)

    print(
        "Hazır. Q/ESC: çıkış | S: kayıt aç/kapat | "
        "P: Normal IK/Policy Powered | R: kişi kilidini sıfırla | D: tanı paneli"
    )
    camera_failed = False
    grab_failure_started: float | None = None
    last_grab_warning = 0.0
    body_failure_started: float | None = None
    last_body_warning = 0.0
    corrupt_consecutive = 0
    corrupt_total = 0
    last_good_frame: np.ndarray | None = None
    try:
        while True:
            grab_result = zed.grab()
            if grab_result != sl.ERROR_CODE.SUCCESS:
                end_of_svo = getattr(sl.ERROR_CODE, "END_OF_SVOFILE_REACHED", None)
                if args.svo_input is not None and end_of_svo is not None and grab_result == end_of_svo:
                    print("SVO2 sonuna ulasildi; offline BODY_38 donusumu tamamlandi.")
                    break
                now = time.monotonic()
                if grab_failure_started is None:
                    grab_failure_started = now
                if now - last_grab_warning >= 1.0:
                    print(
                        "ZED kare uyarısı: "
                        f"{grab_result}; kesinti={now - grab_failure_started:.1f}s",
                        file=sys.stderr,
                    )
                    last_grab_warning = now
                if now - grab_failure_started >= args.camera_timeout:
                    camera_failed = True
                    print(
                        "HATA: ZED kamera akışı zaman aşımına uğradı. "
                        "JSONL/SVO2 güvenli kapatılıyor. USB kablosunu ve "
                        "arka USB 3 portunu kontrol edip uygulamayı yeniden başlatın.",
                        file=sys.stderr,
                    )
                    break
                time.sleep(0.002)
                continue
            grab_failure_started = None

            zed.retrieve_image(left_image, sl.VIEW.LEFT)
            # sl.Mat owns a reusable SDK buffer.  Copy it immediately: the
            # following body inference and GUI work must never observe storage
            # that the SDK may reuse asynchronously.
            frame = np.array(left_image.get_data(), copy=True)
            torn, torn_boundaries, torn_peak = detect_torn_frame(frame)
            if torn:
                corrupt_consecutive += 1
                corrupt_total += 1
                now = time.monotonic()
                status_packet = {
                    "schema": "zed_body38_live/status/v1",
                    "sequence": frame_index,
                    "timestamp_ns": time.time_ns(),
                    "status": "CORRUPT_FRAME",
                    "strong_horizontal_boundaries": torn_boundaries,
                    "peak_row_delta": torn_peak,
                    "consecutive": corrupt_consecutive,
                    "total": corrupt_total,
                }
                if stream_socket is not None and stream_targets:
                    status_payload = json.dumps(
                        status_packet, separators=(",", ":")
                    ).encode("utf-8")
                    for stream_target in stream_targets:
                        try:
                            stream_socket.sendto(status_payload, stream_target)
                        except OSError:
                            pass
                # Never display a torn UVC image and never use its BODY_38
                # result.  Holding the last complete frame gives the operator
                # a short, explicit freeze instead of visually mixing several
                # points in time.  The downstream watchdog receives the status
                # packet above and safely holds its last feasible command.
                display_frame = (
                    last_good_frame.copy()
                    if last_good_frame is not None
                    else frame.copy()
                )
                if display_frame.ndim == 3 and display_frame.shape[2] == 4:
                    display_frame = cv2.cvtColor(
                        display_frame, cv2.COLOR_BGRA2BGR
                    )
                cv2.putText(
                    display_frame,
                    (
                        "USB KARE ATLANDI - son saglam goruntu tutuluyor "
                        f"({corrupt_consecutive}/{args.max_corrupt_consecutive})"
                    ),
                    (20, 45),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                if not args.headless:
                    cv2.imshow(
                        "ZED 2i BODY_38 - G1 Skeleton Extractor",
                        display_frame,
                    )
                if handle_control_key(poll_control_key()):
                    break
                if corrupt_consecutive >= args.max_corrupt_consecutive:
                    camera_failed = True
                    print(
                        "HATA: Ardisik bozuk ZED USB kareleri algilandi; "
                        "kamera guvenli yeniden baslatma icin kapatiliyor.",
                        file=sys.stderr,
                    )
                    break
                frame_index += 1
                continue
            corrupt_consecutive = 0
            retrieve_result = zed.retrieve_bodies(bodies, runtime)
            if retrieve_result != sl.ERROR_CODE.SUCCESS:
                now = time.monotonic()
                if body_failure_started is None:
                    body_failure_started = now
                if now - last_body_warning >= 1.0:
                    print(
                        "BODY_38 uyarısı: "
                        f"{retrieve_result}; kesinti={now - body_failure_started:.1f}s",
                        file=sys.stderr,
                    )
                    last_body_warning = now
                if now - body_failure_started >= args.camera_timeout:
                    camera_failed = True
                    print(
                        "HATA: BODY_38 işlem hattı zaman aşımına uğradı. "
                        "Kayıt güvenli kapatılıyor; GPU/USB bağlantısını kontrol edin.",
                        file=sys.stderr,
                    )
                    break
                continue
            body_failure_started = None

            try:
                timestamp_ns = int(
                    zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
                )
            except Exception:
                timestamp_ns = time.time_ns()
            timestamp_s = timestamp_ns / 1e9

            if frame.ndim == 3 and frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            last_good_frame = frame.copy()

            body_list = list(bodies.body_list) if bodies.is_new else []
            selection = operator_selector.update(
                body_list,
                valid=lambda body: (
                    body.tracking_state == sl.OBJECT_TRACKING_STATE.OK
                    and float(body.confidence) >= args.confidence
                ),
                distance=lambda body: (
                    float(np.linalg.norm(position))
                    if (position := finite_vector(body.position, 3)) is not None
                    else math.inf
                ),
            )
            selected = selection.body
            locked_id = selection.body_id

            for body in body_list:
                draw_skeleton(
                    frame,
                    body,
                    selected is not None and int(body.id) == int(selected.id),
                    args.confidence,
                )

            ready_text = "Kisi bekleniyor"
            current_record: dict[str, Any] | None = None
            if selected is not None:
                raw = np.asarray(selected.keypoint, dtype=np.float64)
                confidence = np.asarray(
                    selected.keypoint_confidence, dtype=np.float64
                )
                filtered = low_pass.update(
                    int(selected.id),
                    raw,
                    confidence,
                    args.confidence,
                    timestamp_s,
                )
                arm_result = arm_optimizer.update(
                    timestamp_s=timestamp_s,
                    points_3d=filtered,
                    points_2d=np.asarray(selected.keypoint_2d, dtype=np.float64),
                    confidence=confidence,
                    index=IDX,
                    threshold=args.confidence,
                    calibration=calibration_manager.profile,
                )
                filtered = arm_result.points
                calibration = calibration_manager.update(
                    operator_id=int(selected.id),
                    timestamp_s=timestamp_s,
                    points=filtered,
                    confidence=confidence,
                    index=IDX,
                    confidence_threshold=args.confidence,
                )
                record = build_record(
                    selected,
                    filtered,
                    timestamp_ns,
                    frame_index,
                    args.confidence,
                    read_imu(zed, sensors),
                )
                try:
                    pelvis_local, pelvis_origin, pelvis_rotation = to_pelvis_local(
                        filtered, IDX
                    )
                except ValueError:
                    pelvis_local = np.full_like(filtered, np.nan)
                    pelvis_origin = np.full(3, np.nan)
                    pelvis_rotation = np.full((3, 3), np.nan)
                relative_neutral_yaw_rad = None
                if calibration.profile is not None and np.isfinite(pelvis_rotation).all():
                    neutral_rotation = np.asarray(
                        calibration.profile.get("neutral_pelvis_rotation_matrix"),
                        dtype=np.float64,
                    )
                    if neutral_rotation.shape == (3, 3) and np.isfinite(neutral_rotation).all():
                        relative_rotation = neutral_rotation.T @ pelvis_rotation
                        relative_neutral_yaw_rad = float(
                            np.arctan2(relative_rotation[1, 0], relative_rotation[0, 0])
                        )
                record["operator_selection"] = {
                    "state": selection.state.value,
                    "locked_body_id": operator_selector.locked_id,
                    "locked_unique_object_id": operator_selector.locked_unique_id,
                    "missing_frames": selection.missing_frames,
                    "acquisition_frames": selection.acquisition_frames,
                    "reason": selection.reason,
                    "automatic_handover": False,
                }
                record["calibration"] = {
                    "state": calibration.state,
                    "progress": calibration.progress,
                    "elapsed_s": calibration.elapsed_s,
                    "sample_count": calibration.sample_count,
                    "reason": calibration.reason,
                    "profile": calibration.profile,
                }
                record["pelvis_frame"] = {
                    "coordinate_system": "PELVIS_LOCAL_X_FWD_Y_LEFT_Z_UP",
                    "origin_camera_m": pelvis_origin,
                    "rotation_camera_from_pelvis": pelvis_rotation,
                    "relative_neutral_yaw_rad": relative_neutral_yaw_rad,
                    "keypoints_m": pelvis_local,
                }
                record["occlusion_analysis"] = {
                    "torso_polygon_names": [
                        "LEFT_SHOULDER", "RIGHT_SHOULDER",
                        "RIGHT_HIP", "LEFT_HIP",
                    ],
                    "arm_torso_overlap": arm_result.overlap,
                    "arm_chain_recovered": arm_result.recovered,
                    "arm_recovery_mode": args.arm_recovery_mode,
                    "akc_candidate_confidence": arm_result.candidate_confidence,
                    "akc_candidate_cost": arm_result.candidate_cost,
                    "elbow_branch_sign": arm_result.branch_sign,
                    "reasons": list(arm_result.reasons),
                }
                record["perception_metrics"] = perception_metrics.update(
                    timestamp_s=timestamp_s,
                    # Left/right ordering is anatomical, not camera-image
                    # ordering. A person facing the camera naturally has the
                    # opposite camera-Y shoulder order. Evaluate swaps in the
                    # pelvis-local frame where +Y always means operator-left.
                    points=pelvis_local,
                    confidence=confidence,
                    index=IDX,
                    threshold=args.confidence,
                    overlap=arm_result.overlap,
                    calibration_profile=calibration.profile,
                )
                record["control_mode_request"] = {
                    "session_ns": control_mode_session_ns,
                    "revision": control_mode_revision,
                    "policy_powered": policy_powered_requested,
                }
                current_record = record
                distance_m = number_or_none(record.get("euclidean_distance_m"))
                record["distance_quality"] = {
                    "recommended_min_m": args.distance_min,
                    "recommended_max_m": args.distance_max,
                    "inside_recommended_range": bool(
                        distance_m is not None
                        and args.distance_min <= distance_m <= args.distance_max
                    ),
                }
                now = time.monotonic()
                if (
                    stream_socket is not None
                    and stream_targets
                    and (
                        stream_every_capture
                        or any(
                            now - last_target_stream_time[target]
                            >= 0.98 / stream_target_rates[target]
                            for target in stream_targets
                        )
                    )
                ):
                    raw_valid = np.isfinite(raw).all(axis=1)
                    filtered_valid = np.isfinite(filtered).all(axis=1)
                    comparable = raw_valid & filtered_valid
                    raw_filtered_rms_m = (
                        float(
                            np.sqrt(
                                np.mean(
                                    np.sum(
                                        (raw[comparable] - filtered[comparable]) ** 2,
                                        axis=1,
                                    )
                                )
                            )
                        )
                        if np.any(comparable)
                        else None
                    )
                    source_interval_ms = (
                        (timestamp_ns - last_stream_source_timestamp_ns) / 1e6
                        if last_stream_source_timestamp_ns is not None
                        and timestamp_ns > last_stream_source_timestamp_ns
                        else None
                    )
                    capture_to_send_ms = max(
                        0.0, (time.time_ns() - timestamp_ns) / 1e6
                    )
                    processing_complete_ns = time.time_ns()
                    packet = sanitize_for_json(
                        {
                            "schema": "zed_body38_live/v1",
                            "sequence": frame_index,
                            "timestamp_ns": timestamp_ns,
                            "coordinate_system": record["coordinate_system"],
                            "units": record["units"],
                            "body_id": record["body_id"],
                            "tracking_state": record["tracking_state"],
                            "action_state": record["action_state"],
                            "body_confidence": record["body_confidence"],
                            "root_position_m": record["root_position_m"],
                            "global_root_orientation_xyzw": record[
                                "global_root_orientation_xyzw"
                            ],
                            "keypoint_names": BODY38_NAMES,
                            "keypoints_2d_px": record["keypoints_2d_px"],
                            "keypoints_3d_raw_m": record["keypoints_3d_raw_m"],
                            "keypoints_3d_m": record[
                                "keypoints_3d_filtered_m"
                            ],
                            "keypoint_confidence": record[
                                "keypoint_confidence"
                            ],
                            "local_orientation_per_joint_xyzw": record[
                                "local_orientation_per_joint_xyzw"
                            ],
                            "local_position_per_joint_m": record[
                                "local_position_per_joint_m"
                            ],
                            "root_relative_keypoints_m": record[
                                "root_relative_keypoints_m"
                            ],
                            "pelvis_frame": record["pelvis_frame"],
                            "operator_selection": record["operator_selection"],
                            "calibration": record["calibration"],
                            "occlusion_analysis": record["occlusion_analysis"],
                            "perception_metrics": record["perception_metrics"],
                            "control_mode_request": record[
                                "control_mode_request"
                            ],
                            "shoulder_width_normalized_keypoints": record[
                                "shoulder_width_normalized_keypoints"
                            ],
                            "reference_ready": {
                                "upper_body": record["g1_reference_features"][
                                    "upper_body_reference_ready"
                                ],
                                "whole_body": record["g1_reference_features"][
                                    "whole_body_reference_ready"
                                ],
                            },
                            "g1_reference_features": record[
                                "g1_reference_features"
                            ],
                            "distance_quality": record["distance_quality"],
                            "euclidean_distance_m": record[
                                "euclidean_distance_m"
                            ],
                            "imu": record["imu"],
                            "latency_trace_ns": {
                                "t0_capture_ns": timestamp_ns,
                                "t1_zed_processing_done_ns": processing_complete_ns,
                                "t2_windows_udp_send_ns": processing_complete_ns,
                            },
                            "transport_metrics": {
                                "source_interval_ms": source_interval_ms,
                                "capture_to_send_ms": capture_to_send_ms,
                                "zed_processing_ms": max(
                                    0.0,
                                    (processing_complete_ns - timestamp_ns) / 1e6,
                                ),
                                "raw_filtered_rms_m": raw_filtered_rms_m,
                                "stream_limit_hz": args.stream_max_hz,
                            },
                        }
                    )
                    payload = json.dumps(
                        packet,
                        ensure_ascii=False,
                        allow_nan=False,
                        separators=(",", ":"),
                    ).encode("utf-8")
                    if len(payload) <= 60000:
                        sent = False
                        for stream_target in stream_targets:
                            target_hz = stream_target_rates[stream_target]
                            if (
                                now - last_target_stream_time[stream_target]
                                < 0.98 / target_hz
                                and not (
                                    stream_target_roles.get(stream_target) == "GMR"
                                    and stream_every_capture
                                )
                            ):
                                continue
                            try:
                                stream_socket.sendto(payload, stream_target)
                                sent = True
                                last_target_stream_time[stream_target] = now
                            except OSError as exc:
                                print(
                                    "UDP yayın uyarısı "
                                    f"({stream_target[0]}:{stream_target[1]}): "
                                    f"{exc}",
                                    file=sys.stderr,
                                )
                        if sent:
                            streamed_frames += 1
                            last_stream_source_timestamp_ns = timestamp_ns
                    else:
                        print(
                            f"UDP paket boyutu fazla: {len(payload)} bayt",
                            file=sys.stderr,
                        )
                features = record["g1_reference_features"]
                upper_ready = features["upper_body_reference_ready"]
                whole_ready = features["whole_body_reference_ready"]
                acquisition_text = (
                    f" {selection.acquisition_frames}/{operator_selector.acquire_frames}"
                    if selection.state == OperatorState.ACQUIRING
                    else ""
                )
                control_text = (
                    "AKTIF"
                    if selection.state == OperatorState.LOCKED
                    and calibration.state == "READY"
                    else "BEKLEME"
                )
                ready_text = (
                    f"ID {locked_id} {selection.state.value}{acquisition_text} "
                    f"CAL={calibration.state} {calibration.progress * 100:.0f}% | "
                    f"kontrol={control_text} | "
                    f"mod={'POLICY' if policy_powered_requested else 'IK'} | "
                    f"ust govde={'HAZIR' if upper_ready else 'EKSIK'} "
                    f"| tum vucut={'HAZIR' if whole_ready else 'EKSIK'}"
                )
                if recording and record_file is not None:
                    record_file.write(
                        json.dumps(
                            sanitize_for_json(record),
                            ensure_ascii=False,
                            allow_nan=False,
                        )
                        + "\n"
                    )
                    recorded_frames += 1
            elif stream_socket is not None and stream_targets:
                now = time.monotonic()
                if (
                    stream_every_capture
                    or any(
                        now - last_target_stream_time[target]
                        >= 0.98 / stream_target_rates[target]
                        for target in stream_targets
                    )
                ):
                    status_packet = {
                        "schema": "zed_body38_live/status/v1",
                        "sequence": frame_index,
                        "timestamp_ns": timestamp_ns,
                        "status": (
                            "OPERATOR_LOST"
                            if selection.state == OperatorState.LOST
                            else "NO_BODY"
                        ),
                        "detected_body_count": len(body_list),
                        "operator_selection": {
                            "state": selection.state.value,
                            "locked_body_id": operator_selector.locked_id,
                            "automatic_handover": False,
                            "reason": selection.reason,
                        },
                    }
                    status_payload = json.dumps(
                        status_packet,
                        separators=(",", ":"),
                    ).encode("utf-8")
                    sent = False
                    for stream_target in stream_targets:
                        target_hz = stream_target_rates[stream_target]
                        if (
                            now - last_target_stream_time[stream_target]
                            < 0.98 / target_hz
                            and not (
                                stream_target_roles.get(stream_target) == "GMR"
                                and stream_every_capture
                            )
                        ):
                            continue
                        try:
                            stream_socket.sendto(status_payload, stream_target)
                            sent = True
                            last_target_stream_time[stream_target] = now
                        except OSError as exc:
                            print(
                                "UDP yayın uyarısı "
                                f"({stream_target[0]}:{stream_target[1]}): "
                                f"{exc}",
                                file=sys.stderr,
                            )
                    if sent:
                        streamed_frames += 1

            fps_frames += 1
            fps_elapsed = time.monotonic() - fps_started_at
            if fps_elapsed >= 1.0:
                measured_fps = fps_frames / fps_elapsed
                fps_frames = 0
                fps_started_at = time.monotonic()

            cv2.rectangle(frame, (0, 0), (frame.shape[1], 70), (20, 20, 20), -1)
            cv2.putText(
                frame,
                f"ZED BODY_38 -> G1 referans | {measured_fps:.1f} FPS",
                (12, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                (
                    f"{ready_text} | "
                    f"Kayit={'ACIK ' + str(recorded_frames) + ' kare' if recording else 'KAPALI'}"
                ),
                (12, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255) if selected is not None else (0, 160, 255),
                2,
                cv2.LINE_AA,
            )
            if diagnostics_visible:
                draw_diagnostics_panel(
                    frame,
                    current_record,
                    args.confidence,
                    recording,
                    recorded_frames,
                    svo_enabled,
                    args.distance_min,
                    args.distance_max,
                    (
                        f"{stream_targets[0][0]}:{stream_targets[0][1]}"
                        if stream_targets
                        else None
                    ),
                    streamed_frames,
                )

            if not args.headless:
                cv2.imshow("ZED 2i BODY_38 - G1 Skeleton Extractor", frame)
            key = poll_control_key()
            if handle_control_key(key):
                break
            frame_index += 1
            if args.seconds > 0 and time.monotonic() - started_at >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if record_file is not None:
            record_file.flush()
            record_file.close()
        if svo_enabled:
            zed.disable_recording()
        if stream_socket is not None:
            stream_socket.close()
        zed.disable_body_tracking()
        zed.disable_positional_tracking()
        zed.close()
        cv2.destroyAllWindows()

    if record_path.exists():
        print(f"Kayıt: {record_path}")
        if recorded_frames == 0:
            elapsed = (
                time.monotonic() - recording_started_at
                if recording_started_at is not None
                else 0.0
            )
            print(
                "UYARI: Kayıtta BODY_38 karesi yok. "
                f"Kayıt süresi={elapsed:.1f}s; görüntüde geçerli bir ID ve "
                "'Kayıt=AÇIK N kare' sayacı görülmelidir."
            )
        else:
            print(f"Kaydedilen BODY_38 kare sayısı: {recorded_frames}")
    print("Kamera güvenli şekilde kapatıldı.")
    print(f"USB frame integrity: corrupt={corrupt_total} total={frame_index}")
    return 7 if camera_failed else 0


if __name__ == "__main__":
    raise SystemExit(main())

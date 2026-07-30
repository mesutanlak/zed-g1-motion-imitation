"""Lossless BODY_38 analysis recording with derived kinematic measurements.

The recorder is perception-only. It stores the received ZED packet and adds
deterministic geometric measurements; it never creates robot commands.
"""

from __future__ import annotations

import csv
import json
import math
import time
from pathlib import Path
from typing import Any


ANGLE_TRIPLETS = {
    "left_elbow_interior_deg": ("LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST"),
    "right_elbow_interior_deg": (
        "RIGHT_SHOULDER",
        "RIGHT_ELBOW",
        "RIGHT_WRIST",
    ),
    "left_knee_interior_deg": ("LEFT_HIP", "LEFT_KNEE", "LEFT_ANKLE"),
    "right_knee_interior_deg": ("RIGHT_HIP", "RIGHT_KNEE", "RIGHT_ANKLE"),
    "left_hip_interior_deg": ("LEFT_SHOULDER", "LEFT_HIP", "LEFT_KNEE"),
    "right_hip_interior_deg": ("RIGHT_SHOULDER", "RIGHT_HIP", "RIGHT_KNEE"),
    "left_ankle_interior_deg": ("LEFT_KNEE", "LEFT_ANKLE", "LEFT_BIG_TOE"),
    "right_ankle_interior_deg": (
        "RIGHT_KNEE",
        "RIGHT_ANKLE",
        "RIGHT_BIG_TOE",
    ),
    "left_wrist_proxy_deg": ("LEFT_ELBOW", "LEFT_WRIST", "LEFT_HAND_INDEX_1"),
    "right_wrist_proxy_deg": (
        "RIGHT_ELBOW",
        "RIGHT_WRIST",
        "RIGHT_HAND_INDEX_1",
    ),
    "neck_interior_deg": ("SPINE_3", "NECK", "NOSE"),
}

SEGMENTS = {
    "pelvis_spine1": ("PELVIS", "SPINE_1"),
    "spine1_spine2": ("SPINE_1", "SPINE_2"),
    "spine2_spine3": ("SPINE_2", "SPINE_3"),
    "left_upper_arm": ("LEFT_SHOULDER", "LEFT_ELBOW"),
    "left_forearm": ("LEFT_ELBOW", "LEFT_WRIST"),
    "right_upper_arm": ("RIGHT_SHOULDER", "RIGHT_ELBOW"),
    "right_forearm": ("RIGHT_ELBOW", "RIGHT_WRIST"),
    "shoulder_width": ("LEFT_SHOULDER", "RIGHT_SHOULDER"),
    "hip_width": ("LEFT_HIP", "RIGHT_HIP"),
    "left_thigh": ("LEFT_HIP", "LEFT_KNEE"),
    "left_shank": ("LEFT_KNEE", "LEFT_ANKLE"),
    "right_thigh": ("RIGHT_HIP", "RIGHT_KNEE"),
    "right_shank": ("RIGHT_KNEE", "RIGHT_ANKLE"),
}

CSV_FIELDS = (
    "analysis_frame",
    "source_sequence",
    "source_timestamp_ns",
    "body_id",
    "body_confidence",
    "valid_keypoints",
    "confident_keypoints",
    "source_hz",
    "capture_to_udp_ms",
    "raw_filter_rms_cm",
    "root_x_m",
    "root_y_m",
    "root_z_m",
    "max_keypoint_speed_m_s",
    "left_arm_occluded",
    "right_arm_occluded",
    "left_arm_crossed",
    "right_arm_crossed",
    *ANGLE_TRIPLETS.keys(),
    "left_shoulder_flexion_deg",
    "left_shoulder_abduction_deg",
    "right_shoulder_flexion_deg",
    "right_shoulder_abduction_deg",
    "left_hip_flexion_deg",
    "left_hip_abduction_deg",
    "right_hip_flexion_deg",
    "right_hip_abduction_deg",
    "trunk_forward_lean_deg",
    "trunk_lateral_lean_deg",
    "head_yaw_deg",
    "head_lateral_lean_deg",
)


def _finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _point(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    if not all(_finite_number(component) for component in value):
        return None
    return tuple(float(component) for component in value)


def _sub(first, second):
    return tuple(first[index] - second[index] for index in range(3))


def _add(first, second):
    return tuple(first[index] + second[index] for index in range(3))


def _scale(vector, scalar):
    return tuple(component * scalar for component in vector)


def _dot(first, second) -> float:
    return sum(first[index] * second[index] for index in range(3))


def _cross(first, second):
    return (
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    )


def _norm(vector) -> float:
    return math.sqrt(_dot(vector, vector))


def _unit(vector):
    length = _norm(vector)
    return _scale(vector, 1.0 / length) if length > 1e-9 else None


def _angle_deg(first, vertex, third) -> float | None:
    first_ray = _sub(first, vertex)
    second_ray = _sub(third, vertex)
    denominator = _norm(first_ray) * _norm(second_ray)
    if denominator < 1e-9:
        return None
    cosine = max(-1.0, min(1.0, _dot(first_ray, second_ray) / denominator))
    return math.degrees(math.acos(cosine))


def _quaternion_xyzw_to_euler_deg(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        return None
    if not all(_finite_number(component) for component in value):
        return None
    x, y, z, w = (float(component) for component in value)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return None
    x, y, z, w = (component / norm for component in (x, y, z, w))
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def _point_lookup(packet: dict, field: str) -> dict[str, tuple[float, float, float]]:
    names = packet.get("keypoint_names", [])
    values = packet.get(field, [])
    return {
        str(name): point
        for index, name in enumerate(names)
        if index < len(values) and (point := _point(values[index])) is not None
    }


class KinematicAnalyzer:
    def __init__(self, confidence_threshold: float = 35.0) -> None:
        self.confidence_threshold = float(confidence_threshold)
        self.previous_points: dict[str, tuple[float, float, float]] = {}
        self.previous_angles: dict[str, float] = {}
        self.previous_timestamp_ns: int | None = None

    def analyze(self, packet: dict) -> dict:
        points = _point_lookup(packet, "keypoints_3d_m")
        raw_points = _point_lookup(packet, "keypoints_3d_raw_m")
        names = [str(name) for name in packet.get("keypoint_names", [])]
        confidence_values = packet.get("keypoint_confidence", [])
        confidence = {
            name: float(confidence_values[index])
            for index, name in enumerate(names)
            if index < len(confidence_values)
            and _finite_number(confidence_values[index])
        }
        timestamp_ns = int(packet.get("timestamp_ns", 0) or 0)
        dt_s = (
            (timestamp_ns - self.previous_timestamp_ns) / 1e9
            if self.previous_timestamp_ns is not None
            and timestamp_ns > self.previous_timestamp_ns
            else None
        )
        if dt_s is not None and not 0.005 <= dt_s <= 0.5:
            dt_s = None

        root = points.get("PELVIS")
        root_relative = {
            name: list(_sub(point, root))
            for name, point in points.items()
        } if root is not None else {}

        segment_lengths = {
            name: (
                _norm(_sub(points[end], points[start]))
                if start in points and end in points
                else None
            )
            for name, (start, end) in SEGMENTS.items()
        }
        geometric_angles = {
            name: (
                _angle_deg(points[first], points[vertex], points[third])
                if first in points and vertex in points and third in points
                else None
            )
            for name, (first, vertex, third) in ANGLE_TRIPLETS.items()
        }

        body_basis = self._body_basis(points)
        anatomical_angles = self._anatomical_angles(points, body_basis)
        geometric_angles.update(anatomical_angles)

        keypoint_velocity: dict[str, list[float] | None] = {}
        keypoint_speed: dict[str, float | None] = {}
        for name, point in points.items():
            if dt_s is not None and name in self.previous_points:
                velocity = _scale(_sub(point, self.previous_points[name]), 1.0 / dt_s)
                keypoint_velocity[name] = list(velocity)
                keypoint_speed[name] = _norm(velocity)
            else:
                keypoint_velocity[name] = None
                keypoint_speed[name] = None

        angle_velocity = {}
        for name, angle in geometric_angles.items():
            previous = self.previous_angles.get(name)
            angle_velocity[name] = (
                (angle - previous) / dt_s
                if angle is not None and previous is not None and dt_s is not None
                else None
            )

        local_quaternions = packet.get("local_orientation_per_joint_xyzw", [])
        local_euler = {
            name: (
                _quaternion_xyzw_to_euler_deg(local_quaternions[index])
                if index < len(local_quaternions)
                else None
            )
            for index, name in enumerate(names)
        }

        raw_filter_error = {
            name: (
                _norm(_sub(raw_points[name], points[name]))
                if name in raw_points and name in points
                else None
            )
            for name in names
        }
        occlusion = self._occlusion_flags(points, body_basis)
        confident_names = [
            name
            for name in points
            if confidence.get(name, 0.0) >= self.confidence_threshold
        ]
        speeds = [speed for speed in keypoint_speed.values() if speed is not None]

        self.previous_points = points.copy()
        self.previous_angles = {
            name: angle for name, angle in geometric_angles.items() if angle is not None
        }
        self.previous_timestamp_ns = timestamp_ns or self.previous_timestamp_ns

        return {
            "dt_s": dt_s,
            "root_relative_keypoints_m": root_relative,
            "segment_lengths_m": segment_lengths,
            "joint_angles_deg": geometric_angles,
            "joint_angle_velocity_deg_s": angle_velocity,
            "local_orientation_euler_xyz_deg": local_euler,
            "keypoint_velocity_m_s": keypoint_velocity,
            "keypoint_speed_m_s": keypoint_speed,
            "raw_filter_error_m": raw_filter_error,
            "visibility": {
                "valid_keypoints": len(points),
                "confident_keypoints": len(confident_names),
                "confident_names": confident_names,
                "missing_names": [name for name in names if name not in points],
                "low_confidence_names": [
                    name
                    for name in points
                    if confidence.get(name, 0.0) < self.confidence_threshold
                ],
            },
            "occlusion": occlusion,
            "quality": {
                "max_keypoint_speed_m_s": max(speeds) if speeds else None,
                "median_raw_filter_error_m": (
                    sorted(error for error in raw_filter_error.values() if error is not None)[
                        len([error for error in raw_filter_error.values() if error is not None]) // 2
                    ]
                    if any(error is not None for error in raw_filter_error.values())
                    else None
                ),
            },
        }

    @staticmethod
    def _body_basis(points: dict) -> dict[str, tuple[float, float, float]] | None:
        required = ("PELVIS", "SPINE_3", "LEFT_SHOULDER", "RIGHT_SHOULDER")
        if any(name not in points for name in required):
            return None
        up = _unit(_sub(points["SPINE_3"], points["PELVIS"]))
        lateral = _unit(_sub(points["LEFT_SHOULDER"], points["RIGHT_SHOULDER"]))
        if up is None or lateral is None:
            return None
        forward = _unit(_cross(lateral, up))
        if forward is None:
            return None
        # Re-orthogonalize lateral after noisy BODY_38 observations.
        lateral = _unit(_cross(up, forward))
        if lateral is None:
            return None
        return {"forward": forward, "lateral": lateral, "up": up}

    @staticmethod
    def _anatomical_angles(points: dict, basis: dict | None) -> dict[str, float | None]:
        result: dict[str, float | None] = {
            "left_shoulder_flexion_deg": None,
            "left_shoulder_abduction_deg": None,
            "right_shoulder_flexion_deg": None,
            "right_shoulder_abduction_deg": None,
            "left_hip_flexion_deg": None,
            "left_hip_abduction_deg": None,
            "right_hip_flexion_deg": None,
            "right_hip_abduction_deg": None,
            "trunk_forward_lean_deg": None,
            "trunk_lateral_lean_deg": None,
            "head_yaw_deg": None,
            "head_lateral_lean_deg": None,
        }
        if basis is None:
            return result
        forward, lateral, up = (basis[name] for name in ("forward", "lateral", "up"))

        for side, outward_sign in (("left", 1.0), ("right", -1.0)):
            shoulder = f"{side.upper()}_SHOULDER"
            elbow = f"{side.upper()}_ELBOW"
            if shoulder in points and elbow in points:
                arm = _unit(_sub(points[elbow], points[shoulder]))
                if arm is not None:
                    downward = -_dot(arm, up)
                    result[f"{side}_shoulder_flexion_deg"] = math.degrees(
                        math.atan2(_dot(arm, forward), downward)
                    )
                    result[f"{side}_shoulder_abduction_deg"] = math.degrees(
                        math.atan2(outward_sign * _dot(arm, lateral), downward)
                    )
            hip = f"{side.upper()}_HIP"
            knee = f"{side.upper()}_KNEE"
            if hip in points and knee in points:
                thigh = _unit(_sub(points[knee], points[hip]))
                if thigh is not None:
                    downward = -_dot(thigh, up)
                    result[f"{side}_hip_flexion_deg"] = math.degrees(
                        math.atan2(_dot(thigh, forward), downward)
                    )
                    result[f"{side}_hip_abduction_deg"] = math.degrees(
                        math.atan2(outward_sign * _dot(thigh, lateral), downward)
                    )

        trunk = _unit(_sub(points["SPINE_3"], points["PELVIS"]))
        if trunk is not None:
            result["trunk_forward_lean_deg"] = math.degrees(
                math.atan2(_dot(trunk, forward), _dot(trunk, up))
            )
            result["trunk_lateral_lean_deg"] = math.degrees(
                math.atan2(_dot(trunk, lateral), _dot(trunk, up))
            )
        if "NECK" in points and "NOSE" in points:
            head = _unit(_sub(points["NOSE"], points["NECK"]))
            if head is not None:
                result["head_yaw_deg"] = math.degrees(
                    math.atan2(_dot(head, lateral), _dot(head, forward))
                )
                result["head_lateral_lean_deg"] = math.degrees(
                    math.atan2(_dot(head, lateral), _dot(head, up))
                )
        return result

    @staticmethod
    def _occlusion_flags(points: dict, basis: dict | None) -> dict:
        flags = {
            "left_arm_torso_projection": False,
            "right_arm_torso_projection": False,
            "left_arm_crossed_centerline": False,
            "right_arm_crossed_centerline": False,
        }
        if basis is None or any(
            name not in points
            for name in ("PELVIS", "LEFT_SHOULDER", "RIGHT_SHOULDER")
        ):
            return flags
        center = _scale(
            _add(points["LEFT_SHOULDER"], points["RIGHT_SHOULDER"]), 0.5
        )
        width = _norm(_sub(points["LEFT_SHOULDER"], points["RIGHT_SHOULDER"]))
        if width < 0.10:
            return flags
        lateral = basis["lateral"]
        for side, own_sign in (("left", 1.0), ("right", -1.0)):
            wrist_name = f"{side.upper()}_WRIST"
            if wrist_name not in points:
                continue
            wrist = points[wrist_name]
            lateral_position = _dot(_sub(wrist, center), lateral)
            inside = (
                abs(lateral_position) < 0.55 * width
                and points["PELVIS"][2] - 0.05
                <= wrist[2]
                <= center[2] + 0.20
            )
            crossed = lateral_position * own_sign < -0.15 * width
            flags[f"{side}_arm_torso_projection"] = inside
            flags[f"{side}_arm_crossed_centerline"] = crossed
        return flags


class SkeletonAnalysisRecorder:
    def __init__(
        self,
        output_dir: Path,
        *,
        confidence_threshold: float = 35.0,
        write_csv: bool = True,
    ) -> None:
        self.output_dir = Path(output_dir)
        self.confidence_threshold = float(confidence_threshold)
        self.write_csv = bool(write_csv)
        self.analyzer = KinematicAnalyzer(confidence_threshold)
        self.json_stream = None
        self.csv_stream = None
        self.csv_writer = None
        self.json_path: Path | None = None
        self.csv_path: Path | None = None
        self.frame_count = 0
        self.status_count = 0
        self.started_ns: int | None = None

    @property
    def active(self) -> bool:
        return self.json_stream is not None

    def start(self) -> tuple[Path, Path | None]:
        if self.active:
            return self.json_path, self.csv_path
        self.output_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.json_path = self.output_dir / f"body38_analysis_{stamp}.jsonl"
        self.csv_path = (
            self.output_dir / f"body38_analysis_{stamp}_summary.csv"
            if self.write_csv
            else None
        )
        self.json_stream = self.json_path.open("w", encoding="utf-8", newline="\n")
        if self.csv_path is not None:
            self.csv_stream = self.csv_path.open(
                "w", encoding="utf-8-sig", newline=""
            )
            self.csv_writer = csv.DictWriter(self.csv_stream, fieldnames=CSV_FIELDS)
            self.csv_writer.writeheader()
        self.started_ns = time.time_ns()
        self.frame_count = 0
        self.status_count = 0
        metadata = {
            "schema": "zed_body38_3d_analysis/metadata/v1",
            "created_unix_ns": self.started_ns,
            "coordinate_convention": "RIGHT_HANDED_Z_UP_X_FWD",
            "angle_units": "degree",
            "position_units": "meter",
            "velocity_units": "meter_per_second",
            "confidence_threshold": self.confidence_threshold,
            "contents": {
                "source_packet": "Lossless zed_body38_live/v1 payload",
                "derived": [
                    "root-relative 3D keypoints",
                    "segment lengths",
                    "geometric and anatomical joint angles",
                    "joint angle velocities",
                    "local quaternion Euler XYZ views",
                    "per-keypoint linear velocities",
                    "raw-filter errors",
                    "visibility and self-occlusion flags",
                ],
            },
            "safety": "Perception analysis only; contains no robot commands.",
        }
        self._write_json(metadata)
        self.json_stream.flush()
        if self.csv_stream is not None:
            self.csv_stream.flush()
        return self.json_path, self.csv_path

    def stop(self) -> tuple[Path | None, Path | None, int]:
        json_path, csv_path, count = self.json_path, self.csv_path, self.frame_count
        if self.json_stream is not None:
            self.json_stream.flush()
            self.json_stream.close()
        if self.csv_stream is not None:
            self.csv_stream.flush()
            self.csv_stream.close()
        self.json_stream = None
        self.csv_stream = None
        self.csv_writer = None
        return json_path, csv_path, count

    def record(self, packet: dict) -> None:
        if not self.active:
            return
        schema = packet.get("schema")
        if schema == "zed_body38_live/status/v1":
            self.status_count += 1
            self._write_json(
                {
                    "schema": "zed_body38_3d_analysis/status/v1",
                    "analysis_receive_timestamp_ns": time.time_ns(),
                    "source": packet,
                }
            )
            return
        if schema != "zed_body38_live/v1":
            return

        derived = self.analyzer.analyze(packet)
        self.frame_count += 1
        frame = {
            "schema": "zed_body38_3d_analysis/frame/v1",
            "analysis_frame": self.frame_count,
            "analysis_receive_timestamp_ns": time.time_ns(),
            "source": packet,
            "derived": derived,
        }
        self._write_json(frame)
        if self.csv_writer is not None:
            self.csv_writer.writerow(self._summary_row(packet, derived))
        if self.frame_count % 15 == 0:
            self.json_stream.flush()
            if self.csv_stream is not None:
                self.csv_stream.flush()

    def _write_json(self, value: dict) -> None:
        json.dump(
            value,
            self.json_stream,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        )
        self.json_stream.write("\n")

    def _summary_row(self, packet: dict, derived: dict) -> dict:
        transport = packet.get("transport_metrics", {})
        interval = transport.get("source_interval_ms")
        source_hz = (
            1000.0 / float(interval)
            if _finite_number(interval) and float(interval) > 0.0
            else None
        )
        root = _point(packet.get("root_position_m")) or (None, None, None)
        visibility = derived["visibility"]
        angles = derived["joint_angles_deg"]
        occlusion = derived["occlusion"]
        row = {
            "analysis_frame": self.frame_count,
            "source_sequence": packet.get("sequence"),
            "source_timestamp_ns": packet.get("timestamp_ns"),
            "body_id": packet.get("body_id"),
            "body_confidence": packet.get("body_confidence"),
            "valid_keypoints": visibility["valid_keypoints"],
            "confident_keypoints": visibility["confident_keypoints"],
            "source_hz": source_hz,
            "capture_to_udp_ms": transport.get("capture_to_send_ms"),
            "raw_filter_rms_cm": (
                float(transport["raw_filtered_rms_m"]) * 100.0
                if _finite_number(transport.get("raw_filtered_rms_m"))
                else None
            ),
            "root_x_m": root[0],
            "root_y_m": root[1],
            "root_z_m": root[2],
            "max_keypoint_speed_m_s": derived["quality"][
                "max_keypoint_speed_m_s"
            ],
            "left_arm_occluded": occlusion["left_arm_torso_projection"],
            "right_arm_occluded": occlusion["right_arm_torso_projection"],
            "left_arm_crossed": occlusion["left_arm_crossed_centerline"],
            "right_arm_crossed": occlusion["right_arm_crossed_centerline"],
        }
        row.update({name: angles.get(name) for name in ANGLE_TRIPLETS})
        row.update(
            {
                name: angles.get(name)
                for name in CSV_FIELDS
                if name.endswith("_deg") and name not in row
            }
        )
        return row

"""Lossless JSONL and long-form CSV session recording for Rerun analysis."""

from __future__ import annotations

import csv
import json
import math
import threading
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

from .model import AnalysisConfig, finite_point, joint_angle_names


FRAME_FIELDS = (
    "analysis_frame", "timestamp_ns", "source_frame", "body_id",
    "tracking_state", "body_confidence", "valid_keypoints",
    "confident_keypoints", "root_x_m", "root_y_m", "root_z_m",
    "source_hz", "capture_to_send_ms", "raw_filter_rms_m",
    "max_keypoint_speed_m_s",
)
JOINT_FIELDS = (
    "analysis_frame", "timestamp_ns", "joint_name", "tracking_source",
    "confidence", "x_m", "y_m", "z_m", "raw_x_m", "raw_y_m", "raw_z_m",
    "root_relative_x_m", "root_relative_y_m", "root_relative_z_m",
    "velocity_x_m_s", "velocity_y_m_s", "velocity_z_m_s", "speed_m_s",
    "filter_error_m", "quat_x", "quat_y", "quat_z", "quat_w",
    "euler_x_deg", "euler_y_deg", "euler_z_deg", "related_angles_deg",
)
ANGLE_FIELDS = (
    "analysis_frame", "timestamp_ns", "angle_name", "angle_deg",
    "angular_velocity_deg_s",
)
IMITATION_FIELDS = (
    "sequence", "timestamp_ns", "safety_level", "safety_reasons",
    "left_human_elbow_deg", "left_raw_elbow_deg", "left_safe_elbow_deg",
    "left_actual_elbow_deg", "left_safe_error_deg", "left_actual_error_deg",
    "right_human_elbow_deg", "right_raw_elbow_deg", "right_safe_elbow_deg",
    "right_actual_elbow_deg", "right_safe_error_deg", "right_actual_error_deg",
    "joint_tracking_rmse_rad", "body_tracking_mpjpe_m", "total_control_ms",
)


def _interior_deg(points: dict[str, Any], first: str, middle: str, last: str) -> float | None:
    try:
        a = [float(v) for v in points[first]]
        b = [float(v) for v in points[middle]]
        c = [float(v) for v in points[last]]
        u = [a[i] - b[i] for i in range(3)]
        v = [c[i] - b[i] for i in range(3)]
        nu = math.sqrt(sum(item * item for item in u))
        nv = math.sqrt(sum(item * item for item in v))
        if nu < 1e-8 or nv < 1e-8:
            return None
        cosine = max(-1.0, min(1.0, sum(u[i] * v[i] for i in range(3)) / (nu * nv)))
        return math.degrees(math.acos(cosine))
    except (KeyError, TypeError, ValueError):
        return None


def _json_safe(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return value


class AnalysisSessionWriter:
    def __init__(
        self,
        output_dir: Path,
        *,
        source_description: str,
        rrd_path: Path,
        svo2_path: Path | None,
        config: AnalysisConfig,
    ) -> None:
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.session_dir = output_dir / f"rerun_body38_{stamp}"
        self.session_dir.mkdir(parents=True, exist_ok=False)
        self.jsonl_path = self.session_dir / "skeleton_analysis.jsonl"
        self.frames_csv_path = self.session_dir / "frames.csv"
        self.joints_csv_path = self.session_dir / "joints.csv"
        self.angles_csv_path = self.session_dir / "angles.csv"
        self.imitation_jsonl_path = self.session_dir / "imitation_comparison.jsonl"
        self.imitation_csv_path = self.session_dir / "imitation_comparison.csv"
        self.manifest_path = self.session_dir / "session_manifest.json"
        self.rrd_path = rrd_path
        self.frame_count = 0
        self.imitation_count = 0
        self._imitation_lock = threading.Lock()

        self._json = self.jsonl_path.open("w", encoding="utf-8", newline="\n")
        self._frames = self.frames_csv_path.open(
            "w", encoding="utf-8-sig", newline=""
        )
        self._joints = self.joints_csv_path.open(
            "w", encoding="utf-8-sig", newline=""
        )
        self._angles = self.angles_csv_path.open(
            "w", encoding="utf-8-sig", newline=""
        )
        self._imitation_json = self.imitation_jsonl_path.open(
            "w", encoding="utf-8", newline="\n"
        )
        self._imitation_csv = self.imitation_csv_path.open(
            "w", encoding="utf-8-sig", newline=""
        )
        self._frame_writer = csv.DictWriter(self._frames, fieldnames=FRAME_FIELDS)
        self._joint_writer = csv.DictWriter(self._joints, fieldnames=JOINT_FIELDS)
        self._angle_writer = csv.DictWriter(self._angles, fieldnames=ANGLE_FIELDS)
        self._imitation_writer = csv.DictWriter(
            self._imitation_csv, fieldnames=IMITATION_FIELDS
        )
        self._frame_writer.writeheader()
        self._joint_writer.writeheader()
        self._angle_writer.writeheader()
        self._imitation_writer.writeheader()

        self.metadata = {
            "schema": "zed_body38_rerun_analysis/metadata/v1",
            "created_unix_ns": time.time_ns(),
            "source": source_description,
            "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
            "units": {
                "position": "meter",
                "linear_velocity": "meter_per_second",
                "angle": "degree",
                "angular_velocity": "degree_per_second",
                "time": "nanosecond",
            },
            "rrd_path": str(rrd_path),
            "native_svo2_path": str(svo2_path) if svo2_path else None,
            "initial_config": asdict(config),
            "files": {
                "lossless_and_derived": self.jsonl_path.name,
                "frames": self.frames_csv_path.name,
                "joints": self.joints_csv_path.name,
                "angles": self.angles_csv_path.name,
                "imitation_lossless": self.imitation_jsonl_path.name,
                "imitation_summary": self.imitation_csv_path.name,
            },
            "safety": "Perception analysis only; no Unitree motor commands.",
        }
        self._json.write(json.dumps(self.metadata, ensure_ascii=False) + "\n")
        self._write_manifest()

    def write_imitation(self, packet: dict[str, Any]) -> None:
        """Persist synchronized human, GMR-safe and measured Isaac geometry."""
        comparison = packet.get("retarget_comparison") or {}
        skeleton = packet.get("g1_skeleton") or {}
        human = comparison.get("human_positions_m") or {}
        isaac = packet.get("isaac_metrics") or {}
        latency = packet.get("latency_breakdown_ms") or {}
        safety = packet.get("safety") or {}
        chains = {
            "human": {
                side: (f"{side}_shoulder", f"{side}_elbow", f"{side}_wrist")
                for side in ("left", "right")
            },
            "g1": {
                side: (
                    f"{side}_shoulder_pitch_link", f"{side}_elbow_link",
                    f"{side}_wrist_roll_rubber_hand",
                ) for side in ("left", "right")
            },
        }
        row: dict[str, Any] = {
            "sequence": packet.get("sequence"),
            "timestamp_ns": packet.get("timestamp_ns"),
            "safety_level": safety.get("level"),
            "safety_reasons": json.dumps(safety.get("reasons", []), ensure_ascii=False),
            "joint_tracking_rmse_rad": isaac.get("joint_tracking_rmse_rad"),
            "body_tracking_mpjpe_m": isaac.get("body_tracking_mpjpe_m"),
            "total_control_ms": latency.get("total_control_ms"),
        }
        for side in ("left", "right"):
            h = _interior_deg(human, *chains["human"][side])
            row[f"{side}_human_elbow_deg"] = h
            for variant in ("raw", "safe", "actual"):
                value = _interior_deg(
                    skeleton.get(f"{variant}_positions_m") or {},
                    *chains["g1"][side],
                )
                row[f"{side}_{variant}_elbow_deg"] = value
                if variant in ("safe", "actual"):
                    row[f"{side}_{variant}_error_deg"] = (
                        abs(value - h) if value is not None and h is not None else None
                    )
        with self._imitation_lock:
            self.imitation_count += 1
            record = {
                "schema": "zed_g1_imitation_comparison/frame/v1",
                "summary": row,
                "telemetry_packet": packet,
            }
            self._imitation_json.write(
                json.dumps(_json_safe(record), ensure_ascii=False, allow_nan=False) + "\n"
            )
            self._imitation_writer.writerow(row)
            if self.imitation_count % 15 == 0:
                self._imitation_json.flush()
                self._imitation_csv.flush()

    def _write_manifest(self) -> None:
        payload = dict(self.metadata)
        payload["frame_count"] = self.frame_count
        payload["updated_unix_ns"] = time.time_ns()
        self.manifest_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def write(
        self,
        source: dict[str, Any],
        processed: dict[str, Any],
        derived: dict[str, Any],
        config: AnalysisConfig,
        tracking_sources: dict[str, str],
    ) -> None:
        self.frame_count += 1
        timestamp_ns = int(source.get("timestamp_ns", 0) or time.time_ns())
        names = [str(name) for name in processed.get("keypoint_names", [])]
        positions = processed.get("keypoints_3d_m", [])
        raw_positions = (
            source.get("keypoints_3d_raw_m")
            or source.get("keypoints_3d_m")
            or []
        )
        confidences = source.get("keypoint_confidence", [])
        quaternions = source.get("local_orientation_per_joint_xyzw", [])
        root_relative = derived.get("root_relative_keypoints_m", {})
        velocities = derived.get("keypoint_velocity_m_s", {})
        speeds = derived.get("keypoint_speed_m_s", {})
        errors = derived.get("raw_filter_error_m", {})
        eulers = derived.get("local_orientation_euler_xyz_deg", {})
        angles = derived.get("joint_angles_deg", {})

        record = {
            "schema": "zed_body38_rerun_analysis/frame/v1",
            "analysis_frame": self.frame_count,
            "timestamp_ns": timestamp_ns,
            "source_packet": source,
            "processed_keypoints_3d_m": positions,
            "tracking_source": tracking_sources,
            "derived": derived,
            "effective_config": asdict(config),
        }
        self._json.write(
            json.dumps(_json_safe(record), ensure_ascii=False, allow_nan=False) + "\n"
        )

        root = None
        if "PELVIS" in names:
            root = finite_point(positions[names.index("PELVIS")])
        metrics = source.get("transport_metrics", {})
        self._frame_writer.writerow(
            {
                "analysis_frame": self.frame_count,
                "timestamp_ns": timestamp_ns,
                "source_frame": source.get("frame_index", source.get("sequence")),
                "body_id": source.get("body_id"),
                "tracking_state": source.get("tracking_state"),
                "body_confidence": source.get("body_confidence"),
                "valid_keypoints": derived.get("visibility", {}).get(
                    "valid_keypoints"
                ),
                "confident_keypoints": derived.get("visibility", {}).get(
                    "confident_keypoints"
                ),
                "root_x_m": root[0] if root else None,
                "root_y_m": root[1] if root else None,
                "root_z_m": root[2] if root else None,
                "source_hz": (
                    1000.0 / metrics["source_interval_ms"]
                    if metrics.get("source_interval_ms")
                    else None
                ),
                "capture_to_send_ms": metrics.get("capture_to_send_ms"),
                "raw_filter_rms_m": metrics.get("raw_filtered_rms_m"),
                "max_keypoint_speed_m_s": derived.get("quality", {}).get(
                    "max_keypoint_speed_m_s"
                ),
            }
        )

        for index, name in enumerate(names):
            point = finite_point(positions[index]) if index < len(positions) else None
            raw = (
                finite_point(raw_positions[index])
                if index < len(raw_positions)
                else None
            )
            relative = finite_point(root_relative.get(name))
            velocity = finite_point(velocities.get(name))
            quaternion = (
                quaternions[index]
                if index < len(quaternions)
                and isinstance(quaternions[index], (list, tuple))
                and len(quaternions[index]) == 4
                else [None] * 4
            )
            euler = eulers.get(name) or [None] * 3
            related = {
                angle_name: angles.get(angle_name)
                for angle_name in joint_angle_names(name, angles)
            }
            self._joint_writer.writerow(
                {
                    "analysis_frame": self.frame_count,
                    "timestamp_ns": timestamp_ns,
                    "joint_name": name,
                    "tracking_source": tracking_sources.get(name),
                    "confidence": (
                        confidences[index] if index < len(confidences) else None
                    ),
                    "x_m": point[0] if point else None,
                    "y_m": point[1] if point else None,
                    "z_m": point[2] if point else None,
                    "raw_x_m": raw[0] if raw else None,
                    "raw_y_m": raw[1] if raw else None,
                    "raw_z_m": raw[2] if raw else None,
                    "root_relative_x_m": relative[0] if relative else None,
                    "root_relative_y_m": relative[1] if relative else None,
                    "root_relative_z_m": relative[2] if relative else None,
                    "velocity_x_m_s": velocity[0] if velocity else None,
                    "velocity_y_m_s": velocity[1] if velocity else None,
                    "velocity_z_m_s": velocity[2] if velocity else None,
                    "speed_m_s": speeds.get(name),
                    "filter_error_m": errors.get(name),
                    "quat_x": quaternion[0],
                    "quat_y": quaternion[1],
                    "quat_z": quaternion[2],
                    "quat_w": quaternion[3],
                    "euler_x_deg": euler[0],
                    "euler_y_deg": euler[1],
                    "euler_z_deg": euler[2],
                    "related_angles_deg": json.dumps(
                        _json_safe(related), ensure_ascii=False
                    ),
                }
            )

        angle_velocity = derived.get("joint_angle_velocity_deg_s", {})
        for name, value in angles.items():
            self._angle_writer.writerow(
                {
                    "analysis_frame": self.frame_count,
                    "timestamp_ns": timestamp_ns,
                    "angle_name": name,
                    "angle_deg": value,
                    "angular_velocity_deg_s": angle_velocity.get(name),
                }
            )
        if self.frame_count % 15 == 0:
            self.flush()

    def flush(self) -> None:
        for stream in (
            self._json, self._frames, self._joints, self._angles,
            self._imitation_json, self._imitation_csv,
        ):
            stream.flush()
        # Keep the manifest useful even if the viewer, WSLg, or the parent
        # PowerShell is terminated without reaching ``close``. At 15/30 FPS
        # this is updated every 1.0/0.5 second by ``write``.
        self._write_manifest()

    def close(self) -> None:
        self.flush()
        for stream in (
            self._json, self._frames, self._joints, self._angles,
            self._imitation_json, self._imitation_csv,
        ):
            stream.close()
        self.metadata["closed_unix_ns"] = time.time_ns()
        self._write_manifest()

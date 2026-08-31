"""Lossless JSONL and long-form CSV session recording for Rerun analysis."""

from __future__ import annotations

import csv
import json
import math
import os
import sys
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
    "source_hz", "effective_output_hz", "capture_to_send_ms",
    "record_queue_depth", "record_dropped", "raw_filter_rms_m",
    "max_keypoint_speed_m_s",
    "fusion_mode", "evidence_views", "selected_single_serial",
    "calibration_agreement_ok", "cross_view_mpjpe_m",
    "cross_view_p95_m", "core_disagreement_m", "pelvis_disagreement_m",
    "left_wrist_disagreement_m", "right_wrist_disagreement_m",
    "camera_timestamp_delta_ms", "camera_sync_ok", "fusion_failure_codes",
    "fused_quality_score", "best_single_quality_score",
    "mean_camera_fused", "camera_fps_min", "camera_fps_max",
    "camera_latency_max_ms", "fusion_timestamp_stdev_ms",
    "left_arm_supporting_views", "right_arm_supporting_views",
    "rig_refinement_state", "rig_refinement_rms_m",
    "rig_refinement_updates",
    "pelvis_quality", "torso_quality", "left_arm_quality",
    "right_arm_quality", "left_leg_quality", "right_leg_quality",
)
JOINT_FIELDS = (
    "analysis_frame", "timestamp_ns", "joint_name", "tracking_source",
    "fusion_source", "fusion_state", "fusion_quality",
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
    "left_raw_elbow_joint_rad", "left_safe_elbow_joint_rad",
    "right_raw_elbow_joint_rad", "right_safe_elbow_joint_rad",
    "left_direct_arm_ik_rms_m", "right_direct_arm_ik_rms_m",
    "left_straight_arm_blend", "right_straight_arm_blend",
    "left_direct_ik_candidate_count", "right_direct_ik_candidate_count",
    "left_gmr_arm_rms_m", "right_gmr_arm_rms_m",
    "left_arm_baseline_preserved", "right_arm_baseline_preserved",
    "left_direct_ik_selected_source", "right_direct_ik_selected_source",
    "left_direct_ik_objective_m", "right_direct_ik_objective_m",
    "left_direct_ik_upper_error_deg", "right_direct_ik_upper_error_deg",
    "left_direct_ik_forearm_error_deg", "right_direct_ik_forearm_error_deg",
    "gmr_upper_relative_residual_m", "mirror_rescue_triggered",
    "mirror_rescue_applied", "fusion_mode", "evidence_views",
    "cross_view_mpjpe_m", "cross_view_p95_m",
    "pelvis_disagreement_m", "left_wrist_disagreement_m",
    "right_wrist_disagreement_m", "camera_timestamp_delta_ms",
    "joint_tracking_rmse_rad", "body_tracking_mpjpe_m", "total_control_ms",
    "target_jerk_rms_rad_s3", "target_jerk_max_rad_s3",
    "reference_target_error_rms_rad", "reference_target_error_max_rad",
    "reference_confidence", "reference_velocity_max_rad_s",
    "reference_acceleration_max_rad_s2", "reference_response_hz",
    "physics_wall_hz", "render_interval",
    "fusion_state",
    "joint_saturation_count", "self_collision_count",
    "left_hand_position_error_m", "right_hand_position_error_m",
    "left_elbow_position_error_m", "right_elbow_position_error_m",
    "policy_control_mode", "policy_inference_wrapper_version",
    "policy_raw_action_abs_max", "policy_raw_action_abs_p90",
    "policy_raw_action_saturation_rate", "policy_clipped_unclipped_l1",
    "policy_residual_rms_rad", "policy_residual_abs_max_rad",
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
        self.quality_summary_path = self.session_dir / "quality_summary.json"
        self.manifest_path = self.session_dir / "session_manifest.json"
        self.rrd_path = rrd_path
        self.frame_count = 0
        self.imitation_count = 0
        self._imitation_lock = threading.Lock()
        self._summary_lock = threading.Lock()
        self._quality_samples: dict[str, list[float]] = {}
        self._fusion_mode_counts: dict[str, int] = {}
        self._safety_level_counts: dict[str, int] = {}
        self._snapshot_warning_at: dict[str, float] = {}

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
                "quality_summary": self.quality_summary_path.name,
            },
            "safety": "Perception analysis only; no Unitree motor commands.",
        }
        self._json.write(json.dumps(self.metadata, ensure_ascii=False) + "\n")
        self._write_manifest()

    def _write_json_snapshot(self, path: Path, payload: dict[str, Any]) -> bool:
        """Atomically publish a derived JSON snapshot without killing capture.

        OneDrive, virus scanners and an editor can briefly open a JSON file
        with an exclusive Windows share mode. Opening that destination with
        ``write_text`` then raises ``PermissionError`` inside the capture
        thread. Write a same-directory pending file first and publish it with
        ``os.replace``. If the reader still owns the destination, retain the
        latest pending snapshot and retry on the next periodic flush.
        """
        text = json.dumps(_json_safe(payload), ensure_ascii=False, indent=2)
        pending = path.with_name(f".{path.name}.pending")
        temporary = path.with_name(
            f".{path.name}.{os.getpid()}.{threading.get_ident()}.tmp"
        )
        error: OSError | None = None
        try:
            temporary.write_text(text, encoding="utf-8")
            os.replace(temporary, pending)
            for delay_s in (0.0, 0.01, 0.025):
                if delay_s:
                    time.sleep(delay_s)
                try:
                    os.replace(pending, path)
                    self._snapshot_warning_at.pop(str(path), None)
                    return True
                except PermissionError as exc:
                    error = exc
        except OSError as exc:
            error = exc
        finally:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass
        now = time.monotonic()
        key = str(path)
        if now - self._snapshot_warning_at.get(key, -float("inf")) >= 5.0:
            print(
                "RERUN_SNAPSHOT_DEFERRED "
                f"path={path} error={error}; capture continues",
                file=sys.stderr,
                flush=True,
            )
            self._snapshot_warning_at[key] = now
        return False

    def _sample_quality(self, name: str, value: Any) -> None:
        if not isinstance(value, (int, float)):
            return
        numeric = float(value)
        if not math.isfinite(numeric):
            return
        with self._summary_lock:
            self._quality_samples.setdefault(name, []).append(numeric)

    @staticmethod
    def _percentile(values: list[float], fraction: float) -> float | None:
        if not values:
            return None
        ordered = sorted(values)
        position = fraction * (len(ordered) - 1)
        lower = int(math.floor(position))
        upper = int(math.ceil(position))
        if lower == upper:
            return ordered[lower]
        blend = position - lower
        return ordered[lower] * (1.0 - blend) + ordered[upper] * blend

    def _write_quality_summary(self) -> None:
        with self._summary_lock:
            samples = {name: list(values) for name, values in self._quality_samples.items()}
            fusion_modes = dict(self._fusion_mode_counts)
            safety_levels = dict(self._safety_level_counts)
        metrics = {}
        for name, values in sorted(samples.items()):
            metrics[name] = {
                "count": len(values),
                "mean": sum(values) / len(values),
                "p50": self._percentile(values, 0.50),
                "p90": self._percentile(values, 0.90),
                "p99": self._percentile(values, 0.99),
                "max": max(values),
            }
        payload = {
            "schema": "zed_g1_quality_summary/v1",
            "created_unix_ns": time.time_ns(),
            "frames": self.frame_count,
            "imitation_frames": self.imitation_count,
            "fusion_mode_counts": fusion_modes,
            "safety_level_counts": safety_levels,
            "metrics": metrics,
        }
        self._write_json_snapshot(self.quality_summary_path, payload)

    def write_imitation(self, packet: dict[str, Any]) -> None:
        """Persist synchronized human, GMR-safe and measured Isaac geometry."""
        comparison = packet.get("retarget_comparison") or {}
        skeleton = packet.get("g1_skeleton") or {}
        human = comparison.get("human_positions_m") or {}
        isaac = packet.get("isaac_metrics") or {}
        latency = packet.get("latency_breakdown_ms") or {}
        safety = packet.get("safety") or {}
        bridge = packet.get("bridge_metrics") or {}
        multi = packet.get("source_multi_camera") or {}
        agreement = multi.get("cross_view_agreement") or {}
        reference_motion = packet.get("reference_motion") or {}
        policy_observation = packet.get("policy_observation_v1") or {}
        policy_raw_action = [
            abs(float(value))
            for value in policy_observation.get("raw_action", [])
            if isinstance(value, (int, float)) and math.isfinite(float(value))
        ]
        policy_residual = [
            float(value)
            for value in policy_observation.get("residual_rad", [])
            if isinstance(value, (int, float)) and math.isfinite(float(value))
        ]
        joint_names = packet.get("joint_names") or []
        raw_joint = packet.get("raw_joint_position_rad") or []
        safe_joint = packet.get("safe_joint_position_rad") or []
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
            "timestamp_ns": packet.get("timestamp_ns")
            or packet.get("source_timestamp_ns"),
            "safety_level": safety.get("level"),
            "safety_reasons": json.dumps(safety.get("reasons", []), ensure_ascii=False),
            "joint_tracking_rmse_rad": isaac.get("joint_tracking_rmse_rad"),
            "body_tracking_mpjpe_m": isaac.get("body_tracking_mpjpe_m"),
            "total_control_ms": latency.get("total_control_ms"),
            "target_jerk_rms_rad_s3": isaac.get("target_jerk_rms_rad_s3"),
            "target_jerk_max_rad_s3": isaac.get("target_jerk_max_rad_s3"),
            "reference_target_error_rms_rad": isaac.get(
                "reference_target_error_rms_rad"
            ),
            "reference_target_error_max_rad": isaac.get(
                "reference_target_error_max_rad"
            ),
            "reference_confidence": reference_motion.get("confidence"),
            "reference_velocity_max_rad_s": max(
                (abs(float(value)) for value in reference_motion.get("velocity_rad_s", [])
                 if isinstance(value, (int, float)) and math.isfinite(float(value))),
                default=None,
            ),
            "reference_acceleration_max_rad_s2": max(
                (abs(float(value)) for value in reference_motion.get("acceleration_rad_s2", [])
                 if isinstance(value, (int, float)) and math.isfinite(float(value))),
                default=None,
            ),
            "reference_response_hz": isaac.get("reference_response_hz"),
            "physics_wall_hz": (packet.get("system_metrics") or {}).get(
                "physics_wall_hz"
            ),
            "render_interval": (packet.get("system_metrics") or {}).get(
                "render_interval"
            ),
            "joint_saturation_count": safety.get("joint_limit_saturation"),
            "self_collision_count": safety.get("safe_self_collision_count"),
            "policy_control_mode": policy_observation.get("control_mode"),
            "policy_inference_wrapper_version": policy_observation.get(
                "inference_wrapper_version"
            ),
            "policy_raw_action_abs_max": max(policy_raw_action, default=0.0),
            "policy_raw_action_abs_p90": (
                self._percentile(policy_raw_action, 0.90)
                if policy_raw_action else 0.0
            ),
            "policy_raw_action_saturation_rate": policy_observation.get(
                "raw_action_saturation_rate"
            ),
            "policy_clipped_unclipped_l1": policy_observation.get(
                "clipped_unclipped_l1"
            ),
            "policy_residual_rms_rad": (
                math.sqrt(sum(value * value for value in policy_residual) / len(policy_residual))
                if policy_residual else 0.0
            ),
            "policy_residual_abs_max_rad": max(
                (abs(value) for value in policy_residual), default=0.0
            ),
            "gmr_upper_relative_residual_m": bridge.get(
                "ik_upper_relative_residual_m"
            ),
            "mirror_rescue_triggered": bridge.get("mirror_rescue_triggered"),
            "mirror_rescue_applied": bridge.get("mirror_rescue_applied"),
            "fusion_mode": multi.get("mode"),
            "fusion_state": multi.get("fusion_state"),
            "evidence_views": multi.get("contributing_views"),
            "cross_view_mpjpe_m": agreement.get("mpjpe_m"),
            "cross_view_p95_m": agreement.get("p95_error_m"),
            "pelvis_disagreement_m": agreement.get("pelvis_error_m"),
            "left_wrist_disagreement_m": agreement.get("left_wrist_error_m"),
            "right_wrist_disagreement_m": agreement.get("right_wrist_error_m"),
            "camera_timestamp_delta_ms": multi.get(
                "camera_timestamp_delta_ms"
            ),
        }
        body_errors = isaac.get("body_position_errors_m") or {}
        for side in ("left", "right"):
            row[f"{side}_hand_position_error_m"] = body_errors.get(
                f"{side}_wrist_roll_rubber_hand"
            )
            row[f"{side}_elbow_position_error_m"] = body_errors.get(
                f"{side}_elbow_link"
            )
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
            elbow_name = f"{side}_elbow_joint"
            elbow_index = (
                joint_names.index(elbow_name)
                if elbow_name in joint_names else -1
            )
            row[f"{side}_raw_elbow_joint_rad"] = (
                raw_joint[elbow_index]
                if 0 <= elbow_index < len(raw_joint) else None
            )
            row[f"{side}_safe_elbow_joint_rad"] = (
                safe_joint[elbow_index]
                if 0 <= elbow_index < len(safe_joint) else None
            )
            row[f"{side}_direct_arm_ik_rms_m"] = bridge.get(
                f"{side}_direct_arm_ik_rms_m"
            )
            row[f"{side}_straight_arm_blend"] = bridge.get(
                f"{side}_straight_arm_blend"
            )
            row[f"{side}_direct_ik_candidate_count"] = bridge.get(
                f"{side}_direct_ik_candidate_count"
            )
            row[f"{side}_gmr_arm_rms_m"] = bridge.get(
                f"{side}_gmr_arm_rms_m"
            )
            row[f"{side}_arm_baseline_preserved"] = bridge.get(
                f"{side}_arm_baseline_preserved"
            )
            row[f"{side}_direct_ik_selected_source"] = bridge.get(
                f"{side}_direct_ik_selected_source"
            )
            row[f"{side}_direct_ik_objective_m"] = bridge.get(
                f"{side}_direct_ik_objective_m"
            )
            direction_error = bridge.get(
                f"{side}_direct_ik_direction_error_deg"
            ) or {}
            row[f"{side}_direct_ik_upper_error_deg"] = direction_error.get(
                "upper"
            )
            row[f"{side}_direct_ik_forearm_error_deg"] = direction_error.get(
                "forearm"
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
        with self._summary_lock:
            level = str(row.get("safety_level") or "UNKNOWN")
            self._safety_level_counts[level] = self._safety_level_counts.get(level, 0) + 1
        for key in (
            "left_safe_error_deg", "right_safe_error_deg",
            "left_actual_error_deg", "right_actual_error_deg",
            "left_direct_arm_ik_rms_m", "right_direct_arm_ik_rms_m",
            "left_direct_ik_objective_m", "right_direct_ik_objective_m",
            "left_direct_ik_upper_error_deg", "right_direct_ik_upper_error_deg",
            "left_direct_ik_forearm_error_deg", "right_direct_ik_forearm_error_deg",
            "gmr_upper_relative_residual_m", "cross_view_mpjpe_m",
            "cross_view_p95_m", "pelvis_disagreement_m",
            "left_wrist_disagreement_m", "right_wrist_disagreement_m",
            "camera_timestamp_delta_ms",
            "joint_tracking_rmse_rad", "body_tracking_mpjpe_m",
            "total_control_ms", "target_jerk_rms_rad_s3",
            "target_jerk_max_rad_s3", "left_hand_position_error_m",
            "reference_target_error_rms_rad",
            "reference_target_error_max_rad",
            "reference_confidence", "reference_velocity_max_rad_s",
            "reference_acceleration_max_rad_s2",
            "physics_wall_hz",
            "right_hand_position_error_m", "left_elbow_position_error_m",
            "right_elbow_position_error_m",
            "policy_raw_action_abs_max", "policy_raw_action_abs_p90",
            "policy_raw_action_saturation_rate", "policy_clipped_unclipped_l1",
            "policy_residual_rms_rad", "policy_residual_abs_max_rad",
        ):
            self._sample_quality(key, row.get(key))

    def _write_manifest(self) -> None:
        payload = dict(self.metadata)
        payload["frame_count"] = self.frame_count
        payload["updated_unix_ns"] = time.time_ns()
        self._write_json_snapshot(self.manifest_path, payload)

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
        multi = source.get("multi_camera") or {}
        human_state = source.get("human_state") or {}
        agreement = multi.get("cross_view_agreement") or {}
        fusion_metrics = multi.get("fusion_metrics") or {}
        cameras = list((fusion_metrics.get("per_camera") or {}).values())
        camera_fps = [
            float(item["received_fps"])
            for item in cameras
            if isinstance(item.get("received_fps"), (int, float))
            and math.isfinite(float(item["received_fps"]))
        ]
        camera_latency = [
            float(item["received_latency_ms"])
            for item in cameras
            if isinstance(item.get("received_latency_ms"), (int, float))
            and math.isfinite(float(item["received_latency_ms"]))
        ]
        arm_evidence = multi.get("arm_evidence") or {}
        rig_refinement = (
            multi.get("rig_extrinsics")
            or multi.get("online_rig_refinement")
            or {}
        )
        source_hz = (
            1000.0 / metrics["source_interval_ms"]
            if metrics.get("source_interval_ms") else None
        )
        fusion_mode = str(multi.get("mode") or "unknown")
        with self._summary_lock:
            self._fusion_mode_counts[fusion_mode] = (
                self._fusion_mode_counts.get(fusion_mode, 0) + 1
            )
        for key, value in {
            "source_hz": source_hz,
            "effective_output_hz": metrics.get("effective_output_hz"),
            "capture_to_send_ms": metrics.get("capture_to_send_ms"),
            "record_queue_depth": metrics.get("record_queue_depth"),
            "record_dropped": metrics.get("record_dropped"),
            "raw_filter_rms_m": metrics.get("raw_filtered_rms_m"),
            "cross_view_mpjpe_m": agreement.get("mpjpe_m"),
            "mean_camera_fused": fusion_metrics.get("mean_camera_fused"),
            "camera_fps_min": min(camera_fps) if camera_fps else None,
            "camera_latency_max_ms": max(camera_latency) if camera_latency else None,
            "fusion_timestamp_stdev_ms": (
                1000.0 * fusion_metrics["mean_stdev_between_camera_s"]
                if isinstance(fusion_metrics.get("mean_stdev_between_camera_s"), (int, float))
                else None
            ),
            "rig_refinement_rms_m": rig_refinement.get("rms_m"),
            "pelvis_disagreement_m": agreement.get("pelvis_error_m"),
            "left_wrist_disagreement_m": agreement.get("left_wrist_error_m"),
            "right_wrist_disagreement_m": agreement.get("right_wrist_error_m"),
            "camera_timestamp_delta_ms": multi.get(
                "camera_timestamp_delta_ms"
            ),
        }.items():
            self._sample_quality(key, value)
        self._frame_writer.writerow(
            {
                "analysis_frame": self.frame_count,
                "timestamp_ns": timestamp_ns,
                "source_frame": source.get(
                    "source_frame_index",
                    source.get("frame_index", source.get("sequence")),
                ),
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
                "source_hz": source_hz,
                "effective_output_hz": metrics.get("effective_output_hz"),
                "capture_to_send_ms": metrics.get("capture_to_send_ms"),
                "record_queue_depth": metrics.get("record_queue_depth"),
                "record_dropped": metrics.get("record_dropped"),
                "raw_filter_rms_m": metrics.get("raw_filtered_rms_m"),
                "max_keypoint_speed_m_s": derived.get("quality", {}).get(
                    "max_keypoint_speed_m_s"
                ),
                "fusion_mode": multi.get("mode"),
                "evidence_views": multi.get("contributing_views"),
                "selected_single_serial": multi.get("selected_single_serial"),
                "calibration_agreement_ok": multi.get(
                    "calibration_agreement_ok"
                ),
                "cross_view_mpjpe_m": agreement.get("mpjpe_m"),
                "cross_view_p95_m": agreement.get("p95_error_m"),
                "core_disagreement_m": human_state.get("core_disagreement_m"),
                "pelvis_disagreement_m": agreement.get("pelvis_error_m"),
                "left_wrist_disagreement_m": agreement.get(
                    "left_wrist_error_m"
                ),
                "right_wrist_disagreement_m": agreement.get(
                    "right_wrist_error_m"
                ),
                "camera_timestamp_delta_ms": multi.get(
                    "camera_timestamp_delta_ms"
                ),
                "camera_sync_ok": multi.get("camera_sync_ok"),
                "fusion_failure_codes": json.dumps(
                    multi.get("failure_codes") or [], ensure_ascii=False
                ),
                "fused_quality_score": multi.get("fused_quality_score"),
                "best_single_quality_score": multi.get(
                    "best_single_quality_score"
                ),
                "mean_camera_fused": fusion_metrics.get("mean_camera_fused"),
                "camera_fps_min": min(camera_fps) if camera_fps else None,
                "camera_fps_max": max(camera_fps) if camera_fps else None,
                "camera_latency_max_ms": (
                    max(camera_latency) if camera_latency else None
                ),
                "fusion_timestamp_stdev_ms": (
                    1000.0 * fusion_metrics["mean_stdev_between_camera_s"]
                    if isinstance(
                        fusion_metrics.get("mean_stdev_between_camera_s"),
                        (int, float),
                    )
                    else None
                ),
                "left_arm_supporting_views": (
                    arm_evidence.get("left") or {}
                ).get("supporting_views"),
                "right_arm_supporting_views": (
                    arm_evidence.get("right") or {}
                ).get("supporting_views"),
                "rig_refinement_state": rig_refinement.get("state"),
                "rig_refinement_rms_m": rig_refinement.get("rms_m"),
                "rig_refinement_updates": rig_refinement.get(
                    "accepted_updates"
                ),
                **{
                    f"{name}_quality": value
                    for name, value in (human_state.get("segment_quality") or {}).items()
                    if name in {"pelvis", "torso", "left_arm", "right_arm", "left_leg", "right_leg"}
                },
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
                    "fusion_source": (human_state.get("joint_source") or {}).get(name),
                    "fusion_state": (human_state.get("joint_state") or {}).get(name),
                    "fusion_quality": (human_state.get("joint_quality") or {}).get(name),
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
        self._write_quality_summary()

    def close(self) -> None:
        self.flush()
        for stream in (
            self._json, self._frames, self._joints, self._angles,
            self._imitation_json, self._imitation_csv,
        ):
            stream.close()
        self.metadata["closed_unix_ns"] = time.time_ns()
        self._write_manifest()

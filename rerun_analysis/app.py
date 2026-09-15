"""Interactive, independent Rerun viewer for ZED BODY_38 skeleton analysis."""

from __future__ import annotations

import argparse
import json
import queue
import shutil
import socket
import sys
import sysconfig
import threading
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

PROJECT_ROOT = Path(__file__).resolve().parents[1]
ANALYSIS_PANEL = PROJECT_ROOT / "analysis_panel"
if str(ANALYSIS_PANEL) not in sys.path:
    sys.path.insert(0, str(ANALYSIS_PANEL))

from skeleton_analysis_recorder import KinematicAnalyzer  # noqa: E402
from motion_pipeline.metrics import latency_breakdown_ms  # noqa: E402

from rerun_analysis.model import (  # noqa: E402
    EDGES,
    ConfigStore,
    SkeletonConditioner,
    finite_point,
    joint_angle_names,
)
from rerun_analysis.session import AnalysisSessionWriter  # noqa: E402
from hand_tracking.rerun_output import HAND_EDGES  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ZED BODY_38 için bağımsız Rerun 3B analiz uygulaması"
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--input", type=Path, help="JSONL kaydını oynat")
    source.add_argument("--demo", action="store_true", help="Sentetik doğrulama")
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=15052)
    parser.add_argument("--gmr-listen-host", default="0.0.0.0")
    parser.add_argument("--gmr-listen-port", type=int, default=15053)
    parser.add_argument(
        "--live-max-hz", type=float, default=10.0,
        help="Canli BODY_38 analiz/render hizi; kontrol akisini etkilemez",
    )
    parser.add_argument(
        "--gmr-log-max-hz", type=float, default=10.0,
        help="Her GMR telemetri semasi icin azami kayit hizi",
    )
    parser.add_argument(
        "--detailed-joint-entities", action="store_true",
        help=(
            "Her BODY_38 eklemini ayri Rerun entity/metric olarak yaz. "
            "Kapaliyken sayisal CSV/JSONL verisi korunur ve canli RRD daha hizlidir."
        ),
    )
    parser.add_argument(
        "--output-dir", type=Path, default=PROJECT_ROOT / "rerun_recordings"
    )
    parser.add_argument("--svo2", type=Path, default=None, help="İlişkili özgün ZED SVO2")
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-realtime", action="store_true")
    parser.add_argument("--seconds", type=float, default=0.0)
    return parser.parse_args()


def source_packets(path: Path) -> Iterable[dict[str, Any]]:
    with path.open("r", encoding="utf-8-sig") as stream:
        for line in stream:
            if not line.strip():
                continue
            payload = json.loads(line)
            if payload.get("schema", "").endswith("/metadata/v1"):
                continue
            if "source_packet" in payload:
                payload = payload["source_packet"]
            if "keypoint_names" in payload:
                yield payload


def demo_packets() -> Iterable[dict[str, Any]]:
    names = [
        "PELVIS", "SPINE_1", "SPINE_2", "SPINE_3", "NECK", "NOSE",
        "LEFT_CLAVICLE", "LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST",
        "RIGHT_CLAVICLE", "RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST",
        "LEFT_HIP", "LEFT_KNEE", "LEFT_ANKLE", "LEFT_HEEL",
        "LEFT_BIG_TOE", "LEFT_SMALL_TOE", "RIGHT_HIP", "RIGHT_KNEE",
        "RIGHT_ANKLE", "RIGHT_HEEL", "RIGHT_BIG_TOE", "RIGHT_SMALL_TOE",
        "LEFT_HAND_THUMB_4", "LEFT_HAND_INDEX_1", "LEFT_HAND_MIDDLE_4",
        "LEFT_HAND_PINKY_1", "RIGHT_HAND_THUMB_4", "RIGHT_HAND_INDEX_1",
        "RIGHT_HAND_MIDDLE_4", "RIGHT_HAND_PINKY_1",
    ]
    for frame in range(180):
        phase = frame * 0.06
        base = {
            "PELVIS": (3.0, 0.0, 1.0), "SPINE_1": (3.0, 0.0, 1.15),
            "SPINE_2": (3.0, 0.0, 1.33), "SPINE_3": (3.0, 0.0, 1.50),
            "NECK": (3.0, 0.0, 1.68), "NOSE": (2.98, 0.0, 1.82),
            "LEFT_CLAVICLE": (3.0, 0.12, 1.52),
            "LEFT_SHOULDER": (3.0, 0.25, 1.50),
            "LEFT_ELBOW": (3.0, 0.50, 1.48 + 0.20 * np.sin(phase)),
            "LEFT_WRIST": (3.0, 0.72, 1.45 + 0.35 * np.sin(phase)),
            "RIGHT_CLAVICLE": (3.0, -0.12, 1.52),
            "RIGHT_SHOULDER": (3.0, -0.25, 1.50),
            "RIGHT_ELBOW": (3.0, -0.50, 1.48 - 0.20 * np.sin(phase)),
            "RIGHT_WRIST": (3.0, -0.72, 1.45 - 0.35 * np.sin(phase)),
            "LEFT_HIP": (3.0, 0.11, 0.98), "LEFT_KNEE": (3.0, 0.11, 0.55),
            "LEFT_ANKLE": (3.0, 0.11, 0.08), "LEFT_HEEL": (2.92, 0.11, 0.03),
            "LEFT_BIG_TOE": (3.16, 0.09, 0.03),
            "LEFT_SMALL_TOE": (3.15, 0.15, 0.03),
            "RIGHT_HIP": (3.0, -0.11, 0.98), "RIGHT_KNEE": (3.0, -0.11, 0.55),
            "RIGHT_ANKLE": (3.0, -0.11, 0.08),
            "RIGHT_HEEL": (2.92, -0.11, 0.03),
            "RIGHT_BIG_TOE": (3.16, -0.09, 0.03),
            "RIGHT_SMALL_TOE": (3.15, -0.15, 0.03),
        }
        for side, sign in (("LEFT", 1.0), ("RIGHT", -1.0)):
            wrist = base[f"{side}_WRIST"]
            base[f"{side}_HAND_THUMB_4"] = (wrist[0], wrist[1] + sign * 0.04, wrist[2])
            base[f"{side}_HAND_INDEX_1"] = (
                wrist[0], wrist[1] + sign * 0.06, wrist[2] + 0.02
            )
            base[f"{side}_HAND_MIDDLE_4"] = (
                wrist[0], wrist[1] + sign * 0.07, wrist[2]
            )
            base[f"{side}_HAND_PINKY_1"] = (
                wrist[0], wrist[1] + sign * 0.05, wrist[2] - 0.02
            )
        yield {
            "schema": "zed_body38_live/v1",
            "sequence": frame,
            "frame_index": frame,
            "timestamp_ns": 1_000_000_000 + frame * 66_666_667,
            "body_id": 0,
            "tracking_state": "OK",
            "body_confidence": 97.0,
            "keypoint_names": names,
            "keypoints_3d_m": [list(base[name]) for name in names],
            "keypoints_3d_raw_m": [list(base[name]) for name in names],
            "keypoint_confidence": [95.0] * len(names),
            "local_orientation_per_joint_xyzw": [[0.0, 0.0, 0.0, 1.0]] * len(names),
            "root_position_m": list(base["PELVIS"]),
            "transport_metrics": {
                "source_interval_ms": 66.67,
                "capture_to_send_ms": 12.0,
                "raw_filtered_rms_m": 0.004,
            },
        }


def make_blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(
                rrb.Spatial3DView(
                    name="ZED BODY_38 — retarget öncesi",
                    origin="/world/skeleton",
                    contents=["+ $origin/**"],
                    line_grid=True,
                ),
                rrb.Spatial3DView(
                    name="İnsan / G1 raw / G1 safe",
                    origin="/comparison",
                    contents=["+ $origin/**"],
                    line_grid=True,
                ),
                column_shares=[1, 1],
            ),
            rrb.Horizontal(
                rrb.TimeSeriesView(
                    name="Eklem açıları (deg)", origin="/world/metrics/angles"
                ),
                rrb.TimeSeriesView(
                    name="Takip kalitesi ve gecikme",
                    origin="/world/metrics/quality",
                ),
                rrb.TimeSeriesView(
                    name="Dex3 el hedefleri (rad)",
                    origin="/world/metrics/dex3",
                ),
                column_shares=[3, 2, 2],
            ),
            row_shares=[3, 2],
        ),
        rrb.SelectionPanel(state="expanded"),
        rrb.TimePanel(state="expanded"),
        rrb.BlueprintPanel(state="collapsed"),
    )


def find_rerun_viewer() -> Path:
    """Locate the Viewer installed by rerun-sdk even when Scripts is not on PATH."""
    executable = "rerun.exe" if sys.platform == "win32" else "rerun"
    on_path = shutil.which(executable)
    if on_path:
        return Path(on_path)
    scripts_candidate = Path(sysconfig.get_path("scripts")) / executable
    if scripts_candidate.is_file():
        return scripts_candidate
    raise RuntimeError(
        "Rerun Viewer executable bulunamadı. "
        "Kurulum: python -m pip install -r requirements-rerun.txt"
    )


class RerunSkeletonApp:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.config = ConfigStore()
        self.conditioner = SkeletonConditioner()
        self.analyzer = KinematicAnalyzer()
        self._logged_joint_names: set[str] = set()
        self.latest_gmr: dict[str, Any] = {}
        self._last_gmr_log_s: dict[str, float] = {}
        self.stop_event = threading.Event()
        self.status_queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=3)
        source = (
            f"jsonl:{args.input.resolve()}" if args.input
            else "demo" if args.demo
            else f"udp:{args.listen_host}:{args.listen_port}"
        )
        stamp = time.strftime("%Y%m%d_%H%M%S")
        args.output_dir.mkdir(parents=True, exist_ok=True)
        self.rrd_path = args.output_dir / f"rerun_body38_{stamp}.rrd"
        rr.init("zed_g1_body38_rerun_analysis")
        if not args.no_viewer:
            rr.spawn(executable_path=str(find_rerun_viewer()))
            rr.set_sinks(rr.GrpcSink(), rr.FileSink(self.rrd_path))
        else:
            rr.set_sinks(rr.FileSink(self.rrd_path))
        rr.send_blueprint(make_blueprint())
        rr.log("/world", rr.ViewCoordinates.FLU, static=True)
        rr.log("/comparison", rr.ViewCoordinates.FLU, static=True)
        rr.log(
            "/world/session_info",
            rr.TextDocument(
                "ZED 2i BODY_38 → G1 EDU algı analizidir; motor komutu üretmez.",
                media_type="text/markdown",
            ),
            static=True,
        )
        self.writer = AnalysisSessionWriter(
            args.output_dir,
            source_description=source,
            rrd_path=self.rrd_path,
            svo2_path=args.svo2,
            config=self.config.snapshot(),
        )
        self.worker = threading.Thread(target=self._run_source, daemon=True)
        self.gmr_worker = threading.Thread(target=self._run_gmr_telemetry, daemon=True)

    def _log_gmr_packet(self, packet: dict[str, Any]) -> None:
        self.latest_gmr = packet
        frame = int(packet.get("sequence", 0) or 0)
        rr.set_time("frame", sequence=frame)
        skeleton = packet.get("g1_skeleton") or {}
        edges = skeleton.get("edges") or []
        for variant, color, lateral_offset in (
            ("raw", [255, 170, 50], -0.65),
            ("safe", [80, 230, 120], 0.65),
        ):
            positions = skeleton.get(f"{variant}_positions_m") or {}
            shifted = {
                name: [float(value[0]), float(value[1]) + lateral_offset, float(value[2])]
                for name, value in positions.items()
                if isinstance(value, (list, tuple)) and len(value) == 3
                and all(isinstance(item, (int, float)) and np.isfinite(item) for item in value)
            }
            strips = [
                [shifted[first], shifted[second]]
                for first, second in edges
                if first in shifted and second in shifted
            ]
            root = f"/world/g1_{variant}"
            if strips:
                rr.log(f"{root}/bones", rr.LineStrips3D(strips, radii=0.012, colors=color))
                rr.log(
                    f"{root}/joints",
                    rr.Points3D(
                        list(shifted.values()), radii=0.022, colors=color,
                        labels=list(shifted.keys()), show_labels=False,
                    ),
                )
            else:
                rr.log(root, rr.Clear(recursive=True))

        comparison = packet.get("retarget_comparison") or {}
        comparison_variants = (
            (
                "human_pre_gmr",
                comparison.get("human_positions_m") or {},
                comparison.get("human_edges") or [],
                [70, 190, 255],
                -0.80,
                "HUMAN PRE-GMR",
            ),
            (
                "g1_raw",
                skeleton.get("raw_positions_m") or {},
                edges,
                [255, 170, 50],
                0.0,
                "G1 RAW",
            ),
            (
                "g1_safe",
                skeleton.get("safe_positions_m") or {},
                edges,
                [80, 230, 120],
                0.80,
                "G1 SAFE",
            ),
        )
        for variant, positions, variant_edges, color, offset, label in comparison_variants:
            shifted = {
                str(name): [float(value[0]), float(value[1]) + offset, float(value[2])]
                for name, value in positions.items()
                if isinstance(value, (list, tuple)) and len(value) == 3
                and all(
                    isinstance(item, (int, float)) and np.isfinite(item)
                    for item in value
                )
            }
            strips = [
                [shifted[first], shifted[second]]
                for first, second in variant_edges
                if first in shifted and second in shifted
            ]
            root = f"/comparison/{variant}"
            if not strips:
                rr.log(root, rr.Clear(recursive=True))
                continue
            rr.log(
                f"{root}/bones",
                rr.LineStrips3D(strips, radii=0.012, colors=color),
            )
            rr.log(
                f"{root}/joints",
                rr.Points3D(list(shifted.values()), radii=0.022, colors=color),
            )
            if "pelvis" in shifted:
                anchor = list(shifted["pelvis"])
                anchor[2] += 0.12
                rr.log(
                    f"{root}/label",
                    rr.Points3D(
                        [anchor], radii=0.001, colors=color,
                        labels=[label], show_labels=True,
                    ),
                )
        joint_names = packet.get("joint_names") or []
        for variant, key in (
            ("raw_q", "raw_joint_position_rad"),
            ("safe_q", "safe_joint_position_rad"),
        ):
            values = packet.get(key) or []
            for name, value in zip(joint_names, values):
                if isinstance(value, (int, float)) and np.isfinite(value):
                    rr.log(f"/world/metrics/{variant}/{name}", rr.Scalars(float(value)))
        safety = packet.get("safety") or {}
        bridge = packet.get("bridge_metrics") or {}
        latency = latency_breakdown_ms(packet.get("latency_trace_ns") or {})
        isaac = packet.get("isaac_metrics") or {}
        rr.log(
            "/world/g1_safety",
            rr.AnyValues(
                level=str(safety.get("level", "UNKNOWN")),
                reasons=json.dumps(safety.get("reasons", []), ensure_ascii=False),
                blend=float(safety.get("blend", 0.0)),
                joint_limit_saturation=int(safety.get("joint_limit_saturation", 0)),
                joint_limit_saturation_names=json.dumps(
                    safety.get("joint_limit_saturation_names", []),
                    ensure_ascii=False,
                ),
                self_collision_count=int(safety.get("safe_self_collision_count", 0)),
                solve_ms=float(bridge.get("solve_ms", 0.0)),
                ik_position_max_m=float(bridge.get("ik_position_max_m", 0.0)),
            ),
        )
        for name, value in {
            **latency,
            **isaac,
            **(packet.get("system_metrics") or {}),
        }.items():
            if isinstance(value, (int, float)) and np.isfinite(value):
                rr.log(f"/world/metrics/system/{name}", rr.Scalars(float(value)))
        reference_motion = packet.get("reference_motion") or {}
        reference_names = reference_motion.get("joint_names") or []
        for metric_name, field_name in (
            ("reference_q", "position_rad"),
            ("reference_qd", "velocity_rad_s"),
            ("reference_qdd", "acceleration_rad_s2"),
        ):
            for name, value in zip(reference_names, reference_motion.get(field_name) or []):
                if isinstance(value, (int, float)) and np.isfinite(value):
                    rr.log(
                        f"/world/metrics/{metric_name}/{name}",
                        rr.Scalars(float(value)),
                    )
        reference_confidence = reference_motion.get("confidence")
        if isinstance(reference_confidence, (int, float)) and np.isfinite(
            reference_confidence
        ):
            rr.log(
                "/world/metrics/reference/confidence",
                rr.Scalars(float(reference_confidence)),
            )
        for name in (
            "solve_ms", "source_age_ms", "ik_position_mean_m",
            "ik_position_max_m", "ik_upper_relative_residual_m",
            "left_direct_arm_ik_rms_m", "right_direct_arm_ik_rms_m",
            "left_straight_arm_blend", "right_straight_arm_blend",
            "left_direct_ik_candidate_count",
            "right_direct_ik_candidate_count", "mirror_rescue_solve_ms",
        ):
            value = bridge.get(name)
            if isinstance(value, (int, float)) and np.isfinite(value):
                rr.log(f"/world/metrics/gmr/{name}", rr.Scalars(float(value)))
        writer = getattr(self, "writer", None)
        if writer is not None:
            writer.write_imitation(packet)

    def _run_gmr_telemetry(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        sock.bind((self.args.gmr_listen_host, self.args.gmr_listen_port))
        sock.settimeout(0.2)
        drain_deadline = None
        try:
            while True:
                if self.stop_event.is_set() and drain_deadline is None:
                    # Isaac telemetry trails the source stream slightly. Drain
                    # the socket before quality_summary is finalized.
                    drain_deadline = time.monotonic() + 0.60
                if drain_deadline is not None and time.monotonic() >= drain_deadline:
                    break
                try:
                    payload, _ = sock.recvfrom(2_000_000)
                except socket.timeout:
                    continue
                try:
                    packet = json.loads(payload.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue
                if packet.get("schema") in {
                    "zed_gmr_g1_23dof_live/v1",
                    "zed_gmr_g1_23dof_isaac_telemetry/v1",
                }:
                    schema = str(packet.get("schema"))
                    now = time.monotonic()
                    last = self._last_gmr_log_s.get(schema, -np.inf)
                    if (
                        self.args.gmr_log_max_hz > 0
                        and now - last < 0.98 / self.args.gmr_log_max_hz
                    ):
                        continue
                    self._last_gmr_log_s[schema] = now
                    self._log_gmr_packet(packet)
                    if drain_deadline is not None:
                        drain_deadline = min(
                            time.monotonic() + 0.20,
                            drain_deadline + 0.10,
                        )
        finally:
            sock.close()

    def _log_hand_packet(self, source: dict[str, Any]) -> None:
        """Log fused 21-landmark hands and safe Dex3 analysis targets."""
        fused = source.get("hand_tracking") or {}
        hands_by_side = {
            str(hand.get("side")): hand
            for hand in fused.get("hands", [])
            if isinstance(hand, dict) and hand.get("side") in {"left", "right"}
        }
        for side, color in (("left", [80, 220, 120]), ("right", [255, 170, 50])):
            root = f"/world/skeleton/hands/{side}"
            hand = hands_by_side.get(side)
            points_value = hand.get("landmarks_world_m") if hand else None
            try:
                points = np.asarray(points_value, dtype=np.float64)
            except (TypeError, ValueError):
                points = np.empty((0, 3), dtype=np.float64)
            valid = bool(
                hand
                and hand.get("valid")
                and points.shape == (21, 3)
                and np.isfinite(points).all()
            )
            if not valid:
                rr.log(root, rr.Clear(recursive=True))
                continue
            rr.log(
                f"{root}/landmarks",
                rr.Points3D(points, radii=0.008, colors=color),
            )
            rr.log(
                f"{root}/bones",
                rr.LineStrips3D(
                    [[points[first], points[second]] for first, second in HAND_EDGES],
                    radii=0.004,
                    colors=color,
                ),
            )
            normalization = hand.get("normalization") or {}
            try:
                origin = np.asarray(normalization["wrist_world_m"], dtype=np.float64)
                rotation = np.asarray(
                    normalization["rotation_world_from_palm"], dtype=np.float64
                )
                scale = float(normalization["scale_m"])
                axes_valid = (
                    origin.shape == (3,)
                    and rotation.shape == (3, 3)
                    and np.isfinite(origin).all()
                    and np.isfinite(rotation).all()
                    and np.isfinite(scale)
                    and scale > 0.0
                )
            except (KeyError, TypeError, ValueError):
                axes_valid = False
            if axes_valid:
                for axis, axis_color in enumerate(
                    ([255, 0, 0], [0, 255, 0], [0, 128, 255])
                ):
                    rr.log(
                        f"{root}/palm_axis_{axis}",
                        rr.Arrows3D(
                            origins=[origin],
                            vectors=[rotation[:, axis] * scale],
                            colors=[axis_color],
                        ),
                    )
            qualities = hand.get("landmark_quality") or []
            camera_counts = [
                float(item["camera_count"])
                for item in qualities
                if isinstance(item, dict)
                and isinstance(item.get("camera_count"), (int, float))
            ]
            confidences = [
                float(item["confidence"])
                for item in qualities
                if isinstance(item, dict)
                and isinstance(item.get("confidence"), (int, float))
            ]
            rr.log(
                f"{root}/status",
                rr.AnyValues(
                    valid=True,
                    capture_spread_ms=float(hand.get("capture_spread_ms", 0.0)),
                    mean_camera_count=(
                        float(np.mean(camera_counts)) if camera_counts else 0.0
                    ),
                    mean_confidence=(
                        float(np.mean(confidences)) if confidences else 0.0
                    ),
                    rejection_reasons=json.dumps(
                        hand.get("rejection_reasons") or [], ensure_ascii=False
                    ),
                ),
            )

        targets = source.get("dex3_targets") or {}
        for side in ("left", "right"):
            target = targets.get(side) or {}
            values = target.get("safe_q_rad") or []
            if not values:
                rr.log(f"/world/metrics/dex3/{side}", rr.Clear(recursive=True))
                continue
            for index, value in enumerate(values):
                if isinstance(value, (int, float)) and np.isfinite(value):
                    rr.log(
                        f"/world/metrics/dex3/{side}/q_{index}",
                        rr.Scalars(float(value)),
                    )
        if targets:
            rr.log(
                "/world/skeleton/hands/output_safety",
                rr.AnyValues(
                    physical_robot_output_enabled=bool(
                        targets.get("physical_robot_output_enabled", False)
                    ),
                    composition_status=str(targets.get("composition_status", "")),
                ),
            )

    def _log_packet(
        self,
        source: dict[str, Any],
        processed: dict[str, Any],
        derived: dict[str, Any],
        states: dict[str, str],
        config,
    ) -> None:
        timestamp_ns = int(source.get("timestamp_ns", 0) or time.time_ns())
        frame = int(source.get("frame_index", source.get("sequence", 0)) or 0)
        rr.set_time("frame", sequence=frame)
        rr.set_time("source_time", timestamp=timestamp_ns / 1e9)
        self._log_hand_packet(source)
        names = [str(name) for name in processed.get("keypoint_names", [])]
        points_list = processed.get("keypoints_3d_m", [])
        points = {
            name: point
            for index, name in enumerate(names)
            if index < len(points_list)
            and (point := finite_point(points_list[index])) is not None
        }
        confidences = source.get("keypoint_confidence", [])
        raw_points = (
            source.get("keypoints_3d_raw_m") or source.get("keypoints_3d_m") or []
        )
        quaternions = source.get("local_orientation_per_joint_xyzw", [])
        eulers = derived.get("local_orientation_euler_xyz_deg", {})
        velocities = derived.get("keypoint_velocity_m_s", {})
        speeds = derived.get("keypoint_speed_m_s", {})
        errors = derived.get("raw_filter_error_m", {})
        angles = derived.get("joint_angles_deg", {})
        angle_velocity = derived.get("joint_angle_velocity_deg_s", {})
        human_state = source.get("human_state") or {}
        fusion_joint_quality = human_state.get("joint_quality") or {}
        fusion_joint_source = human_state.get("joint_source") or {}
        fusion_joint_state = human_state.get("joint_state") or {}

        strips = [
            [points[first], points[second]]
            for first, second in EDGES
            if first in points and second in points
        ]
        if strips:
            rr.log(
                "/world/skeleton/bones",
                rr.LineStrips3D(strips, radii=0.009, colors=[70, 190, 255]),
            )
            rr.log(
                "/fusion/body38_constrained",
                rr.LineStrips3D(strips, radii=0.010, colors=[70, 230, 130]),
            )
        else:
            rr.log("/world/skeleton/bones", rr.Clear(recursive=False))
            rr.log("/fusion/body38_constrained", rr.Clear(recursive=False))

        # The analysis-only UDP copy carries calibrated raw camera skeletons;
        # the WSL/GMR control packet remains compact to avoid fragmentation.
        multi_camera = source.get("multi_camera") or {}
        camera_views = multi_camera.get("per_camera") or []
        views_by_serial = {
            int(view.get("serial_number")): view
            for view in camera_views
            if isinstance(view, dict) and view.get("serial_number") is not None
        }
        camera_serials = sorted(
            int(serial)
            for serial in (
                multi_camera.get("configured_serials")
                or multi_camera.get("connected_serials")
                or views_by_serial
            )
        )[:4]
        camera_colors = (
            [255, 180, 50],
            [180, 90, 255],
            [60, 220, 110],
            [70, 170, 255],
        )
        for camera_index, serial in enumerate(camera_serials, start=1):
            view = views_by_serial.get(serial, {})
            raw_view = view.get("keypoints_3d_fusion_m") or []
            camera_points = {
                name: point
                for index, name in enumerate(names)
                if index < len(raw_view)
                and (point := finite_point(raw_view[index])) is not None
            }
            camera_strips = [
                [camera_points[first], camera_points[second]]
                for first, second in EDGES
                if first in camera_points and second in camera_points
            ]
            entity = f"/cam{camera_index}/body38"
            if camera_strips:
                color = camera_colors[camera_index - 1]
                rr.log(entity, rr.LineStrips3D(camera_strips, radii=0.006, colors=color))
            else:
                rr.log(entity, rr.Clear(recursive=False))
            rr.log(
                f"/cam{camera_index}",
                rr.AnyValues(serial_number=serial, has_current_body=bool(camera_strips)),
            )
        for camera_index in range(len(camera_serials) + 1, 5):
            rr.log(f"/cam{camera_index}/body38", rr.Clear(recursive=False))

        visible_joint_names = [name for name in names if name in points]
        if visible_joint_names:
            joint_colors = []
            for name in visible_joint_names:
                state = states.get(name, "missing")
                joint_colors.append(
                    [80, 220, 120]
                    if state == "tracked"
                    else [255, 190, 50]
                    if state in {"held", "held_outlier"}
                    else [240, 80, 80]
                )
            rr.log(
                "/world/skeleton/joints",
                rr.Points3D(
                    [points[name] for name in visible_joint_names],
                    radii=0.025,
                    colors=joint_colors,
                    labels=visible_joint_names,
                    show_labels=False,
                ),
            )
        else:
            rr.log("/world/skeleton/joints", rr.Clear(recursive=False))

        # The normal live path batches all joints into one Rerun entity.  Full
        # per-joint values remain losslessly available in joints.csv and
        # skeleton_analysis.jsonl.  The opt-in detailed mode is intended for
        # slower offline inspection and creates over 100 log calls per frame.
        current_joint_names = (
            set(names) if self.args.detailed_joint_entities else set()
        )
        for removed_name in self._logged_joint_names - current_joint_names:
            rr.log(
                f"/world/skeleton/joints_detail/{removed_name}",
                rr.Clear(recursive=False),
            )
        self._logged_joint_names = current_joint_names

        detailed_names = names if self.args.detailed_joint_entities else ()
        for index, name in enumerate(detailed_names):
            entity = f"/world/skeleton/joints_detail/{name}"
            if name not in points:
                # Rerun keeps the latest component value until explicitly
                # cleared. Without this, an occluded joint floats indefinitely.
                rr.log(entity, rr.Clear(recursive=False))
                continue
            confidence = (
                float(confidences[index])
                if index < len(confidences)
                and isinstance(confidences[index], (int, float))
                else 0.0
            )
            state = states.get(name, "missing")
            color = (
                [80, 220, 120] if state == "tracked"
                else [255, 190, 50] if state in {"held", "held_outlier"}
                else [240, 80, 80]
            )
            rr.log(
                entity,
                rr.Points3D(
                    [points[name]],
                    radii=0.025,
                    colors=color,
                    labels=[name],
                    show_labels=name in {
                        "PELVIS", "NECK", "LEFT_WRIST", "RIGHT_WRIST",
                        "LEFT_ANKLE", "RIGHT_ANKLE",
                    },
                ),
            )
            raw = finite_point(raw_points[index]) if index < len(raw_points) else None
            quat = (
                quaternions[index]
                if index < len(quaternions)
                and isinstance(quaternions[index], (list, tuple))
                and len(quaternions[index]) == 4
                else [None] * 4
            )
            euler = eulers.get(name) or [None] * 3
            velocity = velocities.get(name) or [None] * 3
            related = {
                angle_name: angles.get(angle_name)
                for angle_name in joint_angle_names(name, angles)
            }
            values = {
                "joint_name": name,
                "tracking_source": state,
                "fusion_source": str(fusion_joint_source.get(name, "unknown")),
                "fusion_state": str(fusion_joint_state.get(name, "unknown")),
                "fusion_quality": fusion_joint_quality.get(name),
                "confidence_percent": confidence,
                "position_x_m": points[name][0],
                "position_y_m": points[name][1],
                "position_z_m": points[name][2],
                "speed_m_s": speeds.get(name),
                "raw_filter_error_m": errors.get(name),
                "velocity_x_m_s": velocity[0],
                "velocity_y_m_s": velocity[1],
                "velocity_z_m_s": velocity[2],
                "quaternion_x": quat[0], "quaternion_y": quat[1],
                "quaternion_z": quat[2], "quaternion_w": quat[3],
                "euler_x_deg": euler[0], "euler_y_deg": euler[1],
                "euler_z_deg": euler[2],
                "raw_position_x_m": raw[0] if raw else None,
                "raw_position_y_m": raw[1] if raw else None,
                "raw_position_z_m": raw[2] if raw else None,
                **{f"angle_{key}": value for key, value in related.items()},
            }
            rr.log(entity, rr.AnyValues(**values))
            if speeds.get(name) is not None:
                rr.log(
                    f"/world/metrics/joint_speed/{name}",
                    rr.Scalars(float(speeds[name])),
                )
            fusion_quality = fusion_joint_quality.get(name)
            if isinstance(fusion_quality, (int, float)) and np.isfinite(fusion_quality):
                rr.log(
                    f"/world/metrics/fusion/joint_quality/{name}",
                    rr.Scalars(float(fusion_quality)),
                )

        for name, value in angles.items():
            if value is not None:
                rr.log(f"/world/metrics/angles/{name}", rr.Scalars(float(value)))
            if angle_velocity.get(name) is not None:
                rr.log(
                    f"/world/metrics/angular_velocity/{name}",
                    rr.Scalars(float(angle_velocity[name])),
                )
        quality = derived.get("quality", {})
        visibility = derived.get("visibility", {})
        transport = source.get("transport_metrics", {})
        perception = source.get("perception_metrics", {})
        multi = source.get("multi_camera") or {}
        agreement = multi.get("cross_view_agreement") or {}
        aligned_agreement = multi.get("post_alignment_agreement") or {}
        fusion_metrics = multi.get("fusion_metrics") or {}
        rig_refinement = (
            multi.get("rig_extrinsics")
            or multi.get("online_rig_refinement")
            or {}
        )
        quality_values = {
            "body_confidence_percent": source.get("body_confidence"),
            "valid_keypoints": visibility.get("valid_keypoints"),
            "confident_keypoints": visibility.get("confident_keypoints"),
            "max_joint_speed_m_s": quality.get("max_keypoint_speed_m_s"),
            "median_filter_error_m": quality.get("median_raw_filter_error_m"),
            "capture_to_send_ms": transport.get("capture_to_send_ms"),
            "source_interval_ms": transport.get("source_interval_ms"),
            "effective_output_hz": transport.get("effective_output_hz"),
            "record_queue_depth": transport.get("record_queue_depth"),
            "record_dropped": transport.get("record_dropped"),
            "visible_keypoint_ratio": perception.get("visible_keypoint_ratio"),
            "bone_length_error": perception.get("bone_length_relative_error_mean"),
            "left_right_swap_count": perception.get("left_right_swap_count"),
            "evidence_views": multi.get("contributing_views"),
            "cross_view_mpjpe_m": agreement.get("mpjpe_m"),
            "cross_view_p95_m": agreement.get("p95_error_m"),
            "post_alignment_mpjpe_m": aligned_agreement.get("mpjpe_m"),
            "post_alignment_p95_m": aligned_agreement.get("p95_error_m"),
            "core_disagreement_m": human_state.get("core_disagreement_m"),
            "pelvis_disagreement_m": agreement.get("pelvis_error_m"),
            "left_wrist_disagreement_m": agreement.get("left_wrist_error_m"),
            "right_wrist_disagreement_m": agreement.get(
                "right_wrist_error_m"
            ),
            "camera_timestamp_delta_ms": multi.get(
                "camera_timestamp_delta_ms"
            ),
            "fused_quality_score": multi.get("fused_quality_score"),
            "best_single_quality_score": multi.get(
                "best_single_quality_score"
            ),
            "mean_camera_fused": fusion_metrics.get("mean_camera_fused"),
            "rig_refinement_rms_m": rig_refinement.get("rms_m"),
            "rig_refinement_updates": rig_refinement.get("accepted_updates"),
        }
        for name, value in quality_values.items():
            if isinstance(value, (int, float)) and np.isfinite(value):
                rr.log(f"/world/metrics/quality/{name}", rr.Scalars(float(value)))
        for name, value in (agreement.get("per_joint_error_m") or {}).items():
            if isinstance(value, (int, float)) and np.isfinite(value):
                rr.log(
                    f"/world/metrics/fusion/disagreement/{name}",
                    rr.Scalars(float(value)),
                )
        for name, value in (human_state.get("segment_quality") or {}).items():
            if isinstance(value, (int, float)) and np.isfinite(value):
                rr.log(
                    f"/world/metrics/fusion/segment_quality/{name}",
                    rr.Scalars(float(value)),
                )
        rr.log(
            "/world/fusion_state",
            rr.AnyValues(
                failure_codes=json.dumps(human_state.get("failure_codes") or []),
                elbow_state=json.dumps(human_state.get("elbow_state") or {}),
            ),
        )
        rr.log(
            "/world/effective_parameters",
            rr.AnyValues(
                **{
                    key: value
                    for key, value in asdict(config).items()
                    if isinstance(value, (str, int, float, bool))
                }
            ),
        )

    def _receive_udp(self) -> Iterable[dict[str, Any]]:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        sock.bind((self.args.listen_host, self.args.listen_port))
        sock.settimeout(0.2)
        try:
            while not self.stop_event.is_set():
                try:
                    payload, _address = sock.recvfrom(2_000_000)
                except socket.timeout:
                    continue
                # Rendering and RRD writes are intentionally slower than the
                # lossless 15 Hz JSONL recorder.  Never replay an accumulated
                # UDP backlog: drain it and visualize the newest complete
                # sample so the live window remains current instead of several
                # seconds behind the robot.
                sock.setblocking(False)
                try:
                    while True:
                        newest, _address = sock.recvfrom(2_000_000)
                        payload = newest
                except BlockingIOError:
                    pass
                finally:
                    sock.settimeout(0.2)
                try:
                    packet = json.loads(payload.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue
                if "keypoint_names" in packet:
                    yield packet
        finally:
            sock.close()

    def _run_source(self) -> None:
        packets = (
            source_packets(self.args.input) if self.args.input
            else demo_packets() if self.args.demo
            else self._receive_udp()
        )
        previous_timestamp = None
        last_live_process_s = -np.inf
        started = time.monotonic()
        try:
            for source in packets:
                if self.stop_event.is_set():
                    break
                if not self.args.input and not self.args.demo:
                    now = time.monotonic()
                    if (
                        self.args.live_max_hz > 0
                        and now - last_live_process_s
                        < 0.98 / self.args.live_max_hz
                    ):
                        continue
                    last_live_process_s = now
                while self.config.snapshot().paused and not self.stop_event.is_set():
                    time.sleep(0.03)
                config = self.config.snapshot()
                self.analyzer.confidence_threshold = config.confidence_threshold
                processed, states = self.conditioner.process(source, config)
                derived = self.analyzer.analyze(processed)
                self._log_packet(source, processed, derived, states, config)
                if config.capture_enabled:
                    self.writer.write(source, processed, derived, config, states)
                latest = {
                    "source": source, "processed": processed, "derived": derived,
                    "states": states, "frame_count": self.writer.frame_count,
                }
                try:
                    while True:
                        self.status_queue.get_nowait()
                except queue.Empty:
                    pass
                self.status_queue.put_nowait(latest)
                timestamp = int(source.get("timestamp_ns", 0) or 0)
                if (
                    self.args.input and not self.args.no_realtime
                    and previous_timestamp is not None and timestamp > previous_timestamp
                ):
                    delay = (timestamp - previous_timestamp) / 1e9
                    time.sleep(min(0.25, delay / max(0.05, config.playback_rate)))
                previous_timestamp = timestamp or previous_timestamp
                if self.args.seconds and time.monotonic() - started >= self.args.seconds:
                    break
        finally:
            self.stop_event.set()

    def start(self) -> int:
        self.worker.start()
        if not self.args.input and not self.args.demo:
            self.gmr_worker.start()
        if self.args.headless:
            self.worker.join()
        else:
            self._run_ui()
        self.stop_event.set()
        self.worker.join(timeout=3.0)
        if self.gmr_worker.is_alive():
            self.gmr_worker.join(timeout=2.0)
        self.writer.close()
        rr.disconnect()
        print(f"Rerun RRD: {self.rrd_path}")
        print(f"Analiz oturumu: {self.writer.session_dir}")
        return 0

    def _run_ui(self) -> None:
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk

        root = tk.Tk()
        root.title("ZED BODY_38 → Rerun 3B Analiz Kontrolü")
        root.geometry("760x760")
        root.protocol("WM_DELETE_WINDOW", lambda: self.stop_event.set())
        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True, padx=8, pady=8)
        live_tab, filter_tab, joint_tab, session_tab = (
            ttk.Frame(notebook) for _ in range(4)
        )
        notebook.add(live_tab, text="Canlı durum")
        notebook.add(filter_tab, text="Filtre ve oynatma")
        notebook.add(joint_tab, text="Eklem düzenleme")
        notebook.add(session_tab, text="Oturum")

        status_var = tk.StringVar(value="Veri bekleniyor…")
        ttk.Label(live_tab, textvariable=status_var, justify="left").pack(
            anchor="nw", padx=12, pady=12
        )
        detail = tk.Text(live_tab, height=28, width=88)
        detail.pack(fill="both", expand=True, padx=12, pady=8)

        variables = {
            "confidence_threshold": tk.DoubleVar(value=35.0),
            "smoothing_alpha": tk.DoubleVar(value=0.55),
            "scale": tk.DoubleVar(value=1.0),
            "offset_x_m": tk.DoubleVar(value=0.0),
            "offset_y_m": tk.DoubleVar(value=0.0),
            "offset_z_m": tk.DoubleVar(value=0.0),
            "max_joint_speed_m_s": tk.DoubleVar(value=6.0),
            "occlusion_hold_frames": tk.IntVar(value=3),
            "playback_rate": tk.DoubleVar(value=1.0),
        }
        controls = (
            ("Güven eşiği (%)", "confidence_threshold", 0.0, 100.0, 1.0),
            ("EMA alpha", "smoothing_alpha", 0.05, 1.0, 0.05),
            ("İskelet ölçeği", "scale", 0.5, 1.5, 0.01),
            ("Global X ofseti (m)", "offset_x_m", -1.0, 1.0, 0.01),
            ("Global Y ofseti (m)", "offset_y_m", -1.0, 1.0, 0.01),
            ("Global Z ofseti (m)", "offset_z_m", -1.0, 1.0, 0.01),
            ("Azami eklem hızı (m/s)", "max_joint_speed_m_s", 0.5, 15.0, 0.1),
            ("Örtülme tutma (kare)", "occlusion_hold_frames", 0, 30, 1),
            ("Oynatma hızı", "playback_rate", 0.1, 4.0, 0.1),
        )
        for row, (label, key, minimum, maximum, resolution) in enumerate(controls):
            ttk.Label(filter_tab, text=label).grid(
                row=row, column=0, sticky="w", padx=10, pady=7
            )
            tk.Scale(
                filter_tab, variable=variables[key], from_=minimum, to=maximum,
                resolution=resolution, orient="horizontal", length=420,
                command=lambda _value, k=key: self.config.update(
                    **{k: variables[k].get()}
                ),
            ).grid(row=row, column=1, sticky="ew", padx=10)
        paused_var = tk.BooleanVar(value=False)
        capture_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            filter_tab, text="Oynatmayı duraklat", variable=paused_var,
            command=lambda: self.config.update(paused=paused_var.get()),
        ).grid(row=len(controls), column=0, columnspan=2, sticky="w", padx=10, pady=8)
        ttk.Checkbutton(
            filter_tab, text="JSONL/CSV kaydını etkinleştir", variable=capture_var,
            command=lambda: self.config.update(capture_enabled=capture_var.get()),
        ).grid(
            row=len(controls) + 1, column=0, columnspan=2,
            sticky="w", padx=10, pady=8,
        )

        joint_var = tk.StringVar(value="PELVIS")
        joint_combo = ttk.Combobox(joint_tab, textvariable=joint_var, state="readonly")
        joint_combo.pack(fill="x", padx=12, pady=12)
        joint_offset_vars = [tk.DoubleVar(value=0.0) for _ in range(3)]
        for axis, variable in zip("XYZ", joint_offset_vars):
            frame = ttk.Frame(joint_tab)
            frame.pack(fill="x", padx=12, pady=6)
            ttk.Label(frame, text=f"{axis} eklem ofseti (m)", width=22).pack(side="left")
            tk.Scale(
                frame, variable=variable, from_=-0.5, to=0.5,
                resolution=0.005, orient="horizontal", length=430,
                command=lambda _value: self.config.set_joint_offset(
                    joint_var.get(), [item.get() for item in joint_offset_vars]
                ),
            ).pack(side="left", fill="x", expand=True)
        joint_details = tk.Text(joint_tab, height=24, width=88)
        joint_details.pack(fill="both", expand=True, padx=12, pady=10)

        def joint_selected(_event=None) -> None:
            self.config.update(selected_joint=joint_var.get())
            offset = self.config.snapshot().joint_offsets_m.get(
                joint_var.get(), [0.0, 0.0, 0.0]
            )
            for variable, value in zip(joint_offset_vars, offset):
                variable.set(value)

        joint_combo.bind("<<ComboboxSelected>>", joint_selected)
        paths_text = tk.Text(session_tab, height=20, width=88)
        paths_text.pack(fill="both", expand=True, padx=12, pady=12)
        paths_text.insert(
            "1.0",
            f"RRD: {self.rrd_path}\nJSONL/CSV: {self.writer.session_dir}\n"
            f"SVO2: {self.args.svo2 or 'ilişkilendirilmedi'}\n\n"
            "RRD zaman çizelgesinde oynatılabilir. SVO2 yalnızca ZED SDK "
            "tarafından üretilir; uygulama mevcut SVO2 yolunu manifestte bağlar.",
        )
        paths_text.configure(state="disabled")

        def save_config() -> None:
            path = filedialog.asksaveasfilename(
                defaultextension=".json", filetypes=[("JSON", "*.json")],
                initialfile="rerun_analysis_config.json",
            )
            if path:
                Path(path).write_text(
                    json.dumps(self.config.as_dict(), ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )

        ttk.Button(
            session_tab, text="Parametreleri JSON kaydet", command=save_config
        ).pack(padx=12, pady=8, anchor="w")
        ttk.Button(
            session_tab, text="Oturum klasörünü göster",
            command=lambda: messagebox.showinfo(
                "Oturum klasörü", str(self.writer.session_dir)
            ),
        ).pack(padx=12, pady=8, anchor="w")

        def refresh() -> None:
            if self.stop_event.is_set():
                root.destroy()
                return
            latest = None
            try:
                while True:
                    latest = self.status_queue.get_nowait()
            except queue.Empty:
                pass
            if latest:
                source = latest["source"]
                processed = latest["processed"]
                derived = latest["derived"]
                names = [str(name) for name in processed.get("keypoint_names", [])]
                joint_combo["values"] = names
                selected = joint_var.get()
                if selected not in names and names:
                    joint_var.set(names[0])
                    selected = names[0]
                visibility = derived.get("visibility", {})
                safety = self.latest_gmr.get("safety") or {}
                safety_reason = ",".join(safety.get("reasons", [])) or "normal"
                status_var.set(
                    f"Body ID: {source.get('body_id')} | "
                    f"Güven: {float(source.get('body_confidence', 0)):.1f}% | "
                    f"Geçerli: {visibility.get('valid_keypoints', 0)}/{len(names)} | "
                    f"Fusion: {(source.get('multi_camera') or {}).get('mode', 'n/a')} "
                    f"({(source.get('multi_camera') or {}).get('contributing_views', 0)} view) | "
                    f"G1: {safety.get('level', 'WAITING')} ({safety_reason}) | "
                    f"Kaydedilen kare: {latest['frame_count']}"
                )
                detail.delete("1.0", "end")
                detail.insert(
                    "1.0",
                    json.dumps(
                        {
                            "tracking_state": source.get("tracking_state"),
                            "root_position_m": source.get("root_position_m"),
                            "transport_metrics": source.get("transport_metrics"),
                            "visibility": visibility,
                            "occlusion": derived.get("occlusion"),
                            "quality": derived.get("quality"),
                            "operator_selection": source.get("operator_selection"),
                            "calibration": source.get("calibration"),
                            "perception_metrics": source.get("perception_metrics"),
                            "multi_camera": source.get("multi_camera"),
                            "g1_safety": self.latest_gmr.get("safety"),
                            "gmr_metrics": self.latest_gmr.get("bridge_metrics"),
                            "isaac_metrics": self.latest_gmr.get("isaac_metrics"),
                            "latency_breakdown_ms": self.latest_gmr.get(
                                "latency_breakdown_ms"
                            ),
                            "effective_config": self.config.as_dict(),
                        },
                        ensure_ascii=False, indent=2,
                    ),
                )
                if selected in names:
                    index = names.index(selected)
                    points = processed.get("keypoints_3d_m", [])
                    confidences = source.get("keypoint_confidence", [])
                    angles = derived.get("joint_angles_deg", {})
                    joint_details.delete("1.0", "end")
                    joint_details.insert(
                        "1.0",
                        json.dumps(
                            {
                                "joint": selected,
                                "position_m": points[index] if index < len(points) else None,
                                "confidence": (
                                    confidences[index]
                                    if index < len(confidences) else None
                                ),
                                "tracking_source": latest["states"].get(selected),
                                "velocity_m_s": derived.get(
                                    "keypoint_velocity_m_s", {}
                                ).get(selected),
                                "speed_m_s": derived.get(
                                    "keypoint_speed_m_s", {}
                                ).get(selected),
                                "raw_filter_error_m": derived.get(
                                    "raw_filter_error_m", {}
                                ).get(selected),
                                "local_euler_xyz_deg": derived.get(
                                    "local_orientation_euler_xyz_deg", {}
                                ).get(selected),
                                "related_angles_deg": {
                                    name: angles.get(name)
                                    for name in joint_angle_names(selected, angles)
                                },
                            },
                            ensure_ascii=False, indent=2,
                        ),
                    )
            root.after(100, refresh)

        refresh()
        root.mainloop()


def main() -> int:
    args = parse_args()
    if args.svo2 and not args.svo2.exists():
        raise SystemExit(f"SVO2 bulunamadı: {args.svo2}")
    return RerunSkeletonApp(args).start()


if __name__ == "__main__":
    raise SystemExit(main())

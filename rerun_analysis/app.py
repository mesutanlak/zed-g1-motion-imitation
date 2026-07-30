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

from rerun_analysis.model import (  # noqa: E402
    EDGES,
    ConfigStore,
    SkeletonConditioner,
    finite_point,
    joint_angle_names,
)
from rerun_analysis.session import AnalysisSessionWriter  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ZED BODY_38 için bağımsız Rerun 3B analiz uygulaması"
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--input", type=Path, help="JSONL kaydını oynat")
    source.add_argument("--demo", action="store_true", help="Sentetik doğrulama")
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=15052)
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
        rrb.Horizontal(
            rrb.Spatial3DView(
                name="BODY_38 — seçilebilir 3B iskelet",
                origin="/world",
                contents=["+ $origin/**", "- /world/metrics/**"],
                line_grid=True,
            ),
            rrb.Vertical(
                rrb.TimeSeriesView(
                    name="Eklem açıları (deg)", origin="/world/metrics/angles"
                ),
                rrb.TimeSeriesView(
                    name="Takip kalitesi ve gecikme",
                    origin="/world/metrics/quality",
                ),
                row_shares=[3, 2],
            ),
            column_shares=[3, 2],
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

        for index, name in enumerate(names):
            if name not in points:
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
                else [255, 190, 50] if state == "held"
                else [240, 80, 80]
            )
            entity = f"/world/skeleton/joints/{name}"
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
        quality_values = {
            "body_confidence_percent": source.get("body_confidence"),
            "valid_keypoints": visibility.get("valid_keypoints"),
            "confident_keypoints": visibility.get("confident_keypoints"),
            "max_joint_speed_m_s": quality.get("max_keypoint_speed_m_s"),
            "median_filter_error_m": quality.get("median_raw_filter_error_m"),
            "capture_to_send_ms": transport.get("capture_to_send_ms"),
            "source_interval_ms": transport.get("source_interval_ms"),
        }
        for name, value in quality_values.items():
            if isinstance(value, (int, float)) and np.isfinite(value):
                rr.log(f"/world/metrics/quality/{name}", rr.Scalars(float(value)))
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
        sock.bind((self.args.listen_host, self.args.listen_port))
        sock.settimeout(0.2)
        try:
            while not self.stop_event.is_set():
                try:
                    payload, _address = sock.recvfrom(2_000_000)
                except socket.timeout:
                    continue
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
        started = time.monotonic()
        try:
            for source in packets:
                if self.stop_event.is_set():
                    break
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
        if self.args.headless:
            self.worker.join()
        else:
            self._run_ui()
        self.stop_event.set()
        self.worker.join(timeout=3.0)
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
            "occlusion_hold_frames": tk.IntVar(value=8),
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
                status_var.set(
                    f"Body ID: {source.get('body_id')} | "
                    f"Güven: {float(source.get('body_confidence', 0)):.1f}% | "
                    f"Geçerli: {visibility.get('valid_keypoints', 0)}/{len(names)} | "
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

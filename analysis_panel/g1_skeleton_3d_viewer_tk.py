"""Native Windows BODY_38 analysis panel.

This viewer intentionally uses only Python's built-in tkinter so the analysis
window does not depend on WSLg/RDP rendering. It consumes the same UDP schema
and writes exactly the same analysis JSONL/CSV format as the Qt viewer.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
import tkinter as tk
from collections import deque
from pathlib import Path

from skeleton_analysis_recorder import SkeletonAnalysisRecorder


EDGES = (
    ("PELVIS", "SPINE_1"), ("SPINE_1", "SPINE_2"),
    ("SPINE_2", "SPINE_3"), ("SPINE_3", "NECK"), ("NECK", "NOSE"),
    ("SPINE_3", "LEFT_CLAVICLE"), ("LEFT_CLAVICLE", "LEFT_SHOULDER"),
    ("LEFT_SHOULDER", "LEFT_ELBOW"), ("LEFT_ELBOW", "LEFT_WRIST"),
    ("SPINE_3", "RIGHT_CLAVICLE"), ("RIGHT_CLAVICLE", "RIGHT_SHOULDER"),
    ("RIGHT_SHOULDER", "RIGHT_ELBOW"), ("RIGHT_ELBOW", "RIGHT_WRIST"),
    ("PELVIS", "LEFT_HIP"), ("LEFT_HIP", "LEFT_KNEE"),
    ("LEFT_KNEE", "LEFT_ANKLE"), ("LEFT_ANKLE", "LEFT_HEEL"),
    ("LEFT_ANKLE", "LEFT_BIG_TOE"), ("LEFT_BIG_TOE", "LEFT_SMALL_TOE"),
    ("PELVIS", "RIGHT_HIP"), ("RIGHT_HIP", "RIGHT_KNEE"),
    ("RIGHT_KNEE", "RIGHT_ANKLE"), ("RIGHT_ANKLE", "RIGHT_HEEL"),
    ("RIGHT_ANKLE", "RIGHT_BIG_TOE"), ("RIGHT_BIG_TOE", "RIGHT_SMALL_TOE"),
    ("LEFT_WRIST", "LEFT_HAND_THUMB_4"),
    ("LEFT_WRIST", "LEFT_HAND_INDEX_1"),
    ("LEFT_WRIST", "LEFT_HAND_MIDDLE_4"),
    ("LEFT_WRIST", "LEFT_HAND_PINKY_1"),
    ("RIGHT_WRIST", "RIGHT_HAND_THUMB_4"),
    ("RIGHT_WRIST", "RIGHT_HAND_INDEX_1"),
    ("RIGHT_WRIST", "RIGHT_HAND_MIDDLE_4"),
    ("RIGHT_WRIST", "RIGHT_HAND_PINKY_1"),
)

LABEL_NAMES = {
    "PELVIS", "NECK", "LEFT_WRIST", "RIGHT_WRIST", "LEFT_KNEE",
    "RIGHT_KNEE", "LEFT_ANKLE", "RIGHT_ANKLE",
}


def valid_point(value) -> bool:
    return (
        isinstance(value, (list, tuple))
        and len(value) == 3
        and all(isinstance(item, (int, float)) and math.isfinite(item) for item in value)
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=15052)
    parser.add_argument("--confidence", type=float, default=35.0)
    parser.add_argument("--stale-after", type=float, default=0.5)
    parser.add_argument("--record", action="store_true")
    parser.add_argument(
        "--record-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "analysis_recordings",
    )
    parser.add_argument("--no-csv", action="store_true")
    return parser.parse_args()


class NativeSkeletonPanel:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.root = tk.Tk()
        self.root.title(
            f"ZED BODY_38 — Canlı 3B İskelet Analizi — UDP {args.listen_port}"
        )
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        width = min(1600, max(1050, int(screen_width * 0.72)))
        height = min(1100, max(760, int(screen_height * 0.78)))
        self.root.geometry(f"{width}x{height}+40+40")
        self.root.minsize(760, 620)
        self.root.configure(bg="#101820")

        self.canvas = tk.Canvas(
            self.root, bg="#101820", highlightthickness=0, bd=0
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.packet: dict | None = None
        self.status = "UDP BEKLENİYOR"
        self.last_packet_time = 0.0
        self.rx_arrivals: deque[float] = deque()
        self.valid_arrivals: deque[float] = deque()
        self.rx_packet_count = 0
        self.valid_packet_count = 0
        self.recorder = SkeletonAnalysisRecorder(
            args.record_dir,
            confidence_threshold=args.confidence,
            write_csv=not args.no_csv,
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((args.listen_host, args.listen_port))
        self.sock.setblocking(False)

        self.root.bind("<Escape>", lambda _event: self.close())
        self.root.bind("<Key-q>", lambda _event: self.close())
        self.root.bind("<Key-Q>", lambda _event: self.close())
        self.root.bind("<Key-s>", lambda _event: self.toggle_recording())
        self.root.bind("<Key-S>", lambda _event: self.toggle_recording())
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        if args.record:
            self.start_recording()

        # Native Windows focus: briefly topmost, then return to a normal window.
        self.root.attributes("-topmost", True)
        self.root.after(350, self.release_topmost)
        self.root.after(16, self.poll)

    def release_topmost(self) -> None:
        self.root.attributes("-topmost", False)
        self.root.lift()
        self.root.focus_force()

    def start_recording(self) -> None:
        json_path, csv_path = self.recorder.start()
        print(f"3B analiz kaydı AÇIK: {json_path}", flush=True)
        if csv_path is not None:
            print(f"3B analiz CSV özeti: {csv_path}", flush=True)

    def stop_recording(self) -> None:
        if not self.recorder.active:
            return
        json_path, csv_path, frame_count = self.recorder.stop()
        print(f"3B analiz kaydı KAPALI: {json_path} ({frame_count} kare)", flush=True)
        if csv_path is not None:
            print(f"3B analiz CSV kaydedildi: {csv_path}", flush=True)

    def toggle_recording(self) -> None:
        if self.recorder.active:
            self.stop_recording()
        else:
            self.start_recording()

    def close(self) -> None:
        self.stop_recording()
        try:
            self.sock.close()
        finally:
            self.root.destroy()

    def accept_packet(self, packet: dict) -> None:
        now = time.monotonic()
        schema = packet.get("schema")
        if schema in ("zed_body38_live/v1", "zed_body38_live/status/v1"):
            self.rx_packet_count += 1
            self.rx_arrivals.append(now)
            self.recorder.record(packet)
        if schema == "zed_body38_live/v1":
            readiness = packet.get("reference_ready", {})
            if readiness and not bool(readiness.get("upper_body", False)):
                self.status = "LOW_QUALITY"
                self.last_packet_time = now
                return
            self.packet = packet
            self.status = (
                "CANLI" if bool(readiness.get("whole_body", False)) else "UPPER_LIVE"
            )
            self.last_packet_time = now
            self.valid_packet_count += 1
            self.valid_arrivals.append(now)
        elif schema == "zed_body38_live/status/v1":
            self.status = str(packet.get("status", "DURUM"))
            self.last_packet_time = now

    def poll(self) -> None:
        while True:
            try:
                payload, _source = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
            except OSError:
                return
            try:
                packet = json.loads(payload)
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            self.accept_packet(packet)

        now = time.monotonic()
        while self.rx_arrivals and now - self.rx_arrivals[0] > 1.0:
            self.rx_arrivals.popleft()
        while self.valid_arrivals and now - self.valid_arrivals[0] > 1.0:
            self.valid_arrivals.popleft()
        if self.last_packet_time and now - self.last_packet_time > self.args.stale_after:
            self.status = "STALE"
        self.redraw()
        self.root.after(16, self.poll)

    @staticmethod
    def depth_color(relative_x: float) -> str:
        value = max(-0.35, min(0.35, relative_x))
        ratio = (value + 0.35) / 0.70
        red = int(55 + 200 * ratio)
        green = int(190 - 80 * abs(ratio - 0.5) * 2)
        blue = int(245 - 190 * ratio)
        return f"#{red:02x}{green:02x}{blue:02x}"

    def redraw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 760)
        height = max(canvas.winfo_height(), 620)
        info_width = 300
        left, top = 45, 82
        right, bottom = width - info_width - 35, height - 45

        canvas.create_text(
            24, 27, anchor="w", fill="#eaf4ff",
            font=("Segoe UI", 16, "bold"),
            text="ZED BODY_38 — 3B KOORDİNAT ANALİZİ",
        )
        canvas.create_text(
            24, 54, anchor="w", fill="#8fa9be", font=("Segoe UI", 9),
            text="FRONT VIEW: yatay Y (sol), dikey Z (yukarı); X derinliği renkle gösterilir",
        )
        status_color = {
            "CANLI": "#54e38e", "UPPER_LIVE": "#54e38e",
            "STALE": "#ffb347", "NO_BODY": "#ff6b6b",
            "LOW_QUALITY": "#ffb347", "CORRUPT_FRAME": "#ff3b30",
        }.get(self.status, "#9fb3c8")
        canvas.create_text(
            width - 24, 28, anchor="e", fill=status_color,
            font=("Segoe UI", 12, "bold"), text=self.status,
        )
        record_text = (
            f"REC {self.recorder.frame_count} | S: DURDUR"
            if self.recorder.active else "S: ANALİZ KAYDI"
        )
        canvas.create_text(
            width - 24, 54, anchor="e",
            fill="#ff5d73" if self.recorder.active else "#708797",
            font=("Segoe UI", 9, "bold"), text=record_text,
        )
        canvas.create_rectangle(
            left, top, right, bottom, fill="#14232e", outline="#2b4356", width=1
        )
        info_left = right + 18
        canvas.create_rectangle(
            info_left, top, width - 18, bottom,
            fill="#13202a", outline="#2b4356", width=1,
        )

        if not self.packet:
            canvas.create_text(
                (left + right) / 2, (top + bottom) / 2,
                fill="#8fa9be", font=("Segoe UI", 13),
                text=f"UDP {self.args.listen_port} üzerinden BODY_38 bekleniyor…",
            )
            self.draw_axis(left, bottom)
            self.draw_info(info_left + 16, top + 20, 0, 0, None)
            return

        names = self.packet.get("keypoint_names", [])
        points = self.packet.get("keypoints_3d_m", [])
        raw_points = self.packet.get("keypoints_3d_raw_m", [])
        confidence = self.packet.get("keypoint_confidence", [])
        point_map: dict[str, tuple[float, float, float]] = {}
        confidence_map: dict[str, float] = {}
        for index, name in enumerate(names):
            if index >= len(points) or not valid_point(points[index]):
                continue
            point_map[str(name)] = tuple(float(value) for value in points[index])
            confidence_map[str(name)] = (
                float(confidence[index])
                if index < len(confidence)
                and isinstance(confidence[index], (int, float))
                else 0.0
            )
        pelvis = point_map.get("PELVIS")
        if pelvis is None:
            canvas.create_text(
                (left + right) / 2, (top + bottom) / 2,
                fill="#ff6b6b", font=("Segoe UI", 13), text="Pelvis noktası geçersiz",
            )
            return
        usable = {
            name: point for name, point in point_map.items()
            if confidence_map.get(name, 0.0) >= self.args.confidence
        }
        relative = {
            name: (
                point[0] - pelvis[0],
                point[1] - pelvis[1],
                point[2] - pelvis[2],
            )
            for name, point in usable.items()
        }
        if not relative:
            return

        ys = [point[1] for point in relative.values()]
        zs = [point[2] for point in relative.values()]
        y_extent = max(0.55, max(abs(min(ys)), abs(max(ys))))
        z_min = min(-1.05, min(zs))
        z_max = max(0.85, max(zs))
        scale = min(
            (right - left - 70) / (2.0 * y_extent),
            (bottom - top - 70) / (z_max - z_min),
        )
        center_x = (left + right) / 2
        plot_bottom = bottom - 28

        def screen(point) -> tuple[float, float]:
            _x, y, z = point
            return center_x - y * scale, plot_bottom - (z - z_min) * scale

        step = 0.25
        tick = -math.ceil(y_extent / step) * step
        while tick <= y_extent + 1e-9:
            x = center_x - tick * scale
            canvas.create_line(x, top + 15, x, bottom - 18, fill="#223746")
            canvas.create_text(x, bottom - 8, fill="#688399", font=("Segoe UI", 7),
                               text=f"{tick:+.2f}")
            tick += step
        tick = math.ceil(z_min / step) * step
        while tick <= z_max + 1e-9:
            y = plot_bottom - (tick - z_min) * scale
            canvas.create_line(left + 15, y, right - 15, y, fill="#223746")
            tick += step

        for raw_point in raw_points:
            if valid_point(raw_point):
                raw_relative = (
                    float(raw_point[0]) - pelvis[0],
                    float(raw_point[1]) - pelvis[1],
                    float(raw_point[2]) - pelvis[2],
                )
                x, y = screen(raw_relative)
                canvas.create_oval(x - 2, y - 2, x + 2, y + 2, outline="#718493")

        for start_name, end_name in EDGES:
            if start_name not in relative or end_name not in relative:
                continue
            start = relative[start_name]
            end = relative[end_name]
            canvas.create_line(
                *screen(start), *screen(end),
                fill=self.depth_color((start[0] + end[0]) * 0.5),
                width=4, capstyle=tk.ROUND,
            )
        for name, point in relative.items():
            x, y = screen(point)
            radius = 6 if name in LABEL_NAMES else 4
            color = self.depth_color(point[0])
            canvas.create_oval(
                x - radius, y - radius, x + radius, y + radius,
                fill=color, outline="#061018",
            )
            if name in LABEL_NAMES:
                canvas.create_text(
                    x + 8, y - 7, anchor="w", fill="#d9e7f2",
                    font=("Segoe UI", 7), text=name.replace("_", " "),
                )
        self.draw_axis(left, bottom)
        self.draw_info(info_left + 16, top + 20, len(usable), len(names), pelvis)

    def draw_axis(self, left: float, bottom: float) -> None:
        origin_x, origin_y = left + 60, bottom - 42
        self.canvas.create_line(origin_x, origin_y, origin_x - 35, origin_y,
                                fill="#5be37d", width=3)
        self.canvas.create_text(origin_x - 44, origin_y - 8, fill="#5be37d",
                                text="+Y", font=("Segoe UI", 9, "bold"))
        self.canvas.create_line(origin_x, origin_y, origin_x, origin_y - 35,
                                fill="#5da9ff", width=3)
        self.canvas.create_text(origin_x + 12, origin_y - 30, fill="#5da9ff",
                                text="+Z", font=("Segoe UI", 9, "bold"))
        self.canvas.create_line(origin_x, origin_y, origin_x + 25, origin_y + 19,
                                fill="#ff776d", width=3)
        self.canvas.create_text(origin_x + 34, origin_y + 23, fill="#ff776d",
                                text="+X", font=("Segoe UI", 9, "bold"))

    def draw_info(
        self, x: float, y: float, valid_count: int, total_count: int, pelvis
    ) -> None:
        packet = self.packet or {}
        transport = packet.get("transport_metrics", {})
        source_interval = transport.get("source_interval_ms")
        source_hz = (
            1000.0 / float(source_interval)
            if isinstance(source_interval, (int, float)) and source_interval > 0
            else 0.0
        )
        angles = packet.get("g1_reference_features", {}).get("geometric_angles", {})

        def numeric(value, suffix: str = "", digits: int = 1) -> str:
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                return "-"
            return f"{float(value):.{digits}f}{suffix}"

        fields = [
            ("Body ID", packet.get("body_id", "-")),
            ("Confidence", numeric(packet.get("body_confidence"), "%")),
            ("UDP RX FPS", len(self.rx_arrivals)),
            ("Geçerli FPS", len(self.valid_arrivals)),
            ("Kaynak Hz", f"{source_hz:.1f}"),
            ("RX / Geçerli", f"{self.rx_packet_count}/{self.valid_packet_count}"),
            ("Geçerli KP", f"{valid_count}/{total_count}"),
            ("Mesafe", numeric(packet.get("euclidean_distance_m"), " m", 2)),
            ("Capture→UDP", numeric(transport.get("capture_to_send_ms"), " ms")),
            ("Root X", f"{pelvis[0]:+.3f} m" if pelvis else "-"),
            ("Root Y", f"{pelvis[1]:+.3f} m" if pelvis else "-"),
            ("Root Z", f"{pelvis[2]:+.3f} m" if pelvis else "-"),
            ("Sol dirsek", numeric(angles.get("left_elbow_interior_deg"), "°")),
            ("Sağ dirsek", numeric(angles.get("right_elbow_interior_deg"), "°")),
        ]
        self.canvas.create_text(
            x, y, anchor="nw", fill="#eaf4ff",
            font=("Segoe UI", 11, "bold"), text="CANLI VERİ",
        )
        cursor_y = y + 32
        for label, value in fields:
            self.canvas.create_text(
                x, cursor_y, anchor="nw", fill="#7892a7",
                font=("Segoe UI", 8), text=label,
            )
            self.canvas.create_text(
                x, cursor_y + 14, anchor="nw", fill="#e2edf5",
                font=("Consolas", 9, "bold"), text=str(value),
            )
            cursor_y += 36

    def run(self) -> int:
        print(
            "3B panel: Windows native Tkinter | "
            f"UDP {self.args.listen_host}:{self.args.listen_port}",
            flush=True,
        )
        self.root.mainloop()
        return 0


def main() -> int:
    return NativeSkeletonPanel(parse_args()).run()


if __name__ == "__main__":
    raise SystemExit(main())

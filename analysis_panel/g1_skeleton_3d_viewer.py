"""Live ZED BODY_38 front-view analysis panel for Ubuntu 22.04 / WSLg.

The panel listens to a dedicated copy of the raw ZED UDP stream.  It never
opens Unitree DDS and cannot send robot commands.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import socket
import sys
import time
from collections import deque
from pathlib import Path

from PyQt5.QtCore import QPoint, QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen
from PyQt5.QtWidgets import QApplication, QWidget

from skeleton_analysis_recorder import SkeletonAnalysisRecorder


EDGES = (
    ("PELVIS", "SPINE_1"),
    ("SPINE_1", "SPINE_2"),
    ("SPINE_2", "SPINE_3"),
    ("SPINE_3", "NECK"),
    ("NECK", "NOSE"),
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
    ("LEFT_ANKLE", "LEFT_HEEL"),
    ("LEFT_ANKLE", "LEFT_BIG_TOE"),
    ("LEFT_BIG_TOE", "LEFT_SMALL_TOE"),
    ("PELVIS", "RIGHT_HIP"),
    ("RIGHT_HIP", "RIGHT_KNEE"),
    ("RIGHT_KNEE", "RIGHT_ANKLE"),
    ("RIGHT_ANKLE", "RIGHT_HEEL"),
    ("RIGHT_ANKLE", "RIGHT_BIG_TOE"),
    ("RIGHT_BIG_TOE", "RIGHT_SMALL_TOE"),
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
    "PELVIS",
    "NECK",
    "LEFT_WRIST",
    "RIGHT_WRIST",
    "LEFT_KNEE",
    "RIGHT_KNEE",
    "LEFT_ANKLE",
    "RIGHT_ANKLE",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=15052)
    parser.add_argument("--confidence", type=float, default=35.0)
    parser.add_argument("--stale-after", type=float, default=0.5)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--seconds", type=float, default=0.0)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--demo-screenshot", type=Path, default=None)
    parser.add_argument("--record", action="store_true")
    parser.add_argument(
        "--maximize",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--screen-index",
        type=int,
        default=0,
        help="Panelin yerleştirileceği Qt/WSLg ekranı (varsayılan: 0)",
    )
    parser.add_argument(
        "--record-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "analysis_recordings",
    )
    parser.add_argument("--no-csv", action="store_true")
    return parser.parse_args()


def valid_point(value) -> bool:
    return (
        isinstance(value, (list, tuple))
        and len(value) == 3
        and all(isinstance(x, (int, float)) and math.isfinite(x) for x in value)
    )


def demo_packet() -> dict:
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
    positions = {
        "PELVIS": (3.0, 0.0, 1.0), "SPINE_1": (3.0, 0.0, 1.15),
        "SPINE_2": (3.0, 0.0, 1.33), "SPINE_3": (3.0, 0.0, 1.50),
        "NECK": (3.0, 0.0, 1.68), "NOSE": (2.98, 0.0, 1.82),
        "LEFT_CLAVICLE": (3.0, 0.12, 1.52), "LEFT_SHOULDER": (3.0, 0.25, 1.50),
        "LEFT_ELBOW": (3.05, 0.48, 1.36), "LEFT_WRIST": (3.10, 0.65, 1.20),
        "RIGHT_CLAVICLE": (3.0, -0.12, 1.52), "RIGHT_SHOULDER": (3.0, -0.25, 1.50),
        "RIGHT_ELBOW": (2.96, -0.48, 1.36), "RIGHT_WRIST": (2.92, -0.65, 1.20),
        "LEFT_HIP": (3.0, 0.11, 0.98), "LEFT_KNEE": (3.02, 0.12, 0.55),
        "LEFT_ANKLE": (3.02, 0.12, 0.08), "LEFT_HEEL": (2.94, 0.12, 0.03),
        "LEFT_BIG_TOE": (3.16, 0.10, 0.03), "LEFT_SMALL_TOE": (3.15, 0.16, 0.03),
        "RIGHT_HIP": (3.0, -0.11, 0.98), "RIGHT_KNEE": (2.98, -0.12, 0.55),
        "RIGHT_ANKLE": (2.98, -0.12, 0.08), "RIGHT_HEEL": (2.90, -0.12, 0.03),
        "RIGHT_BIG_TOE": (3.12, -0.10, 0.03), "RIGHT_SMALL_TOE": (3.11, -0.16, 0.03),
        "LEFT_HAND_THUMB_4": (3.10, 0.69, 1.18),
        "LEFT_HAND_INDEX_1": (3.10, 0.70, 1.22),
        "LEFT_HAND_MIDDLE_4": (3.10, 0.71, 1.20),
        "LEFT_HAND_PINKY_1": (3.10, 0.69, 1.16),
        "RIGHT_HAND_THUMB_4": (2.92, -0.69, 1.18),
        "RIGHT_HAND_INDEX_1": (2.92, -0.70, 1.22),
        "RIGHT_HAND_MIDDLE_4": (2.92, -0.71, 1.20),
        "RIGHT_HAND_PINKY_1": (2.92, -0.69, 1.16),
    }
    return {
        "schema": "zed_body38_live/v1",
        "sequence": 1,
        "timestamp_ns": time.time_ns(),
        "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
        "units": "meter",
        "body_id": 0,
        "tracking_state": "OK",
        "action_state": "IDLE",
        "body_confidence": 97.0,
        "root_position_m": [3.0, 0.0, 1.0],
        "keypoint_names": names,
        "keypoints_2d_px": [[640.0, 360.0] for _name in names],
        "keypoints_3d_raw_m": [
            [positions[name][0] + (0.004 if index % 2 else -0.004),
             positions[name][1], positions[name][2]]
            for index, name in enumerate(names)
        ],
        "keypoints_3d_m": [positions[name] for name in names],
        "keypoint_confidence": [95.0] * len(names),
        "local_position_per_joint_m": [
            list(positions[name]) for name in names
        ],
        "local_orientation_per_joint_xyzw": [
            [0.0, 0.0, 0.0, 1.0] for _name in names
        ],
        "root_relative_keypoints_m": [
            [
                positions[name][0] - positions["PELVIS"][0],
                positions[name][1] - positions["PELVIS"][1],
                positions[name][2] - positions["PELVIS"][2],
            ]
            for name in names
        ],
        "reference_ready": {"upper_body": True, "whole_body": True},
        "euclidean_distance_m": 3.0,
        "g1_reference_features": {
            "geometric_angles": {
                "left_elbow_interior_deg": 145.0,
                "right_elbow_interior_deg": 145.0,
            }
        },
        "imu": {"angular_velocity_deg_s": [0.1, -0.2, 0.1]},
        "transport_metrics": {
            "source_interval_ms": 66.67,
            "capture_to_send_ms": 14.0,
            "raw_filtered_rms_m": 0.004,
        },
    }


class SkeletonPanel(QWidget):
    def __init__(
        self,
        listen_host: str,
        listen_port: int,
        confidence: float,
        stale_after: float,
        record_dir: Path,
        write_csv: bool,
        record_on_start: bool,
    ) -> None:
        super().__init__()
        self.confidence_threshold = confidence
        self.stale_after = stale_after
        self.packet: dict | None = None
        self.status = "UDP verisi bekleniyor"
        self.last_packet_time = 0.0
        self.rx_arrivals: deque[float] = deque()
        self.valid_arrivals: deque[float] = deque()
        self.rx_packet_count = 0
        self.valid_packet_count = 0
        self.recorder = SkeletonAnalysisRecorder(
            record_dir,
            confidence_threshold=confidence,
            write_csv=write_csv,
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((listen_host, listen_port))
        self.sock.setblocking(False)

        self.setWindowTitle(
            f"ZED BODY_38 — Canli 3B Iskelet Analizi — UDP {listen_port}"
        )
        self.resize(1000, 850)
        self.setMinimumSize(720, 600)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(16)
        if record_on_start:
            self.start_recording()

    def closeEvent(self, event) -> None:
        self.stop_recording()
        self.sock.close()
        event.accept()

    def keyPressEvent(self, event) -> None:
        if event.key() in (Qt.Key_Escape, Qt.Key_Q):
            self.close()
        elif event.key() == Qt.Key_S:
            if self.recorder.active:
                self.stop_recording()
            else:
                self.start_recording()
            self.update()
        else:
            super().keyPressEvent(event)

    def start_recording(self) -> None:
        json_path, csv_path = self.recorder.start()
        print(f"3B analiz kaydi ACIK: {json_path}")
        if csv_path is not None:
            print(f"3B analiz CSV ozeti: {csv_path}")

    def stop_recording(self) -> None:
        if not self.recorder.active:
            return
        json_path, csv_path, frame_count = self.recorder.stop()
        print(f"3B analiz kaydi KAPALI: {json_path} ({frame_count} kare)")
        if csv_path is not None:
            print(f"3B analiz CSV kaydedildi: {csv_path}")

    def accept_packet(self, packet: dict) -> None:
        now = time.monotonic()
        schema = packet.get("schema")
        if schema in ("zed_body38_live/v1", "zed_body38_live/status/v1"):
            self.rx_packet_count += 1
            self.rx_arrivals.append(now)
            # Record before display-quality gating. This preserves missing and
            # low-quality intervals for later occlusion analysis.
            self.recorder.record(packet)
        if schema == "zed_body38_live/v1":
            readiness = packet.get("reference_ready", {})
            if readiness and not bool(readiness.get("upper_body", False)):
                # Keep the last coherent skeleton on brief occlusions.  Never
                # redraw a fragmented BODY_38 frame as if it were valid.
                self.status = "LOW_QUALITY"
                self.last_packet_time = now
                return
            self.packet = packet
            self.status = (
                "CANLI"
                if bool(readiness.get("whole_body", False))
                else "UPPER_LIVE"
            )
            self.last_packet_time = now
            self.valid_packet_count += 1
            self.valid_arrivals.append(now)
        elif schema == "zed_body38_live/status/v1":
            self.status = str(packet.get("status", "DURUM"))
            self.last_packet_time = now
        while self.rx_arrivals and now - self.rx_arrivals[0] > 1.0:
            self.rx_arrivals.popleft()
        while self.valid_arrivals and now - self.valid_arrivals[0] > 1.0:
            self.valid_arrivals.popleft()

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
        if self.last_packet_time and time.monotonic() - self.last_packet_time > self.stale_after:
            self.status = "STALE"
        self.update()

    @staticmethod
    def depth_color(relative_x: float) -> QColor:
        value = max(-0.35, min(0.35, relative_x))
        t = (value + 0.35) / 0.70
        return QColor(
            int(55 + 200 * t),
            int(190 - 80 * abs(t - 0.5) * 2),
            int(245 - 190 * t),
        )

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#101820"))

        painter.setFont(QFont("DejaVu Sans", 15, QFont.Bold))
        painter.setPen(QColor("#EAF4FF"))
        painter.drawText(24, 34, "ZED BODY_38 — 3B KOORDINAT ANALIZI")
        painter.setFont(QFont("DejaVu Sans", 9))
        painter.setPen(QColor("#8FA9BE"))
        painter.drawText(
            25,
            55,
            "FRONT VIEW: yatay Y (sol), dikey Z (yukari); X derinligi nokta rengiyle gosterilir",
        )

        status_color = {
            "CANLI": QColor("#54E38E"),
            "UPPER_LIVE": QColor("#54E38E"),
            "STALE": QColor("#FFB347"),
            "NO_BODY": QColor("#FF6B6B"),
            "LOW_QUALITY": QColor("#FFB347"),
            "CORRUPT_FRAME": QColor("#FF3B30"),
        }.get(self.status, QColor("#9FB3C8"))
        painter.setFont(QFont("DejaVu Sans", 11, QFont.Bold))
        painter.setPen(status_color)
        painter.drawText(self.width() - 220, 34, self.status)
        painter.setFont(QFont("DejaVu Sans", 9, QFont.Bold))
        painter.setPen(
            QColor("#FF5D73") if self.recorder.active else QColor("#708797")
        )
        record_text = (
            f"REC {self.recorder.frame_count} | S: DURDUR"
            if self.recorder.active
            else "S: ANALIZ KAYDI"
        )
        painter.drawText(self.width() - 220, 54, record_text)

        plot = QRectF(55, 82, self.width() - 360, self.height() - 130)
        info = QRectF(plot.right() + 22, 82, 270, self.height() - 130)
        painter.setPen(QPen(QColor("#2B4356"), 1))
        painter.setBrush(QColor("#14232E"))
        painter.drawRoundedRect(plot, 8, 8)
        painter.setBrush(QColor("#13202A"))
        painter.drawRoundedRect(info, 8, 8)

        if not self.packet:
            painter.setFont(QFont("DejaVu Sans", 13))
            painter.setPen(QColor("#8FA9BE"))
            painter.drawText(plot, Qt.AlignCenter, "UDP 15052 uzerinden BODY_38 bekleniyor...")
            self.draw_axis_triad(painter, plot)
            return

        names = self.packet.get("keypoint_names", [])
        points = self.packet.get("keypoints_3d_m", [])
        raw_points = self.packet.get("keypoints_3d_raw_m", [])
        confidence = self.packet.get("keypoint_confidence", [])
        point_map = {}
        confidence_map = {}
        for index, name in enumerate(names):
            if index >= len(points) or not valid_point(points[index]):
                continue
            point_map[name] = tuple(float(v) for v in points[index])
            confidence_map[name] = (
                float(confidence[index])
                if index < len(confidence)
                and isinstance(confidence[index], (int, float))
                else 0.0
            )

        pelvis = point_map.get("PELVIS")
        if pelvis is None:
            painter.setPen(QColor("#FF6B6B"))
            painter.drawText(plot, Qt.AlignCenter, "Pelvis noktasi gecersiz")
            return

        usable = {
            name: point
            for name, point in point_map.items()
            if confidence_map.get(name, 0.0) >= self.confidence_threshold
        }
        if not usable:
            painter.setPen(QColor("#FF6B6B"))
            painter.drawText(plot, Qt.AlignCenter, "Guven esigini gecen nokta yok")
            return

        relative = {
            name: (p[0] - pelvis[0], p[1] - pelvis[1], p[2] - pelvis[2])
            for name, p in usable.items()
        }
        ys = [p[1] for p in relative.values()]
        zs = [p[2] for p in relative.values()]
        y_extent = max(0.55, max(abs(min(ys)), abs(max(ys))))
        z_min = min(-1.05, min(zs))
        z_max = max(0.85, max(zs))
        scale = min(
            (plot.width() - 70) / (2.0 * y_extent),
            (plot.height() - 70) / (z_max - z_min),
        )
        center_x = plot.center().x()
        bottom = plot.bottom() - 35

        def screen(point) -> QPointF:
            _x, y, z = point
            return QPointF(center_x - y * scale, bottom - (z - z_min) * scale)

        self.draw_grid(painter, plot, center_x, bottom, scale, z_min, z_max, y_extent)

        # Raw ZED observations remain visible as small hollow points behind the
        # confidence-aware filtered skeleton. They expose sensor jitter without
        # allowing that jitter to become a robot command.
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor("#718493"), 1))
        for raw_point in raw_points:
            if not valid_point(raw_point):
                continue
            raw_relative = (
                float(raw_point[0]) - pelvis[0],
                float(raw_point[1]) - pelvis[1],
                float(raw_point[2]) - pelvis[2],
            )
            painter.drawEllipse(screen(raw_relative), 2, 2)

        for start_name, end_name in EDGES:
            if start_name not in relative or end_name not in relative:
                continue
            a = relative[start_name]
            b = relative[end_name]
            color = self.depth_color((a[0] + b[0]) * 0.5)
            painter.setPen(QPen(color, 4, Qt.SolidLine, Qt.RoundCap))
            painter.drawLine(screen(a), screen(b))

        painter.setFont(QFont("DejaVu Sans", 7))
        for name, point in relative.items():
            position = screen(point)
            color = self.depth_color(point[0])
            painter.setPen(QPen(QColor("#061018"), 1))
            painter.setBrush(color)
            radius = 6 if name in LABEL_NAMES else 4
            painter.drawEllipse(position, radius, radius)
            if name in LABEL_NAMES:
                painter.setPen(QColor("#D9E7F2"))
                painter.drawText(position + QPointF(8, -6), name.replace("_", " "))

        self.draw_axis_triad(painter, plot)
        self.draw_info(painter, info, pelvis, len(usable), len(names))

    def draw_grid(
        self,
        painter: QPainter,
        plot: QRectF,
        center_x: float,
        bottom: float,
        scale: float,
        z_min: float,
        z_max: float,
        y_extent: float,
    ) -> None:
        painter.setFont(QFont("DejaVu Sans", 7))
        step = 0.25
        y_tick = -math.ceil(y_extent / step) * step
        while y_tick <= y_extent + 1e-9:
            x = center_x - y_tick * scale
            painter.setPen(QPen(QColor("#223746"), 1))
            painter.drawLine(QPointF(x, plot.top() + 18), QPointF(x, plot.bottom() - 20))
            painter.setPen(QColor("#688399"))
            painter.drawText(QPointF(x - 13, plot.bottom() - 7), f"{y_tick:+.2f}")
            y_tick += step
        z_tick = math.ceil(z_min / step) * step
        while z_tick <= z_max + 1e-9:
            y = bottom - (z_tick - z_min) * scale
            painter.setPen(QPen(QColor("#223746"), 1))
            painter.drawLine(QPointF(plot.left() + 20, y), QPointF(plot.right() - 20, y))
            painter.setPen(QColor("#688399"))
            painter.drawText(QPointF(plot.left() + 4, y - 3), f"{z_tick:+.2f}")
            z_tick += step
        painter.setPen(QPen(QColor("#4B718A"), 2))
        painter.drawLine(
            QPointF(center_x, plot.top() + 18),
            QPointF(center_x, plot.bottom() - 20),
        )

    def draw_axis_triad(self, painter: QPainter, plot: QRectF) -> None:
        origin = QPointF(plot.left() + 62, plot.bottom() - 48)
        painter.setFont(QFont("DejaVu Sans", 9, QFont.Bold))
        painter.setPen(QPen(QColor("#5BE37D"), 3))
        painter.drawLine(origin, origin + QPointF(-38, 0))
        painter.drawText(origin + QPointF(-53, -6), "+Y")
        painter.setPen(QPen(QColor("#5DA9FF"), 3))
        painter.drawLine(origin, origin + QPointF(0, -38))
        painter.drawText(origin + QPointF(7, -32), "+Z")
        painter.setPen(QPen(QColor("#FF776D"), 3))
        painter.drawLine(origin, origin + QPointF(27, 22))
        painter.drawText(origin + QPointF(31, 27), "+X")

    def draw_info(
        self,
        painter: QPainter,
        info: QRectF,
        pelvis,
        valid_count: int,
        total_count: int,
    ) -> None:
        now = time.monotonic()
        while self.rx_arrivals and now - self.rx_arrivals[0] > 1.0:
            self.rx_arrivals.popleft()
        while self.valid_arrivals and now - self.valid_arrivals[0] > 1.0:
            self.valid_arrivals.popleft()
        age = now - self.last_packet_time if self.last_packet_time else math.inf
        transport = self.packet.get("transport_metrics", {})
        source_interval_ms = transport.get("source_interval_ms")
        source_hz = (
            1000.0 / float(source_interval_ms)
            if isinstance(source_interval_ms, (int, float))
            and float(source_interval_ms) > 0.0
            else 0.0
        )
        raw_filter_rms = transport.get("raw_filtered_rms_m")
        capture_latency = transport.get("capture_to_send_ms")
        features = self.packet.get("g1_reference_features", {})
        angles = features.get("geometric_angles", {})
        imu = self.packet.get("imu") or {}
        gyro = imu.get("angular_velocity_deg_s")
        gyro_norm = (
            math.sqrt(sum(float(value) ** 2 for value in gyro))
            if isinstance(gyro, list)
            and len(gyro) == 3
            and all(isinstance(value, (int, float)) for value in gyro)
            else 0.0
        )
        packet_names = self.packet.get("keypoint_names", [])
        packet_points = self.packet.get("keypoints_3d_m", [])
        point_lookup = {
            str(name): packet_points[index]
            for index, name in enumerate(packet_names)
            if index < len(packet_points) and valid_point(packet_points[index])
        }
        neck = point_lookup.get("NECK")
        nose = point_lookup.get("NOSE")
        head_lean_deg = (
            math.degrees(
                math.atan2(
                    float(nose[1]) - float(neck[1]),
                    float(nose[2]) - float(neck[2]),
                )
            )
            if neck is not None and nose is not None
            else None
        )

        def number_text(value, suffix: str, digits: int = 1) -> str:
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                return "-"
            return f"{float(value):.{digits}f}{suffix}"

        fields = (
            ("Body ID", self.packet.get("body_id", "-")),
            ("Confidence", f"{float(self.packet.get('body_confidence', 0.0)):.1f}%"),
            ("UDP RX FPS", len(self.rx_arrivals)),
            ("Gecerli FPS", len(self.valid_arrivals)),
            ("Kaynak Hz", f"{source_hz:.1f}"),
            ("RX / Gecerli", f"{self.rx_packet_count}/{self.valid_packet_count}"),
            (
                "Analiz kaydi",
                (
                    f"ACIK / {self.recorder.frame_count}"
                    if self.recorder.active
                    else "KAPALI (S)"
                ),
            ),
            ("Yas", f"{age * 1000:.0f} ms"),
            ("Capture->UDP", number_text(capture_latency, " ms")),
            ("Raw/Filter RMS", number_text(
                None if raw_filter_rms is None else float(raw_filter_rms) * 100.0,
                " cm",
                2,
            )),
            ("Gecerli KP", f"{valid_count}/{total_count}"),
            ("Mesafe", number_text(self.packet.get("euclidean_distance_m"), " m", 2)),
            ("Dirsek L/R", (
                f"{number_text(angles.get('left_elbow_interior_deg'), '°', 0)} / "
                f"{number_text(angles.get('right_elbow_interior_deg'), '°', 0)}"
            )),
            ("IMU gyro", f"{gyro_norm:.1f} deg/s"),
            ("Bas yana egim", number_text(head_lean_deg, "°", 1)),
            ("Root X", f"{pelvis[0]:+.3f} m"),
            ("Root Y", f"{pelvis[1]:+.3f} m"),
            ("Root Z", f"{pelvis[2]:+.3f} m"),
            ("Koordinat", self.packet.get("coordinate_system", "-")),
        )
        painter.setFont(QFont("DejaVu Sans", 10, QFont.Bold))
        painter.setPen(QColor("#DCEAF4"))
        painter.drawText(
            QPointF(info.left() + 15, info.top() + 28),
            "CANLI VERI",
        )
        y = info.top() + 60
        for label, value in fields:
            painter.setFont(QFont("DejaVu Sans", 7))
            painter.setPen(QColor("#7F9AAF"))
            painter.drawText(QPointF(info.left() + 15, y), str(label))
            painter.setFont(QFont("DejaVu Sans", 8, QFont.Bold))
            painter.setPen(QColor("#E9F4FB"))
            painter.drawText(QPointF(info.left() + 15, y + 15), str(value))
            y += 32


def run_headless(args: argparse.Namespace) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.listen_host, args.listen_port))
    sock.settimeout(0.2)
    started = time.monotonic()
    body_packets = 0
    status_packets = 0
    recorder = SkeletonAnalysisRecorder(
        args.record_dir,
        confidence_threshold=args.confidence,
        write_csv=not args.no_csv,
    )
    if args.record:
        recorder.start()
    try:
        while not args.seconds or time.monotonic() - started < args.seconds:
            try:
                payload, _source = sock.recvfrom(65535)
            except socket.timeout:
                continue
            try:
                packet = json.loads(payload)
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if packet.get("schema") == "zed_body38_live/v1":
                body_packets += 1
            elif packet.get("schema") == "zed_body38_live/status/v1":
                status_packets += 1
            recorder.record(packet)
    finally:
        if recorder.active:
            json_path, csv_path, count = recorder.stop()
            print(
                f"ANALYSIS_RECORDING_OK frames={count} "
                f"jsonl={json_path} csv={csv_path}"
            )
        sock.close()
    print(
        f"PANEL_TEST body_packets={body_packets} "
        f"status_packets={status_packets} port={args.listen_port}"
    )
    return 0 if body_packets else 3


def main() -> int:
    args = parse_args()
    if not 1 <= args.listen_port <= 65535:
        print("Gecersiz UDP portu.", file=sys.stderr)
        return 2
    if args.self_test:
        packet = demo_packet()
        assert packet["schema"] == "zed_body38_live/v1"
        assert len(packet["keypoint_names"]) == len(packet["keypoints_3d_m"])
        assert all(valid_point(point) for point in packet["keypoints_3d_m"])
        assert all(a in packet["keypoint_names"] and b in packet["keypoint_names"] for a, b in EDGES)
        print("SELF_TEST_OK BODY_38 projection and skeleton topology")
        return 0
    if args.headless:
        return run_headless(args)

    app = QApplication(sys.argv)
    # Convert Ctrl+C into an orderly Qt shutdown.  Raising KeyboardInterrupt
    # inside paintEvent can leave an active QPainter/QBackingStore.
    signal.signal(signal.SIGINT, lambda *_args: app.quit())
    signal_timer = QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(100)
    panel = SkeletonPanel(
        args.listen_host,
        args.listen_port,
        args.confidence,
        args.stale_after,
        args.record_dir,
        not args.no_csv,
        args.record,
    )
    if args.demo_screenshot:
        panel.accept_packet(demo_packet())
        panel.show()
        app.processEvents()
        image = QImage(panel.size(), QImage.Format_ARGB32)
        image.fill(QColor("#101820"))
        painter = QPainter(image)
        panel.render(painter)
        painter.end()
        args.demo_screenshot.parent.mkdir(parents=True, exist_ok=True)
        if not image.save(str(args.demo_screenshot)):
            print("Demo ekrani kaydedilemedi.", file=sys.stderr)
            return 4
        print(f"DEMO_SCREENSHOT_OK {args.demo_screenshot}")
        return 0
    # WSLg/Weston can restore a maximized RAIL window on an invisible virtual
    # monitor (for example x=3840). Create a normal native window and pin it
    # repeatedly to the selected screen instead of trusting saved placement.
    screens = app.screens()
    screen_index = min(max(args.screen_index, 0), max(len(screens) - 1, 0))
    screen = screens[screen_index] if screens else app.primaryScreen()
    panel.show()

    def place_on_visible_screen() -> None:
        if screen is None:
            return
        geometry = screen.availableGeometry()
        width = min(1600, max(1000, int(geometry.width() * 0.72)))
        height = min(1100, max(760, int(geometry.height() * 0.78)))
        width = min(width, max(640, geometry.width() - 80))
        height = min(height, max(520, geometry.height() - 80))
        x = geometry.x() + 40
        y = geometry.y() + 40
        handle = panel.windowHandle()
        if handle is not None:
            handle.setScreen(screen)
        panel.setWindowState(panel.windowState() & ~Qt.WindowMaximized)
        panel.setGeometry(x, y, width, height)
        if handle is not None:
            handle.setPosition(QPoint(x, y))
        panel.raise_()
        panel.activateWindow()

    # Later calls override WSLg's delayed remembered/cascade placement after
    # the native RAIL window has been created.
    place_on_visible_screen()
    QTimer.singleShot(120, place_on_visible_screen)
    QTimer.singleShot(500, place_on_visible_screen)
    if screen is not None:
        geometry = screen.availableGeometry()
        print(
            "3B panel ekran geometrisi: "
            f"{geometry.width()}x{geometry.height()}"
            f"+{geometry.x()}+{geometry.y()} "
            f"screen={screen_index}/{len(screens)} safe-window=ON"
        )
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())

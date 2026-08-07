"""Streaming quality and latency metrics with JSON-safe snapshots."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np


def latency_breakdown_ms(trace: Mapping[str, int | float]) -> dict[str, float | None]:
    def delta(end: str, start: str) -> float | None:
        try:
            value = (int(trace[end]) - int(trace[start])) / 1e6
            return float(value) if value >= 0 else None
        except (KeyError, TypeError, ValueError):
            return None

    return {
        "zed_processing_ms": delta("t1_zed_processing_done_ns", "t0_capture_ns"),
        "windows_to_wsl_ms": delta("t3_wsl_receive_ns", "t2_windows_udp_send_ns"),
        "gmr_ms": delta("t5_gmr_finish_ns", "t4_gmr_start_ns"),
        "isaac_command_ms": delta("t7_isaac_command_applied_ns", "t6_isaac_receive_ns"),
        "total_control_ms": delta("t8_control_observed_ns", "t0_capture_ns"),
    }


class RollingDistribution:
    def __init__(self, maximum: int = 900) -> None:
        self.values: deque[float] = deque(maxlen=maximum)

    def add(self, value: float | None) -> None:
        if value is not None and np.isfinite(value):
            self.values.append(float(value))

    def snapshot(self) -> dict[str, float | None]:
        if not self.values:
            return {"p50": None, "p95": None, "p99": None, "mean": None}
        data = np.asarray(self.values, dtype=np.float64)
        return {
            "p50": float(np.percentile(data, 50)),
            "p95": float(np.percentile(data, 95)),
            "p99": float(np.percentile(data, 99)),
            "mean": float(np.mean(data)),
        }


@dataclass
class PacketMetrics:
    received: int = 0
    lost: int = 0
    watchdog_triggers: int = 0
    previous_sequence: int | None = None
    previous_arrival_ns: int | None = None

    def __post_init__(self) -> None:
        self.interarrival_ms = RollingDistribution()
        self.latency_ms = RollingDistribution()

    def observe(self, sequence: int, arrival_ns: int, total_latency_ms: float | None = None) -> None:
        self.received += 1
        if self.previous_sequence is not None and sequence > self.previous_sequence + 1:
            self.lost += sequence - self.previous_sequence - 1
        if self.previous_arrival_ns is not None:
            self.interarrival_ms.add((arrival_ns - self.previous_arrival_ns) / 1e6)
        self.previous_sequence = sequence
        self.previous_arrival_ns = arrival_ns
        self.latency_ms.add(total_latency_ms)

    def snapshot(self) -> dict[str, object]:
        expected = self.received + self.lost
        interarrival = self.interarrival_ms.snapshot()
        values = np.asarray(self.interarrival_ms.values, dtype=np.float64)
        return {
            "received_packets": self.received,
            "lost_packets": self.lost,
            "packet_loss_ratio": self.lost / expected if expected else 0.0,
            "latency_ms": self.latency_ms.snapshot(),
            "jitter_ms": float(np.std(values)) if values.size else None,
            "interarrival_ms": interarrival,
            "watchdog_triggers": self.watchdog_triggers,
        }


class PerceptionMetrics:
    def __init__(self) -> None:
        self.swap_count = 0
        self._swap_active = False
        self._occlusion_started: dict[str, float] = {}
        self.occlusion_recovery_s = RollingDistribution(200)
        self._angle_history: deque[tuple[float, np.ndarray]] = deque(maxlen=4)

    @staticmethod
    def _angle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
        first, second = a - b, c - b
        denominator = max(float(np.linalg.norm(first) * np.linalg.norm(second)), 1e-8)
        return float(np.degrees(np.arccos(np.clip(np.dot(first, second) / denominator, -1.0, 1.0))))

    def update(
        self, *, timestamp_s: float, points: np.ndarray, confidence: Sequence[float],
        index: Mapping[str, int], threshold: float, overlap: Mapping[str, bool],
        calibration_profile: Mapping[str, object] | None,
    ) -> dict[str, object]:
        xyz = np.asarray(points, dtype=np.float64)
        conf = np.asarray(confidence, dtype=np.float64)
        visible = np.isfinite(xyz).all(axis=1) & (conf >= threshold)
        swap = bool(
            visible[index["LEFT_SHOULDER"]]
            and visible[index["RIGHT_SHOULDER"]]
            and xyz[index["LEFT_SHOULDER"], 1] < xyz[index["RIGHT_SHOULDER"], 1]
        )
        if swap and not self._swap_active:
            self.swap_count += 1
        self._swap_active = swap
        for side in ("left", "right"):
            if overlap.get(side) and side not in self._occlusion_started:
                self._occlusion_started[side] = timestamp_s
            elif not overlap.get(side) and side in self._occlusion_started:
                self.occlusion_recovery_s.add(timestamp_s - self._occlusion_started.pop(side))

        bone_deviations = []
        profile = calibration_profile or {}
        pairs = {
            "left_upper_arm_m": ("LEFT_SHOULDER", "LEFT_ELBOW"),
            "right_upper_arm_m": ("RIGHT_SHOULDER", "RIGHT_ELBOW"),
            "left_forearm_m": ("LEFT_ELBOW", "LEFT_WRIST"),
            "right_forearm_m": ("RIGHT_ELBOW", "RIGHT_WRIST"),
            "left_thigh_m": ("LEFT_HIP", "LEFT_KNEE"),
            "right_thigh_m": ("RIGHT_HIP", "RIGHT_KNEE"),
        }
        for label, (first, second) in pairs.items():
            reference = profile.get(label)
            if reference and visible[index[first]] and visible[index[second]]:
                current = float(np.linalg.norm(xyz[index[first]] - xyz[index[second]]))
                bone_deviations.append(abs(current / float(reference) - 1.0))
        jerk_mean = None
        jerk_max = None
        angle_indices = [
            index[name]
            for name in (
                "LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST",
                "RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST",
                "LEFT_HIP", "LEFT_KNEE", "LEFT_ANKLE",
                "RIGHT_HIP", "RIGHT_KNEE", "RIGHT_ANKLE",
            )
        ]
        if np.isfinite(xyz[angle_indices]).all():
            angle_values = np.asarray(
                [
                    self._angle(xyz[index["LEFT_SHOULDER"]], xyz[index["LEFT_ELBOW"]], xyz[index["LEFT_WRIST"]]),
                    self._angle(xyz[index["RIGHT_SHOULDER"]], xyz[index["RIGHT_ELBOW"]], xyz[index["RIGHT_WRIST"]]),
                    self._angle(xyz[index["LEFT_HIP"]], xyz[index["LEFT_KNEE"]], xyz[index["LEFT_ANKLE"]]),
                    self._angle(xyz[index["RIGHT_HIP"]], xyz[index["RIGHT_KNEE"]], xyz[index["RIGHT_ANKLE"]]),
                ],
                dtype=np.float64,
            )
            self._angle_history.append((timestamp_s, angle_values))
        if len(self._angle_history) == 4:
            times = np.asarray([item[0] for item in self._angle_history])
            dt = float(np.median(np.diff(times)))
            if 0.005 <= dt <= 0.2:
                values = [item[1] for item in self._angle_history]
                jerk = (values[3] - 3 * values[2] + 3 * values[1] - values[0]) / (dt ** 3)
                jerk_mean = float(np.mean(np.abs(jerk)))
                jerk_max = float(np.max(np.abs(jerk)))
        return {
            "visible_keypoint_ratio": float(np.mean(visible)),
            "bone_length_relative_error_mean": float(np.mean(bone_deviations)) if bone_deviations else None,
            "left_right_swap_risk": swap,
            "left_right_swap_count": self.swap_count,
            "occlusion_recovery_s": self.occlusion_recovery_s.snapshot(),
            "joint_angular_jerk_mean_deg_s3": jerk_mean,
            "joint_angular_jerk_max_deg_s3": jerk_max,
        }

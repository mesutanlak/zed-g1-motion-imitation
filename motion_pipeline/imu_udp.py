"""Non-blocking UDP ingress for the optional Raspberry Pi arm IMU stream."""

from __future__ import annotations

import json
import socket
from typing import Any

from .imu_fusion import ArmImuMeasurement, TimeAlignedImuBuffer, parse_arm_imu_packet


class ArmImuUdpReceiver:
    """Drain IMU datagrams without delaying the BODY_38 capture/control loop."""

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 15060,
        *,
        capacity: int = 256,
        maximum_packet_bytes: int = 16_384,
    ) -> None:
        self.buffer = TimeAlignedImuBuffer(capacity=capacity)
        self.maximum_packet_bytes = int(maximum_packet_bytes)
        self.received = 0
        self.rejected = 0
        self.last_error: str | None = None
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((host, int(port)))
        self.socket.setblocking(False)

    @property
    def address(self) -> tuple[str, int]:
        host, port = self.socket.getsockname()[:2]
        return str(host), int(port)

    def poll(self, maximum_datagrams: int = 64) -> int:
        accepted = 0
        for _ in range(maximum_datagrams):
            try:
                payload, _source = self.socket.recvfrom(self.maximum_packet_bytes)
            except BlockingIOError:
                break
            try:
                packet: dict[str, Any] = json.loads(payload.decode("utf-8"))
                sample = parse_arm_imu_packet(packet)
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
                self.rejected += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
                continue
            self.buffer.append(sample)
            self.received += 1
            accepted += 1
        return accepted

    def align(
        self,
        timestamp_ns: int,
        *,
        side: str,
        max_age_ms: float = 100.0,
    ) -> ArmImuMeasurement | None:
        self.poll()
        return self.buffer.align(timestamp_ns, side=side, max_age_ms=max_age_ms)

    def diagnostics(self) -> dict[str, Any]:
        return {
            "received": self.received,
            "rejected": self.rejected,
            "last_error": self.last_error,
            "listen_address": list(self.address),
        }

    def close(self) -> None:
        self.socket.close()

    def __enter__(self) -> "ArmImuUdpReceiver":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

"""Best-effort asynchronous side-channel publishers for live telemetry."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import queue
import socket
import threading
from typing import Any, Callable


@dataclass
class PublisherStatistics:
    submitted: int = 0
    sent: int = 0
    dropped: int = 0
    errors: int = 0
    queue_high_watermark: int = 0


class AsyncJsonUdpPublisher:
    """Serialize and publish analysis packets outside the control loop."""

    def __init__(
        self,
        target: tuple[str, int],
        *,
        transform: Callable[[dict[str, Any]], dict[str, Any] | bytes] | None = None,
        queue_capacity: int = 512,
        maximum_datagram_bytes: int = 65_000,
    ) -> None:
        self.target = target
        self.transform = transform
        self.maximum_datagram_bytes = int(maximum_datagram_bytes)
        self.queue: queue.Queue[dict[str, Any]] = queue.Queue(
            maxsize=max(16, queue_capacity)
        )
        self.statistics = PublisherStatistics()
        self.stop_event = threading.Event()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
        self.thread = threading.Thread(
            target=self._run, name="telemetry-udp-publisher", daemon=True
        )
        self.thread.start()

    def submit(self, packet: dict[str, Any]) -> bool:
        self.statistics.submitted += 1
        try:
            self.queue.put_nowait(packet)
        except queue.Full:
            self.statistics.dropped += 1
            return False
        self.statistics.queue_high_watermark = max(
            self.statistics.queue_high_watermark, self.queue.qsize()
        )
        return True

    def _run(self) -> None:
        while not self.stop_event.is_set() or not self.queue.empty():
            try:
                packet = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                value = self.transform(packet) if self.transform else packet
                payload = (
                    value
                    if isinstance(value, bytes)
                    else json.dumps(
                        value,
                        ensure_ascii=False,
                        allow_nan=False,
                        separators=(",", ":"),
                    ).encode("utf-8")
                )
                if len(payload) > self.maximum_datagram_bytes:
                    self.statistics.dropped += 1
                    continue
                self.socket.sendto(payload, self.target)
                self.statistics.sent += 1
            except (OSError, TypeError, ValueError):
                self.statistics.errors += 1
            finally:
                self.queue.task_done()

    def snapshot(self) -> dict[str, int]:
        return asdict(self.statistics)

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=5.0)
        self.socket.close()
        if self.thread.is_alive():
            raise RuntimeError("telemetry publisher did not drain")

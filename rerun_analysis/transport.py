"""Non-blocking telemetry ingest and durable raw packet journals.

The control path publishes UDP telemetry on a best-effort side channel.  This
module makes the receiving side cheap: one thread only receives datagrams and
hands the newest decoded packet to the visualizer, while a second thread writes
every valid datagram to disk.  Heavy Rerun/CSV work is therefore never allowed
to stop socket draining.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
import queue
import socket
import threading
import time
from typing import Any, Iterable


@dataclass
class IngestStatistics:
    channel: str
    received: int = 0
    decoded: int = 0
    invalid: int = 0
    journal_written: int = 0
    journal_dropped: int = 0
    latest_replaced: int = 0
    sequence_gaps: int = 0
    duplicate_or_reordered: int = 0
    queue_high_watermark: int = 0
    started_unix_ns: int = 0
    closed_unix_ns: int = 0


class RawPacketJournal:
    """Write raw JSON datagrams without blocking their receiver.

    Queue overflow is treated as a visible integrity failure, never hidden.
    A 4096 packet queue holds more than two minutes at 30 Hz and normally stays
    close to zero because writing an already encoded line is much cheaper than
    the downstream analysis.
    """

    def __init__(
        self,
        path: Path,
        statistics: IngestStatistics,
        *,
        capacity: int = 4096,
    ) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.statistics = statistics
        self.queue: queue.Queue[bytes] = queue.Queue(maxsize=max(64, capacity))
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run,
            name=f"{statistics.channel}-journal",
            daemon=True,
        )
        self.thread.start()

    def submit(self, payload: bytes) -> None:
        try:
            self.queue.put_nowait(payload)
            self.statistics.queue_high_watermark = max(
                self.statistics.queue_high_watermark, self.queue.qsize()
            )
        except queue.Full:
            self.statistics.journal_dropped += 1

    def _run(self) -> None:
        with self.path.open("wb", buffering=4 * 1024 * 1024) as stream:
            last_flush = time.monotonic()
            while not self.stop_event.is_set() or not self.queue.empty():
                try:
                    payload = self.queue.get(timeout=0.10)
                except queue.Empty:
                    if time.monotonic() - last_flush >= 0.5:
                        stream.flush()
                        last_flush = time.monotonic()
                    continue
                try:
                    stream.write(payload.rstrip(b"\r\n"))
                    stream.write(b"\n")
                    self.statistics.journal_written += 1
                finally:
                    self.queue.task_done()
                if time.monotonic() - last_flush >= 0.5:
                    stream.flush()
                    last_flush = time.monotonic()
            stream.flush()

    def close(self, timeout_s: float = 5.0) -> None:
        self.stop_event.set()
        self.thread.join(timeout=timeout_s)
        if self.thread.is_alive():
            raise RuntimeError(f"Telemetry journal did not drain: {self.path}")


class LatestUdpReceiver:
    """Continuously drain UDP and expose only the newest packet per schema."""

    def __init__(
        self,
        host: str,
        port: int,
        *,
        channel: str,
        journal_path: Path,
        allowed_schemas: Iterable[str] | None = None,
        require_keypoints: bool = False,
        receive_buffer_bytes: int = 16 * 1024 * 1024,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.channel = channel
        self.allowed_schemas = set(allowed_schemas or ())
        self.require_keypoints = bool(require_keypoints)
        self.statistics = IngestStatistics(
            channel=channel, started_unix_ns=time.time_ns()
        )
        self.journal = RawPacketJournal(journal_path, self.statistics)
        self._latest: dict[str, dict[str, Any]] = {}
        self._last_sequence: dict[str, int] = {}
        self._condition = threading.Condition()
        self.stop_event = threading.Event()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(
            socket.SOL_SOCKET, socket.SO_RCVBUF, int(receive_buffer_bytes)
        )
        self.socket.bind((host, self.port))
        self.port = int(self.socket.getsockname()[1])
        self.socket.settimeout(0.2)
        self.thread = threading.Thread(
            target=self._run, name=f"{channel}-udp-ingest", daemon=True
        )

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        try:
            while not self.stop_event.is_set():
                try:
                    payload, _address = self.socket.recvfrom(2_000_000)
                except socket.timeout:
                    continue
                except OSError:
                    if self.stop_event.is_set():
                        break
                    raise
                self.statistics.received += 1
                try:
                    packet = json.loads(payload.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    self.statistics.invalid += 1
                    continue
                schema = str(packet.get("schema") or "unknown")
                if self.allowed_schemas and schema not in self.allowed_schemas:
                    self.statistics.invalid += 1
                    continue
                if self.require_keypoints and "keypoint_names" not in packet:
                    self.statistics.invalid += 1
                    continue
                self.statistics.decoded += 1
                # Journal only packets that passed the channel contract so a
                # replay never has to guess which malformed datagrams mattered.
                self.journal.submit(payload)
                sequence_value = packet.get("sequence", packet.get("frame_index"))
                if isinstance(sequence_value, int):
                    previous = self._last_sequence.get(schema)
                    if previous is not None:
                        if sequence_value <= previous:
                            self.statistics.duplicate_or_reordered += 1
                        elif sequence_value > previous + 1:
                            self.statistics.sequence_gaps += sequence_value - previous - 1
                    self._last_sequence[schema] = sequence_value
                with self._condition:
                    if schema in self._latest:
                        self.statistics.latest_replaced += 1
                    self._latest[schema] = packet
                    self._condition.notify_all()
        finally:
            self.statistics.closed_unix_ns = time.time_ns()

    def take_latest(
        self,
        *,
        schema: str | None = None,
        timeout_s: float = 0.2,
    ) -> dict[str, Any] | None:
        deadline = time.monotonic() + max(0.0, timeout_s)
        with self._condition:
            while not self.stop_event.is_set():
                if schema is not None and schema in self._latest:
                    return self._latest.pop(schema)
                if schema is None and self._latest:
                    key = max(
                        self._latest,
                        key=lambda item: int(
                            self._latest[item].get(
                                "timestamp_ns",
                                self._latest[item].get("source_timestamp_ns", 0),
                            )
                            or 0
                        ),
                    )
                    return self._latest.pop(key)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
        return None

    def take_all_latest(self, timeout_s: float = 0.2) -> list[dict[str, Any]]:
        deadline = time.monotonic() + max(0.0, timeout_s)
        with self._condition:
            while not self._latest and not self.stop_event.is_set():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return []
                self._condition.wait(remaining)
            packets = list(self._latest.values())
            self._latest.clear()
            return packets

    def snapshot(self) -> dict[str, Any]:
        return asdict(self.statistics)

    def close(self) -> None:
        self.stop_event.set()
        with self._condition:
            self._condition.notify_all()
        self.socket.close()
        self.thread.join(timeout=2.0)
        self.journal.close()
        self.statistics.closed_unix_ns = time.time_ns()

from __future__ import annotations

from collections import deque
import json
from pathlib import Path
import queue
import threading
from typing import Any, Callable

from .contracts import finite_or_none
from .retargeting import compose_targets


class BoundedOutputWorker:
    """Non-blocking visualization/record output with explicit drop metrics."""

    def __init__(self, consumer: Callable[[dict[str, Any]], None], maximum_queue: int = 32) -> None:
        self.consumer = consumer
        self.queue: queue.Queue[dict[str, Any] | None] = queue.Queue(maxsize=maximum_queue)
        self.dropped = 0
        self.errors: deque[str] = deque(maxlen=10)
        self.thread = threading.Thread(target=self._run, name="hand-output", daemon=True)
        self.thread.start()

    def submit(self, item: dict[str, Any]) -> None:
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    def _run(self) -> None:
        while True:
            item = self.queue.get()
            if item is None:
                return
            try:
                self.consumer(item)
            except Exception as exc:  # output cannot stop live perception
                self.errors.append(str(exc))

    def close(self) -> None:
        try:
            self.queue.put_nowait(None)
        except queue.Full:
            pass
        self.thread.join(timeout=2.0)


class JsonlHandWriter:
    def __init__(self, path: str | Path) -> None:
        self.file = Path(path).open("w", encoding="utf-8", buffering=1)

    def __call__(self, packet: dict[str, Any]) -> None:
        self.file.write(json.dumps(finite_or_none(packet), ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n")

    def close(self) -> None:
        self.file.close()


class SimulationTargetAdapter:
    """Writes separate body/left/right articulation groups; never DDS."""

    def __init__(self, body_writer: Callable[[list[float]], None], left_writer: Callable[[list[float]], None], right_writer: Callable[[list[float]], None]) -> None:
        self.body_writer, self.left_writer, self.right_writer = body_writer, left_writer, right_writer

    def write(self, target: dict[str, Any]) -> None:
        if target.get("physical_robot_output_enabled") is not False:
            raise ValueError("simulation contract must explicitly disable physical output")
        self.body_writer(target["q_body"])
        self.left_writer(target["q_left_dex3"])
        self.right_writer(target["q_right_dex3"])


class HandBodyTargetJoiner:
    """Timestamp-gated join for the existing 23-DOF and new 7+7 streams."""

    def __init__(self, maximum_spread_ms: float = 40.0) -> None:
        self.maximum_spread_ns = int(maximum_spread_ms * 1e6)
        self.body: tuple[int, list[float]] | None = None
        self.hands: tuple[int, list[float], list[float]] | None = None

    def update_body(self, timestamp_ns: int, q_body: list[float]) -> dict[str, Any] | None:
        self.body = (int(timestamp_ns), q_body)
        return self._compose()

    def update_hands(self, timestamp_ns: int, q_left: list[float], q_right: list[float]) -> dict[str, Any] | None:
        self.hands = (int(timestamp_ns), q_left, q_right)
        return self._compose()

    def _compose(self) -> dict[str, Any] | None:
        if self.body is None or self.hands is None:
            return None
        if abs(self.body[0] - self.hands[0]) > self.maximum_spread_ns:
            return None
        result = compose_targets(self.body[1], self.hands[1], self.hands[2])
        result["timestamp_ns"] = max(self.body[0], self.hands[0])
        result["join_spread_ms"] = abs(self.body[0] - self.hands[0]) / 1e6
        return result

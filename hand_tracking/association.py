from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class AssociationConfig:
    wrist_weight: float = 1.0
    temporal_weight: float = 0.35
    handedness_weight: float = 25.0
    time_weight: float = 0.08
    maximum_wrist_distance_px: float = 120.0
    switch_hysteresis_px: float = 18.0
    maximum_gap_ms: float = 250.0


class HandAssociator:
    def __init__(self, config: AssociationConfig) -> None:
        self.config = config
        self._previous: dict[str, tuple[np.ndarray, int, float]] = {}

    def reset(self) -> None:
        self._previous.clear()

    def select(
        self,
        side: str,
        candidates: list[dict[str, Any]],
        projected_wrist_px: np.ndarray,
        timestamp_ns: int,
    ) -> dict[str, Any] | None:
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        wrist = np.asarray(projected_wrist_px, dtype=float)
        previous = self._previous.get(side)
        scored: list[tuple[float, dict[str, Any], np.ndarray, float]] = []
        for candidate in candidates:
            points = np.asarray(candidate.get("landmarks_px"), dtype=float)
            if points.shape != (21, 2) or not np.isfinite(points).all():
                continue
            wrist_distance = float(np.linalg.norm(points[0] - wrist))
            if wrist_distance > self.config.maximum_wrist_distance_px:
                continue
            temporal_distance = 0.0
            time_gap_ms = 0.0
            if previous is not None:
                previous_points, previous_ns, _previous_score = previous
                time_gap_ms = max(0.0, (timestamp_ns - previous_ns) / 1e6)
                if time_gap_ms <= self.config.maximum_gap_ms:
                    temporal_distance = float(np.mean(np.linalg.norm(points - previous_points, axis=1)))
                else:
                    temporal_distance = self.config.maximum_wrist_distance_px
            label = str(candidate.get("handedness_label") or "").lower()
            disagreement = 1.0 if label and label != side else 0.0
            score = (
                self.config.wrist_weight * wrist_distance
                + self.config.temporal_weight * temporal_distance
                + self.config.handedness_weight * disagreement
                + self.config.time_weight * time_gap_ms
            )
            scored.append((score, candidate, points, wrist_distance))
        if not scored:
            return None
        scored.sort(key=lambda item: item[0])
        best = scored[0]
        # A challenger must beat continuity by a real margin; crop overlap and
        # crossed hands must not cause frame-by-frame identity flips.
        if previous is not None and len(scored) > 1:
            previous_points = previous[0]
            continuity = min(scored, key=lambda item: float(np.mean(np.linalg.norm(item[2] - previous_points, axis=1))))
            if continuity[0] <= best[0] + self.config.switch_hysteresis_px:
                best = continuity
        self._previous[side] = (best[2].copy(), int(timestamp_ns), float(best[0]))
        selected = dict(best[1])
        selected["association_score"] = float(best[0])
        selected["projected_wrist_distance_px"] = float(best[3])
        return selected

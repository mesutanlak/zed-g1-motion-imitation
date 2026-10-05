from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from .normalization import PalmNormalizer, PalmSE3Filter
from .retargeting import Dex3Retargeter, dex3_control_contract


class SingleCameraDex3Pipeline:
    """Convert one camera's MediaPipe-local hand shapes to DDS-free Dex3 targets."""

    def __init__(self, config_path: str | Path) -> None:
        self.normalizer = PalmNormalizer()
        self.se3_filter = PalmSE3Filter()
        self.retargeter = Dex3Retargeter(config_path)

    def reset(self) -> None:
        self.normalizer.reset()
        self.se3_filter.reset()
        self.retargeter.reset()

    def advance(self, timestamp_ns: int) -> tuple[dict[str, Any], dict[str, Any]]:
        """Advance independent watchdogs between slower ROI inference frames."""
        targets: dict[str, Any] = {
            "schema": "zed_single_camera_dex3_targets/v1",
            "source_schema": "no_new_hand_measurement",
            "coordinate_basis": "mediapipe_local_shape_normalized_se3_filtered",
            "physical_robot_output_enabled": False,
        }
        for side in ("left", "right"):
            targets[side] = self.retargeter.update(side, None, int(timestamp_ns), 0.0)
        return targets, dex3_control_contract(
            int(timestamp_ns), targets["left"], targets["right"]
        )

    def update(
        self, hand_packet: dict[str, Any], timestamp_ns: int
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        hands = {
            str(hand.get("side")): hand
            for hand in hand_packet.get("hands", [])
            if isinstance(hand, dict) and hand.get("side") in {"left", "right"}
        }
        targets: dict[str, Any] = {
            "schema": "zed_single_camera_dex3_targets/v1",
            "source_schema": str(hand_packet.get("schema", "")),
            "coordinate_basis": "mediapipe_local_shape_normalized_se3_filtered",
            "physical_robot_output_enabled": False,
        }
        for side in ("left", "right"):
            hand = hands.get(side) or {}
            normalized = None
            confidence = 0.0
            if hand and hand.get("rejection_reason") is None:
                try:
                    relative = np.asarray(
                        hand.get("relative_landmarks_m"), dtype=np.float64
                    )
                    normalized_hand = self.se3_filter.update(
                        self.normalizer.normalize(relative, side), timestamp_ns
                    )
                    normalized = normalized_hand.landmarks
                    landmark_confidence = [
                        float(value)
                        for value in hand.get("landmark_confidence", [])
                        if isinstance(value, (int, float))
                        and np.isfinite(float(value))
                    ]
                    confidence = float(
                        np.clip(
                            np.mean(landmark_confidence)
                            if landmark_confidence
                            else 1.0,
                            0.0,
                            1.0,
                        )
                    )
                    hand["normalization"] = {
                        "basis": "mediapipe_local_shape",
                        "scale_m": normalized_hand.scale_m,
                        "rotation_local_from_palm": (
                            normalized_hand.rotation_world_from_palm.tolist()
                        ),
                    }
                except (TypeError, ValueError) as exc:
                    hand["retarget_rejection_reason"] = str(exc)
                    confidence = 0.0
            targets[side] = self.retargeter.update(
                side, normalized, int(timestamp_ns), confidence
            )
        control = dex3_control_contract(
            int(timestamp_ns), targets["left"], targets["right"]
        )
        return targets, control

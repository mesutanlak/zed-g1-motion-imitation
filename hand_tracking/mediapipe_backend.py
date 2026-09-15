from __future__ import annotations

from pathlib import Path
import time
from typing import Any

import numpy as np


class MediaPipeHandBackend:
    """Lazy MediaPipe Tasks adapter; one instance is used per camera/side."""

    def __init__(
        self,
        model_path: str | Path,
        *,
        minimum_detection_confidence: float = 0.30,
        minimum_presence_confidence: float = 0.30,
        minimum_tracking_confidence: float = 0.30,
        delegate: str = "cpu",
    ) -> None:
        path = Path(model_path)
        if not path.is_file():
            raise FileNotFoundError(
                f"MediaPipe el modeli bulunamadi: {path}. Model otomatik indirilmez; "
                "--hand-model ile resmi hand_landmarker.task dosyasini belirtin."
            )
        try:
            import mediapipe as mp
        except ImportError as exc:
            raise RuntimeError(
                "El takibi acik fakat mediapipe kurulu degil. Ayrik ZED ortaminda "
                "'python -m pip install -r requirements-hand.txt' calistirin."
            ) from exc
        self._mp = mp
        self._confidence_gates = {
            "detection": float(minimum_detection_confidence),
            "presence": float(minimum_presence_confidence),
            "tracking": float(minimum_tracking_confidence),
        }
        base_options = mp.tasks.BaseOptions
        delegate_value = None
        if delegate.lower() == "gpu" and hasattr(base_options, "Delegate"):
            delegate_value = base_options.Delegate.GPU
        base_kwargs: dict[str, Any] = {"model_asset_path": str(path)}
        if delegate_value is not None:
            base_kwargs["delegate"] = delegate_value
        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=base_options(**base_kwargs),
            # Each instance receives one BODY_38 wrist ROI. IMAGE mode avoids
            # carrying tracker state across a crop coordinate system that moves
            # and rescales every frame.
            running_mode=mp.tasks.vision.RunningMode.IMAGE,
            num_hands=1,
            min_hand_detection_confidence=minimum_detection_confidence,
            min_hand_presence_confidence=minimum_presence_confidence,
            min_tracking_confidence=minimum_tracking_confidence,
        )
        self._landmarker = mp.tasks.vision.HandLandmarker.create_from_options(options)

    def close(self) -> None:
        self._landmarker.close()

    def detect(self, rgb_crop: np.ndarray, timestamp_ns: int) -> tuple[list[dict[str, Any]], float]:
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb_crop))
        started = time.perf_counter()
        result = self._landmarker.detect(image)
        inference_ms = (time.perf_counter() - started) * 1000.0
        candidates: list[dict[str, Any]] = []
        for index, landmarks in enumerate(result.hand_landmarks):
            handedness = result.handedness[index][0] if index < len(result.handedness) and result.handedness[index] else None
            points = [[float(item.x), float(item.y)] for item in landmarks]
            confidence = [
                float(value) if (value := getattr(item, "visibility", None)) is not None else 1.0
                for item in landmarks
            ]
            candidates.append({
                "landmarks_normalized": points,
                "landmark_confidence": confidence,
                "handedness_label": getattr(handedness, "category_name", None),
                "handedness_score": float(getattr(handedness, "score", 0.0) or 0.0),
                # Tasks does not expose three independent per-result scalars.
                # Keep them null instead of mislabelling handedness; record the
                # configured gates explicitly for reproducibility.
                "detection_confidence": None,
                "presence_confidence": None,
                "tracking_confidence": None,
                "confidence_thresholds": dict(self._confidence_gates),
                "relative_landmarks_m": (
                    [[float(item.x), float(item.y), float(item.z)] for item in result.hand_world_landmarks[index]]
                    if index < len(result.hand_world_landmarks)
                    else None
                ),
            })
        return candidates, inference_ms

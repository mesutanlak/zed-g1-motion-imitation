from __future__ import annotations

import hashlib
import time
from typing import Any, Callable

import numpy as np

from .association import AssociationConfig, HandAssociator
from .contracts import HAND_LANDMARK_NAMES, HAND_SCHEMA, encode_packet
from .geometry import CameraIntrinsics, deproject_pixel, robust_depth_patch
from .roi import RoiConfig, clipped_hand_roi, crop_to_full


class HandSourcePipeline:
    """Per-camera operator-locked crop/inference/depth pipeline."""

    def __init__(
        self,
        camera_serial: int,
        source_host_id: str,
        intrinsics: CameraIntrinsics,
        backend_factory: Callable[[str], Any],
        *,
        roi_config: RoiConfig | None = None,
        association_config: AssociationConfig | None = None,
        inference_fps: float = 12.0,
        depth_patch_radius_px: int = 3,
        maximum_depth_wrist_delta_m: float = 0.35,
    ) -> None:
        self.camera_serial = int(camera_serial)
        self.source_host_id = str(source_host_id)
        self.intrinsics = intrinsics
        self.backend_factory = backend_factory
        self.backends = {side: backend_factory(side) for side in ("left", "right")}
        self.roi_config = roi_config or RoiConfig()
        self.associator = HandAssociator(association_config or AssociationConfig())
        self.interval_ns = int(1e9 / max(inference_fps, 0.1))
        self.depth_patch_radius_px = depth_patch_radius_px
        self.maximum_depth_wrist_delta_m = maximum_depth_wrist_delta_m
        self.last_inference_ns: int | None = None
        canonical = (
            f"{intrinsics.fx:.9g},{intrinsics.fy:.9g},{intrinsics.cx:.9g},"
            f"{intrinsics.cy:.9g},{intrinsics.width},{intrinsics.height}"
        )
        self.intrinsics_id = hashlib.sha256(canonical.encode()).hexdigest()[:16]

    def close(self) -> None:
        for backend in self.backends.values():
            close = getattr(backend, "close", None)
            if close:
                close()

    def reset(self, *, recreate_backends: bool = False) -> None:
        self.associator.reset()
        self.last_inference_ns = None
        if recreate_backends:
            for backend in self.backends.values():
                close = getattr(backend, "close", None)
                if close:
                    close()
            self.backends = {
                side: self.backend_factory(side) for side in ("left", "right")
            }

    def due(self, capture_timestamp_ns: int) -> bool:
        return self.last_inference_ns is None or capture_timestamp_ns - self.last_inference_ns >= self.interval_ns

    def process(
        self,
        bgr_image: np.ndarray,
        depth_image_m: np.ndarray | None,
        body_keypoints_2d_px: np.ndarray,
        body_keypoints_3d_m: np.ndarray,
        *,
        body_id: int,
        unique_operator_id: str | None,
        operator_state: str,
        capture_timestamp_ns: int,
        sequence: int,
        indices: dict[str, int],
    ) -> dict[str, Any] | None:
        if operator_state != "LOCKED":
            self.reset()
            return None
        if not self.due(capture_timestamp_ns):
            return None
        self.last_inference_ns = capture_timestamp_ns
        image = np.asarray(bgr_image)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("BGR image must be HxWx3")
        hands: list[dict[str, Any]] = []
        for side in ("left", "right"):
            prefix = side.upper()
            wrist2d = np.asarray(body_keypoints_2d_px[indices[f"{prefix}_WRIST"]], dtype=float)
            elbow2d = np.asarray(body_keypoints_2d_px[indices[f"{prefix}_ELBOW"]], dtype=float)
            wrist3d = np.asarray(body_keypoints_3d_m[indices[f"{prefix}_WRIST"]], dtype=float)
            roi = clipped_hand_roi(wrist2d, elbow2d, (image.shape[1], image.shape[0]), self.roi_config)
            if roi is None:
                hands.append({"side": side, "rejection_reason": "WRIST_OR_ROI_OUTSIDE_IMAGE"})
                continue
            x, y, width, height = roi
            rgb = image[y:y + height, x:x + width, ::-1]
            started = time.perf_counter()
            candidates, inference_ms = self.backends[side].detect(rgb, capture_timestamp_ns)
            inference_ms = float(inference_ms if inference_ms is not None else (time.perf_counter() - started) * 1000)
            full_candidates: list[dict[str, Any]] = []
            for candidate in candidates:
                item = dict(candidate)
                item["landmarks_px"] = crop_to_full(item.pop("landmarks_normalized"), roi).tolist()
                full_candidates.append(item)
            selected = self.associator.select(side, full_candidates, wrist2d, capture_timestamp_ns)
            if selected is None:
                hands.append({
                    "side": side, "roi_xywh": list(roi), "rejection_reason": "NO_ASSOCIATED_HAND",
                    "inference_ms": inference_ms,
                })
                continue
            points_px = np.asarray(selected["landmarks_px"], dtype=float)
            wrist_depth = float(wrist3d[0]) if wrist3d.shape == (3,) and np.isfinite(wrist3d).all() else None
            camera_points: list[list[float] | None] = []
            depth_confidence: list[float] = []
            for pixel in points_px:
                if depth_image_m is None:
                    camera_points.append(None)
                    depth_confidence.append(0.0)
                    continue
                depth, depth_score = robust_depth_patch(
                    depth_image_m, pixel, wrist_depth_m=wrist_depth,
                    radius_px=self.depth_patch_radius_px,
                    maximum_wrist_delta_m=self.maximum_depth_wrist_delta_m,
                )
                camera_points.append(None if depth is None else deproject_pixel(pixel, depth, self.intrinsics).tolist())
                depth_confidence.append(depth_score)
            confidence = selected.get("landmark_confidence") or [selected.get("tracking_confidence", 0.0)] * 21
            optional_number = lambda value: None if value is None else float(value)
            hands.append({
                "side": side, "roi_xywh": list(roi),
                "landmarks_px": points_px.tolist(),
                "landmark_confidence": [float(value) for value in confidence],
                "camera_points_m": camera_points,
                "depth_confidence": depth_confidence,
                "handedness_label": selected.get("handedness_label"),
                "handedness_score": selected.get("handedness_score"),
                "detection_confidence": optional_number(selected.get("detection_confidence")),
                "presence_confidence": optional_number(selected.get("presence_confidence")),
                "tracking_confidence": optional_number(selected.get("tracking_confidence")),
                "confidence_thresholds": selected.get("confidence_thresholds"),
                "association_score": float(selected.get("association_score", 0.0)),
                "projected_wrist_distance_px": float(selected.get("projected_wrist_distance_px", 0.0)),
                "inference_ms": inference_ms, "rejection_reason": None,
            })
        return {
            "schema": HAND_SCHEMA,
            "camera_serial": self.camera_serial,
            "source_host_id": self.source_host_id,
            "sequence": int(sequence),
            "capture_timestamp_ns": int(capture_timestamp_ns),
            "image_size": [int(image.shape[1]), int(image.shape[0])],
            "intrinsics_id": self.intrinsics_id,
            "intrinsics": {
                "fx": self.intrinsics.fx, "fy": self.intrinsics.fy,
                "cx": self.intrinsics.cx, "cy": self.intrinsics.cy,
                "distortion": list(self.intrinsics.distortion),
                "pixels_rectified_by_zed_sdk": True,
                "image_size": {"width": self.intrinsics.width, "height": self.intrinsics.height},
            },
            "operator": {"state": "LOCKED", "body_id": int(body_id), "unique_id": unique_operator_id},
            "landmark_names": list(HAND_LANDMARK_NAMES),
            "hands": hands,
            "transport_metrics": {
                "packet_created_ns": time.time_ns(),
                "frame_age_ms": max(0.0, (time.time_ns() - int(capture_timestamp_ns)) / 1e6),
            },
            "physical_robot_output_enabled": False,
        }

    @staticmethod
    def encode(packet: dict[str, Any]) -> bytes:
        return encode_packet(packet)

from __future__ import annotations

import hashlib
import queue
import threading
import time
from typing import Any, Callable

import numpy as np

from .association import AssociationConfig, HandAssociator
from .contracts import HAND_LANDMARK_NAMES, HAND_SCHEMA, encode_packet
from .geometry import CameraIntrinsics, deproject_pixel, robust_depth_patch
from .roi import RoiConfig, clipped_hand_roi, crop_to_full


PALM_DEPTH_LANDMARKS = frozenset((0, 2, 5, 9, 17))


class HandSourcePipeline:
    """Operator-locked, ROI-only hand inference decoupled from ZED capture.

    The camera thread copies only the two small hand crops into a one-slot
    latest-only queue. A slow detector can drop an obsolete hand job, but can
    never delay BODY_38 capture or build an inference backlog.
    """

    def __init__(
        self,
        camera_serial: int,
        source_host_id: str,
        intrinsics: CameraIntrinsics,
        backend_factory: Callable[[str], Any],
        *,
        roi_config: RoiConfig | None = None,
        association_config: AssociationConfig | None = None,
        inference_fps: float = 8.0,
        depth_patch_radius_px: int = 4,
        maximum_depth_wrist_delta_m: float = 0.55,
        asynchronous: bool = True,
    ) -> None:
        self.camera_serial = int(camera_serial)
        self.source_host_id = str(source_host_id)
        self.intrinsics = intrinsics
        self.backend_factory = backend_factory
        self.backends = {side: backend_factory(side) for side in ("left", "right")}
        self.roi_config = roi_config or RoiConfig()
        self.associator = HandAssociator(association_config or AssociationConfig())
        self.interval_ns = int(1e9 / max(inference_fps, 0.1))
        self.depth_patch_radius_px = int(depth_patch_radius_px)
        self.maximum_depth_wrist_delta_m = float(maximum_depth_wrist_delta_m)
        self.asynchronous = bool(asynchronous)
        self.last_inference_ns: int | None = None
        self._closed = False
        self._jobs: queue.Queue[dict[str, Any] | None] = queue.Queue(maxsize=1)
        self._results: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=1)
        self._worker: threading.Thread | None = None
        self.dropped_jobs = 0
        self._generation = 0
        canonical = (
            f"{intrinsics.fx:.9g},{intrinsics.fy:.9g},{intrinsics.cx:.9g},"
            f"{intrinsics.cy:.9g},{intrinsics.width},{intrinsics.height}"
        )
        self.intrinsics_id = hashlib.sha256(canonical.encode()).hexdigest()[:16]
        if self.asynchronous:
            self._worker = threading.Thread(
                target=self._worker_loop,
                name=f"hand-roi-{self.camera_serial}",
                daemon=True,
            )
            self._worker.start()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._worker is not None:
            try:
                self._jobs.put_nowait(None)
            except queue.Full:
                try:
                    self._jobs.get_nowait()
                except queue.Empty:
                    pass
                self._jobs.put_nowait(None)
            self._worker.join(timeout=3.0)
        for backend in self.backends.values():
            close = getattr(backend, "close", None)
            if close:
                close()

    def reset(self, *, recreate_backends: bool = False) -> None:
        self._generation += 1
        self.associator.reset()
        self.last_inference_ns = None
        for channel in (self._jobs, self._results):
            while True:
                try:
                    channel.get_nowait()
                except queue.Empty:
                    break
        if recreate_backends:
            for backend in self.backends.values():
                close = getattr(backend, "close", None)
                if close:
                    close()
            self.backends = {
                side: self.backend_factory(side) for side in ("left", "right")
            }

    def due(self, capture_timestamp_ns: int) -> bool:
        return (
            self.last_inference_ns is None
            or capture_timestamp_ns - self.last_inference_ns >= self.interval_ns
        )

    @staticmethod
    def _coarse_hand_points(
        side: str, body_keypoints_2d_px: np.ndarray, indices: dict[str, int]
    ) -> list[np.ndarray]:
        prefix = side.upper()
        names = (
            f"{prefix}_HAND_THUMB_4",
            f"{prefix}_HAND_INDEX_1",
            f"{prefix}_HAND_MIDDLE_4",
            f"{prefix}_HAND_PINKY_1",
        )
        return [
            np.asarray(body_keypoints_2d_px[indices[name]], dtype=float)
            for name in names
            if name in indices
        ]

    def _make_job(
        self,
        bgr_image: np.ndarray,
        depth_image_m: np.ndarray | None,
        body_keypoints_2d_px: np.ndarray,
        body_keypoints_3d_m: np.ndarray,
        *,
        body_id: int,
        unique_operator_id: str | None,
        capture_timestamp_ns: int,
        sequence: int,
        indices: dict[str, int],
    ) -> dict[str, Any]:
        image = np.asarray(bgr_image)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("BGR image must be HxWx3")
        depth = None if depth_image_m is None else np.asarray(depth_image_m)
        sides: dict[str, dict[str, Any]] = {}
        for side in ("left", "right"):
            prefix = side.upper()
            wrist2d = np.asarray(body_keypoints_2d_px[indices[f"{prefix}_WRIST"]], dtype=float)
            elbow2d = np.asarray(body_keypoints_2d_px[indices[f"{prefix}_ELBOW"]], dtype=float)
            wrist3d = np.asarray(body_keypoints_3d_m[indices[f"{prefix}_WRIST"]], dtype=float)
            roi = clipped_hand_roi(
                wrist2d,
                elbow2d,
                (image.shape[1], image.shape[0]),
                self.roi_config,
                self._coarse_hand_points(side, body_keypoints_2d_px, indices),
            )
            if roi is None:
                sides[side] = {"roi": None}
                continue
            x, y, width, height = roi
            sides[side] = {
                "roi": roi,
                "rgb": np.ascontiguousarray(image[y:y + height, x:x + width, ::-1]),
                "depth": (
                    None if depth is None
                    else np.array(depth[y:y + height, x:x + width], copy=True)
                ),
                "wrist2d": wrist2d.copy(),
                "wrist3d": wrist3d.copy(),
            }
        return {
            "generation": self._generation,
            "capture_timestamp_ns": int(capture_timestamp_ns),
            "sequence": int(sequence),
            "body_id": int(body_id),
            "unique_operator_id": unique_operator_id,
            "image_size": (int(image.shape[1]), int(image.shape[0])),
            "sides": sides,
            "queued_ns": time.time_ns(),
        }

    def _process_job(self, job: dict[str, Any]) -> dict[str, Any]:
        capture_timestamp_ns = int(job["capture_timestamp_ns"])
        hands: list[dict[str, Any]] = []
        for side in ("left", "right"):
            source = job["sides"][side]
            roi = source["roi"]
            if roi is None:
                hands.append({"side": side, "rejection_reason": "WRIST_OR_ROI_OUTSIDE_IMAGE"})
                continue
            started = time.perf_counter()
            candidates, inference_ms = self.backends[side].detect(source["rgb"], capture_timestamp_ns)
            inference_ms = float(
                inference_ms if inference_ms is not None
                else (time.perf_counter() - started) * 1000.0
            )
            full_candidates: list[dict[str, Any]] = []
            for candidate in candidates:
                item = dict(candidate)
                item["landmarks_px"] = crop_to_full(item.pop("landmarks_normalized"), roi).tolist()
                full_candidates.append(item)
            selected = self.associator.select(side, full_candidates, source["wrist2d"], capture_timestamp_ns)
            if selected is None:
                hands.append({
                    "side": side,
                    "roi_xywh": list(roi),
                    "candidate_count": len(full_candidates),
                    "rejection_reason": self.associator.rejection_reason(
                        side, full_candidates, source["wrist2d"], capture_timestamp_ns
                    ),
                    "inference_ms": inference_ms,
                })
                continue
            points_px = np.asarray(selected["landmarks_px"], dtype=float)
            wrist3d = source["wrist3d"]
            wrist_depth = (
                float(wrist3d[0])
                if wrist3d.shape == (3,) and np.isfinite(wrist3d).all()
                else None
            )
            camera_points: list[list[float] | None] = []
            depth_confidence: list[float] = []
            x, y, _width, _height = roi
            for landmark_index, pixel in enumerate(points_px):
                if source["depth"] is None or landmark_index not in PALM_DEPTH_LANDMARKS:
                    camera_points.append(None)
                    depth_confidence.append(0.0)
                    continue
                local_pixel = pixel - np.asarray([x, y], dtype=float)
                depth_value, depth_score = robust_depth_patch(
                    source["depth"], local_pixel,
                    wrist_depth_m=wrist_depth,
                    radius_px=self.depth_patch_radius_px,
                    maximum_wrist_delta_m=self.maximum_depth_wrist_delta_m,
                )
                camera_points.append(
                    None if depth_value is None
                    else deproject_pixel(pixel, depth_value, self.intrinsics).tolist()
                )
                depth_confidence.append(depth_score)
            confidence = selected.get("landmark_confidence") or [selected.get("tracking_confidence", 0.0)] * 21
            optional_number = lambda value: None if value is None else float(value)
            handedness_label = selected.get("handedness_label")
            hands.append({
                "side": side,
                "roi_xywh": list(roi),
                "candidate_count": len(full_candidates),
                "landmarks_px": points_px.tolist(),
                "relative_landmarks_m": selected.get("relative_landmarks_m"),
                "landmark_confidence": [float(value) for value in confidence],
                "camera_points_m": camera_points,
                "depth_confidence": depth_confidence,
                "depth_valid_count": sum(item is not None for item in camera_points),
                "handedness_label": handedness_label,
                "handedness_score": selected.get("handedness_score"),
                "handedness_mismatch": bool(handedness_label and str(handedness_label).lower() != side),
                "detection_confidence": optional_number(selected.get("detection_confidence")),
                "presence_confidence": optional_number(selected.get("presence_confidence")),
                "tracking_confidence": optional_number(selected.get("tracking_confidence")),
                "confidence_thresholds": selected.get("confidence_thresholds"),
                "association_score": float(selected.get("association_score", 0.0)),
                "projected_wrist_distance_px": float(selected.get("projected_wrist_distance_px", 0.0)),
                "inference_ms": inference_ms,
                "rejection_reason": None,
            })
        completed_ns = time.time_ns()
        return {
            "schema": HAND_SCHEMA,
            "camera_serial": self.camera_serial,
            "source_host_id": self.source_host_id,
            "sequence": int(job["sequence"]),
            "capture_timestamp_ns": capture_timestamp_ns,
            "image_size": list(job["image_size"]),
            "intrinsics_id": self.intrinsics_id,
            "intrinsics": {
                "fx": self.intrinsics.fx, "fy": self.intrinsics.fy,
                "cx": self.intrinsics.cx, "cy": self.intrinsics.cy,
                "distortion": list(self.intrinsics.distortion),
                "pixels_rectified_by_zed_sdk": True,
                "image_size": {"width": self.intrinsics.width, "height": self.intrinsics.height},
            },
            "operator": {"state": "LOCKED", "body_id": int(job["body_id"]), "unique_id": job["unique_operator_id"]},
            "landmark_names": list(HAND_LANDMARK_NAMES),
            "hands": hands,
            "transport_metrics": {
                "queued_ns": int(job["queued_ns"]),
                "packet_created_ns": completed_ns,
                "inference_queue_ms": max(0.0, (completed_ns - int(job["queued_ns"])) / 1e6),
                "frame_age_ms": max(0.0, (completed_ns - capture_timestamp_ns) / 1e6),
                "dropped_jobs": int(self.dropped_jobs),
                "latest_only_async": self.asynchronous,
            },
            "physical_robot_output_enabled": False,
        }

    def _worker_loop(self) -> None:
        while not self._closed:
            job = self._jobs.get()
            if job is None:
                break
            try:
                try:
                    result = self._process_job(job)
                except Exception as exc:
                    # A transient detector/runtime failure must not terminate
                    # the daemon and silently disable hand packets forever.
                    completed_ns = time.time_ns()
                    result = {
                        "schema": HAND_SCHEMA,
                        "camera_serial": self.camera_serial,
                        "source_host_id": self.source_host_id,
                        "sequence": int(job["sequence"]),
                        "capture_timestamp_ns": int(job["capture_timestamp_ns"]),
                        "image_size": list(job["image_size"]),
                        "intrinsics_id": self.intrinsics_id,
                        "intrinsics": {
                            "fx": self.intrinsics.fx, "fy": self.intrinsics.fy,
                            "cx": self.intrinsics.cx, "cy": self.intrinsics.cy,
                            "distortion": list(self.intrinsics.distortion),
                            "pixels_rectified_by_zed_sdk": True,
                            "image_size": {
                                "width": self.intrinsics.width,
                                "height": self.intrinsics.height,
                            },
                        },
                        "operator": {
                            "state": "LOCKED", "body_id": int(job["body_id"]),
                            "unique_id": job["unique_operator_id"],
                        },
                        "landmark_names": list(HAND_LANDMARK_NAMES),
                        "hands": [
                            {
                                "side": side,
                                "rejection_reason": "DETECTOR_RUNTIME_ERROR",
                            }
                            for side in ("left", "right")
                        ],
                        "transport_metrics": {
                            "queued_ns": int(job["queued_ns"]),
                            "packet_created_ns": completed_ns,
                            "inference_queue_ms": max(
                                0.0, (completed_ns - int(job["queued_ns"])) / 1e6
                            ),
                            "frame_age_ms": max(
                                0.0,
                                (completed_ns - int(job["capture_timestamp_ns"])) / 1e6,
                            ),
                            "dropped_jobs": int(self.dropped_jobs),
                            "latest_only_async": True,
                            "worker_error_type": type(exc).__name__,
                        },
                        "physical_robot_output_enabled": False,
                    }
                if int(job.get("generation", -1)) != self._generation:
                    continue
                try:
                    self._results.put_nowait(result)
                except queue.Full:
                    try:
                        self._results.get_nowait()
                    except queue.Empty:
                        pass
                    self._results.put_nowait(result)
            finally:
                self._jobs.task_done()

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
        latest = None
        while True:
            try:
                latest = self._results.get_nowait()
            except queue.Empty:
                break
        if not self.due(capture_timestamp_ns):
            return latest
        job = self._make_job(
            bgr_image, depth_image_m, body_keypoints_2d_px, body_keypoints_3d_m,
            body_id=body_id, unique_operator_id=unique_operator_id,
            capture_timestamp_ns=capture_timestamp_ns, sequence=sequence,
            indices=indices,
        )
        self.last_inference_ns = int(capture_timestamp_ns)
        if not self.asynchronous:
            return self._process_job(job)
        try:
            self._jobs.put_nowait(job)
        except queue.Full:
            # Replace the queued (not currently executing) frame. At most one
            # detector job can wait, so latency is bounded without blocking ZED.
            try:
                self._jobs.get_nowait()
                self._jobs.task_done()
            except queue.Empty:
                pass
            self.dropped_jobs += 1
            self._jobs.put_nowait(job)
        return latest

    @staticmethod
    def encode(packet: dict[str, Any]) -> bytes:
        return encode_packet(packet)

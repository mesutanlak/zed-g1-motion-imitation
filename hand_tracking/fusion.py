from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from .geometry import (
    CameraIntrinsics, camera_ray, project_world_point, triangulate_rays,
    transform_point_camera_to_world, transform_ray_camera_to_world,
)


@dataclass(frozen=True)
class CameraPose:
    rotation_camera_to_world: np.ndarray
    translation_camera_to_world_m: np.ndarray


def interpolate_landmarks(
    previous: dict[str, Any], current: dict[str, Any], target_timestamp_ns: int,
    *, maximum_prediction_ms: float,
) -> dict[str, Any] | None:
    t0, t1 = int(previous["capture_timestamp_ns"]), int(current["capture_timestamp_ns"])
    if t1 <= t0 or target_timestamp_ns < t1:
        return current
    prediction_ms = (target_timestamp_ns - t1) / 1e6
    if prediction_ms > maximum_prediction_ms:
        return None
    result = dict(current)
    result["_source_capture_timestamp_ns"] = t1
    result["hands"] = []
    ratio = (target_timestamp_ns - t1) / (t1 - t0)
    previous_by_side = {item.get("side"): item for item in previous.get("hands", [])}
    for hand in current.get("hands", []):
        item = dict(hand)
        older = previous_by_side.get(hand.get("side"))
        try:
            p0 = np.asarray(older["landmarks_px"], dtype=float)
            p1 = np.asarray(hand["landmarks_px"], dtype=float)
            if p0.shape == p1.shape == (21, 2):
                item["landmarks_px"] = (p1 + (p1 - p0) * ratio).tolist()
                # Depth belongs to the original pixels and must not be moved by
                # image-space prediction. Predicted landmarks contribute rays;
                # fresh measurements still contribute robust depth.
                item["camera_points_m"] = [None] * 21
                item["depth_confidence"] = [0.0] * 21
        except (KeyError, TypeError, ValueError):
            pass
        item["temporal_prediction_ms"] = float(prediction_ms)
        result["hands"].append(item)
    result["capture_timestamp_ns"] = int(target_timestamp_ns)
    return result


def _robust_depth_candidates(candidates: list[tuple[int, np.ndarray, float]], gate_m: float) -> list[tuple[int, np.ndarray, float]]:
    if len(candidates) <= 2:
        return candidates
    points = np.stack([item[1] for item in candidates])
    median = np.median(points, axis=0)
    distance = np.linalg.norm(points - median, axis=1)
    keep = distance <= max(gate_m, float(np.median(distance) * 2.5))
    result = [item for item, accepted in zip(candidates, keep) if accepted]
    return result if result else [candidates[int(np.argmin(distance))]]


def fuse_hand_packets(
    packets: Sequence[dict[str, Any]], camera_poses: dict[int, CameraPose],
    *, target_timestamp_ns: int, maximum_age_ms: float = 70.0,
    maximum_capture_spread_ms: float = 40.0,
    depth_outlier_gate_m: float = 0.08, single_view_depth: bool = True,
) -> dict[str, Any]:
    result: dict[str, Any] = {"schema": "zed_operator_hands_fused/v1", "capture_timestamp_ns": int(target_timestamp_ns), "hands": []}
    for side in ("left", "right"):
        observations: list[tuple[int, dict, CameraIntrinsics, CameraPose, float, int]] = []
        rejection_reasons: list[str] = []
        for packet in packets:
            age_ms = abs(target_timestamp_ns - int(packet.get("capture_timestamp_ns", 0))) / 1e6
            if age_ms > maximum_age_ms:
                rejection_reasons.append(f"STALE:{packet.get('camera_serial')}")
                continue
            serial = int(packet["camera_serial"])
            pose = camera_poses.get(serial)
            if pose is None:
                rejection_reasons.append(f"NO_EXTRINSIC:{serial}")
                continue
            hand = next((item for item in packet.get("hands", []) if item.get("side") == side and not item.get("rejection_reason")), None)
            if hand is None:
                continue
            age_ms = max(age_ms, float(hand.get("temporal_prediction_ms", 0.0) or 0.0))
            source_capture_ns = int(packet.get("_source_capture_timestamp_ns", packet["capture_timestamp_ns"]))
            observations.append((serial, hand, CameraIntrinsics.from_mapping(packet["intrinsics"]), pose, age_ms, source_capture_ns))
        if observations:
            spread_limit_ns = int(maximum_capture_spread_ms * 1e6)
            ordered = sorted(observations, key=lambda item: item[5])
            coherent_sets = [
                [item for item in ordered if anchor[5] <= item[5] <= anchor[5] + spread_limit_ns]
                for anchor in ordered
            ]
            coherent = max(
                coherent_sets,
                key=lambda group: (len(group), -max((item[4] for item in group), default=float("inf"))),
            )
            excluded = {item[0] for item in observations} - {item[0] for item in coherent}
            rejection_reasons.extend(f"CAPTURE_SPREAD:{serial}" for serial in sorted(excluded))
            observations = coherent
        fused = np.full((21, 3), np.nan)
        qualities: list[dict[str, Any]] = []
        for landmark_index in range(21):
            depth_candidates: list[tuple[int, np.ndarray, float]] = []
            ray_candidates: list[tuple[int, np.ndarray, np.ndarray]] = []
            for serial, hand, intrinsics, pose, age_ms, _capture_ns in observations:
                pixel = np.asarray(hand["landmarks_px"][landmark_index], dtype=float)
                confidence = float(hand.get("landmark_confidence", [1.0] * 21)[landmark_index]) * np.exp(-age_ms / 55.0)
                point_value = (hand.get("camera_points_m") or [None] * 21)[landmark_index]
                if point_value is not None:
                    point = transform_point_camera_to_world(point_value, pose.rotation_camera_to_world, pose.translation_camera_to_world_m)
                    depth_score = float((hand.get("depth_confidence") or [1.0] * 21)[landmark_index])
                    depth_candidates.append((serial, point, confidence * max(depth_score, 0.05)))
                ray = transform_ray_camera_to_world(camera_ray(pixel, intrinsics), pose.rotation_camera_to_world)
                ray_candidates.append((serial, pose.translation_camera_to_world_m, ray))
            inliers = _robust_depth_candidates(depth_candidates, depth_outlier_gate_m)
            mode = "invalid"
            residual = None
            used_serials: list[int] = []
            if len(inliers) >= 2:
                weights = np.asarray([item[2] for item in inliers], dtype=float)
                fused[landmark_index] = np.average(np.stack([item[1] for item in inliers]), axis=0, weights=weights)
                used_serials = [item[0] for item in inliers]
                residual = float(np.sqrt(np.mean(np.sum((np.stack([item[1] for item in inliers]) - fused[landmark_index]) ** 2, axis=1))))
                mode = "multi_depth"
            elif len(ray_candidates) >= 2:
                # Pairwise robust triangulation: score each candidate against all
                # calibrated views, then retain the lowest median ray distance.
                candidates: list[tuple[np.ndarray, float, tuple[int, int]]] = []
                for i in range(len(ray_candidates)):
                    for j in range(i + 1, len(ray_candidates)):
                        try:
                            point, _ = triangulate_rays(
                                [ray_candidates[i][1], ray_candidates[j][1]],
                                [ray_candidates[i][2], ray_candidates[j][2]],
                            )
                        except ValueError:
                            continue
                        errors = [np.linalg.norm(np.cross(point - origin, ray)) for _, origin, ray in ray_candidates]
                        candidates.append((point, float(np.median(errors)), (ray_candidates[i][0], ray_candidates[j][0])))
                if candidates:
                    point, residual, pair = min(candidates, key=lambda item: item[1])
                    if residual <= 0.03:
                        fused[landmark_index] = point
                        used_serials = list(pair)
                        mode = "triangulated"
            if mode == "invalid" and single_view_depth and len(inliers) == 1:
                fused[landmark_index] = inliers[0][1]
                used_serials = [inliers[0][0]]
                mode = "single_depth"
            reprojection_errors: list[float] = []
            if mode != "invalid":
                for serial, hand, intrinsics, pose, _age_ms, _capture_ns in observations:
                    if serial not in used_serials:
                        continue
                    try:
                        projected = project_world_point(
                            fused[landmark_index], pose.rotation_camera_to_world,
                            pose.translation_camera_to_world_m, intrinsics,
                        )
                        measured = np.asarray(hand["landmarks_px"][landmark_index], dtype=float)
                        reprojection_errors.append(float(np.linalg.norm(projected - measured)))
                    except ValueError:
                        pass
            confidence = 0.0 if mode == "invalid" else float(np.clip((len(used_serials) / 2.0) * np.exp(-(residual or 0.0) / 0.03), 0.0, 1.0))
            qualities.append({
                "mode": mode, "camera_count": len(used_serials),
                "serials": used_serials, "residual_m": residual,
                "reprojection_error_px": (float(np.median(reprojection_errors)) if reprojection_errors else None),
                "confidence": confidence,
            })
        valid = bool(np.isfinite(fused).all())
        result["hands"].append({
            "side": side, "valid": valid,
            "landmarks_world_m": fused.tolist() if valid else None,
            "landmark_quality": qualities,
            "capture_spread_ms": ((max((item[5] for item in observations), default=target_timestamp_ns) - min((item[5] for item in observations), default=target_timestamp_ns)) / 1e6),
            "rejection_reasons": rejection_reasons if rejection_reasons else ([] if valid else ["INSUFFICIENT_RELIABLE_VIEWS"]),
        })
    return result

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import time

import numpy as np

from hand_tracking.association import AssociationConfig, HandAssociator
from hand_tracking.contracts import (
    HAND_SCHEMA, encode_packet, validate_dex3_control, validate_hand_packet,
)
from hand_tracking.fusion import CameraPose, fuse_hand_packets, interpolate_landmarks
from hand_tracking.geometry import (
    CameraIntrinsics, deproject_pixel, project_world_point, robust_depth_patch,
    transform_point_camera_to_world, triangulate_rays,
)
from hand_tracking.normalization import PalmNormalizer
from hand_tracking.outputs import HandBodyTargetJoiner
from hand_tracking.pipeline import HandSourcePipeline
from hand_tracking.retargeting import (
    Dex3Retargeter, compose_targets, dex3_control_contract,
)
from hand_tracking.roi import RoiConfig, clipped_hand_roi, crop_to_full, full_to_crop
from hand_tracking.mediapipe_backend import MediaPipeHandBackend
from tools.benchmark_hand_tracking import summarize


ROOT = Path(__file__).resolve().parents[1]


def hand_shape() -> np.ndarray:
    p = np.zeros((21, 3), dtype=float)
    p[0] = [0, 0, 0]
    bases = {1: [-0.035, 0.025, 0.01], 5: [-0.03, 0.07, 0], 9: [0, 0.08, 0], 13: [0.025, 0.073, 0], 17: [0.045, 0.06, 0]}
    chains = ((1, 4), (5, 8), (9, 12), (13, 16), (17, 20))
    for start, end in chains:
        p[start] = bases[start]
        for index in range(start + 1, end + 1):
            p[index] = p[index - 1] + [0, 0.025, 0.002 * (index - start)]
    return p


def valid_packet(serial: int = 1, timestamp_ns: int = 1_000_000_000) -> dict:
    points = np.column_stack((np.linspace(300, 360, 21), np.linspace(200, 280, 21)))
    return {
        "schema": HAND_SCHEMA, "camera_serial": serial, "source_host_id": "host",
        "sequence": 1, "capture_timestamp_ns": timestamp_ns,
        "image_size": [1280, 720],
        "intrinsics": {"fx": 700, "fy": 700, "cx": 640, "cy": 360, "image_size": {"width": 1280, "height": 720}},
        "operator": {"state": "LOCKED", "body_id": 7},
        "hands": [{
            "side": "left", "roi_xywh": [250, 150, 200, 200],
            "landmarks_px": points.tolist(), "landmark_confidence": [0.9] * 21,
            "camera_points_m": [[3.0, 0.01 * i, 0.0] for i in range(21)],
            "depth_confidence": [0.9] * 21, "rejection_reason": None,
        }],
    }


def test_body38_wrist_roi_scales_and_clips_at_image_boundary() -> None:
    roi = clipped_hand_roi([15, 20], [80, 100], (1280, 720), RoiConfig(minimum_px=96, maximum_px=300))
    assert roi is not None
    x, y, width, height = roi
    assert x == 0 and y == 0
    assert 24 <= width <= 300 and 24 <= height <= 300


def test_crop_full_image_landmark_round_trip_is_lossless() -> None:
    normalized = np.random.default_rng(4).uniform(0, 1, size=(21, 2))
    roi = (111, 77, 245, 190)
    assert np.allclose(full_to_crop(crop_to_full(normalized, roi), roi), normalized)


def test_association_keeps_anatomical_side_when_handedness_is_wrong_and_hands_cross() -> None:
    associator = HandAssociator(AssociationConfig(switch_hysteresis_px=30))
    base = np.tile([100.0, 100.0], (21, 1))
    first = {"landmarks_px": base.tolist(), "handedness_label": "Right"}
    selected = associator.select("left", [first], np.array([100, 100]), 1_000_000_000)
    assert selected is not None
    continuity = {"landmarks_px": (base + [8, 0]).tolist(), "handedness_label": "Right"}
    challenger = {"landmarks_px": (base + [15, 0]).tolist(), "handedness_label": "Left"}
    selected = associator.select("left", [challenger, continuity], np.array([114, 100]), 1_030_000_000)
    assert np.allclose(selected["landmarks_px"], continuity["landmarks_px"])


def test_timestamp_prediction_is_bounded_and_stale_is_rejected() -> None:
    older = valid_packet(timestamp_ns=1_000_000_000)
    newer = valid_packet(timestamp_ns=1_050_000_000)
    newer["hands"][0]["landmarks_px"] = (np.asarray(older["hands"][0]["landmarks_px"]) + 5).tolist()
    predicted = interpolate_landmarks(older, newer, 1_100_000_000, maximum_prediction_ms=70)
    assert predicted is not None
    assert np.allclose(predicted["hands"][0]["landmarks_px"], np.asarray(newer["hands"][0]["landmarks_px"]) + 5)
    assert interpolate_landmarks(older, newer, 1_130_000_000, maximum_prediction_ms=70) is None


def test_synthetic_triangulation_is_below_ten_mm_with_one_outlier_camera() -> None:
    point = np.array([3.0, 0.1, 1.2])
    origins = [np.array([0, -0.5, 0]), np.array([0, 0.5, 0]), np.array([0, 0, 0.5])]
    rays = [(point - origin) / np.linalg.norm(point - origin) for origin in origins]
    rays[2] = np.array([1.0, 0.6, -0.2]); rays[2] /= np.linalg.norm(rays[2])
    pair_candidates = []
    for i, j in ((0, 1), (0, 2), (1, 2)):
        estimate, _ = triangulate_rays([origins[i], origins[j]], [rays[i], rays[j]])
        residuals = [np.linalg.norm(np.cross(estimate - origin, ray)) for origin, ray in zip(origins, rays)]
        pair_candidates.append((np.median(residuals), estimate))
    estimate = min(pair_candidates, key=lambda item: item[0])[1]
    assert np.linalg.norm(estimate - point) < 0.010


def test_fast_hand_capture_spread_above_40ms_is_not_fused() -> None:
    packets = [
        valid_packet(1, 1_000_000_000),
        valid_packet(2, 1_070_000_000),
        valid_packet(3, 1_005_000_000),
    ]
    poses = {serial: CameraPose(np.eye(3), np.zeros(3)) for serial in (1, 2, 3)}
    fused = fuse_hand_packets(
        packets, poses, target_timestamp_ns=1_070_000_000,
        maximum_age_ms=80, maximum_capture_spread_ms=40,
    )
    left = fused["hands"][0]
    assert "CAPTURE_SPREAD:2" in left["rejection_reasons"]
    assert all(item["serials"] == [1, 3] for item in left["landmark_quality"])


def test_depth_patch_rejects_background_wall_outliers() -> None:
    depth = np.full((15, 15), 5.0)
    depth[4:11, 4:11] = 3.0
    depth[5, 5] = 7.0
    value, confidence = robust_depth_patch(depth, [7, 7], wrist_depth_m=3.05, radius_px=3, maximum_wrist_delta_m=0.25)
    assert value == 3.0 and confidence > 0.7


def test_camera_to_world_direction_and_zed_axis_units() -> None:
    intrinsics = CameraIntrinsics(700, 700, 640, 360, 1280, 720)
    local = deproject_pixel([640, 360], 3.0, intrinsics)
    world = transform_point_camera_to_world(local, np.eye(3), np.array([1.0, 2.0, 0.5]))
    assert np.allclose(local, [3, 0, 0])
    assert np.allclose(world, [4, 2, 0.5])
    assert np.allclose(project_world_point(world, np.eye(3), [1, 2, .5], intrinsics), [640, 360])


def test_left_right_canonical_palm_frame_is_scale_invariant_and_right_handed() -> None:
    normalizer = PalmNormalizer()
    right = normalizer.normalize(hand_shape() + [2, 0, 1], "right")
    left_points = hand_shape().copy(); left_points[:, 0] *= -1
    left = normalizer.normalize(left_points * 2 + [2, 0, 1], "left")
    assert np.linalg.det(right.rotation_world_from_palm) > 0.999
    assert np.linalg.det(left.rotation_world_from_palm) > 0.999
    assert np.allclose(np.abs(right.landmarks), np.abs(left.landmarks), atol=1e-8)
    assert np.isclose(left.scale_m, right.scale_m * 2)


def test_dex3_joint_order_limits_slew_and_independent_watchdogs() -> None:
    retargeter = Dex3Retargeter(ROOT / "config" / "g1_23dof_dex3.json", solver=lambda p, side, previous: (np.full(7, 99.0), {"residual": 0.0, "iterations": 1}))
    points = hand_shape()
    left = retargeter.update("left", points, 1_000_000_000, 1.0)
    right = retargeter.update("right", points, 1_000_000_000, 1.0)
    assert left["joint_order"] == ["thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1"]
    assert left["safety"]["saturated"] and right["safety"]["saturated"]
    left_lost = retargeter.update("left", None, 1_700_000_000, 0.0)
    right_valid = retargeter.update("right", points, 1_700_000_000, 1.0)
    assert left_lost["safety"]["watchdog"] == "FADE_TO_NEUTRAL"
    assert right_valid["safety"]["watchdog"] == "TRACKING"
    target = compose_targets(np.zeros(23), np.zeros(7), np.zeros(7))
    assert len(target["q_target"]) == 37 and target["physical_robot_output_enabled"] is False
    joiner = HandBodyTargetJoiner(maximum_spread_ms=40)
    assert joiner.update_body(1_000_000_000, [0.0] * 23) is None
    joined = joiner.update_hands(1_020_000_000, [0.0] * 7, [0.0] * 7)
    assert joined is not None and len(joined["q_target"]) == 37
    assert joined["physical_robot_output_enabled"] is False
    control = dex3_control_contract(123, left, right)
    assert validate_dex3_control(control) == (True, None)
    assert len(json.dumps(control, separators=(",", ":"))) < 1000
    control["physical_robot_output_enabled"] = True
    assert validate_dex3_control(control)[1] == "PHYSICAL_OUTPUT_MUST_BE_FALSE"


def test_dex3_control_contract_rejects_wrong_joint_order() -> None:
    side = {
        "safe_q_rad": [0.0] * 7,
        "safety": {"confidence": 0.8, "watchdog": "TRACKING"},
    }
    packet = dex3_control_contract(10, side, side)
    packet["joint_order_per_hand"] = list(
        reversed(packet["joint_order_per_hand"])
    )
    assert validate_dex3_control(packet) == (False, "JOINT_ORDER")


def test_json_contract_rejects_nonfinite_and_udp_has_safe_size() -> None:
    packet = valid_packet()
    valid, reason = validate_hand_packet(packet)
    assert valid and reason is None
    assert len(encode_packet(packet)) < 60_000
    packet["hands"][0]["landmarks_px"][3][0] = float("nan")
    valid, reason = validate_hand_packet(packet)
    assert not valid and reason == "LANDMARKS_2D"
    legacy_body = {"schema": "zed_body38_live/v1", "keypoints_3d_m": [[0, 0, 0]] * 38}
    assert validate_hand_packet(legacy_body)[0] is False  # separate channel, no schema collision


def test_mediapipe_is_optional_when_feature_is_disabled() -> None:
    # Importing every core module is dependency-safe. MediaPipe is imported only
    # in MediaPipeHandBackend.__init__, after --hand-tracking is requested.
    assert importlib.util.find_spec("hand_tracking.pipeline") is not None


def test_enabled_mediapipe_requires_explicit_model_file(tmp_path: Path) -> None:
    missing = tmp_path / "hand_landmarker.task"
    try:
        MediaPipeHandBackend(missing)
    except FileNotFoundError as exc:
        assert "otomatik indirilmez" in str(exc)
    else:
        raise AssertionError("missing model must fail before MediaPipe starts")


class MockBackend:
    def detect(self, _rgb: np.ndarray, _timestamp_ns: int):
        shape = hand_shape()[:, :2]
        minimum = shape.min(axis=0); maximum = shape.max(axis=0)
        normalized = (shape - minimum) / np.maximum(maximum - minimum, 1e-6)
        return [{
            "landmarks_normalized": normalized.tolist(),
            "relative_landmarks_m": hand_shape().tolist(),
            "landmark_confidence": [0.95] * 21,
            "handedness_label": "Left", "handedness_score": 0.9,
            "detection_confidence": 0.9, "presence_confidence": 0.9,
            "tracking_confidence": 0.9,
        }], 2.0


def test_mock_backend_end_to_end_two_hand_source_packet_and_fusion() -> None:
    intrinsics = CameraIntrinsics(700, 700, 640, 360, 1280, 720)
    pipeline = HandSourcePipeline(
        1, "host", intrinsics, lambda side: MockBackend(),
        inference_fps=30, asynchronous=False,
    )
    image = np.zeros((720, 1280, 3), dtype=np.uint8)
    depth = np.full((720, 1280), 3.0, dtype=np.float32)
    points2d = np.zeros((38, 2)); points3d = np.zeros((38, 3))
    indices = {"LEFT_WRIST": 16, "RIGHT_WRIST": 17, "LEFT_ELBOW": 14, "RIGHT_ELBOW": 15}
    points2d[16], points2d[14] = [500, 350], [450, 350]
    points2d[17], points2d[15] = [780, 350], [830, 350]
    points3d[16] = points3d[17] = [3, 0, 0]
    packet = pipeline.process(image, depth, points2d, points3d, body_id=9, unique_operator_id="u9", operator_state="LOCKED", capture_timestamp_ns=1_000_000_000, sequence=1, indices=indices)
    assert packet is not None and len(packet["hands"]) == 2
    assert all(hand["rejection_reason"] is None for hand in packet["hands"])
    packet2 = json.loads(json.dumps(packet)); packet2["camera_serial"] = 2
    fused = fuse_hand_packets([packet, packet2], {1: CameraPose(np.eye(3), np.zeros(3)), 2: CameraPose(np.eye(3), np.zeros(3))}, target_timestamp_ns=1_000_000_000)
    assert all(hand["valid"] for hand in fused["hands"])
    assert all(all(q["camera_count"] == 2 for q in hand["landmark_quality"]) for hand in fused["hands"])


def test_async_roi_worker_never_blocks_body_capture() -> None:
    class SlowBackend(MockBackend):
        def detect(self, rgb: np.ndarray, timestamp_ns: int):
            time.sleep(0.06)
            return super().detect(rgb, timestamp_ns)

    intrinsics = CameraIntrinsics(700, 700, 640, 360, 1280, 720)
    pipeline = HandSourcePipeline(1, "host", intrinsics, lambda side: SlowBackend(), inference_fps=30)
    image = np.zeros((720, 1280, 3), dtype=np.uint8)
    depth = np.full((720, 1280), 3.0, dtype=np.float32)
    points2d = np.zeros((38, 2)); points3d = np.zeros((38, 3))
    indices = {"LEFT_WRIST": 16, "RIGHT_WRIST": 17, "LEFT_ELBOW": 14, "RIGHT_ELBOW": 15}
    points2d[16], points2d[14] = [500, 350], [450, 350]
    points2d[17], points2d[15] = [780, 350], [830, 350]
    points3d[16] = points3d[17] = [3, 0, 0]
    started = time.perf_counter()
    first = pipeline.process(
        image, depth, points2d, points3d, body_id=9,
        unique_operator_id="u9", operator_state="LOCKED",
        capture_timestamp_ns=1_000_000_000, sequence=1, indices=indices,
    )
    assert first is None and time.perf_counter() - started < 0.04
    time.sleep(0.14)
    completed = pipeline.process(
        image, depth, points2d, points3d, body_id=9,
        unique_operator_id="u9", operator_state="LOCKED",
        capture_timestamp_ns=1_200_000_000, sequence=2, indices=indices,
    )
    pipeline.close()
    assert completed is not None
    assert completed["transport_metrics"]["latest_only_async"] is True


def test_async_roi_worker_survives_detector_error() -> None:
    class FailingBackend(MockBackend):
        def detect(self, _rgb: np.ndarray, _timestamp_ns: int):
            raise RuntimeError("simulated detector failure")

    intrinsics = CameraIntrinsics(700, 700, 640, 360, 1280, 720)
    pipeline = HandSourcePipeline(
        1, "host", intrinsics, lambda side: FailingBackend(), inference_fps=30
    )
    image = np.zeros((720, 1280, 3), dtype=np.uint8)
    depth = np.full((720, 1280), 3.0, dtype=np.float32)
    points2d = np.zeros((38, 2)); points3d = np.zeros((38, 3))
    indices = {
        "LEFT_WRIST": 16, "RIGHT_WRIST": 17,
        "LEFT_ELBOW": 14, "RIGHT_ELBOW": 15,
    }
    points2d[16], points2d[14] = [500, 350], [450, 350]
    points2d[17], points2d[15] = [780, 350], [830, 350]
    points3d[16] = points3d[17] = [3, 0, 0]
    pipeline.process(
        image, depth, points2d, points3d, body_id=9,
        unique_operator_id="u9", operator_state="LOCKED",
        capture_timestamp_ns=1_000_000_000, sequence=1, indices=indices,
    )
    deadline = time.monotonic() + 1.0
    result = None
    while result is None and time.monotonic() < deadline:
        time.sleep(0.01)
        result = pipeline.process(
            image, depth, points2d, points3d, body_id=9,
            unique_operator_id="u9", operator_state="LOCKED",
            capture_timestamp_ns=1_200_000_000, sequence=2, indices=indices,
        )
    assert result is not None
    assert result["transport_metrics"]["worker_error_type"] == "RuntimeError"
    assert all(
        hand["rejection_reason"] == "DETECTOR_RUNTIME_ERROR"
        for hand in result["hands"]
    )
    assert pipeline._worker is not None and pipeline._worker.is_alive()
    pipeline.close()


def test_benchmark_reads_fused_jsonl_metrics(tmp_path: Path) -> None:
    packet = valid_packet()
    packet["hands"][0]["inference_ms"] = 8.0
    fused = fuse_hand_packets(
        [packet, {**valid_packet(2), "capture_timestamp_ns": 1_005_000_000}],
        {
            1: CameraPose(np.eye(3), np.zeros(3)),
            2: CameraPose(np.eye(3), np.zeros(3)),
        },
        target_timestamp_ns=1_005_000_000,
    )
    fused["per_camera"] = [packet]
    record = {
        "timestamp_ns": 1_005_000_000,
        "latency_trace_ns": {
            "t0_capture_ns": 1_000_000_000,
            "t2_windows_udp_send_ns": 1_025_000_000,
        },
        "hand_tracking": fused,
        "dex3_targets": {
            "left": {
                "solver": {"time_ms": 1.2, "residual": 0.03},
                "safety": {"saturated": False},
            }
        },
    }
    path = tmp_path / "hand.jsonl"
    path.write_text(json.dumps(record) + "\n", encoding="utf-8")
    report = summarize(path)
    assert report["end_to_end_latency_ms"]["p95"] == 25.0
    assert report["per_camera_inference"]["1"]["p95_ms"] == 8.0
    assert report["landmark_camera_count_mean"] == 2.0
    assert report["hand_valid_coverage_percent"] == 50.0
    assert report["solver_ms"]["p95"] == 1.2

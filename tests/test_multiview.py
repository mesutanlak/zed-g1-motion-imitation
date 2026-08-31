from __future__ import annotations

import numpy as np

from motion_pipeline.multiview import (
    arm_evidence,
    body_quality,
    choose_output,
    closest_timestamp_samples,
    keypoint_agreement,
    robust_rigid_alignment,
    torso_overlap_2d,
)


NAMES = [
    "PELVIS", "SPINE_1", "SPINE_2", "SPINE_3", "NECK", "NOSE",
    "LEFT_EYE", "RIGHT_EYE", "LEFT_EAR", "RIGHT_EAR",
    "LEFT_CLAVICLE", "RIGHT_CLAVICLE", "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST",
    "LEFT_HIP", "RIGHT_HIP", "LEFT_KNEE", "RIGHT_KNEE",
    "LEFT_ANKLE", "RIGHT_ANKLE", "LEFT_BIG_TOE", "RIGHT_BIG_TOE",
    "LEFT_SMALL_TOE", "RIGHT_SMALL_TOE", "LEFT_HEEL", "RIGHT_HEEL",
    "LEFT_HAND_THUMB_4", "RIGHT_HAND_THUMB_4", "LEFT_HAND_INDEX_1",
    "RIGHT_HAND_INDEX_1", "LEFT_HAND_MIDDLE_4", "RIGHT_HAND_MIDDLE_4",
    "LEFT_HAND_PINKY_1", "RIGHT_HAND_PINKY_1",
]
IDX = {name: i for i, name in enumerate(NAMES)}


def skeleton() -> np.ndarray:
    points = np.zeros((38, 3), dtype=np.float64)
    points[:, 0] = 3.0
    points[IDX["PELVIS"]] = [3.0, 0.0, 1.0]
    points[IDX["SPINE_2"]] = [3.0, 0.0, 1.3]
    points[IDX["SPINE_3"]] = [3.0, 0.0, 1.5]
    points[IDX["NECK"]] = [3.0, 0.0, 1.65]
    points[IDX["LEFT_SHOULDER"]] = [3.0, 0.2, 1.55]
    points[IDX["RIGHT_SHOULDER"]] = [3.0, -0.2, 1.55]
    points[IDX["LEFT_ELBOW"]] = [2.9, 0.35, 1.35]
    points[IDX["RIGHT_ELBOW"]] = [2.9, -0.35, 1.35]
    points[IDX["LEFT_WRIST"]] = [2.8, 0.25, 1.20]
    points[IDX["RIGHT_WRIST"]] = [2.8, -0.25, 1.20]
    return points


def test_closest_timestamp_samples_avoids_one_frame_latest_pair() -> None:
    histories = {
        1: [(1_000_000_000, "cam1-old"), (1_033_000_000, "cam1-new")],
        2: [(1_001_000_000, "cam2-old"), (1_067_000_000, "cam2-new")],
    }
    selected = closest_timestamp_samples(
        histories, preferred_delta_ns=10_000_000,
    )
    assert selected[1] == (1_000_000_000, "cam1-old")
    assert selected[2] == (1_001_000_000, "cam2-old")


def test_closest_timestamp_samples_prefers_newest_good_pair() -> None:
    histories = {
        1: [(1_000_000_000, "old"), (2_000_000_000, "new")],
        2: [(1_001_000_000, "old"), (2_004_000_000, "new")],
    }
    selected = closest_timestamp_samples(
        histories, preferred_delta_ns=10_000_000,
    )
    assert selected[1][1] == "new"
    assert selected[2][1] == "new"


def test_closest_timestamp_samples_never_replays_old_reference() -> None:
    histories = {
        1: [(1_000_000_000, "old"), (1_033_000_000, "new")],
        2: [(1_001_000_000, "old"), (1_067_000_000, "new")],
    }
    selected = closest_timestamp_samples(
        histories,
        preferred_delta_ns=10_000_000,
        after_timestamp_ns=1_001_000_000,
    )
    assert max(sample[0] for sample in selected.values()) > 1_001_000_000


def test_fusion_preferred_and_fallback_rejects_identity_jump() -> None:
    points = skeleton()
    confidence = np.full(38, 90.0)
    fused = body_quality(serial_number=0, body_id=7, points=points,
                         confidence=confidence, body_confidence=95,
                         index=IDX, threshold=40)
    single = body_quality(serial_number=33773329, body_id=4, points=points,
                          confidence=confidence, body_confidence=95,
                          index=IDX, threshold=40)
    assert choose_output(fused=fused, singles=[single]).mode == "fusion"

    poor_confidence = confidence.copy()
    poor_confidence[[IDX[name] for name in (
        "LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST",
        "RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST",
    )]] = 0
    poor = body_quality(serial_number=0, body_id=7, points=points,
                        confidence=poor_confidence, body_confidence=95,
                        index=IDX, threshold=40)
    assert choose_output(fused=poor, singles=[single]).mode == "single_fallback"
    assert choose_output(
        fused=None, singles=[single], previous_anchor_m=[0.0, 0.0, 0.0]
    ).mode == "hold"


def test_cross_view_agreement_and_clear_oblique_arm_evidence() -> None:
    points = skeleton()
    confidence = np.full(38, 90.0)
    shifted = points.copy()
    shifted[:, 1] += 0.01
    result = keypoint_agreement(points, shifted, confidence, confidence, 40)
    assert result["common_keypoints"] == 38
    assert abs(float(result["mpjpe_m"]) - 0.01) < 1e-9
    evidence = arm_evidence([
        {"serial_number": 1, "arm_confidence": {"left": [90, 90, 90]},
         "arm_overlap": {"left": True}},
        {"serial_number": 2, "arm_confidence": {"left": [85, 88, 82]},
         "arm_overlap": {"left": False}},
    ], side="left", threshold=40)
    assert evidence["supporting_views"] == 2
    assert evidence["reliable_clear_views"] == 1


def test_torso_overlap_is_computed_per_camera_pixels() -> None:
    pixels = np.full((38, 2), -1.0)
    pixels[IDX["LEFT_SHOULDER"]] = [400, 200]
    pixels[IDX["RIGHT_SHOULDER"]] = [200, 200]
    pixels[IDX["RIGHT_HIP"]] = [230, 500]
    pixels[IDX["LEFT_HIP"]] = [370, 500]
    pixels[IDX["LEFT_ELBOW"]] = [300, 300]
    pixels[IDX["LEFT_WRIST"]] = [300, 400]
    assert torso_overlap_2d(pixels, IDX, "LEFT")
    pixels[IDX["LEFT_ELBOW"]] = [500, 300]
    pixels[IDX["LEFT_WRIST"]] = [550, 350]
    assert not torso_overlap_2d(pixels, IDX, "LEFT")


def test_robust_rigid_alignment_rejects_one_bad_joint() -> None:
    rng = np.random.default_rng(4)
    source = rng.normal(size=(40, 3))
    angle = np.deg2rad(17.0)
    rotation = np.array([
        [np.cos(angle), -np.sin(angle), 0.0],
        [np.sin(angle), np.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ])
    translation = np.array([0.7, -0.2, 0.8])
    target = source @ rotation.T + translation
    target[3] += 2.0
    result = robust_rigid_alignment(source, target)
    assert result is not None
    fitted_rotation, fitted_translation, metrics = result
    assert np.allclose(fitted_rotation, rotation, atol=1e-8)
    assert np.allclose(fitted_translation, translation, atol=1e-8)
    assert 12 <= metrics["paired_keypoints"] < 40

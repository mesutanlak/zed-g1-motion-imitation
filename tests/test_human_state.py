from pathlib import Path
from types import SimpleNamespace

import numpy as np

from motion_pipeline.human_state import (
    ConfidenceAwareHumanStateEstimator,
    load_human_state_config,
    quaternion_continuous,
)
from motion_pipeline.reference_motion import (
    BoundedReferenceMotion,
    LowLatencyReferenceMotion,
)


NAMES = (
    "PELVIS", "SPINE_1", "SPINE_2", "SPINE_3", "NECK",
    "LEFT_CLAVICLE", "RIGHT_CLAVICLE", "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST",
    "LEFT_HIP", "RIGHT_HIP", "LEFT_KNEE", "RIGHT_KNEE",
    "LEFT_ANKLE", "RIGHT_ANKLE",
)
EDGES = (
    ("PELVIS", "SPINE_1"), ("SPINE_1", "SPINE_2"),
    ("SPINE_2", "SPINE_3"), ("SPINE_3", "NECK"),
    ("SPINE_3", "LEFT_CLAVICLE"), ("LEFT_CLAVICLE", "LEFT_SHOULDER"),
    ("LEFT_SHOULDER", "LEFT_ELBOW"), ("LEFT_ELBOW", "LEFT_WRIST"),
    ("SPINE_3", "RIGHT_CLAVICLE"), ("RIGHT_CLAVICLE", "RIGHT_SHOULDER"),
    ("RIGHT_SHOULDER", "RIGHT_ELBOW"), ("RIGHT_ELBOW", "RIGHT_WRIST"),
    ("PELVIS", "LEFT_HIP"), ("PELVIS", "RIGHT_HIP"),
    ("LEFT_HIP", "LEFT_KNEE"), ("LEFT_KNEE", "LEFT_ANKLE"),
    ("RIGHT_HIP", "RIGHT_KNEE"), ("RIGHT_KNEE", "RIGHT_ANKLE"),
)
IDX = {name: i for i, name in enumerate(NAMES)}


def config():
    return load_human_state_config(
        Path(__file__).parents[1] / "config" / "dual_teleoperation.json"
    )


def pose() -> np.ndarray:
    p = np.zeros((len(NAMES), 3), dtype=float)
    p[IDX["SPINE_1"]] = [0, 0, 0.2]
    p[IDX["SPINE_2"]] = [0, 0, 0.4]
    p[IDX["SPINE_3"]] = [0, 0, 0.6]
    p[IDX["NECK"]] = [0, 0, 0.8]
    p[IDX["LEFT_CLAVICLE"]] = [0, 0.12, 0.65]
    p[IDX["RIGHT_CLAVICLE"]] = [0, -0.12, 0.65]
    p[IDX["LEFT_SHOULDER"]] = [0, 0.25, 0.62]
    p[IDX["RIGHT_SHOULDER"]] = [0, -0.25, 0.62]
    p[IDX["LEFT_ELBOW"]] = [0.05, 0.48, 0.48]
    p[IDX["RIGHT_ELBOW"]] = [0.05, -0.48, 0.48]
    p[IDX["LEFT_WRIST"]] = [0.12, 0.67, 0.30]
    p[IDX["RIGHT_WRIST"]] = [0.12, -0.67, 0.30]
    p[IDX["LEFT_HIP"]] = [0, 0.12, -0.05]
    p[IDX["RIGHT_HIP"]] = [0, -0.12, -0.05]
    p[IDX["LEFT_KNEE"]] = [0, 0.12, -0.45]
    p[IDX["RIGHT_KNEE"]] = [0, -0.12, -0.45]
    p[IDX["LEFT_ANKLE"]] = [0, 0.12, -0.85]
    p[IDX["RIGHT_ANKLE"]] = [0, -0.12, -0.85]
    return p


def body(points: np.ndarray, confidence: float = 95.0):
    return SimpleNamespace(
        keypoint=points,
        keypoint_confidence=np.full(len(NAMES), confidence),
        confidence=confidence,
    )


def test_joint_outlier_does_not_drop_whole_skeleton() -> None:
    estimator = ConfidenceAwareHumanStateEstimator(NAMES, EDGES, config())
    good = pose()
    bad = good.copy()
    bad[IDX["RIGHT_WRIST"]] += [0.9, 0.4, 0.1]
    bad_conf = np.full(len(NAMES), 95.0)
    bad_conf[IDX["RIGHT_WRIST"]] = 45.0
    second = SimpleNamespace(keypoint=bad, keypoint_confidence=bad_conf, confidence=90.0)
    result = estimator.update(
        {1: body(good), 2: second}, {1: 1_000_000_000, 2: 1_001_000_000},
        camera_positions={1: [-2, 1, 1], 2: [-2, -1, 1]},
    )
    wrist = IDX["RIGHT_WRIST"]
    assert np.linalg.norm(result.points[wrist] - good[wrist]) < 0.08
    assert result.joint_source[wrist] == "CAM_1"
    assert result.joint_source[IDX["PELVIS"]] == "FUSED"
    assert "JOINT_OUTLIER_REJECTED" in result.failure_codes


def test_short_occlusion_predicts_then_expires() -> None:
    estimator = ConfidenceAwareHumanStateEstimator(NAMES, EDGES, config())
    first = pose()
    estimator.update({1: body(first)}, {1: 1_000_000_000})
    second = first.copy()
    second[:, 0] += 0.03
    estimator.update({1: body(second)}, {1: 1_033_000_000})
    missing_conf = np.zeros(len(NAMES))
    missing = SimpleNamespace(keypoint=np.full_like(second, np.nan), keypoint_confidence=missing_conf, confidence=0.0)
    predicted = estimator.update({1: missing}, {1: 1_083_000_000})
    assert predicted.joint_source[IDX["LEFT_WRIST"]] == "PREDICTED"
    assert np.isfinite(predicted.points[IDX["LEFT_WRIST"]]).all()


def test_elbow_plane_flip_requires_hysteresis() -> None:
    estimator = ConfidenceAwareHumanStateEstimator(NAMES, EDGES, config())
    first = pose()
    estimator.update({1: body(first)}, {1: 1_000_000_000})
    flipped = first.copy()
    shoulder = flipped[IDX["LEFT_SHOULDER"]].copy()
    wrist = flipped[IDX["LEFT_WRIST"]].copy()
    flipped[IDX["LEFT_ELBOW"]] = shoulder + wrist - first[IDX["LEFT_ELBOW"]]
    result = estimator.update({1: body(flipped)}, {1: 1_033_000_000})
    assert result.elbow_state["left"] == "ELBOW_RECOVERING"
    assert "ELBOW_PLANE_FLIP_REJECTED" in result.failure_codes


def test_quaternion_continuity_uses_nearest_hemisphere() -> None:
    previous = np.array([0.0, 0.0, 0.0, 1.0])
    current = np.array([0.0, 0.0, 0.0, -1.0])
    fixed = quaternion_continuous(previous, current)
    assert np.dot(previous, fixed) > 0.999


def test_reference_motion_obeys_velocity_acceleration_and_jerk_limits() -> None:
    motion = BoundedReferenceMotion(
        np.zeros(4), response_hz=1.2,
        max_velocity=1.5, max_acceleration=5.0, max_jerk=45.0,
    )
    samples = [motion.update(np.ones(4) * 2.0, 0.005) for _ in range(400)]
    assert max(np.max(np.abs(item.velocity)) for item in samples) <= 1.5 + 1.0e-9
    assert max(np.max(np.abs(item.acceleration)) for item in samples) <= 5.0 + 1.0e-9
    assert max(np.max(np.abs(item.jerk)) for item in samples) <= 45.0 + 1.0e-8
    assert np.all(np.abs(samples[-1].position - 2.0) < 0.08)


def test_low_latency_reference_tracks_human_rate_without_phase_lag() -> None:
    motion = LowLatencyReferenceMotion(
        np.zeros(2), response_hz=4.0,
        max_velocity=5.0, max_acceleration=30.0,
    )
    samples = []
    target_history = []
    for frame in range(1200):
        time_s = frame * 0.005
        target = np.full(2, np.sin(np.pi * time_s))
        target_velocity = np.full(2, np.pi * np.cos(np.pi * time_s))
        samples.append(motion.update(target, target_velocity, 0.005))
        target_history.append(target.copy())
    positions = np.asarray([item.position for item in samples])
    velocities = np.asarray([item.velocity for item in samples])
    accelerations = np.asarray([item.acceleration for item in samples])
    targets = np.asarray(target_history)
    # Ignore the first second while the zero-velocity initial state catches
    # the non-zero sinusoid feed-forward velocity.
    assert np.sqrt(np.mean((positions[200:] - targets[200:]) ** 2)) < 0.04
    assert np.max(np.abs(velocities)) <= 5.0 + 1.0e-9
    assert np.max(np.abs(accelerations)) <= 30.0 + 1.0e-8


def test_low_latency_reference_step_brakes_at_target() -> None:
    motion = LowLatencyReferenceMotion(
        np.zeros(1), response_hz=4.0,
        max_velocity=5.0, max_acceleration=30.0,
    )
    samples = [
        motion.update(np.asarray([2.0]), np.zeros(1), 0.005)
        for _ in range(200)
    ]
    positions = np.asarray([item.position[0] for item in samples])
    assert positions[120] > 1.95
    assert np.max(positions) < 2.03
    assert abs(positions[-1] - 2.0) < 1.0e-3


def test_unitree_live_envelope_caps_velocity_acceleration_and_jerk() -> None:
    motion = LowLatencyReferenceMotion(
        np.zeros(3), response_hz=5.0,
        max_velocity=0.5, max_acceleration=2.0, max_jerk=20.0,
    )
    samples = [
        motion.update(np.full(3, 2.0), np.full(3, 8.0), 1.0 / 24.0)
        for _ in range(120)
    ]
    velocities = np.asarray([item.velocity for item in samples])
    accelerations = np.asarray([item.acceleration for item in samples])
    jerks = np.asarray([item.jerk for item in samples])
    assert np.max(np.abs(velocities)) <= 0.5 + 1.0e-9
    assert np.max(np.abs(accelerations)) <= 2.0 + 1.0e-9
    assert np.max(np.abs(jerks)) <= 20.0 + 1.0e-8

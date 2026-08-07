from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from motion_pipeline.arm_chain import ArmChainOptimizer, _two_bone_candidates
from motion_pipeline.calibration import CalibrationManager, to_pelvis_local
from motion_pipeline.metrics import PerceptionMetrics
from motion_pipeline.operator_selector import OperatorSelector, OperatorState
from motion_pipeline.safety import G1FeasibilityFilter, G1_23_LIMITS_RAD
from motion_pipeline.collision_geometry import (
    CollisionDistanceReport,
    segment_segment_distance,
    upper_body_capsule_report,
)
from motion_pipeline.mirror_rescue import (
    KinematicEvaluation,
    MirrorContinuationRescue,
)


NAMES = [
    "PELVIS", "SPINE_3", "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST",
    "LEFT_HIP", "RIGHT_HIP", "LEFT_KNEE", "RIGHT_KNEE",
    "LEFT_ANKLE", "RIGHT_ANKLE", "LEFT_HEEL", "RIGHT_HEEL",
]
INDEX = {name: index for index, name in enumerate(NAMES)}


def neutral_points(offset=(3.0, 0.0, 0.0)) -> np.ndarray:
    base = {
        "PELVIS": [0, 0, 1.0], "SPINE_3": [0, 0, 1.45],
        "LEFT_SHOULDER": [0, 0.22, 1.48], "RIGHT_SHOULDER": [0, -0.22, 1.48],
        "LEFT_ELBOW": [0, 0.48, 1.35], "RIGHT_ELBOW": [0, -0.48, 1.35],
        "LEFT_WRIST": [0, 0.72, 1.18], "RIGHT_WRIST": [0, -0.72, 1.18],
        "LEFT_HIP": [0, 0.11, 0.98], "RIGHT_HIP": [0, -0.11, 0.98],
        "LEFT_KNEE": [0, 0.11, 0.53], "RIGHT_KNEE": [0, -0.11, 0.53],
        "LEFT_ANKLE": [0, 0.11, 0.08], "RIGHT_ANKLE": [0, -0.11, 0.08],
        "LEFT_HEEL": [-0.08, 0.11, 0.03], "RIGHT_HEEL": [-0.08, -0.11, 0.03],
    }
    return np.asarray([np.asarray(base[name]) + np.asarray(offset) for name in NAMES])


@dataclass
class Body:
    id: int
    unique_object_id: str
    distance: float


def test_operator_never_handover() -> None:
    selector = OperatorSelector(acquire_frames=2)
    first = Body(4, "operator-a", 2.0)
    other = Body(9, "bystander", 1.0)
    valid = lambda body: True
    distance = lambda body: body.distance
    assert selector.update([first], valid=valid, distance=distance).state == OperatorState.ACQUIRING
    assert selector.update([first], valid=valid, distance=distance).state == OperatorState.LOCKED
    lost = selector.update([other], valid=valid, distance=distance)
    assert lost.state == OperatorState.LOST
    assert lost.body is None and lost.body_id == 4


def test_operator_locks_when_pyzed_unique_id_is_unstable() -> None:
    selector = OperatorSelector(acquire_frames=3)
    valid = lambda body: True
    distance = lambda body: body.distance
    states = []
    # The recorded failure had a stable body.id but a newly materialized
    # unique_object_id value on every frame.  This must not keep acquisition at
    # 1 forever.
    for frame in range(3):
        body = Body(0, f"temporary-wrapper-{frame}", 3.2)
        states.append(selector.update([body], valid=valid, distance=distance))
    assert [item.acquisition_frames for item in states] == [1, 2, 3]
    assert states[-1].state == OperatorState.LOCKED
    assert states[-1].body_id == 0
    assert states[-1].unique_object_id == ""


def test_pelvis_frame_translation_invariance_and_calibration() -> None:
    first = neutral_points((3.0, 0.1, 0.0))
    second = neutral_points((4.2, -0.5, 0.2))
    local_first, _, _ = to_pelvis_local(first, INDEX)
    local_second, _, _ = to_pelvis_local(second, INDEX)
    assert np.allclose(local_first, local_second, atol=1e-8)
    manager = CalibrationManager(duration_s=3.0, min_valid_ratio=0.5)
    confidence = np.full(len(NAMES), 100.0)
    result = None
    for frame in range(50):
        result = manager.update(
            operator_id=4, timestamp_s=frame / 15.0, points=first,
            confidence=confidence, index=INDEX, confidence_threshold=40.0,
        )
    assert result is not None and result.state == "READY"
    assert abs(float(result.profile["shoulder_width_m"]) - 0.44) < 1e-6


def test_arm_chain_recovers_torso_overlap() -> None:
    optimizer = ArmChainOptimizer(hold_s=0.3)
    points = neutral_points()
    pixel = np.asarray([
        [320, 350], [320, 240], [250, 220], [390, 220],
        [180, 270], [460, 270], [120, 320], [520, 320],
        [270, 400], [370, 400], [270, 520], [370, 520],
        [270, 650], [370, 650], [260, 660], [380, 660],
    ], dtype=float)
    confidence = np.full(len(NAMES), 100.0)
    optimizer.update(
        timestamp_s=0.0, points_3d=points, points_2d=pixel,
        confidence=confidence, index=INDEX, threshold=40.0, calibration=None,
    )
    pixel[INDEX["LEFT_ELBOW"]] = [300, 300]
    pixel[INDEX["LEFT_WRIST"]] = [330, 330]
    result = optimizer.update(
        timestamp_s=0.05, points_3d=points, points_2d=pixel,
        confidence=confidence, index=INDEX, threshold=40.0, calibration=None,
    )
    assert result.overlap["left"] and result.recovered["left"]


def test_arm_chain_recovers_depth_overlap_missed_by_2d_and_holds_branch() -> None:
    """A frontal wrist can miss the image polygon but still overlap in Y-Z."""
    optimizer = ArmChainOptimizer(hold_s=0.3)
    points = neutral_points()
    pixel = np.asarray([
        [320, 350], [320, 240], [250, 220], [390, 220],
        [180, 270], [460, 270], [120, 320], [520, 320],
        [270, 400], [370, 400], [270, 520], [370, 520],
        [270, 650], [370, 650], [260, 660], [380, 660],
    ], dtype=float)
    confidence = np.full(len(NAMES), 100.0)
    optimizer.update(
        timestamp_s=0.0, points_3d=points, points_2d=pixel,
        confidence=confidence, index=INDEX, threshold=40.0, calibration=None,
    )

    frontal = points.copy()
    frontal[INDEX["LEFT_ELBOW"]] = [3.15, 0.12, 1.36]
    frontal[INDEX["LEFT_WRIST"]] = [3.30, 0.02, 1.20]
    missed_pixel = pixel.copy()
    # Deliberately outside the image-space torso polygon: this reproduces the
    # detector disagreement present in the failing recording.
    missed_pixel[INDEX["LEFT_ELBOW"]] = [180, 270]
    missed_pixel[INDEX["LEFT_WRIST"]] = [255, 300]
    recovered = optimizer.update(
        timestamp_s=0.05, points_3d=frontal, points_2d=missed_pixel,
        confidence=confidence, index=INDEX, threshold=40.0, calibration=None,
    )
    assert recovered.overlap["left"]
    assert recovered.recovered["left"]
    shoulder_i = INDEX["LEFT_SHOULDER"]
    elbow_i = INDEX["LEFT_ELBOW"]
    wrist_i = INDEX["LEFT_WRIST"]
    initial_reach = np.linalg.norm(points[wrist_i] - points[shoulder_i])
    recovered_reach = np.linalg.norm(
        recovered.points[wrist_i] - recovered.points[shoulder_i]
    )
    assert abs(float(recovered_reach - initial_reach)) < 1e-6

    # Leaving the overlap for one frame must not flip the elbow branch.
    held = optimizer.update(
        timestamp_s=0.10, points_3d=points, points_2d=pixel,
        confidence=confidence, index=INDEX, threshold=40.0, calibration=None,
    )
    assert held.overlap["left"]
    assert held.recovered["left"]

    released = optimizer.update(
        timestamp_s=0.40, points_3d=points, points_2d=pixel,
        confidence=confidence, index=INDEX, threshold=40.0, calibration=None,
    )
    assert not released.overlap["left"]
    assert np.allclose(released.points, points)


def test_arm_chain_recovers_front_depth_foreshortening() -> None:
    """A depth-directed upper arm must reuse the stable elbow pole."""
    optimizer = ArmChainOptimizer(hold_s=0.3)
    points = neutral_points()
    pixel = np.asarray([
        [320, 350], [320, 240], [250, 220], [390, 220],
        [180, 270], [460, 270], [120, 320], [520, 320],
        [270, 400], [370, 400], [270, 520], [370, 520],
        [270, 650], [370, 650], [260, 660], [380, 660],
    ], dtype=float)
    confidence = np.full(len(NAMES), 100.0)
    optimizer.update(
        timestamp_s=0.0, points_3d=points, points_2d=pixel,
        confidence=confidence, index=INDEX, threshold=40.0, calibration=None,
    )
    frontal = points.copy()
    frontal[INDEX["LEFT_ELBOW"]] = [3.27, 0.21, 1.44]
    frontal[INDEX["LEFT_WRIST"]] = [3.45, 0.18, 1.34]
    frontal_pixel = pixel.copy()
    frontal_pixel[INDEX["LEFT_ELBOW"]] = [255, 224]
    frontal_pixel[INDEX["LEFT_WRIST"]] = [267, 252]
    result = optimizer.update(
        timestamp_s=0.05, points_3d=frontal, points_2d=frontal_pixel,
        confidence=confidence, index=INDEX, threshold=40.0, calibration=None,
    )
    assert result.recovered["left"]
    assert "left_front_depth_ambiguity" in result.reasons
    assert np.isfinite(result.candidate_confidence["left"])


def test_two_bone_recovery_never_asserts_on_corrupt_front_frame() -> None:
    """A bad frontal frame must yield finite candidates, never kill ZED."""
    positive, negative, height = _two_bone_candidates(
        np.array([0.0, 0.0, 0.0]),
        np.array([np.nan, 0.0, 0.0]),
        np.array([np.nan, np.nan, np.nan]),
        0.29,
        0.25,
    )
    assert np.isfinite(positive[0]).all()
    assert np.isfinite(positive[1]).all()
    assert np.isfinite(negative[0]).all()
    assert np.isfinite(negative[1]).all()
    assert np.isfinite(height)


def test_swap_metric_uses_anatomical_pelvis_frame() -> None:
    metrics = PerceptionMetrics()
    confidence = np.full(len(NAMES), 100.0)
    pelvis_local = neutral_points(offset=(0.0, 0.0, -1.0))
    normal = metrics.update(
        timestamp_s=0.0,
        points=pelvis_local,
        confidence=confidence,
        index=INDEX,
        threshold=40.0,
        overlap={"left": False, "right": False},
        calibration_profile=None,
    )
    assert not normal["left_right_swap_risk"]
    swapped = pelvis_local.copy()
    left = INDEX["LEFT_SHOULDER"]
    right = INDEX["RIGHT_SHOULDER"]
    swapped[[left, right]] = swapped[[right, left]]
    detected = metrics.update(
        timestamp_s=1.0 / 30.0,
        points=swapped,
        confidence=confidence,
        index=INDEX,
        threshold=40.0,
        overlap={"left": False, "right": False},
        calibration_profile=None,
    )
    assert detected["left_right_swap_risk"]
    assert detected["left_right_swap_count"] == 1


def test_feasibility_projects_limits_and_slew() -> None:
    filter_ = G1FeasibilityFilter()
    raw = G1_23_LIMITS_RAD[:, 1] + 1.0
    result = filter_.update(raw, 1.0 / 30.0)
    assert result.joint_limit_saturation_count == 23
    assert result.level.name in {"YELLOW", "ORANGE"}
    assert np.all(result.safe_q <= G1_23_LIMITS_RAD[:, 1])
    assert np.all(result.safe_q >= G1_23_LIMITS_RAD[:, 0])


def test_elbow_command_never_crosses_anatomical_extension_limit() -> None:
    filter_ = G1FeasibilityFilter()
    command = np.zeros(23)
    elbow_indices = np.asarray([16, 21])
    observed = []
    for _ in range(12):
        command[elbow_indices] = 2.0
        observed.append(filter_.update(command, 1.0 / 15.0).safe_q[elbow_indices])
    for _ in range(30):
        command[elbow_indices] = 0.041
        observed.append(filter_.update(command, 1.0 / 15.0).safe_q[elbow_indices])
    values = np.asarray(observed)
    assert np.all(values >= 0.04 - 1.0e-9)
    assert np.allclose(values[-1], 0.041, atol=0.01)


def test_persistent_orange_returns_smoothly_to_neutral() -> None:
    filter_ = G1FeasibilityFilter(
        orange_return_after_s=0.20,
        orange_return_tau_s=0.30,
    )
    command = np.zeros(23)
    command[14] = 1.0
    command[16] = 1.2
    for _ in range(40):
        reliable = filter_.update(command, 1.0 / 60.0)
    held_value = float(reliable.safe_q[14])
    assert held_value > 0.5

    values = []
    reasons = ()
    for _ in range(90):
        result = filter_.update(
            command,
            1.0 / 60.0,
            self_collision=True,
        )
        values.append(float(result.safe_q[14]))
        reasons = result.reasons

    values = np.asarray(values)
    assert np.all(np.isfinite(values))
    assert np.max(np.abs(np.diff(values))) < 0.10
    assert values[-1] < held_value * 0.25
    assert "orange_safe_return" in reasons


def test_capsule_segment_distance_is_continuous() -> None:
    crossing = segment_segment_distance(
        [0, 0, 0], [1, 0, 0], [0.5, -1, 0], [0.5, 1, 0]
    )
    separated = segment_segment_distance(
        [0, 0, 0], [1, 0, 0], [0, 0.2, 0], [1, 0.2, 0]
    )
    assert crossing == 0.0
    assert abs(separated - 0.2) < 1.0e-9


def test_forearm_across_torso_is_measured_as_soft_contact() -> None:
    positions = {
        "pelvis": [0.0, 0.0, 0.0],
        "torso_link": [0.0, 0.0, 0.4],
        "left_shoulder_pitch_link": [0.0, 0.25, 0.4],
        "left_elbow_link": [0.0, 0.30, 0.30],
        "left_wrist_roll_rubber_hand": [0.0, 0.0, 0.25],
        "right_shoulder_pitch_link": [0.0, -0.25, 0.4],
        "right_elbow_link": [0.0, -0.50, 0.30],
        "right_wrist_roll_rubber_hand": [0.0, -0.70, 0.20],
    }
    report = upper_body_capsule_report(positions)
    assert "left_fore__torso" in report.soft_risk_pairs
    assert "left_hand__torso" in report.soft_risk_pairs
    assert report.minimum_margin_m > 0.0
    assert not report.risk_pairs


def test_collision_governor_degrades_only_unsafe_arm() -> None:
    filter_ = G1FeasibilityFilter(yellow_blend=0.82)
    command = np.zeros(23)
    command[13:18] = [0.4, 0.2, 0.1, 0.8, 0.0]
    command[18:23] = [0.4, -0.2, -0.1, 0.8, 0.0]
    result = filter_.update(
        command,
        1.0 / 60.0,
        self_collision=True,
        collision_margin_m=-0.03,
        arm_collision_margins={"left": -0.03, "right": 0.10},
    )
    assert result.arm_blend["left"] == 0.0
    assert result.arm_blend["right"] == 1.0
    assert result.level.name == "ORANGE"
    assert np.linalg.norm(result.safe_q[18:23]) > np.linalg.norm(result.safe_q[13:18])


def test_occlusion_quality_governor_degrades_only_affected_arm() -> None:
    filter_ = G1FeasibilityFilter(yellow_blend=0.55)
    command = np.zeros(23)
    # Keep the step below the velocity/acceleration governor so this test
    # isolates the arm-local quality blend.
    command[13:18] = [0.02, 0.0, 0.0, 0.04, 0.0]
    command[18:23] = [0.02, 0.0, 0.0, 0.04, 0.0]
    result = filter_.update(
        command,
        0.10,
        arm_reasons={"left": ["left_wrist_occluded"]},
        arm_quality_blend={"left": 0.72, "right": 1.0},
    )
    assert result.level.name == "YELLOW"
    assert result.blend == 1.0
    assert result.arm_blend == {"left": 0.72, "right": 1.0}
    assert np.linalg.norm(result.safe_q[18:23]) > np.linalg.norm(result.safe_q[13:18])


def test_continuation_rescue_reduces_arm_task_error() -> None:
    rescue = MirrorContinuationRescue(
        residual_trigger_m=0.01,
        collision_trigger_m=-1.0,
        workers=2,
    )
    limits = np.tile(np.array([-2.0, 2.0]), (23, 1))
    limits[[16, 21], 0] = 0.0

    def evaluate(q: np.ndarray) -> KinematicEvaluation:
        positions = {
            "left_elbow_link": [q[13], 0.0, 0.0],
            "left_wrist_roll_rubber_hand": [q[13] + q[16], 0.0, 0.0],
            "right_elbow_link": [0.0, 0.0, 0.0],
            "right_wrist_roll_rubber_hand": [0.0, 0.0, 0.0],
        }
        collision = CollisionDistanceReport(
            1.0, {}, {"left": 1.0, "right": 1.0}, ()
        )
        return KinematicEvaluation(positions, collision)

    nominal = np.zeros(23)
    nominal[13] = 0.10
    result = rescue.update(
        nominal_q=nominal,
        previous_safe_q=np.zeros(23),
        targets={
            "left_elbow": [0.40, 0.0, 0.0],
            "left_wrist": [0.80, 0.0, 0.0],
            "right_elbow": [0.0, 0.0, 0.0],
            "right_wrist": [0.0, 0.0, 0.0],
        },
        evaluate=evaluate,
        joint_limits=limits,
        residual_m=0.20,
    )
    assert result.triggered and result.applied
    assert result.selected_task_error_m < result.nominal_task_error_m
    assert np.allclose(result.q[:13], nominal[:13])

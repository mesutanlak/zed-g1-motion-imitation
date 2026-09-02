from __future__ import annotations

from collections import deque
import json

import numpy as np

from zed_g1_skeleton import body_in_distance_gate, detect_torn_frame

from zed_four_camera_test.calibrate_distributed_body38 import (
    pelvis_trajectory_metrics,
    write_world_poses_jsonl,
)
from zed_four_camera_test.extrinsic_geometry import rebase_camera_transforms
from zed_four_camera_test.validate_distributed_extrinsics import validate_document
from zed_four_camera_test.distributed_body38_fusion import (
    BODY38_NAMES,
    IDX,
    ClockOffsetEstimator,
    Extrinsic,
    InputEndpoint,
    Sample,
    compact_live_packet,
    covariance_weight,
    load_extrinsics,
    make_output_packet,
    prepare_aligned_views,
    synchronized_samples,
    temporal_compensate_samples,
    valid_body38_payload,
    world_forward_in_workspace,
    workspace_membership_with_hysteresis,
)


def body38_points() -> np.ndarray:
    """Return a non-degenerate, upright BODY_38 pose in Z-up coordinates."""
    value = np.zeros((38, 3), dtype=np.float64)
    value[:] = [2.0, 0.0, 1.0]
    value[IDX["PELVIS"]] = [2.0, 0.0, 1.0]
    value[IDX["SPINE_1"]] = [2.0, 0.0, 1.15]
    value[IDX["SPINE_2"]] = [2.0, 0.0, 1.30]
    value[IDX["SPINE_3"]] = [2.0, 0.0, 1.48]
    value[IDX["NECK"]] = [2.0, 0.0, 1.64]
    value[IDX["LEFT_SHOULDER"]] = [2.0, 0.22, 1.52]
    value[IDX["RIGHT_SHOULDER"]] = [2.0, -0.22, 1.52]
    value[IDX["LEFT_ELBOW"]] = [2.0, 0.44, 1.34]
    value[IDX["RIGHT_ELBOW"]] = [2.0, -0.44, 1.34]
    value[IDX["LEFT_WRIST"]] = [2.0, 0.58, 1.17]
    value[IDX["RIGHT_WRIST"]] = [2.0, -0.58, 1.17]
    value[IDX["LEFT_HIP"]] = [2.0, 0.13, 0.98]
    value[IDX["RIGHT_HIP"]] = [2.0, -0.13, 0.98]
    value[IDX["LEFT_KNEE"]] = [2.0, 0.13, 0.55]
    value[IDX["RIGHT_KNEE"]] = [2.0, -0.13, 0.55]
    value[IDX["LEFT_ANKLE"]] = [2.0, 0.13, 0.10]
    value[IDX["RIGHT_ANKLE"]] = [2.0, -0.13, 0.10]
    return value


def test_frame_integrity_ignores_moderate_scene_edges_but_detects_band_tearing() -> None:
    normal = np.zeros((720, 1280, 3), dtype=np.uint8)
    for row in range(0, 720, 60):
        normal[row:row + 30] = 28
    torn, _boundaries, peak = detect_torn_frame(normal)
    assert torn is False
    assert peak < 38.0

    corrupted = np.zeros((720, 1280, 3), dtype=np.uint8)
    for row in range(0, 720, 80):
        corrupted[row:row + 40] = 255
    torn, boundaries, peak = detect_torn_frame(corrupted)
    assert torn is True
    assert boundaries >= 5
    assert peak >= 38.0


def packet(serial: int, sequence: int, xyz: np.ndarray | None = None) -> dict[str, object]:
    points = body38_points() if xyz is None else xyz
    return {
        "schema": "zed_body38_live/v1",
        "source_serial": serial,
        "sequence": sequence,
        "timestamp_ns": 123_000_000 + sequence,
        "body_confidence": 95.0,
        "global_root_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "keypoint_names": list(BODY38_NAMES),
        "keypoints_3d_m": points.tolist(),
        "keypoint_confidence": np.full(38, 90.0).tolist(),
        "local_orientation_per_joint_xyzw": (
            np.tile(np.array([0.0, 0.0, 0.0, 1.0]), (38, 1)).tolist()
        ),
        "operator_selection": {
            "state": "LOCKED",
            "locked_body_id": sequence,
            "acquisition_frames": 10,
            "missing_frames": 0,
        },
        "calibration": {
            "profile": {"neutral_pelvis_rotation_matrix": np.eye(3).tolist()}
        },
    }


def test_operator_distance_gate_rejects_background_and_keeps_locked_boundary() -> None:
    assert body_in_distance_gate([3.0, 0.0, 0.0], 2.0, 4.0)
    assert not body_in_distance_gate([6.0, 0.0, 0.0], 2.0, 4.0)
    assert not body_in_distance_gate([1.9, 0.0, 0.0], 2.0, 4.0)
    assert body_in_distance_gate([1.9, 0.0, 0.0], 2.0, 4.0, retain_locked=True)
    assert not body_in_distance_gate([-3.0, 0.0, 0.0], 2.0, 4.0)


def test_calibrated_world_workspace_rejects_six_meter_background_person() -> None:
    foreground = Sample(1, packet(1, 1, body38_points()), 1_000_000_000, 1)
    background_points = body38_points().copy()
    background_points[:, 0] += 4.3
    background = Sample(1, packet(1, 2, background_points), 1_000_000_001, 2)
    extrinsic = Extrinsic(np.eye(3), np.zeros(3))

    assert world_forward_in_workspace(foreground, extrinsic, 2.0, 4.0) == (True, 2.0)
    assert world_forward_in_workspace(background, extrinsic, 2.0, 4.0) == (False, 6.3)


def test_workspace_hysteresis_prevents_boundary_flapping() -> None:
    assert workspace_membership_with_hysteresis(
        2.0, was_inside=False, minimum_x_m=2.0, maximum_x_m=4.0,
        hysteresis_m=0.15,
    )
    assert workspace_membership_with_hysteresis(
        1.90, was_inside=True, minimum_x_m=2.0, maximum_x_m=4.0,
        hysteresis_m=0.15,
    )
    assert not workspace_membership_with_hysteresis(
        1.80, was_inside=True, minimum_x_m=2.0, maximum_x_m=4.0,
        hysteresis_m=0.15,
    )
    assert not workspace_membership_with_hysteresis(
        6.0, was_inside=True, minimum_x_m=2.0, maximum_x_m=4.0,
        hysteresis_m=0.15,
    )


def test_body38_payload_validation_checks_raw_shape_and_joint_names() -> None:
    document = packet(1, 1)
    assert valid_body38_payload(document)
    document["keypoints_3d_m"] = [[1.0, 2.0, 3.0]]
    assert not valid_body38_payload(document)
    document = packet(1, 1)
    document["keypoint_names"] = list(BODY38_NAMES[:-1])
    assert not valid_body38_payload(document)


def test_clock_offset_estimator_turns_negative_cross_host_latency_positive() -> None:
    estimator = ClockOffsetEstimator(window=30)
    document = packet(1, 1)
    document["timestamp_ns"] = 1_085_000_000
    document["latency_trace_ns"] = {
        "t0_capture_ns": 1_085_000_000,
        "t2_windows_udp_send_ns": 1_100_000_000,
    }
    corrected_ns, offset_ns, metrics = estimator.update(
        "Laptop", document, 1_003_000_000
    )
    assert offset_ns == -97_000_000
    assert corrected_ns == 988_000_000
    assert metrics["raw_clock_mixed_capture_to_receive_ms"] == -82.0
    assert metrics["corrected_capture_to_receive_ms"] == 15.0
    assert metrics["network_queue_ms"] == 0.0


def test_zero_sdk_covariance_is_missing_data_not_perfect_certainty() -> None:
    document = packet(1, 1)
    document["keypoints_covariance"] = np.zeros((38, 6)).tolist()
    assert covariance_weight(document, IDX["LEFT_WRIST"]) == 0.25


def test_temporal_compensation_predicts_only_to_bounded_common_capture_time() -> None:
    previous_xyz = body38_points()
    current_xyz = previous_xyz.copy()
    current_xyz[IDX["LEFT_WRIST"], 1] += 0.10
    target_xyz = body38_points()
    previous_packet = packet(1, 1, previous_xyz)
    current_packet = packet(1, 2, current_xyz)
    target_packet = packet(2, 2, target_xyz)
    previous_packet["body_id"] = current_packet["body_id"] = 7
    target_packet["body_id"] = 7
    previous = Sample(1, previous_packet, 1_000_000_000, 1, 1_000_000_000, 0, "Laptop")
    current = Sample(1, current_packet, 1_050_000_000, 2, 1_050_000_000, 0, "Laptop")
    target = Sample(2, target_packet, 1_100_000_000, 2, 1_100_000_000, 0, "MainPc")
    compensated = temporal_compensate_samples(
        [current, target],
        {1: deque([previous, current], maxlen=32), 2: deque([target], maxlen=32)},
        minimum_confidence=45.0,
        maximum_prediction_ms=70.0,
    )
    wrist = np.asarray(compensated[0].packet["keypoints_3d_m"])[IDX["LEFT_WRIST"]]
    assert np.isclose(wrist[1], previous_xyz[IDX["LEFT_WRIST"], 1] + 0.20)
    assert compensated[0].packet["_fusion_temporal_prediction_ms"] == 50.0


def test_synchronized_samples_prefers_four_fresh_sources_and_ignores_stale() -> None:
    now = 2_000_000_000
    histories = {
        1: deque([Sample(1, packet(1, 1), now - 12_000_000, 1)], maxlen=16),
        2: deque([Sample(2, packet(2, 2), now - 9_000_000, 2)], maxlen=16),
        3: deque([Sample(3, packet(3, 3), now - 8_000_000, 3)], maxlen=16),
        4: deque([Sample(4, packet(4, 4), now - 400_000_000, 4),
                  Sample(4, packet(4, 5), now - 10_000_000, 5)], maxlen=16),
    }
    selected = synchronized_samples(
        histories,
        now_ns=now,
        source_timeout_ns=250_000_000,
        maximum_spread_ns=25_000_000,
        minimum_sources=4,
    )
    assert selected is not None
    samples, spread_ms = selected
    assert {sample.serial for sample in samples} == {1, 2, 3, 4}
    assert next(sample for sample in samples if sample.serial == 4).sequence == 5
    assert spread_ms == 4.0


def test_synchronized_samples_advances_past_an_older_tighter_bundle() -> None:
    now = 2_000_000_000
    histories = {
        serial: deque([
            Sample(serial, packet(serial, serial), now - 200_000_000, serial),
            Sample(serial, packet(serial, serial + 10), now - (20 - serial * 2) * 1_000_000, serial + 10),
        ], maxlen=16)
        for serial in (1, 2, 3, 4)
    }

    selected = synchronized_samples(
        histories,
        now_ns=now,
        source_timeout_ns=750_000_000,
        maximum_spread_ns=110_000_000,
        minimum_sources=4,
    )

    assert selected is not None
    samples, spread_ms = selected
    assert {sample.sequence for sample in samples} == {11, 12, 13, 14}
    assert spread_ms == 6.0


def test_synchronized_samples_prefers_fresh_three_views_over_stale_four() -> None:
    now = 1_000_000_000
    histories = {
        serial: deque([
            Sample(serial, packet(serial, 1), 800_000_000, 1),
            Sample(serial, packet(serial, 2), 995_000_000 + serial, 2),
        ], maxlen=16)
        for serial in (1, 2, 3)
    }
    histories[4] = deque([
        Sample(4, packet(4, 1), 800_000_000, 1),
    ], maxlen=16)

    selected = synchronized_samples(
        histories,
        now_ns=now,
        source_timeout_ns=250_000_000,
        maximum_spread_ns=80_000_000,
        minimum_sources=3,
    )

    assert selected is not None
    samples, spread_ms = selected
    assert {sample.serial for sample in samples} == {1, 2, 3}
    assert {sample.sequence for sample in samples} == {2}
    assert spread_ms < 0.001


def test_synchronized_samples_prefers_coherent_four_over_slightly_newer_three() -> None:
    now = 1_000_000_000
    capture_times = {
        # Stable two-host camera phase can be slightly above half a 15 Hz
        # frame; this must still form a true four-view current-cycle bundle.
        1: 955_000_000,
        2: 985_000_000,
        3: 988_000_000,
        4: 990_000_000,
    }
    histories = {
        serial: deque([
            Sample(serial, packet(serial, 2), capture_ns, 2, capture_ns)
        ], maxlen=16)
        for serial, capture_ns in capture_times.items()
    }

    selected = synchronized_samples(
        histories,
        now_ns=now,
        source_timeout_ns=250_000_000,
        maximum_spread_ns=80_000_000,
        minimum_sources=3,
    )

    assert selected is not None
    samples, spread_ms = selected
    assert {sample.serial for sample in samples} == {1, 2, 3, 4}
    assert spread_ms == 35.0


def test_synchronized_samples_waits_for_new_common_cycle_instead_of_smearing() -> None:
    old_ns = 934_000_000
    histories = {
        serial: deque(
            [Sample(serial, packet(serial, 1), old_ns, 1, old_ns)],
            maxlen=16,
        )
        for serial in (1, 2, 3, 4)
    }
    histories[1].append(
        Sample(1, packet(1, 2), 1_000_000_000, 2, 1_000_000_000)
    )

    selected = synchronized_samples(
        histories,
        now_ns=1_005_000_000,
        source_timeout_ns=250_000_000,
        maximum_spread_ns=80_000_000,
        minimum_sources=3,
    )

    assert selected is not None
    samples, spread_ms = selected
    assert {sample.sequence for sample in samples} == {1}
    assert spread_ms == 0.0


def test_synchronized_samples_emits_three_new_views_without_old_fourth() -> None:
    old_ns = 934_000_000
    histories = {
        serial: deque(
            [Sample(serial, packet(serial, 1), old_ns, 1, old_ns)],
            maxlen=16,
        )
        for serial in (1, 2, 3, 4)
    }
    for serial, jitter_ns in ((1, 0), (2, 2_000_000), (3, 4_000_000)):
        capture_ns = 1_000_000_000 + jitter_ns
        histories[serial].append(
            Sample(serial, packet(serial, 2), capture_ns, 2, capture_ns)
        )

    selected = synchronized_samples(
        histories,
        now_ns=1_010_000_000,
        source_timeout_ns=250_000_000,
        maximum_spread_ns=80_000_000,
        minimum_sources=3,
    )

    assert selected is not None
    samples, spread_ms = selected
    assert {sample.serial for sample in samples} == {1, 2, 3}
    assert {sample.sequence for sample in samples} == {2}
    assert spread_ms == 4.0


def test_synchronized_samples_keeps_up_with_four_15_hz_sources() -> None:
    histories = {serial: deque(maxlen=16) for serial in (1, 2, 3, 4)}
    selected_markers = []
    period_ns = 66_666_667

    for frame in range(15):
        frame_start_ns = 1_000_000_000 + frame * period_ns
        for serial in histories:
            # The first bundle is perfectly aligned; subsequent bundles have
            # realistic receive jitter and must still replace it immediately.
            jitter_ns = 0 if frame == 0 else (serial - 1) * 2_000_000
            received_ns = frame_start_ns + jitter_ns
            histories[serial].append(Sample(
                serial,
                packet(serial, frame),
                received_ns,
                frame,
            ))

        selected = synchronized_samples(
            histories,
            now_ns=frame_start_ns + 10_000_000,
            source_timeout_ns=750_000_000,
            maximum_spread_ns=110_000_000,
            minimum_sources=4,
        )
        assert selected is not None
        samples, _ = selected
        selected_markers.append(tuple((sample.serial, sample.sequence) for sample in samples))

    assert len(set(selected_markers)) == 15
    assert {sequence for _, sequence in selected_markers[-1]} == {14}


def test_load_extrinsics_preserves_explicit_reference_serial(tmp_path) -> None:
    path = tmp_path / "extrinsics.json"
    path.write_text(json.dumps({
        "schema": "zed_body38_distributed_extrinsics/v1",
        "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
        "units": "meter",
        "reference_world_serial": 33773329,
        "cameras": {
            str(serial): {
                "rotation_camera_to_world": np.eye(3).tolist(),
                "translation_camera_to_world_m": [0.0, 0.0, 0.0],
            }
            for serial in (31571870, 33773329, 34760587, 39504762)
        },
    }), encoding="utf-8")
    endpoints = [InputEndpoint(serial, 16000 + index * 2) for index, serial in enumerate(
        (39504762, 31571870, 33773329, 34760587)
    )]
    loaded = load_extrinsics(path, endpoints)
    assert loaded is not None
    assert loaded.reference_serial == 33773329
    assert set(loaded.cameras) == {39504762, 31571870, 33773329, 34760587}


def test_zed360_transforms_are_rebased_to_selected_reference_camera() -> None:
    world_from_reference = np.eye(4)
    world_from_reference[:3, 3] = [4.0, -2.0, 0.5]
    reference_from_second = np.eye(4)
    reference_from_second[:3, 3] = [1.2, 0.4, -0.1]
    world_from_second = world_from_reference @ reference_from_second

    rebased = rebase_camera_transforms(
        {10: world_from_reference, 20: world_from_second},
        reference_serial=10,
    )

    assert np.allclose(rebased[10], np.eye(4))
    assert np.allclose(rebased[20], reference_from_second)


def test_runtime_validator_accepts_zed360_and_rejects_false_string_gate() -> None:
    serials = {1, 2, 3, 4}
    document = {
        "schema": "zed_body38_distributed_extrinsics/v1",
        "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
        "units": "meter",
        "reference_world_serial": 1,
        "method": "zed_sdk_read_fusion_configuration_file_from_zed360",
        "cameras": {
            str(serial): {
                "rotation_camera_to_world": np.eye(3).tolist(),
                "translation_camera_to_world_m": (
                    [0.0, 0.0, 0.0] if serial == 1 else [float(serial), 0.0, 0.0]
                ),
                "fit": {
                    "source": "zed360_sdk_configuration",
                    "quality_gate_passed": True,
                },
            }
            for serial in serials
        },
    }
    validate_document(document, expected_serials=serials, reference_serial=1)
    document["cameras"]["2"]["fit"]["quality_gate_passed"] = "false"
    try:
        validate_document(document, expected_serials=serials, reference_serial=1)
    except ValueError as exc:
        assert "kalite" in str(exc)
    else:
        raise AssertionError("JSON string 'false' must not pass the Boolean gate")


def test_runtime_validator_recomputes_body38_quality_thresholds() -> None:
    serials = {1, 2, 3, 4}
    document = {
        "schema": "zed_body38_distributed_extrinsics/v1",
        "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
        "units": "meter",
        "reference_world_serial": 1,
        "method": "trimmed_kabsch_from_synchronized_body38_core_joints",
        "cameras": {},
    }
    for serial in serials:
        document["cameras"][str(serial)] = {
            "rotation_camera_to_world": np.eye(3).tolist(),
            "translation_camera_to_world_m": (
                [0.0, 0.0, 0.0] if serial == 1 else [float(serial), 0.0, 0.0]
            ),
            "fit": (
                {"reference": True}
                if serial == 1
                else {
                    "capture_samples": 100,
                    "candidate_keypoints": 1000,
                    "paired_keypoints": 600,
                    "inlier_ratio": 0.6,
                    "rms_m": 0.08,
                    "pelvis_p95_m": 0.12,
                    "pelvis_reference_motion_m": 0.25,
                    "pelvis_motion_scale_ratio": 0.90,
                    "pelvis_trajectory_correlation": 0.95,
                    "quality_gate_passed": True,
                }
            ),
        }
    validate_document(document, expected_serials=serials, reference_serial=1)
    document["cameras"]["3"]["fit"]["capture_samples"] = 20
    try:
        validate_document(document, expected_serials=serials, reference_serial=1)
    except ValueError as exc:
        assert "yetersiz" in str(exc)
    else:
        raise AssertionError("Relaxed producer thresholds must not bypass runtime policy")


def test_pelvis_trajectory_gate_rejects_stationary_wrong_person() -> None:
    reference = np.column_stack((
        np.linspace(0.0, 1.0, 120),
        0.1 * np.sin(np.linspace(0.0, 4.0, 120)),
        np.zeros(120),
    ))
    wrong_stationary = np.column_stack((
        0.01 * np.sin(np.linspace(0.0, 10.0, 120)),
        0.01 * np.cos(np.linspace(0.0, 10.0, 120)),
        np.zeros(120),
    ))
    pairs = np.stack((wrong_stationary, reference), axis=1)
    metrics = pelvis_trajectory_metrics(pairs, np.eye(3), np.zeros(3))

    assert metrics["pelvis_reference_motion_m"] >= 0.10
    assert metrics["pelvis_motion_scale_ratio"] < 0.60
    assert metrics["pelvis_trajectory_correlation"] < 0.75


def test_fused_packet_rebuilds_world_and_g1_reference_features() -> None:
    base = body38_points()
    # Camera 2 observes coordinates translated by -1 m; its extrinsic maps
    # them back to the same fusion world as camera 1.
    second_camera_points = base - np.array([1.0, 0.0, 0.0])
    views = [
        (Sample(33773329, packet(33773329, 10, base), 1_000_000_000, 10),
         Extrinsic(np.eye(3), np.zeros(3))),
        (Sample(39504762, packet(39504762, 11, second_camera_points), 1_003_000_000, 11),
         Extrinsic(np.eye(3), np.array([1.0, 0.0, 0.0]))),
    ]
    fused = make_output_packet(
        views,
        minimum_confidence=45.0,
        maximum_spread_m=0.30,
        output_sequence=7,
        reference_serial=33773329,
        arrival_spread_ms=3.0,
    )
    assert fused["schema"] == "zed_body38_live/v1"
    assert fused["source_serial"] == 0
    assert fused["fusion"]["reference_world_serial"] == 33773329
    assert set(fused["fusion"]["contributing_serials"]) == {33773329, 39504762}
    assert fused["fusion"]["per_joint_contributions"][IDX["PELVIS"]] == 2
    assert np.allclose(fused["root_position_m"], base[IDX["PELVIS"]])
    assert fused["pelvis_frame"]["reference_frame"] == "FUSION_WORLD"
    assert fused["reference_ready"]["whole_body"] is True


def test_four_view_packet_contains_analysis_data_but_control_copy_is_compact() -> None:
    base = body38_points()
    views = [
        (
            Sample(serial, packet(serial, index), 1_000_000_000 + index * 1_000_000, index),
            Extrinsic(np.eye(3), np.zeros(3)),
        )
        for index, serial in enumerate((39504762, 31571870, 33773329, 34760587), start=1)
    ]
    fused = make_output_packet(
        views,
        minimum_confidence=45.0,
        maximum_spread_m=0.30,
        output_sequence=8,
        reference_serial=33773329,
        arrival_spread_ms=3.0,
        source_metrics={serial: {"body_fps": 15.0} for serial in (39504762, 31571870, 33773329, 34760587)},
        connected_serials=[39504762, 31571870, 33773329, 34760587],
        effective_output_hz=14.8,
        record_queue_depth=2,
    )

    assert fused["multi_camera"]["mode"] == "FOUR_FUSED"
    assert fused["multi_camera"]["contributing_views"] == 4
    assert len(fused["multi_camera"]["per_camera"]) == 4
    assert all(len(view["keypoints_3d_fusion_m"]) == 38 for view in fused["multi_camera"]["per_camera"])
    assert fused["multi_camera"]["cross_view_agreement"]["mpjpe_m"] == 0.0
    assert fused["transport_metrics"]["effective_output_hz"] == 14.8

    compact = compact_live_packet(fused)
    assert "camera_pose_fusion_from_local" not in compact["multi_camera"]
    assert all("keypoints_3d_fusion_m" not in view for view in compact["multi_camera"]["per_camera"])
    assert compact["keypoints_3d_m"] == fused["keypoints_3d_m"]


def test_fused_calibration_is_ready_when_one_contributing_view_has_ready_profile() -> None:
    views = []
    for serial, state in ((1, "FAILED"), (2, "READY"), (3, "COLLECTING")):
        document = packet(serial, serial)
        document["calibration"].update({
            "state": state,
            "progress": 1.0 if state != "COLLECTING" else 0.5,
            "sample_count": 120 if state == "READY" else 20,
        })
        views.append((
            Sample(serial, document, 1_000_000_000 + serial, serial),
            Extrinsic(np.eye(3), np.zeros(3)),
        ))

    fused = make_output_packet(
        views,
        minimum_confidence=45.0,
        maximum_spread_m=0.30,
        output_sequence=1,
        reference_serial=1,
        arrival_spread_ms=2.0,
    )

    assert fused["calibration"]["state"] == "READY"
    assert fused["calibration"]["profile_source_serial"] == 2
    assert fused["calibration"]["ready_profile_source_serials"] == [2]
    assert fused["calibration"]["source_states"] == {
        "1": "FAILED", "2": "READY", "3": "COLLECTING",
    }


def test_fused_calibration_waits_when_no_contributing_view_has_ready_profile() -> None:
    views = []
    for serial, state, progress in (
        (1, "FAILED", 1.0), (2, "COLLECTING", 0.4), (3, "COLLECTING", 0.7)
    ):
        document = packet(serial, serial)
        document["calibration"].update({
            "state": state,
            "progress": progress,
            "sample_count": 10,
        })
        views.append((
            Sample(serial, document, 1_000_000_000 + serial, serial),
            Extrinsic(np.eye(3), np.zeros(3)),
        ))

    fused = make_output_packet(
        views,
        minimum_confidence=45.0,
        maximum_spread_m=0.30,
        output_sequence=1,
        reference_serial=1,
        arrival_spread_ms=2.0,
    )

    assert fused["calibration"]["state"] == "ACQUIRING"
    assert fused["calibration"]["ready_profile_source_serials"] == []


def test_arm_fusion_prefers_two_clear_views_over_two_torso_occluded_views() -> None:
    base = body38_points()
    bad = base.copy()
    bad[IDX["LEFT_ELBOW"]] += np.array([0.0, 0.20, 0.0])
    bad[IDX["LEFT_WRIST"]] += np.array([0.0, 0.25, 0.0])
    views = []
    for serial, xyz, overlap in (
        (1, base, False),
        (2, base, False),
        (3, bad, True),
        (4, bad, True),
    ):
        document = packet(serial, serial, xyz)
        document["occlusion_analysis"] = {
            "arm_torso_overlap": {"left": overlap, "right": False},
            "arm_chain_recovered": {"left": overlap, "right": False},
        }
        if overlap:
            document["local_orientation_per_joint_xyzw"][IDX["LEFT_WRIST"]] = [
                1.0, 0.0, 0.0, 0.0
            ]
        views.append((
            Sample(serial, document, 1_000_000_000 + serial, serial),
            Extrinsic(np.eye(3), np.zeros(3)),
        ))

    fused = make_output_packet(
        views,
        minimum_confidence=45.0,
        maximum_spread_m=0.30,
        output_sequence=1,
        reference_serial=1,
        arrival_spread_ms=1.0,
    )

    assert np.allclose(
        np.asarray(fused["keypoints_3d_m"])[IDX["LEFT_WRIST"]],
        base[IDX["LEFT_WRIST"]],
    )
    assert fused["fusion"]["per_joint_contributing_serials"][IDX["LEFT_WRIST"]] == [1, 2]
    assert fused["multi_camera"]["arm_evidence"]["left"]["reliable_clear_views"] == 2
    assert fused["occlusion_analysis"]["arm_torso_overlap"]["left"] is False
    assert np.allclose(
        fused["local_orientation_per_joint_xyzw"][IDX["LEFT_WRIST"]],
        [0.0, 0.0, 0.0, 1.0],
    )
    assert fused["fusion"]["per_joint_orientation_serials"][IDX["LEFT_WRIST"]] in (1, 2)
    assert fused["operator_selection"]["state"] == "LOCKED"
    assert fused["timestamp_ns"] == fused["latency_trace_ns"]["t0_capture_ns"]
    assert fused["latency_trace_ns"]["t2_windows_udp_send_ns"] >= fused["timestamp_ns"]


def test_one_clear_arm_view_rejects_disagreeing_occluded_views() -> None:
    base = body38_points()
    views = []
    for serial, offset, overlap in (
        (1, 0.0, False),
        (2, 0.22, True),
        (3, -0.20, True),
        (4, 0.18, True),
    ):
        xyz = base.copy()
        xyz[IDX["LEFT_ELBOW"], 1] += offset
        xyz[IDX["LEFT_WRIST"], 1] += offset
        document = packet(serial, serial, xyz)
        document["occlusion_analysis"] = {
            "arm_torso_overlap": {"left": overlap, "right": False},
            "arm_chain_recovered": {"left": overlap, "right": False},
        }
        views.append((
            Sample(serial, document, 1_000_000_000 + serial, serial),
            Extrinsic(np.eye(3), np.zeros(3)),
        ))

    fused = make_output_packet(
        views,
        minimum_confidence=45.0,
        maximum_spread_m=0.30,
        output_sequence=1,
        reference_serial=1,
        arrival_spread_ms=1.0,
    )

    wrist_serials = fused["fusion"]["per_joint_contributing_serials"][
        IDX["LEFT_WRIST"]
    ]
    assert wrist_serials == [1]
    assert fused["fusion"]["arm_evidence"]["left"][
        "occlusion_rejected_serials"
    ] == [2, 3, 4]
    assert np.allclose(
        np.asarray(fused["keypoints_3d_m"])[IDX["LEFT_WRIST"]],
        base[IDX["LEFT_WRIST"]],
    )


def test_dynamic_pelvis_alignment_repairs_translation_biased_extrinsic() -> None:
    base = body38_points()
    views = [
        (Sample(1, packet(1, 1, base), 1_000_000_000, 1), Extrinsic(np.eye(3), np.zeros(3))),
        (Sample(2, packet(2, 2, base), 1_002_000_000, 2), Extrinsic(np.eye(3), np.array([0.60, -0.20, 0.10]))),
    ]
    prepared = prepare_aligned_views(views, minimum_confidence=45.0)
    assert all(item.accepted for item in prepared)
    assert np.allclose(prepared[0].aligned_points, prepared[1].aligned_points)
    fused = make_output_packet(
        views,
        minimum_confidence=45.0,
        maximum_spread_m=0.30,
        output_sequence=1,
        reference_serial=1,
        arrival_spread_ms=2.0,
    )
    assert fused["multi_camera"]["post_alignment_agreement"]["mpjpe_m"] < 1.0e-12
    relative = np.asarray(fused["root_relative_keypoints_m"])
    assert np.linalg.norm(relative[IDX["LEFT_WRIST"]] - relative[IDX["LEFT_ELBOW"]]) > 0.1


def test_cross_person_pose_outlier_is_excluded_from_fusion() -> None:
    base = body38_points()
    outlier = base.copy()
    for name in (
        "SPINE_3", "NECK", "LEFT_SHOULDER", "RIGHT_SHOULDER",
        "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST",
        "LEFT_HIP", "RIGHT_HIP", "LEFT_KNEE", "RIGHT_KNEE",
    ):
        outlier[IDX[name]] += np.array([0.0, 0.9, 0.4])
    views = [
        (Sample(1, packet(1, 1, base), 1_000_000_000, 1), Extrinsic(np.eye(3), np.zeros(3))),
        (Sample(2, packet(2, 2, base), 1_002_000_000, 2), Extrinsic(np.eye(3), np.zeros(3))),
        (Sample(3, packet(3, 3, outlier), 1_003_000_000, 3), Extrinsic(np.eye(3), np.zeros(3))),
    ]
    fused = make_output_packet(
        views,
        minimum_confidence=45.0,
        maximum_spread_m=0.30,
        output_sequence=1,
        reference_serial=1,
        arrival_spread_ms=3.0,
    )
    assert fused["multi_camera"]["contributing_views"] == 2
    assert fused["multi_camera"]["excluded_pose_serials"] == [3]
    assert fused["multi_camera"]["failure_codes"] == ["CROSS_PERSON_OR_POSE_OUTLIER"]
    assert np.allclose(fused["keypoints_3d_m"], base, equal_nan=True)


def test_extreme_pelvis_translation_cannot_be_pulled_into_operator_fusion() -> None:
    base = body38_points()
    views = [
        (Sample(1, packet(1, 1, base), 1_000_000_000, 1), Extrinsic(np.eye(3), np.zeros(3))),
        (Sample(2, packet(2, 2, base), 1_001_000_000, 2), Extrinsic(np.eye(3), np.array([0.05, 0.0, 0.0]))),
        (Sample(3, packet(3, 3, base), 1_002_000_000, 3), Extrinsic(np.eye(3), np.array([-0.05, 0.0, 0.0]))),
        (Sample(4, packet(4, 4, base), 1_003_000_000, 4), Extrinsic(np.eye(3), np.array([4.0, 0.0, 0.0]))),
    ]
    fused = make_output_packet(
        views,
        minimum_confidence=45.0,
        maximum_spread_m=0.30,
        output_sequence=1,
        reference_serial=1,
        arrival_spread_ms=3.0,
    )
    assert fused["multi_camera"]["contributing_views"] == 3
    assert fused["multi_camera"]["excluded_pose_serials"] == [4]
    assert np.linalg.norm(np.asarray(fused["root_position_m"]) - base[IDX["PELVIS"]]) < 0.06


def test_world_pose_jsonl_contains_metadata_and_one_line_per_camera(tmp_path) -> None:
    path = tmp_path / "world.jsonl"
    serials = (33773329, 39504762, 31571870, 34760587)
    output = {
        "created_unix_ns": 77,
        "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
        "units": "meter",
        "reference_world_serial": 33773329,
        "source_capture": "capture.jsonl",
        "cameras": {
            str(serial): {
                "rotation_camera_to_world": np.eye(3).tolist(),
                "translation_camera_to_world_m": [float(index), 0.0, 0.0],
                "fit": {"reference": serial == 33773329},
            }
            for index, serial in enumerate(serials)
        },
    }
    write_world_poses_jsonl(
        path,
        output=output,
        source_ports={serial: 16000 + 2 * index for index, serial in enumerate(serials)},
    )
    lines = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert len(lines) == 5
    assert lines[0]["schema"] == "zed_camera_world_poses_metadata/v1"
    camera_lines = lines[1:]
    assert {line["serial"] for line in camera_lines} == set(serials)
    reference = next(line for line in camera_lines if line["serial"] == 33773329)
    assert reference["is_reference"] is True
    assert reference["orientation_camera_to_world_xyzw"] == [0.0, 0.0, 0.0, 1.0]
    assert np.asarray(reference["transform_camera_to_world_4x4"]).shape == (4, 4)

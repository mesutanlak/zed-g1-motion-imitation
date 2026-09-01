from __future__ import annotations

from collections import deque
import json

import numpy as np

from zed_four_camera_test.calibrate_distributed_body38 import write_world_poses_jsonl
from zed_four_camera_test.distributed_body38_fusion import (
    BODY38_NAMES,
    IDX,
    Extrinsic,
    InputEndpoint,
    Sample,
    load_extrinsics,
    make_output_packet,
    synchronized_samples,
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
        "calibration": {
            "profile": {"neutral_pelvis_rotation_matrix": np.eye(3).tolist()}
        },
    }


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


def test_load_extrinsics_preserves_explicit_reference_serial(tmp_path) -> None:
    path = tmp_path / "extrinsics.json"
    path.write_text(json.dumps({
        "schema": "zed_body38_distributed_extrinsics/v1",
        "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
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

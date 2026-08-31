from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).resolve().parents[1] / "tools" / "prepare_reference_motion_clips.py"
SPEC = importlib.util.spec_from_file_location("prepare_reference_motion_clips", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_contiguous_runs_split_invalid_and_timestamp_gap():
    timestamps = np.asarray([0, 20, 40, 500, 520, 540], dtype=np.int64) * 1_000_000
    mask = np.asarray([True, True, False, True, True, True])
    runs = MODULE.contiguous_runs(mask, timestamps, max_gap_s=0.20)
    assert [run.tolist() for run in runs] == [[0, 1], [3, 4, 5]]


def test_feasibility_filter_respects_velocity_and_acceleration():
    dt = 0.02
    values = np.zeros((100, 2))
    values[10:, 0] = 4.0
    values[40:, 1] = -4.0
    filtered = MODULE.feasibility_filter(values, dt, max_velocity=5.0, max_acceleration=30.0)
    velocity = np.diff(filtered, axis=0) / dt
    acceleration = np.diff(velocity, axis=0) / dt
    assert np.max(np.abs(velocity)) <= 5.0 + 1.0e-9
    assert np.max(np.abs(acceleration)) <= 30.0 + 1.0e-8


def test_feasibility_filter_brakes_before_joint_limit_without_impulse():
    dt = 0.02
    values = np.zeros((200, 1))
    values[5:, 0] = 4.0
    lower = np.asarray([-2.0])
    upper = np.asarray([2.618])
    filtered = MODULE.feasibility_filter(
        values, dt, max_velocity=5.0, max_acceleration=30.0,
        lower_limits=lower, upper_limits=upper,
    )
    velocity = np.diff(filtered, axis=0) / dt
    acceleration = np.diff(velocity, axis=0) / dt
    assert filtered[:, 0].max() <= upper[0] + 1.0e-9
    assert np.max(np.abs(velocity)) <= 5.0 + 1.0e-9
    assert np.max(np.abs(acceleration)) <= 30.0 + 1.0e-8


def test_quaternion_continuity_removes_antipodal_flip():
    values = np.asarray([[1.0, 0.0, 0.0, 0.0], [-1.0, 0.0, 0.0, 0.0]])
    normalized = MODULE.normalize_quaternions_wxyz(values)
    assert np.dot(normalized[0], normalized[1]) > 0.999999


def test_cleanest_clip_is_held_out_and_others_train():
    manifest = {
        "clips": [
            {"id": "clip_000", "frames": 1000, "confidence_mean": 0.92, "confidence_min": 0.75},
            {"id": "clip_001", "frames": 1800, "confidence_mean": 0.94, "confidence_min": 0.84},
            {"id": "clip_002", "frames": 640, "confidence_mean": 0.95, "confidence_min": 0.90},
            {"id": "clip_003", "frames": 2100, "confidence_mean": 0.93, "confidence_min": 0.81},
        ]
    }
    result = MODULE.assign_dataset_splits(manifest)
    assert result["splits"]["validation"] == ["clip_002"]
    assert result["splits"]["train"] == ["clip_000", "clip_001", "clip_003"]
    assert result["split_strategy"]["concatenate_clips"] is False


def test_split_requires_independent_train_and_validation_clips():
    try:
        MODULE.assign_dataset_splits(
            {"clips": [{"id": "only", "frames": 10, "confidence_mean": 1.0, "confidence_min": 1.0}]}
        )
    except RuntimeError as exc:
        assert "at least two" in str(exc)
    else:
        raise AssertionError("single-clip dataset must not leak into validation")

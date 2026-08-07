from __future__ import annotations

import json
import tempfile
from pathlib import Path

from tools.benchmark_motion_capture import benchmark


NAMES = [
    "PELVIS", "SPINE_3", "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST",
    "LEFT_HIP", "RIGHT_HIP", "LEFT_KNEE", "RIGHT_KNEE",
    "LEFT_ANKLE", "RIGHT_ANKLE",
]
POINTS = [
    [2.0, 0.0, 1.0], [2.0, 0.0, 1.5], [2.0, 0.2, 1.5], [2.0, -0.2, 1.5],
    [2.0, 0.4, 1.3], [2.0, -0.4, 1.3], [2.0, 0.5, 1.1], [2.0, -0.5, 1.1],
    [2.0, 0.15, 1.0], [2.0, -0.15, 1.0], [2.0, 0.15, 0.55], [2.0, -0.15, 0.55],
    [2.0, 0.15, 0.1], [2.0, -0.15, 0.1],
]


def test_capture_benchmark_clean_stream() -> None:
    with tempfile.TemporaryDirectory() as temp:
        path = Path(temp) / "capture.jsonl"
        with path.open("w", encoding="utf-8") as stream:
            for frame in range(61):
                stream.write(json.dumps({
                    "schema": "zed_body38_live/v1",
                    "timestamp_ns": 1_000_000_000 + frame * 33_333_333,
                    "frame_index": frame,
                    "body_id": 7,
                    "body_confidence": 95.0,
                    "keypoint_names": NAMES,
                    "keypoints_3d_filtered_m": POINTS,
                    "keypoint_confidence": [95.0] * len(NAMES),
                    "operator_selection": {"state": "LOCKED"},
                    "calibration": {"state": "READY"},
                }) + "\n")
        result = benchmark(path, 40.0, 30.0)
        assert result.quality_level == "GREEN", result
        assert result.body_id_switches == 0
        assert result.sequence_gap_frames == 0
        assert result.core_visible_ratio_mean == 1.0
        assert result.bone_cv_p95 == 0.0

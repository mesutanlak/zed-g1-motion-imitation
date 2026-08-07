"""Regression checks for BODY_38 occlusion and re-acquisition handling."""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import numpy as np

from body38_to_gmr import Body38ToGMR


def body_frames(path: Path) -> list[dict]:
    frames = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            if record.get("schema") == "zed_body38_g1_reference/v1":
                frames.append(record)
    return frames


def landmark_index(frame: dict, name: str) -> int:
    return frame["keypoint_names"].index(name)


def set_landmark(
    frame: dict,
    name: str,
    *,
    filtered: list[float] | None,
    raw: list[float] | None,
    confidence: float,
) -> None:
    index = landmark_index(frame, name)
    frame["keypoints_3d_filtered_m"][index] = filtered
    frame["keypoints_3d_raw_m"][index] = raw
    pelvis_points = (frame.get("pelvis_frame") or {}).get("keypoints_m")
    if pelvis_points is not None and index < len(pelvis_points):
        pelvis_points[index] = filtered
    frame["keypoint_confidence"][index] = confidence


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: test_occlusion_resilience.py RECORDING.jsonl")

    frames = body_frames(Path(sys.argv[1]))
    if len(frames) < 20:
        raise RuntimeError("recording has too few BODY_38 frames")

    adapter = Body38ToGMR(
        nominal_fps=15.0,
        memory_seconds=0.20,
        max_raw_fallback_frames=2,
        reacquire_frames=3,
    )
    seed = None
    required = set(adapter_name for adapter_name in (
        "pelvis",
        "spine3",
        "left_shoulder",
        "right_shoulder",
        "right_wrist",
    ))
    for frame in frames:
        if adapter.adapt(frame) is not None and required.issubset(adapter._memory):
            seed = frame
            break
    if seed is None:
        raise RuntimeError("no valid seed frame")

    trusted_wrist = adapter._memory["right_wrist"][0].copy()

    # A wildly displaced point must neither reach output nor overwrite the
    # last trusted landmark.
    torn = copy.deepcopy(seed)
    bad_wrist = (trusted_wrist + np.array([1.5, 0.0, 0.0])).tolist()
    set_landmark(
        torn,
        "RIGHT_WRIST",
        filtered=bad_wrist,
        raw=bad_wrist,
        confidence=100.0,
    )
    torn_result = adapter.adapt(torn)
    assert torn_result is not None
    assert np.allclose(adapter._memory["right_wrist"][0], trusted_wrist)

    # A complete-frame geometry rejection is transactional: speculative
    # shoulder updates must be rolled back.
    trusted_left = adapter._memory["left_shoulder"][0].copy()
    trusted_right = adapter._memory["right_shoulder"][0].copy()
    overlap = copy.deepcopy(seed)
    midpoint = ((trusted_left + trusted_right) * 0.5).tolist()
    set_landmark(
        overlap,
        "LEFT_SHOULDER",
        filtered=midpoint,
        raw=midpoint,
        confidence=100.0,
    )
    set_landmark(
        overlap,
        "RIGHT_SHOULDER",
        filtered=midpoint,
        raw=midpoint,
        confidence=100.0,
    )
    assert adapter.adapt(overlap) is None
    assert adapter.last_rejection_reason == "implausible_geometry"
    assert np.allclose(adapter._memory["left_shoulder"][0], trusted_left)
    assert np.allclose(adapter._memory["right_shoulder"][0], trusted_right)

    # After memory expires, a reappearing landmark needs three coherent
    # observations. This prevents a one-frame snap after a long occlusion.
    adapter = Body38ToGMR(
        nominal_fps=15.0,
        memory_seconds=0.20,
        max_raw_fallback_frames=2,
        reacquire_frames=3,
    )
    assert adapter.adapt(copy.deepcopy(seed)) is not None
    missing = copy.deepcopy(seed)
    set_landmark(
        missing,
        "RIGHT_WRIST",
        filtered=None,
        raw=None,
        confidence=0.0,
    )
    for _ in range(adapter.max_memory_frames + 2):
        adapter.adapt(copy.deepcopy(missing))

    first = adapter.adapt(copy.deepcopy(seed))
    second = adapter.adapt(copy.deepcopy(seed))
    third = adapter.adapt(copy.deepcopy(seed))
    assert first is not None and "right_wrist" not in first.human_data
    assert second is not None and "right_wrist" not in second.human_data
    assert third is not None and "right_wrist" in third.human_data

    # In upper-body mode the verified lower-body geometry is a persistent IK
    # constraint. Losing an ankle must not stop otherwise valid arm tracking.
    upper_adapter = Body38ToGMR(
        nominal_fps=15.0,
        memory_seconds=0.20,
        persistent_memory_targets=(
            "left_hip",
            "right_hip",
            "left_knee",
            "right_knee",
            "left_foot",
            "right_foot",
        ),
    )
    for frame in frames:
        upper_result = upper_adapter.adapt(frame)
        if upper_result is not None and "right_foot" in upper_result.human_data:
            upper_seed = frame
            break
    else:
        raise RuntimeError("no valid upper-body seed frame")
    missing_ankle = copy.deepcopy(upper_seed)
    set_landmark(
        missing_ankle,
        "RIGHT_ANKLE",
        filtered=None,
        raw=None,
        confidence=0.0,
    )
    for _ in range(upper_adapter.max_memory_frames + 10):
        upper_result = upper_adapter.adapt(copy.deepcopy(missing_ankle))
    assert upper_result is not None and "right_foot" in upper_result.human_data

    print(
        "OCCLUSION_RESILIENCE_OK "
        f"memory={adapter.max_memory_frames} "
        f"raw_fallback={adapter.max_raw_fallback_frames} "
        f"reacquire={adapter.reacquire_frames}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

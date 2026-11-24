from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from hand_tracking.gesture import (
    GestureDex3Retargeter, compact_gesture_packet, fuse_gesture_packets, grip_observation,
    valid_gesture_packet,
)


def _shape(closed: bool) -> np.ndarray:
    points = np.zeros((21, 3), dtype=float)
    bases = {1: [-.035, .025, .01], 5: [-.03, .07, 0], 9: [0, .08, 0],
             13: [.025, .073, 0], 17: [.045, .06, 0]}
    for start, base in bases.items():
        points[start] = base
        for index in range(start + 1, min(start + 4, 21)):
            points[index] = points[index - 1] + [0, .025, .002]
    if closed:
        for start in (5, 9, 13, 17):
            points[start + 1] = points[start] + [0, .018, 0]
            points[start + 2] = points[start + 1] + [.012, .002, -.012]
            points[start + 3] = points[start + 2] + [.002, -.016, -.010]
    return points


def _packet(serial: int, closed: bool, stamp: int = 1_000_000_000) -> dict:
    return compact_gesture_packet({
        "camera_serial": serial, "source_host_id": "test", "sequence": 1,
        "capture_timestamp_ns": stamp,
        "operator": {"state": "LOCKED", "body_id": 7},
        "hands": [{"side": "right", "relative_landmarks_m": _shape(closed).tolist(),
                   "landmark_confidence": [.9] * 21, "rejection_reason": None}],
    })


def test_open_and_fist_have_compact_valid_datagrams() -> None:
    opened, closed = _packet(1, False), _packet(1, True)
    assert opened["hands"][0]["state"] == "OPEN"
    assert closed["hands"][0]["state"] == "CLOSED"
    assert valid_gesture_packet(opened) and valid_gesture_packet(closed)
    assert len(json.dumps(closed, separators=(",", ":")).encode()) < 1000


def test_missing_or_disagreeing_views_do_not_invent_fist() -> None:
    opened, closed = _packet(1, False), _packet(2, True)
    result = fuse_gesture_packets([opened, closed], 1_000_000_000)
    assert result["hands"][1]["state"] == "UNKNOWN"
    stale = fuse_gesture_packets([closed], 1_200_000_000)
    assert stale["hands"][1]["state"] == "UNKNOWN"
    assert fuse_gesture_packets([closed], 1_000_000_000)["hands"][1]["state"] == "CLOSED"


def test_low_confidence_and_partial_shape_are_unknown() -> None:
    hand = {"side": "left", "relative_landmarks_m": _shape(True).tolist(),
            "landmark_confidence": [.1] * 21, "rejection_reason": None}
    assert grip_observation(hand)["state"] == "UNKNOWN"
    assert grip_observation({"side": "left", "rejection_reason": "OCCLUDED"})["state"] == "UNKNOWN"


def test_binary_grip_simulation_target_is_limited_and_unknown_holds() -> None:
    config = Path(__file__).resolve().parents[1] / "config/g1_23dof_dex3.json"
    retargeter = GestureDex3Retargeter(config)
    first = retargeter.update("right", "CLOSED", 1_000_000_000, .9)
    assert first["solver"]["pose"] == "FIST"
    assert first["safety"]["watchdog"] == "TRACKING"
    last = first
    for index in range(1, 20):
        last = retargeter.update("right", "CLOSED", 1_000_000_000 + index * 33_333_333, .9)
    assert last["safe_q_rad"][3] > .5
    unknown = retargeter.update("right", "UNKNOWN", 1_650_000_000, 0.0)
    assert unknown["safety"]["watchdog"] == "HOLD"
    assert unknown["safe_q_rad"][3] > 0

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from typing import Any

import numpy as np


HAND_LANDMARK_NAMES = (
    "WRIST", "THUMB_CMC", "THUMB_MCP", "THUMB_IP", "THUMB_TIP",
    "INDEX_MCP", "INDEX_PIP", "INDEX_DIP", "INDEX_TIP",
    "MIDDLE_MCP", "MIDDLE_PIP", "MIDDLE_DIP", "MIDDLE_TIP",
    "RING_MCP", "RING_PIP", "RING_DIP", "RING_TIP",
    "PINKY_MCP", "PINKY_PIP", "PINKY_DIP", "PINKY_TIP",
)
HAND_SCHEMA = "zed_operator_hand/v1"
FUSED_HAND_SCHEMA = "zed_operator_hands_fused/v1"
UDP_SAFE_BYTES = 60_000
DEX3_CONTROL_SCHEMA = "unitree_g1_dex3_control/v1"
DEX3_JOINT_ORDER = (
    "thumb_0", "thumb_1", "thumb_2",
    "middle_0", "middle_1", "index_0", "index_1",
)


@dataclass(frozen=True)
class HandObservation:
    side: str
    roi_xywh: tuple[int, int, int, int]
    landmarks_px: list[list[float]]
    landmark_confidence: list[float]
    camera_points_m: list[list[float] | None]
    handedness_label: str | None
    handedness_score: float | None
    detection_confidence: float
    presence_confidence: float
    tracking_confidence: float
    inference_ms: float
    rejection_reason: str | None = None

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def finite_or_none(value: Any) -> Any:
    """Recursively replace non-finite numbers; JSON must never emit NaN/Inf."""
    if isinstance(value, np.ndarray):
        return finite_or_none(value.tolist())
    if isinstance(value, np.generic):
        return finite_or_none(value.item())
    if isinstance(value, dict):
        return {str(k): finite_or_none(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_or_none(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def encode_packet(packet: dict[str, Any], *, maximum_bytes: int = UDP_SAFE_BYTES) -> bytes:
    valid, reason = validate_hand_packet(packet)
    if not valid:
        raise ValueError(f"invalid hand packet: {reason}")
    encoded = json.dumps(
        packet, ensure_ascii=False, allow_nan=False,
        separators=(",", ":"),
    ).encode("utf-8")
    if len(encoded) > maximum_bytes:
        raise ValueError(f"hand UDP packet is {len(encoded)} bytes; limit is {maximum_bytes}")
    return encoded


def _finite_pair_list(value: Any, count: int) -> bool:
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        return False
    return array.shape == (count, 2) and bool(np.isfinite(array).all())


def validate_hand_packet(packet: Any) -> tuple[bool, str | None]:
    if not isinstance(packet, dict) or packet.get("schema") != HAND_SCHEMA:
        return False, "SCHEMA"
    for field in ("camera_serial", "source_host_id", "sequence", "capture_timestamp_ns"):
        if field not in packet:
            return False, f"MISSING_{field.upper()}"
    if not isinstance(packet.get("image_size"), list) or len(packet["image_size"]) != 2:
        return False, "IMAGE_SIZE"
    operator = packet.get("operator") or {}
    if operator.get("state") != "LOCKED" or operator.get("body_id") is None:
        return False, "OPERATOR_NOT_LOCKED"
    hands = packet.get("hands")
    if not isinstance(hands, list) or len(hands) > 2:
        return False, "HANDS"
    seen: set[str] = set()
    for hand in hands:
        side = hand.get("side")
        if side not in ("left", "right") or side in seen:
            return False, "SIDE"
        seen.add(side)
        if hand.get("rejection_reason") is None:
            if not _finite_pair_list(hand.get("landmarks_px"), 21):
                return False, "LANDMARKS_2D"
            confidence = np.asarray(hand.get("landmark_confidence"), dtype=float)
            if confidence.shape != (21,) or not np.isfinite(confidence).all():
                return False, "CONFIDENCE"
        try:
            json.dumps(hand, allow_nan=False)
        except (TypeError, ValueError):
            return False, "NONFINITE_OR_NON_JSON"
    return True, None


def validate_dex3_control(value: Any) -> tuple[bool, str | None]:
    if not isinstance(value, dict) or value.get("schema") != DEX3_CONTROL_SCHEMA:
        return False, "SCHEMA"
    if value.get("physical_robot_output_enabled") is not False:
        return False, "PHYSICAL_OUTPUT_MUST_BE_FALSE"
    if not isinstance(value.get("timestamp_ns"), int):
        return False, "TIMESTAMP"
    if tuple(value.get("joint_order_per_hand") or ()) != DEX3_JOINT_ORDER:
        return False, "JOINT_ORDER"
    for side in ("left", "right"):
        try:
            q = np.asarray(value[f"q_{side}"], dtype=float)
        except (KeyError, TypeError, ValueError):
            return False, f"Q_{side.upper()}"
        if q.shape != (7,) or not np.isfinite(q).all():
            return False, f"Q_{side.upper()}"
        try:
            confidence = float(value[f"confidence_{side}"])
        except (KeyError, TypeError, ValueError):
            return False, f"CONFIDENCE_{side.upper()}"
        if not np.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            return False, f"CONFIDENCE_{side.upper()}"
        watchdog = value.get(f"watchdog_{side}")
        if not isinstance(watchdog, str) or not watchdog:
            return False, f"WATCHDOG_{side.upper()}"
    return True, None

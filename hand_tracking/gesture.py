"""Small, operator-locked open/closed hand observations without 3-D fusion."""

from __future__ import annotations

import math
import json
from pathlib import Path
from typing import Any

import numpy as np

from .contracts import DEX3_JOINT_ORDER
from .retargeting import Dex3SafetyFilter, SafetyConfig


GESTURE_SCHEMA = "zed_operator_hand_gesture/v1"
FUSED_GESTURE_SCHEMA = "zed_operator_hand_gestures_fused/v1"
GESTURE_STATES = frozenset(("OPEN", "CLOSED", "UNKNOWN"))


def _bend(first: np.ndarray, centre: np.ndarray, last: np.ndarray) -> float:
    a, b = first - centre, last - centre
    denominator = float(np.linalg.norm(a) * np.linalg.norm(b))
    if denominator < 1e-8:
        raise ValueError("degenerate finger landmarks")
    return float(np.pi - np.arccos(np.clip(float(np.dot(a, b)) / denominator, -1, 1)))


def grip_observation(hand: dict[str, Any]) -> dict[str, Any]:
    """Classify four fingers; partial/ambiguous hands remain UNKNOWN.

    Uses MediaPipe's relative 3-D *shape*, never its absolute hand position.
    A closed state needs several curled fingers; a pinch alone is not a fist.
    """
    base = {
        "side": hand.get("side"),
        "state": "UNKNOWN", "fist_score": None, "confidence": 0.0,
        "hand_bbox_px": hand.get("hand_bbox_px"),
        "inference_ms": hand.get("inference_ms"),
        "reason": hand.get("rejection_reason") or "AMBIGUOUS_SHAPE",
    }
    if hand.get("rejection_reason") is not None:
        return base
    try:
        points = np.asarray(hand.get("relative_landmarks_m"), dtype=float)
        if points.shape != (21, 3) or not np.isfinite(points).all():
            return base
        curls = []
        for mcp in (5, 9, 13, 17):
            total = (
                0.45 * _bend(points[0], points[mcp], points[mcp + 1])
                + _bend(points[mcp], points[mcp + 1], points[mcp + 2])
                + _bend(points[mcp + 1], points[mcp + 2], points[mcp + 3])
            )
            curls.append(float(np.clip(total / 2.75, 0, 1)))
        score = float(np.mean(curls))
        confidence_values = np.asarray(hand.get("landmark_confidence", []), dtype=float)
        quality = float(np.clip(np.median(confidence_values), 0, 1)) if confidence_values.shape == (21,) and np.isfinite(confidence_values).all() else 0.0
        if quality < 0.30:
            base["reason"] = "LOW_LANDMARK_CONFIDENCE"
            return base
        state = "CLOSED" if score >= 0.62 else "OPEN" if score <= 0.22 else "UNKNOWN"
        base.update(state=state, fist_score=score, confidence=quality,
                    reason=None if state != "UNKNOWN" else "PARTIAL_OR_AMBIGUOUS")
    except (TypeError, ValueError, IndexError):
        pass
    return base


def compact_gesture_packet(packet: dict[str, Any]) -> dict[str, Any]:
    """Retain only scalar gesture evidence and transport identity."""
    return {
        "schema": GESTURE_SCHEMA,
        "camera_serial": int(packet["camera_serial"]),
        "source_host_id": str(packet["source_host_id"]),
        "research_session_id": packet.get("research_session_id"),
        "sequence": int(packet["sequence"]),
        "capture_timestamp_ns": int(packet["capture_timestamp_ns"]),
        "operator": dict(packet["operator"]),
        "hands": [grip_observation(hand) for hand in packet.get("hands", [])],
        "transport_metrics": packet.get("transport_metrics"),
        "physical_robot_output_enabled": False,
    }


def valid_gesture_packet(packet: Any) -> bool:
    if not isinstance(packet, dict) or packet.get("schema") != GESTURE_SCHEMA:
        return False
    try:
        if int(packet["camera_serial"]) <= 0 or int(packet["capture_timestamp_ns"]) <= 0:
            return False
        if int(packet["sequence"]) < 0 or packet["operator"]["state"] != "LOCKED":
            return False
        if packet["operator"]["body_id"] is None or packet.get("physical_robot_output_enabled") is not False:
            return False
        hands = packet["hands"]
        if not isinstance(hands, list) or len(hands) > 2:
            return False
        seen = set()
        for hand in hands:
            side, state = hand["side"], hand["state"]
            if side not in ("left", "right") or side in seen or state not in GESTURE_STATES:
                return False
            seen.add(side)
            confidence = float(hand["confidence"])
            if not math.isfinite(confidence) or not 0 <= confidence <= 1:
                return False
        return True
    except (KeyError, TypeError, ValueError):
        return False


def fuse_gesture_packets(
    packets: list[dict[str, Any]], target_timestamp_ns: int,
    *, maximum_age_ms: float = 120.0,
    maximum_capture_spread_ms: float = 70.0,
) -> dict[str, Any]:
    """Time align scalar votes; disagreement and missing hands stay UNKNOWN."""
    result: dict[str, Any] = {
        "schema": FUSED_GESTURE_SCHEMA,
        "capture_timestamp_ns": int(target_timestamp_ns), "hands": [],
        "physical_robot_output_enabled": False,
    }
    for side in ("left", "right"):
        votes = []
        for packet in packets:
            stamp = int(packet.get("_source_capture_timestamp_ns", packet["capture_timestamp_ns"]))
            age_ms = abs(target_timestamp_ns - stamp) / 1e6
            if age_ms > maximum_age_ms:
                continue
            hand = next((item for item in packet.get("hands", []) if item.get("side") == side), None)
            if hand is None or hand.get("state") not in ("OPEN", "CLOSED"):
                continue
            votes.append((stamp, hand, int(packet["camera_serial"]), age_ms))
        if votes:
            latest = max(item[0] for item in votes)
            votes = [item for item in votes if latest - item[0] <= maximum_capture_spread_ms * 1e6]
        weights = {"OPEN": 0.0, "CLOSED": 0.0}
        for _stamp, hand, _serial, age_ms in votes:
            weights[hand["state"]] += float(hand["confidence"]) * math.exp(-age_ms / max(maximum_age_ms, 1.0))
        total = sum(weights.values())
        leader = max(weights, key=weights.get)
        dominance = weights[leader] / total if total > 0 else 0.0
        state = leader if votes and dominance >= 0.70 else "UNKNOWN"
        quality = float(np.mean([
            float(hand["confidence"]) for _stamp, hand, _serial, _age in votes
            if hand["state"] == state
        ])) if state != "UNKNOWN" else 0.0
        result["hands"].append({
            "side": side, "state": state,
            "confidence": float(dominance * quality),
            "camera_serials": [serial for _stamp, hand, serial, _age in votes if hand["state"] == state],
            "observation_count": len(votes),
            "reason": None if state != "UNKNOWN" else "MISSING_STALE_OR_DISAGREEING_VIEWS",
        })
    return result


class GestureDex3Retargeter:
    """Open/closed presets for DDS-free simulation, with existing joint limits/watchdog."""

    def __init__(self, config_path: str | Path) -> None:
        hands = json.loads(Path(config_path).read_text())["hands"]
        if tuple(hands["joint_order_per_hand"]) != DEX3_JOINT_ORDER:
            raise ValueError("Dex3 joint order mismatch")
        safety_data = hands.get("safety") or {}
        safety = SafetyConfig(**{
            key: safety_data[key] for key in SafetyConfig.__dataclass_fields__
            if key in safety_data
        })
        self.filters = {
            side: Dex3SafetyFilter(hands[f"{side}_limits_rad"]["min"],
                                   hands[f"{side}_limits_rad"]["max"],
                                   hands.get("neutral_rad", [0.0] * 7), safety)
            for side in ("left", "right")
        }

    def update(self, side: str, state: str, timestamp_ns: int, confidence: float) -> dict[str, Any]:
        if side not in self.filters:
            raise ValueError("invalid side")
        if state == "CLOSED":
            sign = -1.0 if side == "left" else 1.0
            raw = np.asarray([.55, .65, -sign * 1.35,
                              sign * 1.25, sign * 1.50,
                              sign * 1.25, sign * 1.50])
            pose = "FIST"
        elif state == "OPEN":
            raw = np.zeros(7)
            pose = "OPEN"
        else:
            raw = None
            pose = None
        safe, metrics = self.filters[side].update(
            raw, timestamp_ns, confidence if raw is not None else 0.0, pose
        )
        return {
            "joint_order": list(DEX3_JOINT_ORDER),
            "raw_q_rad": None if raw is None else raw.tolist(),
            "safe_q_rad": safe.tolist(),
            "solver": {"backend": "binary_grip_presets_v1", "pose": pose},
            "safety": metrics,
        }

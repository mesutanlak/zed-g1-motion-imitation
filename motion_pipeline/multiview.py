"""Pure multi-view BODY_38 quality, association, and comparison helpers.

The ZED Fusion SDK remains responsible for synchronization, calibration and
3-D fusion.  This module only decides whether a fused result is usable and
which already-calibrated per-camera result is the safest temporary fallback.
It intentionally has no PyZED dependency so the rules can be unit tested.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

import numpy as np


UPPER_BODY_NAMES = (
    "PELVIS", "SPINE_2", "SPINE_3", "NECK",
    "LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST",
    "RIGHT_SHOULDER", "RIGHT_ELBOW", "RIGHT_WRIST",
)


@dataclass(frozen=True)
class ViewQuality:
    serial_number: int
    body_id: int
    score: float
    visible_ratio: float
    upper_visible_ratio: float
    mean_confidence: float
    anchor_m: tuple[float, float, float]


@dataclass(frozen=True)
class MultiViewDecision:
    mode: str
    selected_serial: int | None
    reason: str
    fused_score: float | None
    best_single_score: float | None


def _finite_rows(points: np.ndarray) -> np.ndarray:
    xyz = np.asarray(points, dtype=np.float64)
    return np.isfinite(xyz).all(axis=1)


def body_quality(
    *,
    serial_number: int,
    body_id: int,
    points: np.ndarray,
    confidence: Sequence[float],
    body_confidence: float,
    index: Mapping[str, int],
    threshold: float,
) -> ViewQuality:
    """Score one body without rewarding a single spuriously confident joint."""
    xyz = np.asarray(points, dtype=np.float64)
    conf = np.asarray(confidence, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or conf.shape[0] != xyz.shape[0]:
        return ViewQuality(serial_number, body_id, 0.0, 0.0, 0.0, 0.0, (math.nan,) * 3)
    finite = _finite_rows(xyz)
    visible = finite & (conf >= float(threshold))
    upper_ids = [index[name] for name in UPPER_BODY_NAMES if name in index]
    upper_visible = visible[upper_ids] if upper_ids else np.zeros(0, dtype=bool)
    visible_ratio = float(np.mean(visible)) if visible.size else 0.0
    upper_ratio = float(np.mean(upper_visible)) if upper_visible.size else 0.0
    mean_conf = float(np.mean(conf[visible])) if np.any(visible) else 0.0
    # Upper-body completeness dominates because the current G1 task fixes the
    # lower body. Body confidence is capped to prevent a high global value from
    # hiding a missing elbow/wrist chain.
    score = float(np.clip(
        0.50 * upper_ratio
        + 0.25 * visible_ratio
        + 0.15 * (mean_conf / 100.0)
        + 0.10 * (float(body_confidence) / 100.0),
        0.0,
        1.0,
    ))
    pelvis = xyz[index["PELVIS"]] if "PELVIS" in index else np.full(3, np.nan)
    anchor = tuple(float(value) for value in pelvis)
    return ViewQuality(
        int(serial_number), int(body_id), score, visible_ratio, upper_ratio,
        mean_conf, anchor,
    )


def association_distance_m(first: ViewQuality, second: ViewQuality) -> float:
    a = np.asarray(first.anchor_m, dtype=np.float64)
    b = np.asarray(second.anchor_m, dtype=np.float64)
    if not np.isfinite(a).all() or not np.isfinite(b).all():
        return math.inf
    return float(np.linalg.norm(a - b))


def choose_output(
    *,
    fused: ViewQuality | None,
    singles: Sequence[ViewQuality],
    minimum_fused_score: float = 0.58,
    maximum_operator_jump_m: float = 0.75,
    previous_anchor_m: Sequence[float] | None = None,
) -> MultiViewDecision:
    """Prefer official fusion, otherwise choose a conservative single view."""
    valid_singles = [item for item in singles if item.score > 0.0]
    best = max(valid_singles, key=lambda item: item.score, default=None)
    # A global score alone can hide the exact failure that matters to this
    # project: both arm chains disappearing while torso/leg landmarks remain
    # excellent. Require an almost-complete upper body before accepting fused
    # output over a clearer calibrated camera.
    if (
        fused is not None
        and fused.score >= minimum_fused_score
        and fused.upper_visible_ratio >= 0.80
    ):
        return MultiViewDecision(
            "fusion", None, "official_fusion_valid", fused.score,
            best.score if best else None,
        )
    if best is None:
        return MultiViewDecision(
            "hold", None, "no_valid_fused_or_single_body",
            fused.score if fused else None, None,
        )
    if previous_anchor_m is not None:
        previous = np.asarray(previous_anchor_m, dtype=np.float64)
        current = np.asarray(best.anchor_m, dtype=np.float64)
        if (
            previous.shape == (3,) and np.isfinite(previous).all()
            and current.shape == (3,) and np.isfinite(current).all()
            and float(np.linalg.norm(current - previous)) > maximum_operator_jump_m
        ):
            return MultiViewDecision(
                "hold", None, "fallback_operator_jump_rejected",
                fused.score if fused else None, best.score,
            )
    return MultiViewDecision(
        "single_fallback", best.serial_number,
        "fusion_low_quality_best_calibrated_camera_selected",
        fused.score if fused else None, best.score,
    )


def keypoint_agreement(
    first: np.ndarray,
    second: np.ndarray,
    first_confidence: Sequence[float],
    second_confidence: Sequence[float],
    threshold: float,
) -> dict[str, float | int | None]:
    """Compare synchronized SDK-transformed skeletons in the common frame."""
    a = np.asarray(first, dtype=np.float64)
    b = np.asarray(second, dtype=np.float64)
    ca = np.asarray(first_confidence, dtype=np.float64)
    cb = np.asarray(second_confidence, dtype=np.float64)
    valid = (
        _finite_rows(a) & _finite_rows(b)
        & (ca >= float(threshold)) & (cb >= float(threshold))
    )
    count = int(np.count_nonzero(valid))
    if not count:
        return {"common_keypoints": 0, "mpjpe_m": None, "p95_error_m": None}
    errors = np.linalg.norm(a[valid] - b[valid], axis=1)
    return {
        "common_keypoints": count,
        "mpjpe_m": float(np.mean(errors)),
        "p95_error_m": float(np.percentile(errors, 95)),
    }


def arm_evidence(
    views: Sequence[dict[str, object]],
    *,
    side: str,
    threshold: float,
) -> dict[str, object]:
    """Summarize independent camera evidence for one complete arm chain.

    Each view dictionary contains ``confidence`` and a precomputed ``overlap``
    flag from that camera's own 2-D torso polygon. A single clear, complete
    oblique view can therefore prevent an unnecessary AKC recovery, while two
    ambiguous views keep the existing conservative recovery active.
    """
    label = side.lower()
    supporting = 0
    reliable_clear = 0
    confidences: list[float] = []
    serials: list[int] = []
    for view in views:
        chain = np.asarray(view.get("arm_confidence", {}).get(label, []), dtype=np.float64)
        if chain.size != 3 or not np.isfinite(chain).all():
            continue
        minimum = float(np.min(chain))
        confidences.append(minimum)
        if minimum >= threshold:
            supporting += 1
            serials.append(int(view.get("serial_number", 0)))
            if not bool(view.get("arm_overlap", {}).get(label, True)):
                reliable_clear += 1
    return {
        "supporting_views": supporting,
        "reliable_clear_views": reliable_clear,
        "best_chain_confidence": max(confidences, default=0.0),
        "supporting_serials": serials,
    }


def _inside_convex(point: np.ndarray, polygon: np.ndarray) -> bool:
    p = np.asarray(point, dtype=np.float64)
    poly = np.asarray(polygon, dtype=np.float64)
    if p.shape != (2,) or poly.shape != (4, 2):
        return False
    if not np.isfinite(p).all() or not np.isfinite(poly).all():
        return False
    signs: list[float] = []
    for start, end in zip(poly, np.roll(poly, -1, axis=0)):
        edge = end - start
        relative = p - start
        signs.append(float(edge[0] * relative[1] - edge[1] * relative[0]))
    return all(value >= -1e-6 for value in signs) or all(
        value <= 1e-6 for value in signs
    )


def torso_overlap_2d(
    points_2d: np.ndarray,
    index: Mapping[str, int],
    side: str,
) -> bool:
    """Camera-local overlap test; never mix pixels from different cameras."""
    pixel = np.asarray(points_2d, dtype=np.float64)
    try:
        torso = pixel[[
            index["LEFT_SHOULDER"], index["RIGHT_SHOULDER"],
            index["RIGHT_HIP"], index["LEFT_HIP"],
        ]]
        elbow = pixel[index[f"{side.upper()}_ELBOW"]]
        wrist = pixel[index[f"{side.upper()}_WRIST"]]
    except (KeyError, IndexError):
        return True
    return _inside_convex(elbow, torso) or _inside_convex(wrist, torso)

#!/usr/bin/env python3
"""Replay a dual BODY_38 JSONL through the confidence-aware estimator.

This is an offline regression tool: it performs no camera, UDP, Isaac or robot
I/O.  It reports whether the new fusion is at least as continuous as the
recorded control skeleton and how often both views can safely contribute.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from motion_pipeline.human_state import (
    CORE_NAMES,
    ConfidenceAwareHumanStateEstimator,
    load_human_state_config,
)
from zed_g1_skeleton import BODY38_EDGES, BODY38_NAMES


def percentile(values: list[float], q: float) -> float | None:
    return float(np.percentile(values, q)) if values else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("recording", type=Path)
    parser.add_argument(
        "--config", type=Path,
        default=Path(__file__).parents[1] / "config" / "dual_teleoperation.json",
    )
    parser.add_argument("--fallback-serial", type=int, default=33773329)
    args = parser.parse_args()
    config = load_human_state_config(args.config)
    estimator = ConfidenceAwareHumanStateEstimator(BODY38_NAMES, BODY38_EDGES, config)
    index = {name: i for i, name in enumerate(BODY38_NAMES)}
    core_ids = [index[name] for name in CORE_NAMES]
    old_modes: Counter[str] = Counter()
    new_modes: Counter[str] = Counter()
    new_states: Counter[str] = Counter()
    estimator_view_counts: Counter[str] = Counter()
    joint_sources: Counter[str] = Counter()
    failures: Counter[str] = Counter()
    old_jumps: list[float] = []
    new_jumps: list[float] = []
    core_errors: list[float] = []
    previous_old = previous_new = None
    previous_camera: dict[int, np.ndarray] = {}
    camera_jumps: dict[int, list[float]] = {}
    evaluated = 0
    old_transitions = new_transitions = 0
    previous_old_mode = previous_new_mode = None
    fusion_latched = False
    fusion_lock_frames = 0

    with args.recording.open("r", encoding="utf-8") as stream:
        next(stream, None)
        for line in stream:
            frame = json.loads(line)
            multi = frame.get("multi_camera") or {}
            views = multi.get("per_camera") or []
            bodies: dict[int, SimpleNamespace] = {}
            timestamps: dict[int, int] = {}
            for view in views:
                points = np.asarray(view.get("keypoints_3d_fusion_m"), dtype=float)
                confidence = np.asarray(view.get("keypoint_confidence"), dtype=float)
                if points.shape != (38, 3) or confidence.shape != (38,):
                    continue
                serial = int(view["serial_number"])
                quality = view.get("quality") or {}
                bodies[serial] = SimpleNamespace(
                    keypoint=points,
                    keypoint_confidence=confidence,
                    confidence=float(quality.get("mean_confidence", 0.0)),
                )
                timestamps[serial] = int(view.get("capture_timestamp_ns") or frame["timestamp_ns"])
                if serial in previous_camera:
                    valid = np.isfinite(points).all(axis=1) & np.isfinite(previous_camera[serial]).all(axis=1)
                    camera_jumps.setdefault(serial, []).extend(
                        np.linalg.norm(points[valid] - previous_camera[serial][valid], axis=1)
                    )
                previous_camera[serial] = points.copy()
            if not bodies:
                continue
            old_mode = str(multi.get("mode") or "unknown")
            old_modes[old_mode] += 1
            if previous_old_mode is not None and old_mode != previous_old_mode:
                old_transitions += 1
            previous_old_mode = old_mode

            use = dict(bodies)
            core_error = None
            if len(bodies) >= 2:
                first, second = list(bodies.values())[:2]
                valid = (
                    np.isfinite(first.keypoint[core_ids]).all(axis=1)
                    & np.isfinite(second.keypoint[core_ids]).all(axis=1)
                    & (first.keypoint_confidence[core_ids] >= config["fusion"]["confidence_threshold"])
                    & (second.keypoint_confidence[core_ids] >= config["fusion"]["confidence_threshold"])
                )
                if np.any(valid):
                    core_error = float(np.median(np.linalg.norm(
                        first.keypoint[core_ids] - second.keypoint[core_ids], axis=1
                    )[valid]))
                    core_errors.append(core_error)
            if core_error is None or core_error > config["fusion"]["core_calibration_max_m"]:
                serial = args.fallback_serial if args.fallback_serial in bodies else max(bodies)
                use = {serial: bodies[serial]}
            calibration_good = len(use) >= 2
            if calibration_good:
                fusion_lock_frames += 1
                if fusion_lock_frames >= int(
                    config["fusion"].get("fusion_lock_acquire_frames", 3)
                ):
                    fusion_latched = True
            else:
                fusion_lock_frames = max(0, fusion_lock_frames - 1)
            new_mode = (
                "joint_fusion"
                if fusion_latched or calibration_good
                else "single_fallback"
            )
            new_state = (
                "LOCKED_DUAL"
                if calibration_good
                else "LOCKED_CALIBRATION_GUARD"
                if fusion_latched and len(bodies) >= 2
                else "LOCKED_PARTIALLY_VISIBLE"
                if fusion_latched
                else "SINGLE_ACQUISITION"
            )
            new_modes[new_mode] += 1
            new_states[new_state] += 1
            estimator_view_counts[str(len(use))] += 1
            if previous_new_mode is not None and new_mode != previous_new_mode:
                new_transitions += 1
            previous_new_mode = new_mode
            result = estimator.update(
                use, {serial: timestamps[serial] for serial in use},
                target_timestamp_ns=max(timestamps[serial] for serial in use),
            )
            evaluated += 1
            joint_sources.update(result.joint_source)
            failures.update(result.failure_codes)
            old = np.asarray(frame.get("keypoints_3d_filtered_m"), dtype=float)
            if previous_old is not None and old.shape == previous_old.shape:
                valid = np.isfinite(old).all(axis=1) & np.isfinite(previous_old).all(axis=1)
                old_jumps.extend(np.linalg.norm(old[valid] - previous_old[valid], axis=1))
            if previous_new is not None:
                valid = np.isfinite(result.points).all(axis=1) & np.isfinite(previous_new).all(axis=1)
                new_jumps.extend(np.linalg.norm(result.points[valid] - previous_new[valid], axis=1))
            previous_old = old
            previous_new = result.points.copy()

    summary = {
        "schema": "dual_fusion_regression/v1",
        "recording": str(args.recording.resolve()),
        "evaluated_frames": evaluated,
        "old_mode_counts": dict(old_modes),
        "new_mode_counts": dict(new_modes),
        "new_fusion_state_counts": dict(new_states),
        "estimator_view_counts": dict(estimator_view_counts),
        "old_mode_transitions": old_transitions,
        "new_mode_transitions": new_transitions,
        "core_disagreement_m": {
            "p50": percentile(core_errors, 50), "p95": percentile(core_errors, 95),
        },
        "per_joint_frame_displacement_m": {
            "old_p50": percentile(old_jumps, 50), "old_p95": percentile(old_jumps, 95),
            "new_p50": percentile(new_jumps, 50), "new_p95": percentile(new_jumps, 95),
            "camera_only": {
                str(serial): {
                    "p50": percentile(values, 50), "p95": percentile(values, 95),
                }
                for serial, values in sorted(camera_jumps.items())
            },
        },
        "joint_source_counts": dict(joint_sources),
        "failure_counts": dict(failures),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

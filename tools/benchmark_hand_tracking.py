#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import statistics
from typing import Any


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round((len(ordered) - 1) * q)))]


def _run_durations(samples: list[tuple[int, bool]], wanted: bool) -> list[float]:
    durations: list[float] = []
    started: int | None = None
    previous: int | None = None
    for timestamp, state in samples:
        if state == wanted:
            if started is None:
                started = timestamp
            previous = timestamp
        elif started is not None:
            durations.append(max(0.0, ((previous or started) - started) / 1e9))
            started = previous = None
    if started is not None:
        durations.append(max(0.0, ((previous or started) - started) / 1e9))
    return durations


def _depth_count(hand: dict[str, Any]) -> int:
    if hand.get("depth_valid_count") is not None:
        return int(hand["depth_valid_count"])
    return sum(item is not None for item in (hand.get("camera_points_m") or []))


def summarize(path: Path) -> dict[str, Any]:
    inference: dict[str, list[float]] = {}
    camera_timestamps: dict[str, list[int]] = {}
    camera_detector: dict[str, Counter] = {}
    camera_depth: dict[str, list[int]] = {}
    camera_depth_detected: dict[str, list[int]] = {}
    latency: list[float] = []
    multi_view_spread: list[float] = []
    reprojection: list[float] = []
    fusion_residual_mm: list[float] = []
    camera_counts: list[int] = []
    side_samples = {side: [] for side in ("left", "right")}
    side_valid = Counter()
    frame_total = both_valid = either_valid = 0
    stale = spread_rejected = 0
    solver_times: list[float] = []
    residuals: list[float] = []
    joint_clip = velocity_limit = acceleration_limit = 0
    annotated_previous: dict[str, str] = {}
    annotated_switches = 0
    annotation_samples = 0

    with path.open(encoding="utf-8") as stream:
        for line in stream:
            try:
                packet = json.loads(line)
            except json.JSONDecodeError:
                continue
            hands_root = packet.get("hand_tracking") or (
                packet if packet.get("schema") == "zed_operator_hands_fused/v1" else None
            )
            if not hands_root:
                continue
            timestamp = int(hands_root.get("capture_timestamp_ns") or packet.get("timestamp_ns") or 0)
            trace = packet.get("latency_trace_ns") or {}
            if trace.get("t2_windows_udp_send_ns") and trace.get("t0_capture_ns"):
                latency.append((trace["t2_windows_udp_send_ns"] - trace["t0_capture_ns"]) / 1e6)
            for camera in hands_root.get("per_camera", []):
                serial = str(camera.get("camera_serial"))
                camera_timestamps.setdefault(serial, []).append(int(camera.get("capture_timestamp_ns", 0)))
                counts = camera_detector.setdefault(serial, Counter())
                for hand in camera.get("hands", []):
                    side = str(hand.get("side", "unknown"))
                    counts[f"{side}_samples"] += 1
                    reason = hand.get("rejection_reason")
                    if reason:
                        counts[f"{side}_rejected"] += 1
                        counts[f"reason:{reason}"] += 1
                    else:
                        counts[f"{side}_detected"] += 1
                    if hand.get("inference_ms") is not None:
                        inference.setdefault(serial, []).append(float(hand["inference_ms"]))
                    depth_count = _depth_count(hand)
                    camera_depth.setdefault(f"{serial}:{side}", []).append(depth_count)
                    if not reason:
                        camera_depth_detected.setdefault(f"{serial}:{side}", []).append(depth_count)
            hands = {
                str(hand.get("side")): hand
                for hand in hands_root.get("hands", [])
                if hand.get("side") in {"left", "right"}
            }
            if len(hands) == 2:
                frame_total += 1
                valid_now = {side: bool(hands[side].get("valid")) for side in ("left", "right")}
                both_valid += int(all(valid_now.values()))
                either_valid += int(any(valid_now.values()))
                for side in ("left", "right"):
                    side_valid[side] += int(valid_now[side])
                    side_samples[side].append((timestamp, valid_now[side]))
            for side, hand in hands.items():
                spread_value = float(hand.get("capture_spread_ms", 0.0) or 0.0)
                reasons = hand.get("rejection_reasons", [])
                stale += sum(str(item).startswith("STALE:") for item in reasons)
                spread_rejected += sum(str(item).startswith("CAPTURE_SPREAD:") for item in reasons)
                counts_this_hand = []
                for quality in hand.get("landmark_quality", []):
                    camera_count = int(quality.get("camera_count", 0) or 0)
                    camera_counts.append(camera_count)
                    counts_this_hand.append(camera_count)
                    if quality.get("residual_m") is not None:
                        fusion_residual_mm.append(float(quality["residual_m"]) * 1000.0)
                    if quality.get("reprojection_error_px") is not None:
                        reprojection.append(float(quality["reprojection_error_px"]))
                if any(value >= 2 for value in counts_this_hand):
                    multi_view_spread.append(spread_value)
                identity = hand.get("annotated_identity")
                if identity is not None:
                    annotation_samples += 1
                    identity = str(identity)
                    if side in annotated_previous and annotated_previous[side] != identity:
                        annotated_switches += 1
                    annotated_previous[side] = identity
            for target in (packet.get("dex3_targets") or {}).values():
                if not isinstance(target, dict):
                    continue
                solver = target.get("solver") or {}
                safety = target.get("safety") or {}
                if solver.get("time_ms") is not None:
                    solver_times.append(float(solver["time_ms"]))
                if solver.get("residual") is not None:
                    residuals.append(float(solver["residual"]))
                joint_clip += int(safety.get("joint_limit_clipped_count", 0) or 0)
                velocity_limit += int(safety.get("velocity_limited_count", 0) or 0)
                acceleration_limit += int(safety.get("acceleration_limited_count", 0) or 0)

    per_camera = {}
    for serial, values in inference.items():
        timestamps = sorted(set(camera_timestamps.get(serial, [])))
        counts = camera_detector.get(serial, Counter())
        detector_by_side = {}
        for side in ("left", "right"):
            total = counts[f"{side}_samples"]
            detector_by_side[side] = {
                "samples": total,
                "detected": counts[f"{side}_detected"],
                "success_percent": 100.0 * counts[f"{side}_detected"] / total if total else 0.0,
                "depth_points_mean_of_21": (
                    statistics.mean(camera_depth.get(f"{serial}:{side}", []))
                    if camera_depth.get(f"{serial}:{side}") else None
                ),
                "depth_points_mean_when_detected": (
                    statistics.mean(camera_depth_detected.get(f"{serial}:{side}", []))
                    if camera_depth_detected.get(f"{serial}:{side}") else None
                ),
            }
        per_camera[serial] = {
            "samples": len(values),
            "p50_ms": percentile(values, 0.5),
            "p95_ms": percentile(values, 0.95),
            "measured_fps": (
                (len(timestamps) - 1) * 1e9 / (timestamps[-1] - timestamps[0])
                if len(timestamps) > 1 and timestamps[-1] > timestamps[0] else None
            ),
            "detector_by_side": detector_by_side,
            "rejection_reasons": {
                key.removeprefix("reason:"): value
                for key, value in counts.items() if key.startswith("reason:")
            },
        }
    histogram = Counter(camera_counts)
    observed_camera_counts = [value for value in camera_counts if value > 0]
    side_report = {}
    for side in ("left", "right"):
        track = _run_durations(side_samples[side], True)
        gaps = _run_durations(side_samples[side], False)
        side_report[side] = {
            "valid_frames": side_valid[side],
            "coverage_percent": 100.0 * side_valid[side] / frame_total if frame_total else 0.0,
            "longest_tracking_s": max(track, default=0.0),
            "longest_loss_s": max(gaps, default=0.0),
        }
    return {
        "schema": "zed_hand_benchmark/v2",
        "measured_not_claimed": True,
        "frames": frame_total,
        "coverage": {
            "left": side_report["left"], "right": side_report["right"],
            "both_hands_percent": 100.0 * both_valid / frame_total if frame_total else 0.0,
            "either_hand_percent": 100.0 * either_valid / frame_total if frame_total else 0.0,
        },
        "per_camera_inference": per_camera,
        "end_to_end_latency_ms": {"p50": percentile(latency, 0.5), "p95": percentile(latency, 0.95)},
        "landmark_camera_count": {
            "mean": statistics.mean(camera_counts) if camera_counts else None,
            "mean_when_observed": statistics.mean(observed_camera_counts) if observed_camera_counts else None,
            "histogram": {str(key): histogram[key] for key in range(5)},
            "percent": {str(key): 100.0 * histogram[key] / len(camera_counts) if camera_counts else 0.0 for key in range(5)},
        },
        "reprojection_error_px": {"p50": percentile(reprojection, 0.5), "p95": percentile(reprojection, 0.95)},
        "fusion_residual_mm": {"p50": percentile(fusion_residual_mm, 0.5), "p95": percentile(fusion_residual_mm, 0.95)},
        "multi_view_capture_spread_ms": {"p50": percentile(multi_view_spread, 0.5), "p95": percentile(multi_view_spread, 0.95), "samples": len(multi_view_spread)},
        "stale_rejections": stale,
        "capture_spread_rejections": spread_rejected,
        "solver_ms": {"p50": percentile(solver_times, 0.5), "p95": percentile(solver_times, 0.95)},
        "solver_residual": {"p50": percentile(residuals, 0.5), "p95": percentile(residuals, 0.95)},
        "safety_events": {
            "joint_limit_clipped": joint_clip,
            "velocity_limited": velocity_limit,
            "acceleration_limited": acceleration_limit,
        },
        "annotated_id_switch": {
            "samples": annotation_samples,
            "count": annotated_switches if annotation_samples else None,
            "status": "measured" if annotation_samples else "requires annotated_identity in replay",
        },
        # v1 compatibility for existing notebooks.
        "hand_valid_coverage_percent": (
            100.0 * (side_valid["left"] + side_valid["right"]) / (2 * frame_total)
            if frame_total else 0.0
        ),
        "landmark_camera_count_mean": statistics.mean(observed_camera_counts) if observed_camera_counts else None,
        "capture_spread_ms": {"p50": percentile(multi_view_spread, 0.5), "p95": percentile(multi_view_spread, 0.95)},
        "limit_saturation_count": joint_clip + velocity_limit + acceleration_limit,
        "id_switch_count": annotated_switches if annotation_samples else "requires annotated real sequence",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = summarize(args.input)
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

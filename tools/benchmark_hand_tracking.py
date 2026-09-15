#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics
from typing import Any


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round((len(ordered) - 1) * q)))]


def summarize(path: Path) -> dict[str, Any]:
    inference: dict[str, list[float]] = {}
    camera_timestamps: dict[str, list[int]] = {}
    latency: list[float] = []
    spread: list[float] = []
    reprojection: list[float] = []
    fusion_residual_mm: list[float] = []
    camera_counts: list[float] = []
    valid = total = stale = spread_rejected = saturated = 0
    solver_times: list[float] = []
    residuals: list[float] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            packet = json.loads(line)
        except json.JSONDecodeError:
            continue
        hands_root = packet.get("hand_tracking") or (packet if packet.get("schema") == "zed_operator_hands_fused/v1" else None)
        if not hands_root:
            continue
        if packet.get("latency_trace_ns"):
            trace = packet["latency_trace_ns"]
            if trace.get("t2_windows_udp_send_ns") and trace.get("t0_capture_ns"):
                latency.append((trace["t2_windows_udp_send_ns"] - trace["t0_capture_ns"]) / 1e6)
        for camera in hands_root.get("per_camera", []):
            serial = str(camera.get("camera_serial"))
            camera_timestamps.setdefault(serial, []).append(int(camera.get("capture_timestamp_ns", 0)))
            for hand in camera.get("hands", []):
                if hand.get("inference_ms") is not None:
                    inference.setdefault(serial, []).append(float(hand["inference_ms"]))
        for hand in hands_root.get("hands", []):
            total += 1
            valid += int(bool(hand.get("valid")))
            spread.append(float(hand.get("capture_spread_ms", 0.0)))
            stale += sum(str(item).startswith("STALE:") for item in hand.get("rejection_reasons", []))
            spread_rejected += sum(str(item).startswith("CAPTURE_SPREAD:") for item in hand.get("rejection_reasons", []))
            for quality in hand.get("landmark_quality", []):
                camera_count = float(quality.get("camera_count", 0))
                if camera_count > 0:
                    camera_counts.append(camera_count)
                if quality.get("residual_m") is not None:
                    fusion_residual_mm.append(float(quality["residual_m"]) * 1000.0)
                if quality.get("reprojection_error_px") is not None:
                    reprojection.append(float(quality["reprojection_error_px"]))
        for target in (packet.get("dex3_targets") or {}).values():
            if not isinstance(target, dict):
                continue
            solver = target.get("solver") or {}
            if solver.get("time_ms") is not None:
                solver_times.append(float(solver["time_ms"]))
            if solver.get("residual") is not None:
                residuals.append(float(solver["residual"]))
            saturated += int(bool((target.get("safety") or {}).get("saturated")))
    return {
        "schema": "zed_hand_benchmark/v1",
        "measured_not_claimed": True,
        "per_camera_inference": {
            serial: {
                "samples": len(values), "p50_ms": percentile(values, .5),
                "p95_ms": percentile(values, .95),
                "measured_fps": (
                    (len(set(camera_timestamps.get(serial, []))) - 1) * 1e9
                    / (max(camera_timestamps[serial]) - min(camera_timestamps[serial]))
                    if len(set(camera_timestamps.get(serial, []))) > 1
                    and max(camera_timestamps[serial]) > min(camera_timestamps[serial])
                    else None
                ),
            }
            for serial, values in inference.items()
        },
        "end_to_end_latency_ms": {"p50": percentile(latency, .5), "p95": percentile(latency, .95)},
        "landmark_camera_count_mean": statistics.mean(camera_counts) if camera_counts else None,
        "reprojection_error_px": {"p50": percentile(reprojection, .5), "p95": percentile(reprojection, .95)},
        "fusion_residual_mm": {"p50": percentile(fusion_residual_mm, .5), "p95": percentile(fusion_residual_mm, .95)},
        "capture_spread_ms": {"p50": percentile(spread, .5), "p95": percentile(spread, .95)},
        "stale_rejections": stale,
        "capture_spread_rejections": spread_rejected,
        "hand_valid_coverage_percent": 100.0 * valid / total if total else 0.0,
        "solver_ms": {"p50": percentile(solver_times, .5), "p95": percentile(solver_times, .95)},
        "solver_residual": {"p50": percentile(residuals, .5), "p95": percentile(residuals, .95)},
        "limit_saturation_count": saturated,
        "id_switch_count": "requires annotated real sequence",
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

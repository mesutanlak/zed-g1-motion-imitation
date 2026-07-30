#!/usr/bin/env python3
"""Rank BODY_38 3D-analysis captures for calibration and replay.

The output is a perception-data manifest, not robot ground truth. Analysis
records must still pass through BODY_38 -> GMR -> G1 constraints before they
can become simulation references.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path
from typing import Any


def number(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def quantile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    low = int(position)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def values(rows: list[dict[str, str]], field: str) -> list[float]:
    return [
        result
        for row in rows
        if (result := number(row.get(field))) is not None
    ]


def truth(value: Any) -> bool:
    return str(value).strip().lower() == "true"


def analyze(path: Path, expected_hz: float) -> dict[str, Any]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    result: dict[str, Any] = {
        "id": path.stem.removesuffix("_summary"),
        "summary_csv": str(path.resolve()),
        "analysis_jsonl": str(
            path.with_name(path.name.replace("_summary.csv", ".jsonl")).resolve()
        ),
        "frames": len(rows),
        "status": "metadata_only" if not rows else "analyzed",
    }
    if not rows:
        result.update({"eligible": False, "role": "reject", "score": 0.0})
        return result

    timestamps = [
        int(value)
        for row in rows
        if (value := row.get("source_timestamp_ns"))
    ]
    duration_s = (
        (timestamps[-1] - timestamps[0]) / 1e9 if len(timestamps) > 1 else 0.0
    )
    effective_hz = len(rows) / duration_s if duration_s > 0 else 0.0
    confidence = values(rows, "body_confidence")
    keypoints = values(rows, "confident_keypoints")
    latency = values(rows, "capture_to_udp_ms")
    rms = values(rows, "raw_filter_rms_cm")
    speed = values(rows, "max_keypoint_speed_m_s")
    occlusion_ratio = sum(
        truth(row.get("left_arm_occluded"))
        or truth(row.get("right_arm_occluded"))
        for row in rows
    ) / len(rows)
    crossed_ratio = sum(
        truth(row.get("left_arm_crossed"))
        or truth(row.get("right_arm_crossed"))
        for row in rows
    ) / len(rows)
    spike_ratio = (
        sum(value > 5.0 for value in speed) / len(speed) if speed else 1.0
    )
    gaps = sum(
        second - first > 200_000_000
        for first, second in zip(timestamps, timestamps[1:])
    )
    confidence_mean = statistics.mean(confidence) if confidence else 0.0
    confidence_p10 = quantile(confidence, 0.10) or 0.0
    keypoints_mean = statistics.mean(keypoints) if keypoints else 0.0
    latency_p95 = quantile(latency, 0.95) or math.inf
    rms_p95 = quantile(rms, 0.95) or math.inf
    delivery_ratio = min(1.0, effective_hz / expected_hz)

    eligible = bool(
        len(rows) >= 150
        and confidence_p10 >= 85.0
        and keypoints_mean >= 26.0
        and latency_p95 <= 50.0
        and rms_p95 <= 3.0
        and spike_ratio <= 0.01
    )
    score = (
        20.0 * min(1.0, confidence_mean / 97.0)
        + 25.0 * min(1.0, keypoints_mean / 36.0)
        + 20.0 * delivery_ratio
        + 15.0 * max(0.0, 1.0 - rms_p95 / 4.0)
        + 10.0 * max(0.0, 1.0 - latency_p95 / 80.0)
        + 10.0 * max(0.0, 1.0 - 4.0 * spike_ratio)
    )
    role = (
        "occlusion_challenge"
        if eligible and (occlusion_ratio >= 0.15 or crossed_ratio >= 0.03)
        else "baseline"
        if eligible
        else "reject"
    )
    result.update(
        {
            "eligible": eligible,
            "role": role,
            "score": round(score, 3),
            "duration_s": round(duration_s, 3),
            "effective_hz": round(effective_hz, 3),
            "delivery_ratio": round(delivery_ratio, 4),
            "body_confidence_mean": round(confidence_mean, 3),
            "body_confidence_p10": round(confidence_p10, 3),
            "confident_keypoints_mean": round(keypoints_mean, 3),
            "capture_to_udp_p95_ms": round(latency_p95, 3),
            "raw_filter_rms_p95_cm": round(rms_p95, 4),
            "speed_spike_ratio": round(spike_ratio, 5),
            "arm_occlusion_ratio": round(occlusion_ratio, 5),
            "arm_crossed_ratio": round(crossed_ratio, 5),
            "timestamp_gaps_over_200ms": gaps,
        }
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "analysis_dir", type=Path, nargs="?", default=Path("analysis_recordings")
    )
    parser.add_argument("--expected-hz", type=float, default=15.0)
    parser.add_argument(
        "--output", type=Path,
        default=Path("datasets/analysis_quality_manifest.json"),
    )
    args = parser.parse_args()
    records = [
        analyze(path, args.expected_hz)
        for path in sorted(args.analysis_dir.glob("*_summary.csv"))
    ]
    records.sort(key=lambda item: float(item["score"]), reverse=True)
    manifest = {
        "schema": "zed_body38_analysis_quality_manifest/v1",
        "expected_live_hz": args.expected_hz,
        "use": (
            "Perception/filter calibration and GMR replay selection only; "
            "not direct G1 joint ground truth."
        ),
        "selected_baselines": [
            item["id"]
            for item in records
            if item["eligible"] and item["role"] == "baseline"
        ],
        "selected_occlusion_challenges": [
            item["id"]
            for item in records
            if item["eligible"] and item["role"] == "occlusion_challenge"
        ],
        "records": records,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    csv_path = args.output.with_suffix(".csv")
    fields = sorted({key for item in records for key in item})
    with csv_path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)
    print(f"records={len(records)} eligible={sum(r['eligible'] for r in records)}")
    print(f"manifest={args.output.resolve()}")
    for item in records[:5]:
        print(
            f"{item['id']} score={item['score']:.1f} "
            f"role={item['role']} frames={item['frames']} "
            f"hz={item.get('effective_hz', 0):.1f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

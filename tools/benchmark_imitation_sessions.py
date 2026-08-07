#!/usr/bin/env python3
"""Aggregate recorded ZED->GMR->Isaac imitation telemetry.

Bridge and Isaac telemetry can write two rows for the same sequence.  The
row containing measured Isaac state is preferred so aggregate metrics are not
biased by duplicate command-only samples.
"""

from __future__ import annotations

import argparse
import ast
import csv
import json
from collections import Counter
from pathlib import Path
from typing import Iterable

import numpy as np


METRICS = (
    "left_safe_error_deg",
    "right_safe_error_deg",
    "left_actual_error_deg",
    "right_actual_error_deg",
    "joint_tracking_rmse_rad",
    "body_tracking_mpjpe_m",
    "total_control_ms",
)


def finite(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if np.isfinite(number) else None


def percentile(values: Iterable[float], q: float) -> float | None:
    data = np.asarray(list(values), dtype=np.float64)
    if data.size == 0:
        return None
    return float(np.percentile(data, q))


def reasons(value: str) -> list[str]:
    if not value:
        return []
    for loader in (json.loads, ast.literal_eval):
        try:
            parsed = loader(value)
            if isinstance(parsed, (list, tuple)):
                return [str(item) for item in parsed]
        except (ValueError, SyntaxError, json.JSONDecodeError):
            pass
    return [value]


def discover(paths: list[Path]) -> list[Path]:
    found: set[Path] = set()
    for path in paths:
        if path.is_file() and path.name == "imitation_comparison.csv":
            found.add(path.resolve())
        elif path.is_dir():
            found.update(item.resolve() for item in path.rglob("imitation_comparison.csv"))
    return sorted(found)


def preferred_rows(path: Path) -> list[dict[str, str]]:
    by_sequence: dict[str, dict[str, str]] = {}
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        for row in csv.DictReader(handle):
            sequence = row.get("sequence", "")
            previous = by_sequence.get(sequence)
            score = sum(bool(row.get(name)) for name in METRICS)
            old_score = sum(bool(previous.get(name)) for name in METRICS) if previous else -1
            if score >= old_score:
                by_sequence[sequence] = row
    return list(by_sequence.values())


def summarize(path: Path) -> dict[str, object]:
    rows = preferred_rows(path)
    levels = Counter((row.get("safety_level") or "UNKNOWN") for row in rows)
    reason_counts: Counter[str] = Counter()
    values: dict[str, list[float]] = {name: [] for name in METRICS}
    joint_following_error: dict[str, list[float]] = {"left": [], "right": []}
    for row in rows:
        reason_counts.update(reasons(row.get("safety_reasons", "")))
        for name in METRICS:
            number = finite(row.get(name))
            if number is not None:
                values[name].append(number)
        for side in ("left", "right"):
            safe_angle = finite(row.get(f"{side}_safe_elbow_deg"))
            actual_angle = finite(row.get(f"{side}_actual_elbow_deg"))
            if safe_angle is not None and actual_angle is not None:
                joint_following_error[side].append(abs(actual_angle - safe_angle))

    count = len(rows)
    return {
        "session": path.parent.name,
        "file": str(path),
        "frames": count,
        "safety_level_counts": dict(levels),
        "green_ratio": levels["GREEN"] / count if count else None,
        "orange_red_ratio": (levels["ORANGE"] + levels["RED"]) / count if count else None,
        "safety_reason_counts": dict(reason_counts.most_common()),
        "metrics": {
            name: {
                "count": len(series),
                "p50": percentile(series, 50),
                "p95": percentile(series, 95),
                "max": max(series) if series else None,
            }
            for name, series in values.items()
        },
        "elbow_command_tracking_error_deg": {
            side: {
                "count": len(series),
                "p50": percentile(series, 50),
                "p95": percentile(series, 95),
                "max": max(series) if series else None,
            }
            for side, series in joint_following_error.items()
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    files = discover(args.paths)
    if not files:
        raise SystemExit("imitation_comparison.csv bulunamadi")
    sessions = [summarize(path) for path in files]
    payload = {"schema": "g1_imitation_benchmark/v1", "sessions": sessions}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    csv_path = args.output.with_suffix(".csv")
    with csv_path.open("w", encoding="utf-8-sig", newline="") as handle:
        fields = [
            "session", "frames", "green_ratio", "orange_red_ratio",
            "safe_elbow_error_p95_deg", "actual_elbow_error_p95_deg",
            "elbow_command_tracking_p95_deg", "joint_tracking_rmse_p95_rad", "body_tracking_mpjpe_p95_m",
            "total_control_p95_ms", "top_safety_reason",
        ]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for session in sessions:
            metrics = session["metrics"]
            safe = [
                value for value in (
                    metrics["left_safe_error_deg"]["p95"],
                    metrics["right_safe_error_deg"]["p95"],
                ) if value is not None
            ]
            actual = [
                value for value in (
                    metrics["left_actual_error_deg"]["p95"],
                    metrics["right_actual_error_deg"]["p95"],
                ) if value is not None
            ]
            reason_counts = session["safety_reason_counts"]
            following = session["elbow_command_tracking_error_deg"]
            following_p95 = [
                item["p95"] for item in following.values() if item["p95"] is not None
            ]
            writer.writerow({
                "session": session["session"],
                "frames": session["frames"],
                "green_ratio": session["green_ratio"],
                "orange_red_ratio": session["orange_red_ratio"],
                "safe_elbow_error_p95_deg": max(safe) if safe else None,
                "actual_elbow_error_p95_deg": max(actual) if actual else None,
                "elbow_command_tracking_p95_deg": max(following_p95) if following_p95 else None,
                "joint_tracking_rmse_p95_rad": metrics["joint_tracking_rmse_rad"]["p95"],
                "body_tracking_mpjpe_p95_m": metrics["body_tracking_mpjpe_m"]["p95"],
                "total_control_p95_ms": metrics["total_control_ms"]["p95"],
                "top_safety_reason": next(iter(reason_counts), ""),
            })
    print(f"JSON: {args.output}")
    print(f"CSV : {csv_path}")
    print(f"Oturum: {len(sessions)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

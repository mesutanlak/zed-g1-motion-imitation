#!/usr/bin/env python3
"""Fail-closed evidence gate before any physical G1 command adapter exists."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


def nested(value: dict[str, Any], *keys: str) -> Any:
    current: Any = value
    for key in keys:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    return current


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("quality_summary", type=Path)
    parser.add_argument("--charuco", type=Path)
    parser.add_argument("--planar-reference", type=Path)
    parser.add_argument("--shadow", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--phase", choices=("simulation", "physical-shadow"),
        default="simulation",
    )
    args = parser.parse_args()
    summary = json.loads(args.quality_summary.read_text(encoding="utf-8"))
    checks: list[dict[str, Any]] = []

    def check(name: str, value: Any, operation: str, limit: float) -> None:
        measured = isinstance(value, (int, float)) and math.isfinite(float(value))
        passed = bool(
            measured
            and (
                float(value) <= limit if operation == "<="
                else float(value) >= limit if operation == ">="
                else float(value) == limit
            )
        )
        checks.append({
            "name": name,
            "status": "PASS" if passed else "FAIL" if measured else "NOT_MEASURED",
            "value": value,
            "requirement": f"{operation} {limit}",
        })

    metrics = summary.get("metrics") or {}
    check("fusion_output_hz_p50", nested(metrics, "effective_output_hz", "p50"), ">=", 27.0)
    check("minimum_camera_fps_p50", nested(metrics, "camera_fps_min", "p50"), ">=", 27.0)
    check("camera_capture_spread_p95_ms", nested(metrics, "camera_timestamp_delta_ms", "p95"), "<=", 40.0)
    check("total_control_p95_ms", nested(metrics, "total_control_ms", "p95"), "<=", 100.0)
    check("gmr_solve_p95_ms", nested(metrics, "gmr_solve_ms", "p95"), "<=", 25.0)
    check("post_alignment_p95_m_p95", nested(metrics, "post_alignment_p95_m", "p95"), "<=", 0.08)
    check("joint_tracking_rmse_p95_rad", nested(metrics, "joint_tracking_rmse_rad", "p95"), "<=", 0.20)
    check("filtered_collision_margin_min_m", nested(metrics, "robot_body_barrier_safe_margin_m", "min"), ">=", -1.0e-6)
    check("red_safety_frames", (summary.get("safety_level_counts") or {}).get("RED", 0), "==", 0.0)

    transport = summary.get("transport_integrity") or {}
    for channel in ("body", "control"):
        item = transport.get(channel) or {}
        check(f"{channel}_journal_dropped", item.get("journal_dropped"), "==", 0.0)
        check(f"{channel}_sequence_gaps", item.get("sequence_gaps"), "==", 0.0)
        written = item.get("journal_written")
        decoded = item.get("decoded")
        equality = (
            int(written) - int(decoded)
            if isinstance(written, int) and isinstance(decoded, int)
            else None
        )
        check(f"{channel}_journal_written_minus_decoded", equality, "==", 0.0)

    reference_path = args.planar_reference or args.charuco
    if reference_path is not None:
        reference = json.loads(reference_path.read_text(encoding="utf-8"))
        checks.append({
            "name": "planar_reference_metric_grade",
            "status": "PASS" if reference.get("metric_grade") == "FULL_CORNER" else "FAIL",
            "value": reference.get("metric_grade"),
            "requirement": "== FULL_CORNER",
        })
        check("planar_reference_camera_count", reference.get("camera_count"), ">=", 4.0)
        check("planar_reference_translation_p95_m", nested(reference, "translation_error_m", "p95"), "<=", 0.03)
        check("planar_reference_rotation_p95_deg", nested(reference, "rotation_error_deg", "p95"), "<=", 2.0)
        check("planar_reference_reprojection_p95_px", nested(reference, "reprojection_rms_px", "p95"), "<=", 2.0)
    elif args.phase == "physical-shadow":
        checks.append({"name": "planar_reference", "status": "NOT_MEASURED", "value": None, "requirement": "required"})

    if args.shadow is not None:
        ages: list[float] = []
        fresh: list[bool] = []
        publishing_flags: list[bool] = []
        with args.shadow.open(encoding="utf-8") as stream:
            for line in stream:
                try:
                    item = json.loads(line)
                except json.JSONDecodeError:
                    continue
                age = item.get("target_age_ms")
                if isinstance(age, (int, float)) and math.isfinite(float(age)):
                    ages.append(float(age))
                fresh.append(bool(item.get("target_fresh")))
                publishing_flags.append(bool(item.get("command_publishing_enabled")))
        check("shadow_samples", len(ages), ">=", 1000.0)
        check("shadow_target_age_p95_ms", float(np.percentile(ages, 95)) if ages else None, "<=", 150.0)
        check("shadow_fresh_ratio", float(np.mean(fresh)) if fresh else None, ">=", 0.99)
        check("shadow_command_publish_true_count", sum(publishing_flags), "==", 0.0)
    elif args.phase == "physical-shadow":
        checks.append({"name": "robot_shadow_recording", "status": "NOT_MEASURED", "value": None, "requirement": "required"})

    failed = [item for item in checks if item["status"] != "PASS"]
    report = {
        "schema": "g1_physical_readiness_gate/v1",
        "phase": args.phase,
        "ready": not failed,
        "checks": checks,
        "failed_or_missing": [item["name"] for item in failed],
        "note": "PASS does not enable motor commands; e-stop, deadman, harness and Unitree service-state validation remain mandatory.",
    }
    rendered = json.dumps(report, indent=2, ensure_ascii=False)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0 if report["ready"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

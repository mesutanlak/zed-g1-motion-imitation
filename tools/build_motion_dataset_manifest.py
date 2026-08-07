#!/usr/bin/env python3
"""Build leakage-safe train/validation/test splits from capture benchmarks.

Splits are assigned per complete recording, never per adjacent frame, so the
test set cannot contain near-duplicates from a training trajectory.  Torso
overlap and other difficult but valid recordings are explicitly tagged for a
separate challenge-set evaluation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def exclusion_reasons(item: dict) -> list[str]:
    reasons: list[str] = []
    if int(item.get("frames") or 0) < 300:
        reasons.append("too_short")
    if item.get("quality_level") == "RED":
        reasons.append("capture_red")
    if float(item.get("core_visible_ratio_mean") or 0.0) < 0.97:
        reasons.append("core_visibility_below_97pct")
    if float(item.get("bone_cv_p95") or 1.0) > 0.05:
        reasons.append("bone_length_cv_above_5pct")
    if int(item.get("body_id_switches") or 0) > 0:
        reasons.append("body_id_switch")
    locked = item.get("locked_ratio")
    if locked is not None and float(locked) < 0.95:
        reasons.append("operator_lock_below_95pct")
    return reasons


def split_for(name: str, train: float, validation: float) -> str:
    value = int(hashlib.sha256(name.encode("utf-8")).hexdigest()[:8], 16) / 0xFFFFFFFF
    if value < train:
        return "train"
    if value < train + validation:
        return "validation"
    return "test"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("benchmark", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--train-ratio", type=float, default=0.75)
    parser.add_argument("--validation-ratio", type=float, default=0.15)
    args = parser.parse_args()
    if not 0 < args.train_ratio < 1:
        raise SystemExit("train ratio 0..1 arasinda olmali")
    if not 0 <= args.validation_ratio < 1 - args.train_ratio:
        raise SystemExit("validation ratio gecersiz")

    source = json.loads(args.benchmark.read_text(encoding="utf-8"))
    accepted: list[dict] = []
    excluded: list[dict] = []
    for item in source.get("results", []):
        recording = Path(item["recording"])
        reject = exclusion_reasons(item)
        record = {
            "recording": str(recording),
            "frames": item.get("frames"),
            "effective_fps": item.get("effective_fps"),
            "core_visible_ratio": item.get("core_visible_ratio_mean"),
            "bone_cv_p95": item.get("bone_cv_p95"),
            "torso_overlap_ratio": item.get("torso_overlap_ratio"),
            "quality_level": item.get("quality_level"),
            "quality_reasons": item.get("quality_reasons", []),
        }
        if reject:
            record["excluded_reasons"] = reject
            excluded.append(record)
            continue
        record["split"] = split_for(
            recording.name, args.train_ratio, args.validation_ratio
        )
        record["challenge_tags"] = [
            tag for condition, tag in (
                (float(item.get("torso_overlap_ratio") or 0.0) >= 0.15, "torso_overlap"),
                (int(item.get("teleport_events") or 0) > 0, "source_outlier"),
                (int(item.get("sequence_gap_frames") or 0) > 0, "packet_gap"),
            ) if condition
        ]
        accepted.append(record)

    counts = {name: sum(row["split"] == name for row in accepted) for name in ("train", "validation", "test")}
    payload = {
        "schema": "zed_g1_motion_dataset_manifest/v1",
        "source_benchmark": str(args.benchmark),
        "split_policy": "sha256_by_complete_recording",
        "ratios": {"train": args.train_ratio, "validation": args.validation_ratio, "test": 1.0 - args.train_ratio - args.validation_ratio},
        "counts": {**counts, "excluded": len(excluded)},
        "accepted": accepted,
        "excluded": excluded,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Manifest: {args.output}")
    print(f"train={counts['train']} validation={counts['validation']} test={counts['test']} excluded={len(excluded)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

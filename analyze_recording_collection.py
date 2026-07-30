#!/usr/bin/env python3
"""Build a compact index of every ZED BODY_38 JSONL recording."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

from analyze_body38_recording import analyze, parse_recording


def value_at(data: dict[str, Any], *keys: str, default: Any = None) -> Any:
    current: Any = data
    for key in keys:
        if not isinstance(current, dict):
            return default
        current = current.get(key)
    return default if current is None else current


def main() -> int:
    parser = argparse.ArgumentParser(
        description="recordings dizinindeki tüm BODY_38 kayıtlarını karşılaştırır."
    )
    parser.add_argument("recordings_dir", type=Path, nargs="?", default=Path("recordings"))
    parser.add_argument("--confidence", type=float, default=50.0)
    args = parser.parse_args()

    rows: list[dict[str, Any]] = []
    for path in sorted(args.recordings_dir.glob("zed_body38_*.jsonl")):
        row: dict[str, Any] = {
            "file": path.name,
            "jsonl_bytes": path.stat().st_size,
            "status": "invalid",
        }
        try:
            metadata, frames, invalid = parse_recording(path)
            if not frames:
                row["status"] = "metadata_only"
                rows.append(row)
                continue
            summary, _ = analyze(metadata, frames, invalid, args.confidence)
            declared_svo = value_at(summary, "native_svo2", "declared_path")
            svo_path = Path(declared_svo) if declared_svo else path.with_suffix(".svo2")
            row.update(
                {
                    "status": "analyzed",
                    "classification": value_at(summary, "classification", "label"),
                    "frames": value_at(summary, "recording", "frame_count"),
                    "duration_s": value_at(summary, "recording", "duration_s"),
                    "wall_clock_span_s": value_at(
                        summary, "recording", "wall_clock_span_s"
                    ),
                    "segment_count": value_at(
                        summary, "recording", "segment_count"
                    ),
                    "fps": value_at(summary, "recording", "effective_fps"),
                    "dropped_frames": value_at(
                        summary, "recording", "estimated_dropped_frames"
                    ),
                    "body_confidence_mean": value_at(
                        summary, "body_confidence", "mean"
                    ),
                    "distance_mean_m": value_at(summary, "distance_m", "mean"),
                    "distance_inside_2_4m_percent": value_at(
                        summary,
                        "distance_quality",
                        "inside_recommended_range_percent",
                    ),
                    "upper_body_percent": value_at(
                        summary,
                        "classification",
                        "upper_body_observability_percent",
                    ),
                    "whole_body_core_percent": value_at(
                        summary,
                        "classification",
                        "whole_body_core_observability_percent",
                    ),
                    "left_dex3_source_percent": value_at(
                        summary, "dex3_1", "left_source_observability_percent"
                    ),
                    "right_dex3_source_percent": value_at(
                        summary, "dex3_1", "right_source_observability_percent"
                    ),
                    "imu_coverage_percent": value_at(
                        summary, "imu", "coverage_percent", default=0.0
                    ),
                    "svo2_exists": svo_path.is_file(),
                    "svo2_bytes": svo_path.stat().st_size if svo_path.is_file() else 0,
                }
            )
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            row["error"] = str(exc)
        rows.append(row)

    output_dir = args.recordings_dir / "analysis"
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "collection_analysis.json"
    csv_path = output_dir / "collection_analysis.csv"
    ranked = sorted(
        rows,
        key=lambda row: (
            row.get("classification") == "whole_body_candidate",
            float(row.get("whole_body_core_percent") or 0.0),
            float(row.get("duration_s") or 0.0),
        ),
        reverse=True,
    )
    json_path.write_text(
        json.dumps(
            {
                "schema": "zed_body38_collection_analysis/v1",
                "confidence_threshold": args.confidence,
                "recommended_baseline": next(
                    (
                        row["file"]
                        for row in ranked
                        if row.get("classification") == "whole_body_candidate"
                    ),
                    None,
                ),
                "recordings": ranked,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    fields = sorted({key for row in rows for key in row})
    with csv_path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(ranked)
    print(f"Kayıt sayısı: {len(rows)}")
    print(f"Önerilen başlangıç: {ranked[0].get('file') if ranked else 'yok'}")
    print(f"Özet: {json_path}")
    print(f"Tablo: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

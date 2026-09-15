#!/usr/bin/env python3
"""Replay/version-convert hand JSONL; output is simulation/visualization only."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from hand_tracking.contracts import finite_or_none
from hand_tracking.rerun_output import RerunHandLogger


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="El JSONL kaydini fiziksel robot cikisi olmadan replay et")
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--rerun", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--landmark-only", action="store_true")
    parser.add_argument("--simulation", action="store_true", help="Safe Dex3 hedeflerini sim adaptoru icin koru")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.input.is_file():
        print(f"Kayit bulunamadi: {args.input}", file=sys.stderr)
        return 2
    logger = RerunHandLogger() if args.rerun else None
    output = args.output.open("w", encoding="utf-8") if args.output else None
    previous_ns = None
    frames = 0
    try:
        for raw_line in args.input.read_text(encoding="utf-8").splitlines():
            try:
                packet = json.loads(raw_line)
            except json.JSONDecodeError:
                continue
            if "hand_tracking" not in packet and packet.get("schema") not in ("zed_operator_hand/v1", "zed_operator_hands_fused/v1"):
                continue
            timestamp_ns = int(packet.get("timestamp_ns") or packet.get("capture_timestamp_ns") or 0)
            if args.realtime and previous_ns is not None and timestamp_ns > previous_ns:
                time.sleep(min((timestamp_ns - previous_ns) / 1e9, 0.2))
            previous_ns = timestamp_ns
            packet["replay"] = {
                "mode": "simulation" if args.simulation else "landmark_only",
                "physical_robot_output_enabled": False,
            }
            if args.landmark_only:
                packet.pop("dex3_targets", None)
            if logger:
                logger.log(packet)
            if output:
                output.write(json.dumps(finite_or_none(packet), ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n")
            frames += 1
    finally:
        if output:
            output.close()
    print(f"Replay tamamlandi: {frames} el karesi | fiziksel robot cikisi KAPALI")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

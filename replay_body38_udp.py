#!/usr/bin/env python3
"""Replay a recorded BODY_38 JSONL over the live UDP protocol."""

from __future__ import annotations

import argparse
import json
import socket
import time
from pathlib import Path
from typing import Any


def packet_from_record(record: dict[str, Any]) -> dict[str, Any]:
    if record.get("schema") == "zed_body38_live/v1":
        # Analysis recordings preserve the complete original live packet.
        # Copy it instead of rebuilding a lossy subset.
        return dict(record)
    return {
        "schema": "zed_body38_live/v1",
        "sequence": record.get("frame_index"),
        "timestamp_ns": record.get("timestamp_ns"),
        "coordinate_system": record.get("coordinate_system"),
        "units": record.get("units"),
        "body_id": record.get("body_id"),
        "body_confidence": record.get("body_confidence"),
        "root_position_m": record.get("root_position_m"),
        "global_root_orientation_xyzw": record.get(
            "global_root_orientation_xyzw"
        ),
        "keypoint_names": record.get("keypoint_names"),
        "keypoints_3d_m": record.get("keypoints_3d_filtered_m"),
        "keypoint_confidence": record.get("keypoint_confidence"),
        "local_orientation_per_joint_xyzw": record.get(
            "local_orientation_per_joint_xyzw"
        ),
        "reference_ready": {
            "upper_body": record.get("g1_reference_features", {}).get(
                "upper_body_reference_ready", False
            ),
            "whole_body": record.get("g1_reference_features", {}).get(
                "whole_body_reference_ready", False
            ),
        },
        "g1_reference_features": record.get("g1_reference_features", {}),
    }


def load_frames(path: Path) -> list[dict[str, Any]]:
    frames: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            item = json.loads(line)
            if item.get("schema") == "zed_body38_g1_reference/v1":
                frames.append(item)
            elif item.get("schema") == "zed_body38_live/v1":
                frames.append(item)
            elif item.get("schema") == "zed_body38_3d_analysis/frame/v1":
                source = item.get("source")
                if (
                    isinstance(source, dict)
                    and source.get("schema") == "zed_body38_live/v1"
                ):
                    frames.append(source)
    if not frames:
        raise ValueError("Kayıtta BODY_38 karesi yok.")
    return frames


def main() -> int:
    parser = argparse.ArgumentParser(
        description="BODY_38 JSONL kaydını MuJoCo canlı alıcısına yollar."
    )
    parser.add_argument("recording", type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=15050)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    args = parser.parse_args()
    if not args.recording.is_file() or args.fps <= 0:
        print("Kayıt yolu veya FPS geçersiz.")
        return 2

    frames = load_frames(args.recording)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.host, args.port)
    frame_period = 1.0 / args.fps
    sent = 0
    print(f"UDP replay: {target[0]}:{target[1]} | {len(frames)} kare")
    try:
        while True:
            next_deadline = time.perf_counter()
            for frame in frames:
                payload = json.dumps(
                    packet_from_record(frame),
                    ensure_ascii=False,
                    allow_nan=False,
                    separators=(",", ":"),
                ).encode("utf-8")
                if len(payload) > 60000:
                    raise ValueError(f"UDP paketi çok büyük: {len(payload)} bayt")
                sock.sendto(payload, target)
                sent += 1
                if args.max_frames and sent >= args.max_frames:
                    print(f"Gönderilen kare: {sent}")
                    return 0
                next_deadline += frame_period
                time.sleep(max(0.0, next_deadline - time.perf_counter()))
            if not args.loop:
                break
    finally:
        sock.close()
    print(f"Gönderilen kare: {sent}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

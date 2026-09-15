#!/usr/bin/env python3
"""Re-fuse recorded per-camera hands with the current DDS-free pipeline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from hand_tracking.contracts import finite_or_none
from hand_tracking.fusion import CameraPose, fuse_hand_packets
from hand_tracking.normalization import PalmNormalizer
from hand_tracking.retargeting import Dex3Retargeter, dex3_control_contract
from zed_four_camera_test.distributed_body38_fusion import research_record_packet


def _poses(header: dict) -> dict[int, CameraPose]:
    cameras = ((header.get("extrinsics") or {}).get("cameras") or {})
    return {
        int(serial): CameraPose(
            np.asarray(value["rotation_camera_to_world"], dtype=float),
            np.asarray(value["translation_camera_to_world_m"], dtype=float),
        )
        for serial, value in cameras.items()
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Kayitli el goruslerini guncel geometriyle yeniden fuse et")
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--max-age-ms", type=float, default=120.0)
    parser.add_argument("--max-spread-ms", type=float, default=70.0)
    parser.add_argument("--dex3-config", type=Path, default=ROOT / "config" / "g1_23dof_dex3.json")
    args = parser.parse_args()
    normalizer = PalmNormalizer()
    retargeter = Dex3Retargeter(args.dex3_config)
    frames = valid_left = valid_right = 0
    poses: dict[int, CameraPose] = {}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.input.open(encoding="utf-8") as source, args.output.open("w", encoding="utf-8") as target:
        for line in source:
            try:
                packet = json.loads(line)
            except json.JSONDecodeError:
                continue
            if packet.get("schema", "").endswith("metadata/v1"):
                poses = _poses(packet)
                packet["reprocessing"] = {
                    "schema": "zed_hand_refusion/v1",
                    "max_age_ms": args.max_age_ms,
                    "max_spread_ms": args.max_spread_ms,
                    "physical_robot_output_enabled": False,
                }
                target.write(json.dumps(finite_or_none(packet), ensure_ascii=False, separators=(",", ":")) + "\n")
                continue
            hand_root = packet.get("hand_tracking") or {}
            per_camera = hand_root.get("per_camera") or []
            if not per_camera or not poses:
                continue
            timestamp_ns = int(packet.get("timestamp_ns") or hand_root.get("capture_timestamp_ns") or 0)
            refused = fuse_hand_packets(
                per_camera, poses, target_timestamp_ns=timestamp_ns,
                maximum_age_ms=args.max_age_ms,
                maximum_capture_spread_ms=args.max_spread_ms,
                single_view_depth=True,
            )
            refused["per_camera"] = per_camera
            targets = {"physical_robot_output_enabled": False}
            for hand in refused["hands"]:
                side = hand["side"]
                normalized = None
                confidence = 0.0
                if hand.get("valid"):
                    try:
                        normalized_hand = normalizer.normalize(hand["landmarks_world_m"], side)
                        normalized = normalized_hand.landmarks
                        confidence = float(np.mean([item["confidence"] for item in hand["landmark_quality"]]))
                        hand["normalization"] = {
                            "wrist_world_m": normalized_hand.wrist_world_m.tolist(),
                            "rotation_world_from_palm": normalized_hand.rotation_world_from_palm.tolist(),
                            "scale_m": normalized_hand.scale_m,
                        }
                    except ValueError as exc:
                        hand["valid"] = False
                        hand.setdefault("rejection_reasons", []).append(str(exc))
                targets[side] = retargeter.update(side, normalized, timestamp_ns, confidence)
                if side == "left": valid_left += int(bool(hand.get("valid")))
                else:
                    valid_right += int(bool(hand.get("valid")))
            packet["hand_tracking"] = refused
            packet["dex3_targets"] = targets
            packet["dex3_control"] = dex3_control_contract(timestamp_ns, targets["left"], targets["right"])
            packet["reprocessing"] = {"schema": "zed_hand_refusion/v1", "physical_robot_output_enabled": False}
            target.write(json.dumps(finite_or_none(research_record_packet(packet)), ensure_ascii=False, separators=(",", ":")) + "\n")
            frames += 1
    print(
        f"REFUSION_OK frames={frames} left={valid_left} right={valid_right} "
        "physical_robot_output=FALSE"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

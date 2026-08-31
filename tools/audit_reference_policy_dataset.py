"""Audit policy NPZ clips for lower-body motion and arm self-clearance."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from motion_pipeline.reference_policy import (
    LOWER_POLICY_JOINTS,
    POLICY_COLLISION_CONSTRAINTS,
)


def _strings(values) -> list[str]:
    return [
        value.decode("utf-8") if isinstance(value, bytes) else str(value)
        for value in np.asarray(values).reshape(-1)
    ]


def audit_clip(path: Path) -> dict:
    data = np.load(path, allow_pickle=False)
    joint_names = _strings(data["joint_names"])
    body_names = _strings(data["body_names"])
    joint_pos = np.asarray(data["joint_pos"], dtype=float)
    joint_vel = np.asarray(data["joint_vel"], dtype=float)
    body_pos = np.asarray(data["body_pos_w"], dtype=float)
    lower_ids = [joint_names.index(name) for name in LOWER_POLICY_JOINTS]
    lower_pos = joint_pos[:, lower_ids]
    lower_vel = joint_vel[:, lower_ids]
    collision = {}
    unsafe_any = np.zeros(len(joint_pos), dtype=bool)
    severe_any = np.zeros(len(joint_pos), dtype=bool)
    for constraint in POLICY_COLLISION_CONSTRAINTS:
        first = body_names.index(str(constraint["first"]))
        second = body_names.index(str(constraint["second"]))
        distance = np.linalg.norm(body_pos[:, first] - body_pos[:, second], axis=-1)
        hard = float(constraint["hard_margin_m"])
        unsafe = distance < hard
        severe = distance < 0.65 * hard
        unsafe_any |= unsafe
        severe_any |= severe
        collision[str(constraint["name"])] = {
            "minimum_m": float(distance.min()),
            "p01_m": float(np.quantile(distance, 0.01)),
            "below_margin_rate": float(unsafe.mean()),
            "severe_rate": float(severe.mean()),
        }
    lower_range = np.ptp(lower_pos, axis=0)
    lower_speed = np.abs(lower_vel)
    return {
        "path": str(path.resolve()),
        "frames": int(len(joint_pos)),
        "lower_body": {
            "max_joint_range_rad": float(lower_range.max()),
            "moving_joint_count_over_0p02_rad": int(np.sum(lower_range > 0.02)),
            "velocity_p95_rad_s": float(np.quantile(lower_speed, 0.95)),
            "velocity_max_rad_s": float(lower_speed.max()),
        },
        "collision": collision,
        "unsafe_frame_rate": float(unsafe_any.mean()),
        "severe_frame_rate": float(severe_any.mean()),
    }


def audit_manifest(path: Path) -> dict:
    manifest = json.loads(path.read_text(encoding="utf-8-sig"))
    clips = []
    for item in manifest["clips"]:
        clip_path = Path(str(item["reference_npz"]))
        if not clip_path.is_absolute():
            clip_path = (path.parent / clip_path).resolve()
        result = audit_clip(clip_path)
        result["id"] = str(item["id"])
        result["session_id"] = item.get("session_id")
        result["split"] = (
            "train" if item["id"] in set(manifest["splits"]["train"]) else "validation"
        )
        clips.append(result)
    total_frames = sum(item["frames"] for item in clips)
    weighted = lambda name: sum(item["frames"] * item[name] for item in clips) / max(total_frames, 1)
    return {
        "schema": "g1_reference_policy_dataset_audit/v1",
        "manifest": str(path.resolve()),
        "summary": {
            "clip_count": len(clips),
            "frames": total_frames,
            "unsafe_frame_rate": weighted("unsafe_frame_rate"),
            "severe_frame_rate": weighted("severe_frame_rate"),
            "clips_with_lower_body_motion": sum(
                item["lower_body"]["moving_joint_count_over_0p02_rad"] > 0
                for item in clips
            ),
            "clips_with_severe_clearance": sum(item["severe_frame_rate"] > 0 for item in clips),
        },
        "clips": clips,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest", type=Path)
    args = parser.parse_args()
    print(json.dumps(audit_manifest(args.manifest), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

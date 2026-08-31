from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from motion_pipeline.reference_policy import LOWER_POLICY_JOINTS, TRACKED_BODIES

from tools.update_reference_motion_catalog import build_catalog


def _session(root: Path, name: str, quality: float) -> None:
    clips = root / name / "clips"
    clips.mkdir(parents=True)
    entries = []
    for index in range(2):
        clip_id = f"{name}_clip_{index:03d}"
        npz = clips / f"{clip_id}.npz"
        joint_names = list(LOWER_POLICY_JOINTS) + [
            "waist_yaw_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
        ]
        body = np.zeros((3, len(TRACKED_BODIES), 3), dtype=np.float32)
        positions = {
            "pelvis": (0.0, 0.0, 0.0),
            "torso_link": (0.0, 0.0, 0.4),
            "left_shoulder_pitch_link": (0.0, 0.25, 0.55),
            "left_elbow_link": (0.0, 0.50, 0.50),
            "left_wrist_roll_rubber_hand": (0.0, 0.75, 0.45),
            "right_shoulder_pitch_link": (0.0, -0.25, 0.55),
            "right_elbow_link": (0.0, -0.50, 0.50),
            "right_wrist_roll_rubber_hand": (0.0, -0.75, 0.45),
        }
        for body_index, body_name in enumerate(TRACKED_BODIES):
            body[:, body_index] = positions[body_name]
        np.savez_compressed(
            npz,
            joint_names=np.asarray(joint_names),
            body_names=np.asarray(TRACKED_BODIES),
            joint_pos=np.zeros((3, len(joint_names)), dtype=np.float32),
            joint_vel=np.zeros((3, len(joint_names)), dtype=np.float32),
            body_pos_w=body,
        )
        entries.append(
            {
                "id": clip_id,
                "reference_npz": str(npz),
                "duration_s": 2.0,
                "quality_score": quality,
            }
        )
    manifest = {
        "schema": "g1_reference_motion_clips/v2",
        "source_jsonl": str(root / f"zed_dual_body38_{name}.jsonl"),
        "clips": entries,
        "splits": {"train": [entries[0]["id"]], "validation": [entries[1]["id"]]},
    }
    (clips / "clips_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")


def test_catalog_keeps_old_sessions_and_stable_holdout(tmp_path):
    _session(tmp_path, "20260824_100000", 0.8)
    _session(tmp_path, "20260826_100000", 0.9)
    output = tmp_path / "multi_session_manifest.json"
    catalog = build_catalog(tmp_path, output)
    assert catalog["summary"]["session_count"] == 2
    assert catalog["summary"]["train_session_ids"] == ["zed_dual_body38_20260824_100000"]
    assert catalog["summary"]["validation_session_ids"] == ["zed_dual_body38_20260826_100000"]
    assert set(catalog["splits"]["train"]).isdisjoint(catalog["splits"]["validation"])
    assert output.is_file()
    assert catalog["summary"]["excluded_policy_clip_count"] == 0

    # A newer recording must immediately enter training; the prior validation
    # session remains untouched for comparable, leakage-free evaluation.
    _session(tmp_path, "20260827_100000", 0.95)
    updated = build_catalog(tmp_path, output)
    assert updated["summary"]["validation_session_ids"] == [
        "zed_dual_body38_20260826_100000"
    ]
    assert "zed_dual_body38_20260827_100000" in updated["summary"][
        "train_session_ids"
    ]
    assert updated["split_strategy"]["name"] == "stable_session_holdout"

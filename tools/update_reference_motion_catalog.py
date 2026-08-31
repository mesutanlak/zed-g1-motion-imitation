"""Build the persistent, leakage-safe multi-session policy dataset catalog."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.audit_reference_policy_dataset import audit_clip


SCHEMA = "g1_reference_motion_multisession/v1"
SESSION_SCHEMA = "g1_reference_motion_clips/v2"
MAX_SEVERE_COLLISION_FRAME_RATE = 0.01


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _session_id(manifest_path: Path, manifest: dict) -> str:
    source = Path(str(manifest.get("source_jsonl", ""))).stem
    return source or manifest_path.parent.parent.name


def build_catalog(dataset_root: Path, output: Path) -> dict:
    dataset_root = dataset_root.expanduser().resolve()
    output = output.expanduser().resolve()
    # Keep an established held-out session stable when new recordings arrive.
    # Otherwise today's recording would immediately become validation-only and
    # would not contribute to continual learning until yet another session was
    # prepared.  A stable holdout also makes policy reports comparable over
    # time while every subsequent recording is added to the train pool.
    previous_validation_ids: list[str] = []
    if output.is_file():
        try:
            previous = json.loads(output.read_text(encoding="utf-8-sig"))
            if previous.get("schema") == SCHEMA:
                previous_validation_ids = [
                    str(value)
                    for value in (
                        (previous.get("split_strategy") or {}).get(
                            "validation_session_ids", []
                        )
                    )
                ]
        except (OSError, ValueError, TypeError):
            previous_validation_ids = []
    sessions: list[dict] = []
    all_clips: list[dict] = []
    seen_ids: set[str] = set()

    for path in sorted(dataset_root.glob("*/clips/clips_manifest.json")):
        # Never ingest the generated catalog itself, and never publish a
        # session until every referenced policy NPZ is present.
        manifest = json.loads(path.read_text(encoding="utf-8-sig"))
        if manifest.get("schema") != SESSION_SCHEMA:
            continue
        raw_clips = manifest.get("clips")
        if not isinstance(raw_clips, list) or not raw_clips:
            continue
        session_id = _session_id(path, manifest)
        session_clips: list[dict] = []
        complete = True
        for raw in raw_clips:
            npz = Path(str(raw.get("reference_npz", "")))
            if not npz.is_absolute():
                npz = (path.parent / npz).resolve()
            if not npz.is_file():
                complete = False
                break
            local_id = str(raw.get("id", npz.stem))
            clip_id = f"{session_id}::{local_id}"
            if clip_id in seen_ids:
                raise ValueError(f"duplicate catalog clip id: {clip_id}")
            clip = dict(raw)
            audit = audit_clip(npz)
            severe_rate = float(audit["severe_frame_rate"])
            exclusion_reasons = []
            if severe_rate > MAX_SEVERE_COLLISION_FRAME_RATE:
                exclusion_reasons.append("SEVERE_ARM_SELF_CLEARANCE")
            clip.update(
                {
                    "id": clip_id,
                    "session_id": session_id,
                    "session_clip_id": local_id,
                    "reference_npz": str(npz),
                    "reference_npz_sha256": _sha256(npz),
                    "policy_training_eligible": not exclusion_reasons,
                    "policy_exclusion_reasons": exclusion_reasons,
                    "policy_quality": {
                        "unsafe_frame_rate": float(audit["unsafe_frame_rate"]),
                        "severe_frame_rate": severe_rate,
                        "lower_body_reference_ignored": True,
                        "lower_body_max_joint_range_rad": float(
                            audit["lower_body"]["max_joint_range_rad"]
                        ),
                    },
                }
            )
            session_clips.append(clip)
        if not complete:
            continue
        seen_ids.update(clip["id"] for clip in session_clips)
        all_clips.extend(session_clips)
        sessions.append(
            {
                "id": session_id,
                "manifest": str(path.resolve()),
                "manifest_sha256": _sha256(path),
                "source_jsonl": manifest.get("source_jsonl"),
                "clip_ids": [clip["id"] for clip in session_clips],
                "clip_count": len(session_clips),
                "eligible_clip_count": sum(
                    bool(clip["policy_training_eligible"])
                    for clip in session_clips
                ),
                "duration_s": float(sum(float(clip.get("duration_s", 0.0)) for clip in session_clips)),
            }
        )

    if not sessions:
        raise ValueError(f"no complete {SESSION_SCHEMA} sessions found under {dataset_root}")

    # Output directories are often short clock labels (114253, 150201), so
    # path ordering is not chronological across days. BODY_38 session ids
    # contain YYYYMMDD_HHMMSS and provide a deterministic acquisition order.
    sessions.sort(key=lambda session: session["id"])
    eligible_ids = {
        clip["id"]
        for clip in all_clips
        if bool(clip.get("policy_training_eligible"))
    }

    # Validation is session-disjoint once two recordings exist.  The first
    # catalog build holds out the newest complete recording. Later rebuilds
    # preserve that same validation session so newly prepared data immediately
    # strengthens training instead of being validation-only.
    validation_session_ids: list[str]
    if len(sessions) >= 2:
        session_by_id = {session["id"]: session for session in sessions}
        validation_session_ids = [
            session_id
            for session_id in previous_validation_ids
            if session_id in session_by_id
        ]
        if not validation_session_ids or len(validation_session_ids) == len(sessions):
            validation_session_ids = [sessions[-1]["id"]]
        validation = [
            clip_id
            for session_id in validation_session_ids
            for clip_id in session_by_id[session_id]["clip_ids"]
            if clip_id in eligible_ids
        ]
        train = [
            clip_id
            for session in sessions
            if session["id"] not in set(validation_session_ids)
            for clip_id in session["clip_ids"]
            if clip_id in eligible_ids
        ]
        split_name = "stable_session_holdout"
    else:
        session_manifest = json.loads(
            Path(sessions[0]["manifest"]).read_text(encoding="utf-8-sig")
        )
        local_split = session_manifest["splits"]
        prefix = sessions[0]["id"] + "::"
        train = [
            prefix + str(clip_id)
            for clip_id in local_split["train"]
            if prefix + str(clip_id) in eligible_ids
        ]
        validation = [
            prefix + str(clip_id)
            for clip_id in local_split["validation"]
            if prefix + str(clip_id) in eligible_ids
        ]
        validation_session_ids = [sessions[0]["id"]]
        split_name = "single_session_cleanest_clip_holdout"

    if not train or not validation or set(train).intersection(validation):
        raise ValueError("catalog must have non-empty, non-overlapping splits")

    catalog = {
        "schema": SCHEMA,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "dataset_root": str(dataset_root),
        "clips": all_clips,
        "sessions": sessions,
        "splits": {
            "train": train,
            "validation": validation,
            # Never sampled by the imitation actor. Retained so future safety
            # classifiers/regression tests can use the real failure examples.
            "safety_negative": sorted(
                clip["id"] for clip in all_clips if clip["id"] not in eligible_ids
            ),
        },
        "split_strategy": {
            "name": split_name,
            "session_disjoint": len(sessions) >= 2,
            "validation_session_ids": validation_session_ids,
            "concatenate_clips": False,
            "environment_sampling": "balanced_per_environment",
            "policy_quality_gate": {
                "max_severe_collision_frame_rate": MAX_SEVERE_COLLISION_FRAME_RATE,
                "excluded_clips_are_retained_as_safety_negatives": True,
            },
        },
        "summary": {
            "session_count": len(sessions),
            "clip_count": len(all_clips),
            "train_clip_count": len(train),
            "validation_clip_count": len(validation),
            "excluded_policy_clip_count": len(all_clips) - len(eligible_ids),
            "excluded_policy_clip_ids": sorted(
                clip["id"] for clip in all_clips if clip["id"] not in eligible_ids
            ),
            "train_session_ids": sorted(
                {clip["session_id"] for clip in all_clips if clip["id"] in set(train)}
            ),
            "validation_session_ids": validation_session_ids,
        },
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(catalog, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    temporary.replace(output)
    return catalog


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    catalog = build_catalog(args.dataset_root, args.output)
    print(
        json.dumps(
            {
                "catalog": str(args.output.resolve()),
                **catalog["summary"],
                "split_strategy": catalog["split_strategy"]["name"],
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

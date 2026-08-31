"""Revalidate and atomically promote an already exported policy candidate.

This deliberately reuses the recorded held-out rollout metrics.  It is meant
for acceptance-gate corrections, not for changing the model or pretending a
new validation rollout occurred.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import shutil
import sys
import uuid
from datetime import datetime, timezone


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from reference_policy.evaluation_report import (  # noqa: E402
    ACCEPTANCE_GATE_VERSION,
    build_evaluation_report,
    report_markdown,
)


def _sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_copy(source: pathlib.Path, destination: pathlib.Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".{destination.name}.{uuid.uuid4().hex}.tmp")
    shutil.copy2(source, temporary)
    os.replace(temporary, destination)


def _load_json(path: pathlib.Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8-sig"))


def _write_json(path: pathlib.Path, value: dict) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("candidate_dir", type=pathlib.Path)
    parser.add_argument(
        "--stable-dir",
        type=pathlib.Path,
        default=REPO_ROOT / "policies" / "g1_reference_upper_body",
    )
    args = parser.parse_args()

    candidate_dir = args.candidate_dir.expanduser().resolve()
    stable_dir = args.stable_dir.expanduser().resolve()
    source_evaluation_path = candidate_dir / "policy_evaluation.json"
    source_metadata_path = candidate_dir / "policy_metadata.json"
    onnx_path = candidate_dir / "policy.onnx"
    sweep_path = candidate_dir / "checkpoint_sweep.json"
    for required in (source_evaluation_path, source_metadata_path, onnx_path, sweep_path):
        if not required.is_file():
            raise FileNotFoundError(required)

    previous = _load_json(source_evaluation_path)
    metadata = _load_json(source_metadata_path)
    checkpoint_name = str(previous["checkpoint"]["name"])
    checkpoint_path = pathlib.Path(previous["training_run"]) / checkpoint_name
    if not checkpoint_path.is_file():
        raise FileNotFoundError(checkpoint_path)

    provenance = metadata.get("provenance", {})
    expected_onnx = provenance.get("onnx_sha256")
    expected_checkpoint = provenance.get("checkpoint_sha256")
    if expected_onnx and _sha256(onnx_path) != expected_onnx:
        raise RuntimeError("candidate ONNX hash does not match policy metadata")
    if expected_checkpoint and _sha256(checkpoint_path) != expected_checkpoint:
        raise RuntimeError("selected checkpoint hash does not match policy metadata")

    evaluation = build_evaluation_report(
        manifest_path=previous["dataset_manifest"],
        train_clip_ids=list(previous["train_clip_ids"]),
        validation_clip_ids=list(previous["validation_clip_ids"]),
        train_session_ids=list(previous.get("train_session_ids", [])),
        validation_session_ids=list(previous.get("validation_session_ids", [])),
        baseline=dict(previous["baseline"]),
        policy=dict(previous["policy"]),
        steps=int(previous["evaluation_steps"]),
        num_envs=int(previous["evaluation_environments"]),
        training_run=previous["training_run"],
        checkpoint_name=checkpoint_name,
        checkpoint_iteration=int(previous["checkpoint"]["iteration"]),
        stress=dict(previous.get("stress_tests", {})),
        contract=dict(previous.get("live_contract", {})),
        action_diagnostics=dict(previous.get("action_diagnostics", {})),
    )
    evaluation["revalidation"] = {
        "reused_held_out_rollout_metrics": True,
        "source_report": str(source_evaluation_path),
        "source_generated_at_utc": previous.get("generated_at_utc"),
        "revalidated_at_utc": datetime.now(timezone.utc).isoformat(),
        "reason": "baseline-relative stationary/lower-body gates and bounded collision-demand relaxation",
    }

    revalidated_json = candidate_dir / "policy_evaluation.revalidated.json"
    revalidated_md = candidate_dir / "policy_evaluation.revalidated.md"
    _write_json(revalidated_json, evaluation)
    revalidated_md.write_text(report_markdown(evaluation), encoding="utf-8")
    if not evaluation["accepted"]:
        print(f"[POLICY STILL REJECTED] report={revalidated_md}")
        return 2

    deployment_id = uuid.uuid4().hex
    metadata["deployment_id"] = deployment_id
    metadata.setdefault("provenance", {})["acceptance_gate_version"] = ACCEPTANCE_GATE_VERSION
    metadata["validation"] = {
        "accepted": True,
        "policy_strength_percent": float(evaluation["policy_strength_percent"]),
        "report": "policy_evaluation.json",
        "validation_clip_ids": list(evaluation["validation_clip_ids"]),
        "validation_session_ids": list(evaluation["validation_session_ids"]),
        "checkpoint_sweep": "checkpoint_sweep.json",
        "revalidated_existing_rollout": True,
        "acceptance_gate_version": ACCEPTANCE_GATE_VERSION,
    }
    revalidated_metadata = candidate_dir / "policy_metadata.revalidated.json"
    _write_json(revalidated_metadata, metadata)

    sweep = _load_json(sweep_path)
    sweep["deployment_revalidation"] = {
        "selected_checkpoint": checkpoint_name,
        "accepted": True,
        "acceptance_gate_version": ACCEPTANCE_GATE_VERSION,
        "report": "policy_evaluation.json",
    }
    revalidated_sweep = candidate_dir / "checkpoint_sweep.revalidated.json"
    _write_json(revalidated_sweep, sweep)

    # Preserve the previously deployed live policy before replacing it.
    if stable_dir.exists():
        previous_deployment = "unknown"
        stable_metadata = stable_dir / "policy_metadata.json"
        if stable_metadata.is_file():
            previous_deployment = str(_load_json(stable_metadata).get("deployment_id", "unknown"))
        backup_dir = stable_dir / "backups" / previous_deployment
        backup_dir.mkdir(parents=True, exist_ok=True)
        for name in (
            "policy.onnx",
            "policy_checkpoint.pt",
            "policy_evaluation.json",
            "policy_evaluation.md",
            "policy_metadata.json",
            "checkpoint_sweep.json",
        ):
            source = stable_dir / name
            if source.is_file() and not (backup_dir / name).exists():
                shutil.copy2(source, backup_dir / name)

    # Metadata is replaced last so a live reload never sees a partial bundle.
    _atomic_copy(onnx_path, stable_dir / "policy.onnx")
    _atomic_copy(checkpoint_path, stable_dir / "policy_checkpoint.pt")
    _atomic_copy(revalidated_json, stable_dir / "policy_evaluation.json")
    _atomic_copy(revalidated_md, stable_dir / "policy_evaluation.md")
    _atomic_copy(revalidated_sweep, stable_dir / "checkpoint_sweep.json")
    _atomic_copy(revalidated_metadata, stable_dir / "policy_metadata.json")
    print(f"[POLICY READY] checkpoint={checkpoint_name}")
    print(f"[POLICY READY] contribution={evaluation['policy_strength_percent']:+.3f}%")
    print(f"[POLICY READY] gate={ACCEPTANCE_GATE_VERSION}")
    print(f"[POLICY READY] deployment={stable_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

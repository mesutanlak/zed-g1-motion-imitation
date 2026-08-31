#!/usr/bin/env python3
"""Estimate static four-camera extrinsics from a captured BODY_38 session.

The input is produced by ``distributed_body38_fusion.py --calibration-record``.
All four cameras must stay fixed and see the same single person.  The result
maps each camera's ``RIGHT_HANDED_Z_UP_X_FWD`` coordinates to the reference
camera's frame; it is consumed only by the application-level fusion receiver.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np

# A script launched by its path has ``zed_four_camera_test`` as sys.path[0],
# whereas motion_pipeline lives at the repository root.  Keep the command-line
# calibration tool independent of the shell's current directory.
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from motion_pipeline.multiview import robust_rigid_alignment


CORE_NAMES = (
    "PELVIS", "SPINE_1", "SPINE_2", "SPINE_3", "NECK",
    "LEFT_CLAVICLE", "RIGHT_CLAVICLE", "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST",
    "LEFT_HIP", "RIGHT_HIP", "LEFT_KNEE", "RIGHT_KNEE",
    "LEFT_ANKLE", "RIGHT_ANKLE",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dortlu BODY_38 ham kaydindan statik kamera extrinsic olusturur")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--reference-serial", type=int, required=True)
    parser.add_argument("--confidence", type=float, default=55.0)
    parser.add_argument("--max-residual-m", type=float, default=0.16)
    parser.add_argument("--max-samples", type=int, default=240)
    return parser.parse_args()


def read_samples(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    metadata: dict[str, Any] = {}
    samples: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for number, line in enumerate(handle, start=1):
            try:
                item = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"Satir {number} JSON degil: {exc}") from exc
            schema = item.get("schema") if isinstance(item, dict) else None
            if schema == "zed_body38_multihost_calibration_metadata/v1":
                metadata = item
            elif schema == "zed_body38_multihost_calibration_sample/v1":
                samples.append(item)
    if not metadata:
        raise ValueError("Kalibrasyon metadata satiri bulunamadi.")
    if not samples:
        raise ValueError("Kalibrasyon ornek karesi bulunamadi.")
    return metadata, samples


def array(value: Any, shape: tuple[int, ...]) -> np.ndarray | None:
    try:
        result = np.asarray(value, dtype=np.float64)
        if result.shape == shape:
            return result
    except (TypeError, ValueError):
        pass
    return None


def collect_correspondences(
    samples: list[dict[str, Any]],
    *,
    reference_serial: int,
    target_serial: int,
    threshold: float,
    maximum_samples: int,
) -> tuple[np.ndarray, np.ndarray, int]:
    source_rows: list[np.ndarray] = []
    target_rows: list[np.ndarray] = []
    used_samples = 0
    for sample in samples[:maximum_samples]:
        sources = sample.get("sources", {})
        if not isinstance(sources, dict):
            continue
        reference = sources.get(str(reference_serial))
        target = sources.get(str(target_serial))
        if not isinstance(reference, dict) or not isinstance(target, dict):
            continue
        names = reference.get("keypoint_names")
        if names != target.get("keypoint_names") or not isinstance(names, list):
            continue
        indices = {str(name): index for index, name in enumerate(names)}
        required = [indices[name] for name in CORE_NAMES if name in indices]
        if len(required) < 12:
            continue
        reference_points = array(reference.get("keypoints_3d_m"), (len(names), 3))
        target_points = array(target.get("keypoints_3d_m"), (len(names), 3))
        reference_confidence = array(reference.get("keypoint_confidence"), (len(names),))
        target_confidence = array(target.get("keypoint_confidence"), (len(names),))
        if any(item is None for item in (reference_points, target_points, reference_confidence, target_confidence)):
            continue
        keep = [
            index for index in required
            if np.isfinite(reference_points[index]).all()
            and np.isfinite(target_points[index]).all()
            and reference_confidence[index] >= threshold
            and target_confidence[index] >= threshold
        ]
        if len(keep) < 12:
            continue
        source_rows.extend(target_points[keep])
        target_rows.extend(reference_points[keep])
        used_samples += 1
    if not source_rows:
        return np.empty((0, 3)), np.empty((0, 3)), used_samples
    return np.asarray(source_rows), np.asarray(target_rows), used_samples


def main() -> int:
    args = parse_args()
    if not 0.0 <= args.confidence <= 100.0 or args.max_residual_m <= 0.0:
        print("HATA: confidence veya max-residual-m gecersiz.", file=sys.stderr)
        return 2
    try:
        metadata, samples = read_samples(args.input.expanduser().resolve())
    except (OSError, ValueError) as exc:
        print(f"HATA: {exc}", file=sys.stderr)
        return 2
    serials = [int(item["serial"]) for item in metadata.get("sources", []) if isinstance(item, dict) and "serial" in item]
    if len(serials) != 4 or len(set(serials)) != 4:
        print("HATA: kayit dort benzersiz ZED serisi icermiyor.", file=sys.stderr)
        return 2
    if args.reference_serial not in serials:
        print(f"HATA: referans ZED {args.reference_serial} kayitta yok.", file=sys.stderr)
        return 2

    cameras: dict[str, Any] = {
        str(args.reference_serial): {
            "rotation_camera_to_world": np.eye(3).tolist(),
            "translation_camera_to_world_m": [0.0, 0.0, 0.0],
            "fit": {"reference": True},
        }
    }
    failures = 0
    for serial in serials:
        if serial == args.reference_serial:
            continue
        source, target, used_samples = collect_correspondences(
            samples,
            reference_serial=args.reference_serial,
            target_serial=serial,
            threshold=args.confidence,
            maximum_samples=args.max_samples,
        )
        fit = robust_rigid_alignment(source, target, maximum_residual_m=args.max_residual_m)
        if fit is None:
            print(f"HATA: ZED {serial} icin yeterli eslesmis BODY_38 noktasi yok (ornek={used_samples}, nokta={len(source)}).", file=sys.stderr)
            failures += 1
            continue
        rotation, translation, metrics = fit
        cameras[str(serial)] = {
            "rotation_camera_to_world": rotation.tolist(),
            "translation_camera_to_world_m": translation.tolist(),
            "fit": {"capture_samples": used_samples, **metrics},
        }
        print(
            f"ZED {serial} -> {args.reference_serial} | kare={used_samples} "
            f"nokta={metrics['paired_keypoints']} | rms={metrics['rms_m']:.3f}m "
            f"p95={metrics['p95_m']:.3f}m"
        )
        if float(metrics["rms_m"]) > args.max_residual_m:
            print(f"UYARI: ZED {serial} RMS esigi asti; tripod/ortak gorus ve T-poz kaydini tekrarlayin.", file=sys.stderr)
            failures += 1
    if failures or len(cameras) != 4:
        print("Kalibrasyon dosyasi yazilmadi; once tum kameralar icin dusuk hatali fit gerekli.", file=sys.stderr)
        return 3

    output = {
        "schema": "zed_body38_distributed_extrinsics/v1",
        "created_unix_ns": time.time_ns(),
        "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
        "units": "meter",
        "reference_world_serial": args.reference_serial,
        "source_capture": str(args.input.expanduser().resolve()),
        "method": "trimmed_kabsch_from_synchronized_body38_core_joints",
        "cameras": cameras,
        "warning": "Tripodlar veya kameralar hareket ederse bu dosya gecersizdir; yeniden kalibre edin.",
    }
    path = args.output.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"KALIBRASYON HAZIR: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

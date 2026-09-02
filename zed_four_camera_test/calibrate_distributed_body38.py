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
    parser.add_argument("--max-samples", type=int, default=900)
    parser.add_argument("--min-inlier-ratio", type=float, default=0.25)
    parser.add_argument("--min-inlier-points", type=int, default=300)
    parser.add_argument("--min-capture-samples", type=int, default=60)
    parser.add_argument("--max-pelvis-p95-m", type=float, default=0.25)
    parser.add_argument(
        "--min-pelvis-motion-m", type=float, default=0.10,
        help=(
            "Referans operator pelvisinin kalibrasyon boyunca yapmasi gereken "
            "asgari saglam hareket genisligi. Yanlis/sabit kisi kilidini yakalar."
        ),
    )
    parser.add_argument(
        "--min-pelvis-motion-ratio", type=float, default=0.60,
        help="Kamera ve referans pelvis hareket genisliklerinin asgari orani.",
    )
    parser.add_argument(
        "--min-pelvis-trajectory-correlation", type=float, default=0.75,
        help="Donusturulmus kamera ile referans pelvis yollari arasindaki asgari korelasyon.",
    )
    parser.add_argument(
        "--world-poses-jsonl",
        type=Path,
        default=None,
        help="Kamera-dunya pozlarini satir bazli yaz. Varsayilan: OUTPUT yaninda *_world_poses.jsonl.",
    )
    return parser.parse_args()


def matrix_to_quaternion_xyzw(matrix: np.ndarray) -> list[float]:
    value = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(value))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quat = np.array([
            (value[2, 1] - value[1, 2]) / scale,
            (value[0, 2] - value[2, 0]) / scale,
            (value[1, 0] - value[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(value)))
        if index == 0:
            scale = math.sqrt(max(1.0e-12, 1.0 + value[0, 0] - value[1, 1] - value[2, 2])) * 2.0
            quat = np.array([0.25 * scale, (value[0, 1] + value[1, 0]) / scale,
                             (value[0, 2] + value[2, 0]) / scale, (value[2, 1] - value[1, 2]) / scale])
        elif index == 1:
            scale = math.sqrt(max(1.0e-12, 1.0 + value[1, 1] - value[0, 0] - value[2, 2])) * 2.0
            quat = np.array([(value[0, 1] + value[1, 0]) / scale, 0.25 * scale,
                             (value[1, 2] + value[2, 1]) / scale, (value[0, 2] - value[2, 0]) / scale])
        else:
            scale = math.sqrt(max(1.0e-12, 1.0 + value[2, 2] - value[0, 0] - value[1, 1])) * 2.0
            quat = np.array([(value[0, 2] + value[2, 0]) / scale, (value[1, 2] + value[2, 1]) / scale,
                             0.25 * scale, (value[1, 0] - value[0, 1]) / scale])
    quat /= max(float(np.linalg.norm(quat)), 1.0e-12)
    return quat.tolist()


def write_world_poses_jsonl(
    path: Path,
    *,
    output: dict[str, Any],
    source_ports: dict[int, int],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [{
        "schema": "zed_camera_world_poses_metadata/v1",
        "created_unix_ns": output["created_unix_ns"],
        "coordinate_system": output["coordinate_system"],
        "units": output["units"],
        "reference_world_serial": output["reference_world_serial"],
        "source_capture": output["source_capture"],
        "transform_semantics": "point_world = rotation_camera_to_world @ point_camera + translation_camera_to_world_m",
    }]
    for serial_text, camera in output["cameras"].items():
        serial = int(serial_text)
        rotation = np.asarray(camera["rotation_camera_to_world"], dtype=np.float64)
        translation = np.asarray(camera["translation_camera_to_world_m"], dtype=np.float64)
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation
        lines.append({
            "schema": "zed_camera_world_pose/v1",
            "serial": serial,
            "source_udp_port": source_ports.get(serial),
            "is_reference": serial == int(output["reference_world_serial"]),
            "camera_optical_origin_world_m": translation.tolist(),
            "rotation_camera_to_world": rotation.tolist(),
            "orientation_camera_to_world_xyzw": matrix_to_quaternion_xyzw(rotation),
            "transform_camera_to_world_4x4": transform.tolist(),
            "fit": camera.get("fit", {}),
        })
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        for item in lines:
            handle.write(json.dumps(item, ensure_ascii=False, allow_nan=False) + "\n")


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
) -> tuple[np.ndarray, np.ndarray, int, np.ndarray]:
    source_rows: list[np.ndarray] = []
    target_rows: list[np.ndarray] = []
    used_samples = 0
    pelvis_pairs: list[np.ndarray] = []
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
        # One oblique camera can legitimately lose one complete arm/leg.  Six
        # non-collinear torso/limb correspondences in one synchronized frame
        # are enough; robust_rigid_alignment still requires at least twelve
        # accumulated points before any transform can be emitted.
        if len(required) < 6:
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
        if len(keep) < 6:
            continue
        source_rows.extend(target_points[keep])
        target_rows.extend(reference_points[keep])
        pelvis_index = indices.get("PELVIS")
        if (
            pelvis_index is not None
            and np.isfinite(reference_points[pelvis_index]).all()
            and np.isfinite(target_points[pelvis_index]).all()
            and reference_confidence[pelvis_index] >= threshold
            and target_confidence[pelvis_index] >= threshold
        ):
            pelvis_pairs.append(np.stack((
                target_points[pelvis_index], reference_points[pelvis_index]
            )))
        used_samples += 1
    if not source_rows:
        return np.empty((0, 3)), np.empty((0, 3)), used_samples, np.empty((0, 2, 3))
    return (
        np.asarray(source_rows), np.asarray(target_rows), used_samples,
        np.asarray(pelvis_pairs, dtype=np.float64).reshape((-1, 2, 3)),
    )


def pelvis_trajectory_metrics(
    pelvis_pairs: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> dict[str, float]:
    """Measure whether both cameras tracked the same moving pelvis.

    A rigid fit over many BODY_38 joints can appear numerically acceptable
    even when one camera stays locked to a different, mostly stationary
    person.  Translation/rotation cannot change trajectory scale; comparing
    the centered pelvis paths therefore catches that cross-person failure.
    """
    if len(pelvis_pairs) < 3:
        return {
            "pelvis_source_motion_m": 0.0,
            "pelvis_reference_motion_m": 0.0,
            "pelvis_motion_scale_ratio": 0.0,
            "pelvis_trajectory_correlation": 0.0,
        }
    source = pelvis_pairs[:, 0] @ rotation.T + translation
    reference = pelvis_pairs[:, 1]

    def robust_motion_extent(values: np.ndarray) -> float:
        centered_radius = np.linalg.norm(
            values - np.median(values, axis=0), axis=1
        )
        return float(
            np.percentile(centered_radius, 95)
            - np.percentile(centered_radius, 5)
        )

    source_motion = robust_motion_extent(source)
    reference_motion = robust_motion_extent(reference)
    maximum_motion = max(source_motion, reference_motion, 1.0e-9)
    motion_ratio = min(source_motion, reference_motion) / maximum_motion
    source_centered = source - np.mean(source, axis=0)
    reference_centered = reference - np.mean(reference, axis=0)
    denominator = float(
        np.linalg.norm(source_centered) * np.linalg.norm(reference_centered)
    )
    correlation = (
        float(np.sum(source_centered * reference_centered) / denominator)
        if denominator > 1.0e-9
        else 0.0
    )
    return {
        "pelvis_source_motion_m": source_motion,
        "pelvis_reference_motion_m": reference_motion,
        "pelvis_motion_scale_ratio": float(np.clip(motion_ratio, 0.0, 1.0)),
        "pelvis_trajectory_correlation": float(np.clip(correlation, -1.0, 1.0)),
    }


def main() -> int:
    args = parse_args()
    if (
        not 0.0 <= args.confidence <= 100.0
        or args.max_residual_m <= 0.0
        or not 0.0 < args.min_inlier_ratio <= 1.0
        or args.min_inlier_points < 12
        or args.min_capture_samples < 1
        or args.max_pelvis_p95_m <= 0.0
        or args.min_pelvis_motion_m <= 0.0
        or not 0.0 < args.min_pelvis_motion_ratio <= 1.0
        or not -1.0 <= args.min_pelvis_trajectory_correlation <= 1.0
    ):
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
        source, target, used_samples, pelvis_pairs = collect_correspondences(
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
        candidate_points = int(len(source))
        inlier_ratio = (
            float(metrics["paired_keypoints"]) / candidate_points
            if candidate_points else 0.0
        )
        pelvis_errors = (
            np.linalg.norm(
                pelvis_pairs[:, 0] @ rotation.T + translation - pelvis_pairs[:, 1],
                axis=1,
            )
            if len(pelvis_pairs)
            else np.empty(0, dtype=np.float64)
        )
        pelvis_median_m = float(np.median(pelvis_errors)) if pelvis_errors.size else math.inf
        pelvis_p95_m = float(np.percentile(pelvis_errors, 95)) if pelvis_errors.size else math.inf
        trajectory = pelvis_trajectory_metrics(pelvis_pairs, rotation, translation)
        quality_ok = bool(
            used_samples >= args.min_capture_samples
            and int(metrics["paired_keypoints"]) >= args.min_inlier_points
            and inlier_ratio >= args.min_inlier_ratio
            and float(metrics["rms_m"]) <= args.max_residual_m
            and pelvis_p95_m <= args.max_pelvis_p95_m
            and trajectory["pelvis_reference_motion_m"] >= args.min_pelvis_motion_m
            and trajectory["pelvis_motion_scale_ratio"] >= args.min_pelvis_motion_ratio
            and trajectory["pelvis_trajectory_correlation"]
            >= args.min_pelvis_trajectory_correlation
        )
        fit_metrics = {
            "capture_samples": used_samples,
            "candidate_keypoints": candidate_points,
            "inlier_ratio": inlier_ratio,
            "pelvis_frames": int(pelvis_errors.size),
            "pelvis_median_m": pelvis_median_m,
            "pelvis_p95_m": pelvis_p95_m,
            "quality_gate_passed": quality_ok,
            **trajectory,
            **metrics,
        }
        cameras[str(serial)] = {
            "rotation_camera_to_world": rotation.tolist(),
            "translation_camera_to_world_m": translation.tolist(),
            "fit": fit_metrics,
        }
        print(
            f"ZED {serial} -> {args.reference_serial} | kare={used_samples} "
            f"nokta={metrics['paired_keypoints']}/{candidate_points} "
            f"({100.0 * inlier_ratio:.1f}%) | rms={metrics['rms_m']:.3f}m "
            f"p95={metrics['p95_m']:.3f}m | pelvis_p95={pelvis_p95_m:.3f}m "
            f"| pelvis_hareket={trajectory['pelvis_source_motion_m']:.3f}/"
            f"{trajectory['pelvis_reference_motion_m']:.3f}m "
            f"oran={trajectory['pelvis_motion_scale_ratio']:.2f} "
            f"korelasyon={trajectory['pelvis_trajectory_correlation']:.2f}"
        )
        if not quality_ok:
            print(
                f"HATA: ZED {serial} kalibrasyon kalite kapisini gecemedi "
                f"(en az {args.min_capture_samples} kare, {args.min_inlier_points} nokta, "
                f"%{100.0 * args.min_inlier_ratio:.0f} inlier, pelvis p95 <= "
                f"{args.max_pelvis_p95_m:.2f} m, referans pelvis hareketi >= "
                f"{args.min_pelvis_motion_m:.2f} m, hareket orani >= "
                f"{args.min_pelvis_motion_ratio:.2f}, yol korelasyonu >= "
                f"{args.min_pelvis_trajectory_correlation:.2f} gerekli).",
                file=sys.stderr,
            )
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
    source_ports = {
        int(item["serial"]): int(item["port"])
        for item in metadata.get("sources", [])
        if isinstance(item, dict) and "serial" in item and "port" in item
    }
    poses_path = (
        args.world_poses_jsonl.expanduser().resolve()
        if args.world_poses_jsonl is not None
        else path.with_name(path.stem + "_world_poses.jsonl")
    )
    write_world_poses_jsonl(poses_path, output=output, source_ports=source_ports)
    print(f"DUNYA POZLARI JSONL HAZIR: {poses_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

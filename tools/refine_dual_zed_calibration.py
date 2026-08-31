#!/usr/bin/env python3
"""Create one immutable dual-ZED extrinsic from a recorded dual session.

This is an offline recovery tool, not a live calibration loop. It reconstructs
sender-local torso points from the diagnostic poses stored in a JSONL session,
fits one trimmed rigid transform across the whole recording, validates it, and
writes a normal ZED Fusion configuration through the ZED SDK.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys
import tempfile

import numpy as np
import pyzed.sl as sl

PROJECT = Path(__file__).resolve().parents[1]
if str(PROJECT) not in sys.path:
    sys.path.insert(0, str(PROJECT))

from motion_pipeline.multiview import robust_rigid_alignment
from zed_g1_skeleton import IDX


CORE = (
    "PELVIS", "SPINE_1", "SPINE_2", "SPINE_3", "NECK",
    "LEFT_SHOULDER", "RIGHT_SHOULDER", "LEFT_HIP", "RIGHT_HIP",
)


def _pose(value: dict[str, object]) -> tuple[np.ndarray, np.ndarray]:
    return (
        np.asarray(value["rotation"], dtype=np.float64),
        np.asarray(value["translation_m"], dtype=np.float64),
    )


def _local_points(
    world_points: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> np.ndarray:
    # Forward convention in the recorder is world = local @ R.T + t.
    return (world_points - translation) @ rotation


def _sdk_transform(rotation: np.ndarray, translation: np.ndarray) -> sl.Transform:
    sdk_rotation = sl.Rotation()
    for row in range(3):
        for column in range(3):
            sdk_rotation[row, column] = float(rotation[row, column])
    sdk_translation = sl.Translation()
    sdk_translation.init_vector(*map(float, translation))
    transform = sl.Transform()
    transform.set_rotation_matrix(sdk_rotation)
    transform.set_translation(sdk_translation)
    return transform


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("recording", type=Path)
    parser.add_argument("input_config", type=Path)
    parser.add_argument("output_config", type=Path)
    parser.add_argument("--confidence", type=float, default=40.0)
    parser.add_argument("--minimum-frames", type=int, default=300)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    # ZED SDK 5.x on Windows cannot read non-ASCII paths. Use SDK-only cache
    # files, while preserving the user-facing source/output locations.
    sdk_cache = Path(tempfile.mkdtemp(prefix="zed_fusion_calibration_"))
    sdk_input = sdk_cache / "input.json"
    sdk_output = sdk_cache / "output.json"
    shutil.copy2(args.input_config.resolve(), sdk_input)
    configurations = sl.read_fusion_configuration_file(
        str(sdk_input), sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD,
        sl.UNIT.METER,
    )
    if len(configurations) != 2:
        raise RuntimeError("Exactly two ZED configurations are required.")
    by_serial = {int(item.serial_number): item for item in configurations}
    reference_serial, moving_serial = sorted(by_serial)
    reference_pose = by_serial[reference_serial].pose
    reference_rotation = np.asarray(
        reference_pose.get_rotation_matrix().r, dtype=np.float64
    )
    reference_translation = np.asarray(
        reference_pose.get_translation().get(), dtype=np.float64
    )

    source: list[np.ndarray] = []
    target: list[np.ndarray] = []
    paired_frames = 0
    core_ids = np.asarray([IDX[name] for name in CORE], dtype=np.int32)
    with args.recording.open("r", encoding="utf-8") as stream:
        next(stream, None)  # metadata
        for line in stream:
            record = json.loads(line)
            multi = record.get("multi_camera") or {}
            views = {
                int(view["serial_number"]): view
                for view in multi.get("per_camera", [])
                if "keypoints_3d_fusion_m" in view
            }
            poses = multi.get("camera_pose_fusion_from_local") or {}
            if (
                reference_serial not in views
                or moving_serial not in views
                or str(reference_serial) not in poses
                or str(moving_serial) not in poses
            ):
                continue
            reference_view = views[reference_serial]
            moving_view = views[moving_serial]
            reference_world_recorded = np.asarray(
                reference_view["keypoints_3d_fusion_m"], dtype=np.float64
            )
            moving_world_recorded = np.asarray(
                moving_view["keypoints_3d_fusion_m"], dtype=np.float64
            )
            reference_recorded_rotation, reference_recorded_translation = _pose(
                poses[str(reference_serial)]
            )
            moving_recorded_rotation, moving_recorded_translation = _pose(
                poses[str(moving_serial)]
            )
            reference_local = _local_points(
                reference_world_recorded,
                reference_recorded_rotation,
                reference_recorded_translation,
            )
            moving_local = _local_points(
                moving_world_recorded,
                moving_recorded_rotation,
                moving_recorded_translation,
            )
            reference_world = (
                reference_local @ reference_rotation.T + reference_translation
            )
            reference_confidence = np.asarray(
                reference_view["keypoint_confidence"], dtype=np.float64
            )
            moving_confidence = np.asarray(
                moving_view["keypoint_confidence"], dtype=np.float64
            )
            first = reference_world[core_ids]
            second = moving_local[core_ids]
            valid = (
                np.isfinite(first).all(axis=1)
                & np.isfinite(second).all(axis=1)
                & (reference_confidence[core_ids] >= args.confidence)
                & (moving_confidence[core_ids] >= args.confidence)
            )
            if int(np.count_nonzero(valid)) < 7:
                continue
            source.extend(second[valid])
            target.extend(first[valid])
            paired_frames += 1

    if paired_frames < args.minimum_frames:
        raise RuntimeError(
            f"Only {paired_frames} paired frames; need {args.minimum_frames}."
        )
    fitted = robust_rigid_alignment(
        np.asarray(source), np.asarray(target), maximum_residual_m=0.18
    )
    if fitted is None:
        raise RuntimeError("Robust rigid fit failed.")
    rotation, translation, metrics = fitted
    if metrics["rms_m"] > 0.04 or metrics["p95_m"] > 0.07:
        raise RuntimeError(f"Calibration rejected: {metrics}")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1.0e-5):
        raise RuntimeError("Calibration produced an improper rotation.")

    by_serial[moving_serial].pose = _sdk_transform(rotation, translation)
    args.output_config.parent.mkdir(parents=True, exist_ok=True)
    sl.write_configuration_file(
        str(sdk_output),
        configurations,
        sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD,
        sl.UNIT.METER,
    )
    reloaded = sl.read_fusion_configuration_file(
        str(sdk_output),
        sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD,
        sl.UNIT.METER,
    )
    check = {
        int(item.serial_number): item for item in reloaded
    }[moving_serial].pose
    check_rotation = np.asarray(check.get_rotation_matrix().r, dtype=np.float64)
    check_translation = np.asarray(
        check.get_translation().get(), dtype=np.float64
    )
    if (
        not np.allclose(check_rotation, rotation, atol=2.0e-5)
        or not np.allclose(check_translation, translation, atol=2.0e-5)
    ):
        raise RuntimeError("ZED SDK write/read verification failed.")
    shutil.copy2(sdk_output, args.output_config.resolve())
    shutil.rmtree(sdk_cache, ignore_errors=True)
    print(json.dumps({
        "reference_serial": reference_serial,
        "updated_serial": moving_serial,
        "paired_frames": paired_frames,
        "fit": metrics,
        "rotation": rotation.tolist(),
        "translation_m": translation.tolist(),
        "output": str(args.output_config.resolve()),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

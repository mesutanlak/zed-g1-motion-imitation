"""Turn an offline GMR retargeting result into clean fixed-rate training clips.

Input is the NPZ written by ``isaaclab_bridge/gmr_retarget_jsonl.py``.  The
source BODY_38 JSONL is used only for confidence/calibration gates.  Output CSV
has Unitree's canonical layout: root xyz, root quaternion xyzw, 23 joints.
Each CSV has a metadata NPZ consumed by the Isaac FK conversion step.
"""

from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Quality:
    confidence: float
    usable: bool


def quality_from_record(record: dict) -> Quality:
    body = float(record.get("body_confidence", 0.0) or 0.0) / 100.0
    human = record.get("human_state") if isinstance(record.get("human_state"), dict) else {}
    segment = human.get("segment_quality") if isinstance(human.get("segment_quality"), dict) else {}
    upper_values = [float(segment[name]) for name in ("pelvis", "torso", "left_arm", "right_arm") if name in segment]
    segment_score = float(np.mean(upper_values)) if upper_values else body
    fused = record.get("multi_camera") if isinstance(record.get("multi_camera"), dict) else {}
    fused_score = float(fused.get("fused_quality_score", segment_score) or segment_score)
    confidence = float(np.clip(min(body, segment_score, fused_score), 0.0, 1.0))

    operator = record.get("operator_selection") if isinstance(record.get("operator_selection"), dict) else {}
    calibration = record.get("calibration") if isinstance(record.get("calibration"), dict) else {}
    operator_ok = not operator or operator.get("state") == "LOCKED"
    calibration_ok = not calibration or calibration.get("state") == "READY"
    failure_codes = human.get("failure_codes", [])
    hard_failure = any(
        code in {
            "CAMERA_CALIBRATION_DISAGREEMENT",
            "LEFT_RIGHT_SWAP_RISK",
            "BONE_LENGTH_VIOLATION",
            "BOTH_CAMERAS_LOW_CONFIDENCE",
        }
        for code in failure_codes
    )
    return Quality(confidence, bool(operator_ok and calibration_ok and not hard_failure))


def load_quality(jsonl: Path) -> dict[int, Quality]:
    result: dict[int, Quality] = {}
    with jsonl.open("r", encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            if str(record.get("schema", "")).endswith("/metadata/v1"):
                continue
            if record.get("schema") == "zed_body38_3d_analysis/frame/v1":
                record = record.get("source", {})
            elif record.get("schema") == "zed_body38_rerun_analysis/frame/v1":
                record = record.get("source_packet", {})
            timestamp = record.get("timestamp_ns")
            if timestamp is not None:
                result[int(timestamp)] = quality_from_record(record)
    return result


def contiguous_runs(mask: np.ndarray, timestamps_ns: np.ndarray, max_gap_s: float) -> list[np.ndarray]:
    valid = np.flatnonzero(mask)
    if not len(valid):
        return []
    boundaries = [0]
    for index in range(1, len(valid)):
        consecutive = valid[index] == valid[index - 1] + 1
        gap_s = (timestamps_ns[valid[index]] - timestamps_ns[valid[index - 1]]) * 1.0e-9
        if not consecutive or gap_s > max_gap_s:
            boundaries.append(index)
    boundaries.append(len(valid))
    return [valid[boundaries[i] : boundaries[i + 1]] for i in range(len(boundaries) - 1)]


def normalize_quaternions_wxyz(values: np.ndarray) -> np.ndarray:
    output = np.asarray(values, dtype=np.float64).copy()
    for index in range(1, len(output)):
        if np.dot(output[index - 1], output[index]) < 0.0:
            output[index] *= -1.0
    output /= np.maximum(np.linalg.norm(output, axis=1, keepdims=True), 1.0e-12)
    return output


def interp_columns(times: np.ndarray, values: np.ndarray, target: np.ndarray) -> np.ndarray:
    return np.column_stack([np.interp(target, times, values[:, column]) for column in range(values.shape[1])])


def triangular_smooth(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return values.copy()
    if window % 2 == 0:
        raise ValueError("smoothing window must be odd")
    half = window // 2
    weights = np.arange(1, half + 2, dtype=np.float64)
    weights = np.concatenate((weights, weights[-2::-1]))
    weights /= weights.sum()
    padded = np.pad(values, ((half, half), (0, 0)), mode="edge")
    return np.column_stack([np.convolve(padded[:, i], weights, mode="valid") for i in range(values.shape[1])])


def feasibility_filter(
    values: np.ndarray,
    dt: float,
    max_velocity: float,
    max_acceleration: float,
    lower_limits: np.ndarray | None = None,
    upper_limits: np.ndarray | None = None,
) -> np.ndarray:
    """Project an offline reference onto velocity, acceleration and position limits.

    The velocity envelope starts braking *before* a position boundary.  Clipping
    positions after rate limiting would stop a moving joint in one sample and
    manufacture a large acceleration spike in the training reference.
    """
    values = np.asarray(values, dtype=np.float64)
    lower = (
        np.full(values.shape[1], -np.inf, dtype=np.float64)
        if lower_limits is None else np.asarray(lower_limits, dtype=np.float64)
    )
    upper = (
        np.full(values.shape[1], np.inf, dtype=np.float64)
        if upper_limits is None else np.asarray(upper_limits, dtype=np.float64)
    )
    if lower.shape != (values.shape[1],) or upper.shape != (values.shape[1],):
        raise ValueError("joint limit arrays must match the number of columns")
    if np.any(lower >= upper):
        raise ValueError("every lower joint limit must be below its upper limit")

    output = np.empty_like(values)
    output[0] = np.clip(values[0], lower, upper)
    velocity = np.zeros(values.shape[1], dtype=np.float64)
    for index in range(1, len(values)):
        target = np.clip(values[index], lower, upper)
        desired_velocity = np.clip(
            (target - output[index - 1]) / dt,
            -max_velocity,
            max_velocity,
        )

        # A conservative discrete-time stopping envelope.  At distance d from
        # a hard boundary, sqrt(2*a*d)-a*dt begins braking one control sample
        # earlier than the continuous-time bound and avoids a terminal clip.
        distance_to_lower = np.maximum(output[index - 1] - lower, 0.0)
        distance_to_upper = np.maximum(upper - output[index - 1], 0.0)
        max_negative = np.maximum(
            0.0, np.sqrt(2.0 * max_acceleration * distance_to_lower) - max_acceleration * dt
        )
        max_positive = np.maximum(
            0.0, np.sqrt(2.0 * max_acceleration * distance_to_upper) - max_acceleration * dt
        )
        desired_velocity = np.clip(desired_velocity, -max_negative, max_positive)
        velocity = np.clip(
            desired_velocity,
            velocity - max_acceleration * dt,
            velocity + max_acceleration * dt,
        )
        velocity = np.clip(velocity, -max_velocity, max_velocity)
        output[index] = output[index - 1] + velocity * dt

        # This is only a floating-point guard; the stopping envelope above is
        # responsible for respecting the boundary without an impulsive stop.
        output[index] = np.clip(output[index], lower, upper)
    return output


def root_from_retarget(data: np.lib.npyio.NpzFile, count: int) -> tuple[np.ndarray, np.ndarray]:
    if "q_g1_full_qpos" in data.files and data["q_g1_full_qpos"].shape[1] >= 7:
        qpos = np.asarray(data["q_g1_full_qpos"], dtype=np.float64)
        root_xyz = qpos[:, :3]
        root_quat = normalize_quaternions_wxyz(qpos[:, 3:7])
    else:
        root_xyz = np.repeat(np.array([[0.0, 0.0, 0.80]]), count, axis=0)
        root_quat = np.repeat(np.array([[1.0, 0.0, 0.0, 0.0]]), count, axis=0)
    return root_xyz, root_quat


def clip_to_robot_limits(values: np.ndarray, names: list[str], xml_path: Path | None) -> np.ndarray:
    if xml_path is None:
        return values.copy()
    root = ET.parse(xml_path).getroot()
    limits = {}
    for node in root.findall(".//joint"):
        if node.get("name") and node.get("range"):
            limits[node.get("name")] = tuple(float(item) for item in node.get("range").split())
    missing = [name for name in names if name not in limits]
    if missing:
        raise ValueError(f"robot XML is missing joint limits: {missing}")
    output = values.copy()
    for index, name in enumerate(names):
        low, high = limits[name]
        output[:, index] = np.clip(output[:, index], low, high)
    return output


def robot_limit_arrays(names: list[str], xml_path: Path | None) -> tuple[np.ndarray, np.ndarray]:
    if xml_path is None:
        count = len(names)
        return np.full(count, -np.inf), np.full(count, np.inf)
    root = ET.parse(xml_path).getroot()
    limits = {
        node.get("name"): tuple(float(item) for item in node.get("range").split())
        for node in root.findall(".//joint")
        if node.get("name") and node.get("range")
    }
    missing = [name for name in names if name not in limits]
    if missing:
        raise ValueError(f"robot XML is missing joint limits: {missing}")
    return (
        np.asarray([limits[name][0] for name in names], dtype=np.float64),
        np.asarray([limits[name][1] for name in names], dtype=np.float64),
    )


def validation_quality_score(clip: dict) -> float:
    """Return the deterministic held-out-clip quality score.

    Validation should measure generalization on the cleanest independent run,
    not merely on the largest file.  The confidence floor receives almost as
    much weight as the mean so that a clip with a short unreliable interval is
    not silently selected as the reference benchmark.
    """

    mean = float(np.clip(clip["confidence_mean"], 0.0, 1.0))
    minimum = float(np.clip(clip["confidence_min"], 0.0, 1.0))
    return 0.55 * mean + 0.45 * minimum


def assign_dataset_splits(manifest: dict) -> dict:
    """Assign exactly one clean clip to validation and all others to train."""

    clips = manifest.get("clips", [])
    if len(clips) < 2:
        raise RuntimeError(
            "at least two independent clean clips are required for a leakage-free "
            "train/validation split"
        )
    for clip in clips:
        clip["quality_score"] = validation_quality_score(clip)
    validation = max(
        clips,
        key=lambda clip: (
            clip["quality_score"],
            float(clip["confidence_min"]),
            float(clip["confidence_mean"]),
            int(clip["frames"]),
            str(clip["id"]),
        ),
    )
    for clip in clips:
        clip["split"] = "validation" if clip is validation else "train"
    train_ids = [str(clip["id"]) for clip in clips if clip["split"] == "train"]
    validation_ids = [str(validation["id"])]
    manifest["splits"] = {"train": train_ids, "validation": validation_ids}
    manifest["split_strategy"] = {
        "name": "cleanest_clip_holdout",
        "quality_formula": "0.55*confidence_mean + 0.45*confidence_min",
        "concatenate_clips": False,
        "environment_sampling": "balanced_per_environment",
    }
    manifest["schema"] = "g1_reference_motion_clips/v2"
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("retarget_npz", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--source-jsonl", type=Path)
    parser.add_argument("--output-fps", type=float, default=50.0)
    parser.add_argument("--minimum-confidence", type=float, default=0.60)
    parser.add_argument("--minimum-seconds", type=float, default=2.0)
    parser.add_argument("--max-gap-seconds", type=float, default=0.20)
    parser.add_argument("--smoothing-window", type=int, default=3)
    parser.add_argument("--max-joint-velocity", type=float, default=5.0)
    parser.add_argument("--max-joint-acceleration", type=float, default=30.0)
    parser.add_argument("--robot-xml", type=Path, default=None)
    args = parser.parse_args()

    data = np.load(args.retarget_npz, allow_pickle=False)
    required = {"timestamp_ns", "q_g1_23dof", "joint_names_23"}
    missing = sorted(required.difference(data.files))
    if missing:
        raise ValueError(f"retarget NPZ is missing: {missing}")
    timestamps = np.asarray(data["timestamp_ns"], dtype=np.int64)
    joints = np.asarray(data["q_g1_23dof"], dtype=np.float64)
    if joints.shape != (len(timestamps), 23) or not np.isfinite(joints).all():
        raise ValueError("retarget NPZ must contain finite Nx23 q_g1_23dof")
    joint_names = [str(value) for value in np.asarray(data["joint_names_23"]).reshape(-1)]
    joints = clip_to_robot_limits(joints, joint_names, args.robot_xml)
    lower_limits, upper_limits = robot_limit_arrays(joint_names, args.robot_xml)
    root_xyz, root_quat = root_from_retarget(data, len(timestamps))

    source_jsonl = args.source_jsonl
    if source_jsonl is None and "source_jsonl" in data.files:
        candidate = Path(str(np.asarray(data["source_jsonl"]).item()))
        if candidate.exists():
            source_jsonl = candidate
    qualities = load_quality(source_jsonl) if source_jsonl else {}
    confidence = np.ones(len(timestamps), dtype=np.float64)
    usable = np.ones(len(timestamps), dtype=bool)
    for index, timestamp in enumerate(timestamps):
        if timestamp in qualities:
            confidence[index] = qualities[timestamp].confidence
            usable[index] = qualities[timestamp].usable
    usable &= confidence >= args.minimum_confidence

    runs = contiguous_runs(usable, timestamps, args.max_gap_seconds)
    minimum_frames = max(3, int(np.ceil(args.minimum_seconds * args.output_fps)))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    manifest = {
        "schema": "g1_reference_motion_clips/v2",
        "source_retarget_npz": str(args.retarget_npz.resolve()),
        "source_jsonl": str(source_jsonl.resolve()) if source_jsonl else None,
        "output_fps": args.output_fps,
        "clips": [],
    }
    stem = args.retarget_npz.stem.replace("_gmr", "")
    for run in runs:
        source_time = (timestamps[run] - timestamps[run[0]]).astype(np.float64) * 1.0e-9
        duration = float(source_time[-1])
        target_time = np.arange(0.0, duration + 0.5 / args.output_fps, 1.0 / args.output_fps)
        if len(target_time) < minimum_frames:
            continue
        joint_interp = interp_columns(source_time, joints[run], target_time)
        joint_smooth = triangular_smooth(joint_interp, args.smoothing_window)
        joint_safe = feasibility_filter(
            joint_smooth,
            1.0 / args.output_fps,
            args.max_joint_velocity,
            args.max_joint_acceleration,
            lower_limits,
            upper_limits,
        )
        xyz = interp_columns(source_time, root_xyz[run], target_time)
        quat = normalize_quaternions_wxyz(interp_columns(source_time, root_quat[run], target_time))
        conf = np.interp(target_time, source_time, confidence[run]).clip(0.0, 1.0)
        source_timestamp = np.interp(target_time, source_time, timestamps[run].astype(np.float64)).round().astype(np.int64)

        index = len(manifest["clips"])
        clip_stem = f"{stem}_clip_{index:03d}"
        csv_path = args.output_dir / f"{clip_stem}.csv"
        metadata_path = args.output_dir / f"{clip_stem}.metadata.npz"
        reference_path = args.output_dir / f"{clip_stem}.npz"
        # Unitree CSV root quaternion is xyzw; internal/source quaternions are wxyz.
        output = np.column_stack((xyz, quat[:, [1, 2, 3, 0]], joint_safe))
        np.savetxt(csv_path, output, delimiter=",", fmt="%.9f")
        phase = np.linspace(0.0, 1.0, len(output), dtype=np.float32)
        np.savez_compressed(
            metadata_path,
            reference_confidence=conf.astype(np.float32),
            phase=phase,
            foot_contact=np.ones((len(output), 2), dtype=np.float32),
            source_timestamp_ns=source_timestamp,
            joint_names=np.asarray(data["joint_names_23"]),
        )
        manifest["clips"].append(
            {
                "id": clip_stem,
                "csv": str(csv_path.resolve()),
                "metadata_npz": str(metadata_path.resolve()),
                "reference_npz": str(reference_path.resolve()),
                "frames": len(output),
                "duration_s": duration,
                "confidence_mean": float(conf.mean()),
                "confidence_min": float(conf.min()),
            }
        )

    if not manifest["clips"]:
        raise RuntimeError("no clean clip passed the confidence, duration and timestamp-gap gates")
    assign_dataset_splits(manifest)
    manifest_path = args.output_dir / "clips_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(
        f"REFERENCE_CLIPS_OK clips={len(manifest['clips'])} "
        f"train={len(manifest['splits']['train'])} validation=1 "
        f"manifest={manifest_path.resolve()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Replay an imitation JSONL through the final reference-motion governor."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from motion_pipeline.reference_motion import LowLatencyReferenceMotion


UPPER_BODY = {
    "waist_yaw_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
}


def distribution(values: list[float]) -> dict[str, float | int | None]:
    data = np.asarray(values, dtype=np.float64)
    return {
        "count": int(data.size),
        "p50": float(np.percentile(data, 50)) if data.size else None,
        "p90": float(np.percentile(data, 90)) if data.size else None,
        "p99": float(np.percentile(data, 99)) if data.size else None,
        "max": float(np.max(data)) if data.size else None,
    }


def load_samples(path: Path) -> list[dict]:
    samples = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            packet = json.loads(line).get("telemetry_packet") or {}
            isaac = packet.get("isaac_metrics") or {}
            if not isaac or not packet.get("safe_joint_position_rad"):
                continue
            samples.append(packet)
    if len(samples) < 2:
        raise ValueError("At least two Isaac telemetry samples are required")
    return samples


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("imitation_jsonl", type=Path)
    parser.add_argument("--response-hz", type=float, default=5.0)
    parser.add_argument("--max-velocity", type=float, default=0.65)
    parser.add_argument("--max-acceleration", type=float, default=2.5)
    parser.add_argument("--max-jerk", type=float, default=25.0)
    parser.add_argument("--physics-dt", type=float, default=0.005)
    parser.add_argument("--input-fps", type=float, default=30.0)
    args = parser.parse_args()

    samples = load_samples(args.imitation_jsonl)
    robot_names = list(samples[0]["isaac_metrics"]["joint_names"])
    source_names = list(samples[0]["joint_names"])
    source_index = {name: index for index, name in enumerate(source_names)}
    upper_names = [name for name in robot_names if name in UPPER_BODY]
    upper_robot = np.asarray([robot_names.index(name) for name in upper_names])
    upper_source = np.asarray([source_index[name] for name in upper_names])
    timestamps = np.asarray(
        [int(sample["source_timestamp_ns"]) for sample in samples],
        dtype=np.float64,
    ) / 1.0e9
    targets = np.asarray(
        [sample["safe_joint_position_rad"] for sample in samples],
        dtype=np.float64,
    )[:, upper_source]
    recorded_reference = np.asarray(
        [sample["isaac_metrics"]["reference_joint_position_rad"] for sample in samples],
        dtype=np.float64,
    )[:, upper_robot]

    governor = LowLatencyReferenceMotion(
        recorded_reference[0],
        response_hz=args.response_hz,
        max_velocity=args.max_velocity,
        max_acceleration=args.max_acceleration,
        max_jerk=args.max_jerk,
    )
    interpolated = targets[0].copy()
    interpolated_velocity = np.zeros_like(interpolated)
    trajectory_start = interpolated.copy()
    trajectory_start_velocity = interpolated_velocity.copy()
    trajectory_target = interpolated.copy()
    trajectory_started = timestamps[0]
    trajectory_duration = 1.0 / max(1.0, args.input_fps)
    now = timestamps[0]
    event_index = 1
    low_latency_errors: list[float] = []
    recorded_errors: list[float] = []
    replayed_velocity: list[float] = []
    replayed_acceleration: list[float] = []
    replayed_jerk: list[float] = []

    while event_index < len(samples):
        next_event = timestamps[event_index]
        while now + 1.0e-12 < next_event:
            previous_now = now
            now = min(next_event, now + args.physics_dt)
            step_dt = now - previous_now
            phase = float(np.clip(
                (now - trajectory_started) / trajectory_duration, 0.0, 1.0
            ))
            phase2 = phase * phase
            phase3 = phase2 * phase
            h00 = 2.0 * phase3 - 3.0 * phase2 + 1.0
            h10 = phase3 - 2.0 * phase2 + phase
            h01 = -2.0 * phase3 + 3.0 * phase2
            interpolated = (
                h00 * trajectory_start
                + h10 * trajectory_duration * trajectory_start_velocity
                + h01 * trajectory_target
            )
            dh00 = (6.0 * phase2 - 6.0 * phase) / trajectory_duration
            dh10 = 3.0 * phase2 - 4.0 * phase + 1.0
            dh01 = (-6.0 * phase2 + 6.0 * phase) / trajectory_duration
            interpolated_velocity = (
                dh00 * trajectory_start
                + dh10 * trajectory_start_velocity
                + dh01 * trajectory_target
            )
            governed = governor.update(
                interpolated, interpolated_velocity,
                step_dt,
            )
            replayed_velocity.extend(np.abs(governed.velocity).tolist())
            replayed_acceleration.extend(np.abs(governed.acceleration).tolist())
            replayed_jerk.extend(np.abs(governed.jerk).tolist())

        target = targets[event_index]
        low_latency_errors.extend(np.abs(target - governor.position).tolist())
        recorded_errors.extend(
            np.abs(target - recorded_reference[event_index]).tolist()
        )
        trajectory_start = interpolated.copy()
        trajectory_start_velocity = interpolated_velocity.copy()
        trajectory_target = target.copy()
        trajectory_started = now
        event_index += 1

    payload = {
        "samples": len(samples),
        "upper_joint_names": upper_names,
        "recorded_safe_to_reference_abs_error_rad": distribution(recorded_errors),
        "replayed_low_latency_safe_to_reference_abs_error_rad": distribution(
            low_latency_errors
        ),
        "replayed_reference_velocity_rad_s": distribution(replayed_velocity),
        "replayed_reference_acceleration_rad_s2": distribution(
            replayed_acceleration
        ),
        "replayed_reference_jerk_rad_s3": distribution(replayed_jerk),
        "profile": {
            "response_hz": args.response_hz,
            "max_velocity_rad_s": args.max_velocity,
            "max_acceleration_rad_s2": args.max_acceleration,
            "max_jerk_rad_s3": args.max_jerk,
        },
        "note": (
            "Offline approximation uses recorded source timestamps and the same "
            "30-to-200 Hz Hermite target resampler as the live simulator."
        ),
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Clean GMR references into uniform, segmented RL tracking clips."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfiltfilt


MAX_VELOCITY_RAD_S = np.asarray([6.0] * 12 + [4.0] + [8.0] * 10)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--cutoff-hz", type=float, default=6.0)
    parser.add_argument("--gap-s", type=float, default=0.20)
    parser.add_argument("--min-frames", type=int, default=30)
    return parser.parse_args()


def slew_limit(q: np.ndarray, dt: float) -> np.ndarray:
    result = q.copy()
    max_delta = MAX_VELOCITY_RAD_S * dt
    for i in range(1, len(result)):
        result[i] = result[i - 1] + np.clip(
            result[i] - result[i - 1], -max_delta, max_delta
        )
    return result


def main() -> int:
    args = parse_args()
    source = np.load(args.input)
    timestamps = source["timestamp_ns"].astype(np.int64)
    q = source["q_g1_23dof"].astype(np.float64)
    joint_names = source["joint_names_23"]
    gaps = np.diff(timestamps) * 1e-9
    cuts = np.flatnonzero(gaps > args.gap_s) + 1
    intervals = np.split(np.arange(len(timestamps)), cuts)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    index = {"schema": "g1_23dof_tracking_dataset/v1", "clips": []}
    sos = butter(4, args.cutoff_hz, fs=args.fps, output="sos")
    for source_segment, ids in enumerate(intervals):
        if len(ids) < args.min_frames:
            continue
        t = (timestamps[ids] - timestamps[ids[0]]) * 1e-9
        uniform_t = np.arange(0.0, t[-1], 1.0 / args.fps)
        if len(uniform_t) < args.min_frames:
            continue
        uniform_q = np.column_stack(
            [np.interp(uniform_t, t, q[ids, j]) for j in range(q.shape[1])]
        )
        filtered_q = sosfiltfilt(sos, uniform_q, axis=0)
        filtered_q = slew_limit(filtered_q, 1.0 / args.fps)
        qd = np.gradient(filtered_q, 1.0 / args.fps, axis=0)
        clip_name = f"{args.input.stem}_clip_{source_segment:03d}.npz"
        clip_path = args.output_dir / clip_name
        np.savez_compressed(
            clip_path,
            schema="g1_23dof_tracking_clip/v1",
            fps=args.fps,
            time_s=uniform_t,
            joint_names=joint_names,
            q_ref=filtered_q,
            qd_ref=qd,
            source=str(args.input.resolve()),
        )
        index["clips"].append(
            {
                "file": clip_name,
                "frames": len(uniform_t),
                "duration_s": float(uniform_t[-1]),
                "source_segment": source_segment,
            }
        )
    (args.output_dir / "index.json").write_text(
        json.dumps(index, indent=2), encoding="utf-8"
    )
    print(json.dumps(index, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


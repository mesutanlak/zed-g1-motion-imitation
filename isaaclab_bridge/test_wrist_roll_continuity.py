"""Replay a BODY_38 recording and guard against wrist wrap-around flips."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from gmr_live_bridge import RelativeHandRoll


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("recording", type=Path)
    args = parser.parse_args()

    tracker = RelativeHandRoll()
    values: dict[str, list[float]] = {"left": [], "right": []}
    calibrated = 0
    frames = 0
    with args.recording.open(encoding="utf-8") as stream:
        for line in stream:
            frame = json.loads(line)
            if "frame_index" not in frame:
                continue
            roll, ready = tracker.update(frame)
            frames += 1
            calibrated += int(ready)
            for side in values:
                values[side].append(roll[side])

    if frames < 2:
        raise AssertionError("recording contains fewer than two BODY_38 frames")
    if calibrated / frames < 0.90:
        raise AssertionError("wrist estimator did not calibrate reliably")

    summaries = {}
    for side, samples in values.items():
        steps = np.abs(np.diff(np.asarray(samples, dtype=np.float64)))
        maximum = float(np.max(steps))
        if maximum > 0.451:
            raise AssertionError(f"{side} wrist discontinuity: {maximum:.6f} rad")
        summaries[side] = {
            "step_p95_rad": float(np.percentile(steps, 95)),
            "step_max_rad": maximum,
            "jumps_over_0_5_rad": int(np.count_nonzero(steps > 0.5)),
        }

    print(
        "WRIST_ROLL_CONTINUITY_OK "
        f"frames={frames} calibrated={calibrated} metrics={summaries}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

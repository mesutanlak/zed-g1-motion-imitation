"""Regression test for the BODY_38 analysis JSONL/CSV recorder."""

from __future__ import annotations

import csv
import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from g1_skeleton_3d_viewer import demo_packet
from skeleton_analysis_recorder import SkeletonAnalysisRecorder


def main() -> int:
    with tempfile.TemporaryDirectory() as directory:
        recorder = SkeletonAnalysisRecorder(Path(directory))
        json_path, csv_path = recorder.start()
        first = demo_packet()
        recorder.record(first)

        second = demo_packet()
        second["sequence"] = 2
        second["timestamp_ns"] = first["timestamp_ns"] + 66_666_667
        wrist = second["keypoint_names"].index("LEFT_WRIST")
        second["keypoints_3d_m"][wrist] = [3.10, 0.61, 1.28]
        recorder.record(second)
        recorder.record(
            {
                "schema": "zed_body38_live/status/v1",
                "sequence": 3,
                "timestamp_ns": second["timestamp_ns"] + 66_666_667,
                "status": "NO_BODY",
            }
        )
        saved_json, saved_csv, count = recorder.stop()

        assert saved_json == json_path and saved_csv == csv_path
        assert count == 2
        records = [
            json.loads(line)
            for line in json_path.read_text(encoding="utf-8").splitlines()
        ]
        assert records[0]["schema"] == "zed_body38_3d_analysis/metadata/v1"
        frames = [
            record
            for record in records
            if record["schema"] == "zed_body38_3d_analysis/frame/v1"
        ]
        assert len(frames) == 2
        derived = frames[-1]["derived"]
        assert derived["joint_angles_deg"]["left_elbow_interior_deg"] is not None
        assert derived["segment_lengths_m"]["left_forearm"] > 0.1
        assert derived["keypoint_speed_m_s"]["LEFT_WRIST"] is not None
        assert len(derived["root_relative_keypoints_m"]) >= 30
        assert derived["local_orientation_euler_xyz_deg"]["LEFT_WRIST"] == [
            0.0,
            0.0,
            0.0,
        ]
        assert any(
            record["schema"] == "zed_body38_3d_analysis/status/v1"
            for record in records
        )
        with csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
            csv_rows = list(csv.DictReader(stream))
        assert len(csv_rows) == 2
        assert csv_rows[-1]["left_elbow_interior_deg"]
        print(
            "ANALYSIS_RECORDER_OK "
            f"frames={count} json_records={len(records)} csv_rows={len(csv_rows)}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

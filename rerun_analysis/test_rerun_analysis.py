"""Fast headless regression checks for the independent Rerun analyzer."""

from __future__ import annotations

import json
import csv
import subprocess
import sys
import tempfile
from pathlib import Path
from unittest.mock import patch

import rerun as rr

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from rerun_analysis.model import AnalysisConfig, SkeletonConditioner
from rerun_analysis.app import RerunSkeletonApp
from rerun_analysis.session import AnalysisSessionWriter
import rerun_analysis.session as session_module


def check_stale_joint_reacquisition() -> None:
    """An outlier must not leave a joint frozen or permanently rejected."""
    conditioner = SkeletonConditioner()
    config = AnalysisConfig(
        smoothing_alpha=1.0,
        max_joint_speed_m_s=1.0,
        occlusion_hold_frames=2,
    )

    def process(frame: int, x_m: float):
        return conditioner.process(
            {
                "timestamp_ns": 1_000_000_000 + frame * 100_000_000,
                "keypoint_names": ["RIGHT_WRIST"],
                "keypoints_3d_m": [[x_m, 0.0, 0.0]],
                "keypoint_confidence": [100.0],
            },
            config,
        )

    first, first_states = process(0, 0.0)
    if first_states["RIGHT_WRIST"] != "tracked":
        raise AssertionError(first_states)
    for frame in (1, 2):
        held, states = process(frame, 10.0)
        if states["RIGHT_WRIST"] != "held_outlier":
            raise AssertionError((frame, states, held))
    missing, missing_states = process(3, 10.0)
    if missing_states["RIGHT_WRIST"] != "velocity_rejected":
        raise AssertionError(missing_states)
    if missing["keypoints_3d_m"][0] != [None, None, None]:
        raise AssertionError(missing)
    reacquired, reacquired_states = process(4, 10.0)
    if reacquired_states["RIGHT_WRIST"] != "tracked":
        raise AssertionError((reacquired_states, reacquired))
    if reacquired["keypoints_3d_m"][0] != [10.0, 0.0, 0.0]:
        raise AssertionError(reacquired)


def check_retarget_comparison_logging(output: Path) -> None:
    rr.init("zed_g1_retarget_comparison_test")
    rr.set_sinks(rr.FileSink(output / "comparison_test.rrd"))
    app = object.__new__(RerunSkeletonApp)
    app.latest_gmr = {}
    positions = {
        "pelvis": [0.0, 0.0, 0.8],
        "torso_link": [0.0, 0.0, 1.1],
        "left_shoulder_yaw_link": [0.0, 0.2, 1.25],
        "left_elbow_link": [0.0, 0.35, 1.1],
    }
    app._log_gmr_packet(
        {
            "sequence": 1,
            "joint_names": ["left_elbow_joint"],
            "raw_joint_position_rad": [0.4],
            "safe_joint_position_rad": [0.35],
            "g1_skeleton": {
                "edges": [
                    ["pelvis", "torso_link"],
                    ["torso_link", "left_shoulder_yaw_link"],
                    ["left_shoulder_yaw_link", "left_elbow_link"],
                ],
                "raw_positions_m": positions,
                "safe_positions_m": positions,
            },
            "retarget_comparison": {
                "human_edges": [["pelvis", "spine3"]],
                "human_positions_m": {
                    "pelvis": [0.0, 0.0, 0.8],
                    "spine3": [0.0, 0.0, 1.1],
                },
            },
            "safety": {"level": "GREEN", "reasons": [], "blend": 1.0},
        }
    )
    landmarks = [
        [3.0, 0.20 + index * 0.005, 1.30 + index * 0.004]
        for index in range(21)
    ]
    app._log_hand_packet(
        {
            "timestamp_ns": 1_000_000_000,
            "hand_tracking": {
                "hands": [
                    {
                        "side": "left",
                        "valid": True,
                        "landmarks_world_m": landmarks,
                        "landmark_quality": [
                            {"camera_count": 2, "confidence": 0.9}
                            for _ in range(21)
                        ],
                        "capture_spread_ms": 8.0,
                        "rejection_reasons": [],
                    }
                ]
            },
            "dex3_targets": {
                "physical_robot_output_enabled": False,
                "left": {"safe_q_rad": [0.1] * 7},
            },
        }
    )
    rr.disconnect()
    if not (output / "comparison_test.rrd").exists():
        raise AssertionError("comparison RRD was not written")


def check_snapshot_lock_is_nonfatal(output: Path) -> None:
    """A transient OneDrive/editor lock must not terminate capture."""
    writer = object.__new__(AnalysisSessionWriter)
    writer._snapshot_warning_at = {}
    target = output / "locked_quality_summary.json"
    target.write_text("{}", encoding="utf-8")
    real_replace = session_module.os.replace

    def replace_with_locked_destination(source, destination):
        if Path(destination) == target:
            raise PermissionError(13, "simulated exclusive reader", str(target))
        return real_replace(source, destination)

    with patch.object(session_module.os, "replace", replace_with_locked_destination):
        published = writer._write_json_snapshot(target, {"frames": 42})
    if published:
        raise AssertionError("locked destination unexpectedly published")
    pending = target.with_name(f".{target.name}.pending")
    if json.loads(pending.read_text(encoding="utf-8"))["frames"] != 42:
        raise AssertionError(pending)


def main() -> int:
    check_stale_joint_reacquisition()
    project = PROJECT_ROOT
    with tempfile.TemporaryDirectory(prefix="zed_g1_rerun_test_") as temp:
        output = Path(temp)
        check_snapshot_lock_is_nonfatal(output)
        check_retarget_comparison_logging(output)
        completed = subprocess.run(
            [
                sys.executable, "-m", "rerun_analysis.app", "--demo",
                # Rerun 0.26 startup can consume most of 250 ms on Windows,
                # leaving only one demo frame and making this smoke test
                # timing-dependent.  Keep it short but allow three frames.
                "--no-viewer", "--headless", "--seconds", "0.75",
                "--output-dir", str(output),
            ],
            cwd=project,
            check=False,
            text=True,
            capture_output=True,
            timeout=45,
        )
        if completed.returncode:
            print(completed.stdout)
            print(completed.stderr, file=sys.stderr)
            return completed.returncode
        manifests = list(output.glob("rerun_body38_*/session_manifest.json"))
        rrd_files = list(output.glob("rerun_body38_*.rrd"))
        if len(manifests) != 1 or len(rrd_files) != 1:
            raise AssertionError((manifests, rrd_files))
        manifest = json.loads(manifests[0].read_text(encoding="utf-8"))
        if manifest.get("frame_count", 0) < 2:
            raise AssertionError(manifest)
        session = manifests[0].parent
        for name in (
            "skeleton_analysis.jsonl", "frames.csv", "joints.csv", "angles.csv",
            "imitation_comparison.csv", "quality_summary.json",
        ):
            path = session / name
            if not path.exists() or path.stat().st_size < 50:
                raise AssertionError(path)
        if not (session / "imitation_comparison.jsonl").exists():
            raise AssertionError(session / "imitation_comparison.jsonl")
        quality = json.loads(
            (session / "quality_summary.json").read_text(encoding="utf-8")
        )
        if quality.get("schema") != "zed_g1_quality_summary/v1":
            raise AssertionError(quality)
        with (session / "frames.csv").open(encoding="utf-8-sig") as stream:
            frame_fields = next(csv.reader(stream))
        for field in (
            "fusion_mode", "evidence_views", "cross_view_mpjpe_m",
            "mean_camera_fused", "camera_latency_max_ms",
            "camera_network_queue_max_ms", "camera_clock_offset_span_ms",
            "selection_timestamp_delta_ms", "workspace_excluded_serials",
            "left_arm_clear_views", "left_arm_selected_serials",
            "left_arm_orientation_source",
        ):
            if field not in frame_fields:
                raise AssertionError((field, frame_fields))
        with (session / "imitation_comparison.csv").open(
            encoding="utf-8-sig"
        ) as stream:
            imitation_fields = next(csv.reader(stream))
        for field in (
            "left_direct_arm_ik_rms_m", "left_raw_elbow_joint_rad",
            "gmr_upper_relative_residual_m", "mirror_rescue_applied",
            "reference_confidence", "reference_velocity_max_rad_s",
            "fusion_state", "left_direct_ik_selected_source",
            "right_direct_ik_forearm_error_deg",
            "left_front_clearance_blend", "right_front_clearance_shift_m",
            "robot_body_barrier_projection_alpha",
        ):
            if field not in imitation_fields:
                raise AssertionError((field, imitation_fields))
        print(
            "RERUN_ANALYSIS_OK "
            f"frames={manifest['frame_count']} rrd={rrd_files[0].name}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

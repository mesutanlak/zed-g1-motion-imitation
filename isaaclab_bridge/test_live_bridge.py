"""Loopback smoke test for the GMR UDP bridge."""

from __future__ import annotations

import argparse
import json
import math
import socket
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np


def to_live(record: dict) -> dict:
    # Preserve pelvis-local coordinates, calibration and occlusion metadata so
    # this regression test exercises the exact live retargeting path.
    packet = dict(record)
    packet.update({
        "schema": "zed_body38_live/v1",
        "sequence": record["frame_index"],
        "timestamp_ns": record["timestamp_ns"],
        "coordinate_system": record["coordinate_system"],
        "units": record["units"],
        "body_id": record["body_id"],
        "body_confidence": record["body_confidence"],
        "root_position_m": record["root_position_m"],
        "global_root_orientation_xyzw": record["global_root_orientation_xyzw"],
        "keypoint_names": record["keypoint_names"],
        "keypoints_3d_m": record["keypoints_3d_filtered_m"],
        "keypoint_confidence": record["keypoint_confidence"],
        "local_orientation_per_joint_xyzw": record[
            "local_orientation_per_joint_xyzw"
        ],
        "operator_selection": record.get("operator_selection") or {
            "state": "LOCKED",
            "locked_body_id": record["body_id"],
            "automatic_handover": False,
        },
        "calibration": record.get("calibration") or {
            "state": "READY", "profile": {}
        },
    })
    # Old captures include acquisition frames before the operator lock. The
    # solver smoke test is about retargeting, not acquisition timing.
    packet["operator_selection"] = {
        **packet["operator_selection"], "state": "LOCKED"
    }
    if (packet.get("calibration") or {}).get("state") != "READY":
        packet["calibration"] = {"state": "READY", "profile": {}}
    return packet


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("recording", type=Path)
    parser.add_argument("--start-sequence", type=int, default=0)
    parser.add_argument("--packet-count", type=int, default=20)
    parser.add_argument("--disable-mirror", action="store_true")
    args = parser.parse_args()
    recording = args.recording
    bridge = Path(__file__).with_name("gmr_live_bridge.py")
    # Regression replay must coexist with a real Isaac/ZED session.  Fixed
    # 15050/15051 ports could silently feed the live bridge or block forever.
    def available_udp_port() -> int:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.bind(("127.0.0.1", 0))
            return int(probe.getsockname()[1])
        finally:
            probe.close()

    input_port = available_udp_port()
    output_port = available_udp_port()
    bridge_command = [
            sys.executable,
            str(bridge),
            "--listen-host",
            "127.0.0.1",
            "--listen-port",
            str(input_port),
            "--output-host",
            "127.0.0.1",
            "--output-port",
            str(output_port),
        ]
    if args.disable_mirror:
        bridge_command.append("--no-mirror-rescue")
    process = subprocess.Popen(bridge_command)
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", output_port))
    receiver.settimeout(10.0)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    received_packets = 0
    relative_residuals: list[float] = []
    absolute_residuals: list[float] = []
    safety_levels: Counter[str] = Counter()
    safety_reasons: Counter[str] = Counter()
    mirror_solve_ms: list[float] = []
    mirror_triggered = 0
    mirror_applied = 0
    mirror_reasons: Counter[str] = Counter()
    mirror_task_improvement_m: list[float] = []
    mirror_margin_improvement_m: list[float] = []
    try:
        time.sleep(2.0)
        with recording.open("r", encoding="utf-8") as stream:
            for line in stream:
                record = json.loads(line)
                if record.get("schema") != "zed_body38_g1_reference/v1":
                    continue
                if int(record.get("frame_index", 0)) < args.start_sequence:
                    continue
                if (record.get("calibration") or {}).get("state") != "READY":
                    continue
                if (record.get("operator_selection") or {}).get("state") != "LOCKED":
                    continue
                sender.sendto(
                    json.dumps(to_live(record), separators=(",", ":")).encode(),
                    ("127.0.0.1", input_port),
                )
                try:
                    packet = json.loads(receiver.recv(65535))
                except socket.timeout:
                    continue
                if packet.get("schema") == "zed_gmr_g1_23dof_live/v1":
                    assert len(packet["joint_names"]) == 23
                    assert len(packet["joint_position_rad"]) == 23
                    assert all(
                        math.isfinite(float(value))
                        for value in packet["joint_position_rad"]
                    )
                    raw_by_name = dict(
                        zip(packet["joint_names"], packet["raw_joint_position_rad"])
                    )
                    safe_by_name = dict(
                        zip(packet["joint_names"], packet["safe_joint_position_rad"])
                    )
                    for elbow in ("left_elbow_joint", "right_elbow_joint"):
                        assert float(raw_by_name[elbow]) >= -1.0e-5, (
                            elbow, raw_by_name[elbow]
                        )
                        assert float(safe_by_name[elbow]) >= -1.0e-5, (
                            f"safe_{elbow}", safe_by_name[elbow]
                        )
                    comparison = packet.get("retarget_comparison") or {}
                    assert comparison.get("human_positions_m"), comparison
                    skeleton = packet.get("g1_skeleton") or {}
                    assert skeleton.get("raw_positions_m"), skeleton
                    assert skeleton.get("safe_positions_m"), skeleton
                    received_packets += 1
                    metrics = packet.get("bridge_metrics") or {}
                    relative = metrics.get("ik_upper_relative_residual_m")
                    absolute = metrics.get("ik_upper_position_max_m")
                    if relative is not None and math.isfinite(float(relative)):
                        relative_residuals.append(float(relative))
                    if absolute is not None and math.isfinite(float(absolute)):
                        absolute_residuals.append(float(absolute))
                    safety = packet.get("safety") or {}
                    safety_levels[str(safety.get("level", "UNKNOWN"))] += 1
                    safety_reasons.update(str(reason) for reason in safety.get("reasons", []))
                    if bool(metrics.get("mirror_rescue_triggered")):
                        mirror_triggered += 1
                        mirror_reasons.update(
                            str(reason)
                            for reason in metrics.get("mirror_rescue_reasons", [])
                        )
                    if bool(metrics.get("mirror_rescue_applied")):
                        mirror_applied += 1
                        mirror_task_improvement_m.append(
                            float(metrics.get("mirror_nominal_task_error_m", 0.0))
                            - float(metrics.get("mirror_selected_task_error_m", 0.0))
                        )
                        mirror_margin_improvement_m.append(
                            float(metrics.get("mirror_selected_collision_margin_m", 0.0))
                            - float(metrics.get("mirror_nominal_collision_margin_m", 0.0))
                        )
                    mirror_ms = metrics.get("mirror_rescue_solve_ms")
                    if mirror_ms is not None and math.isfinite(float(mirror_ms)):
                        mirror_solve_ms.append(float(mirror_ms))
                    max_abs_q = max(
                        abs(float(value))
                        for value in packet["joint_position_rad"]
                    )
                    if received_packets >= args.packet_count:
                        print(
                            "LIVE_BRIDGE_OK "
                            f"sequence={packet['sequence']} "
                            f"packets={received_packets} "
                            f"targets={packet['valid_targets']} "
                            f"safety={packet.get('safety', {}).get('level')} "
                            f"reasons={packet.get('safety', {}).get('reasons')} "
                            f"collision_pairs={packet.get('safety', {}).get('collision_risk_pairs')} "
                            f"collision_margin={packet.get('safety', {}).get('minimum_collision_margin_m')} "
                            f"mirror_triggered={packet.get('bridge_metrics', {}).get('mirror_rescue_triggered')} "
                            f"mirror_applied={packet.get('bridge_metrics', {}).get('mirror_rescue_applied')} "
                            f"mirror_ms={packet.get('bridge_metrics', {}).get('mirror_rescue_solve_ms')} "
                            f"mirror_counts={mirror_applied}/{mirror_triggered} "
                            f"mirror_reasons={dict(mirror_reasons)} "
                            f"mirror_p95_ms={np.percentile(mirror_solve_ms, 95):.2f} "
                            f"mirror_task_gain_mm={1000*np.mean(mirror_task_improvement_m) if mirror_task_improvement_m else 0:.2f} "
                            f"mirror_margin_gain_mm={1000*np.mean(mirror_margin_improvement_m) if mirror_margin_improvement_m else 0:.2f} "
                            f"saturation={packet.get('safety', {}).get('joint_limit_saturation_names')} "
                            f"ik_upper_max={packet.get('bridge_metrics', {}).get('ik_upper_position_max_m')} "
                            f"ik_upper_relative={packet.get('bridge_metrics', {}).get('ik_upper_relative_residual_m')} "
                            f"relative_p50={np.percentile(relative_residuals, 50):.4f} "
                            f"relative_p95={np.percentile(relative_residuals, 95):.4f} "
                            f"absolute_p95={np.percentile(absolute_residuals, 95):.4f} "
                            f"levels={dict(safety_levels)} "
                            f"reason_counts={dict(safety_reasons)} "
                            f"max_abs_q={max_abs_q:.3f}"
                        )
                        return 0
        raise RuntimeError("bridge produced no valid packet")
    finally:
        sender.close()
        receiver.close()
        process.terminate()
        process.wait(timeout=5)


if __name__ == "__main__":
    raise SystemExit(main())

"""Loopback smoke test for the GMR UDP bridge."""

from __future__ import annotations

import json
import math
import socket
import subprocess
import sys
import time
from pathlib import Path


def to_live(record: dict) -> dict:
    return {
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
    }


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: test_live_bridge.py RECORDING.jsonl")
    recording = Path(sys.argv[1])
    bridge = Path(__file__).with_name("gmr_live_bridge.py")
    process = subprocess.Popen(
        [
            sys.executable,
            str(bridge),
            "--listen-host",
            "127.0.0.1",
            "--listen-port",
            "15050",
            "--output-host",
            "127.0.0.1",
            "--output-port",
            "15051",
        ]
    )
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 15051))
    receiver.settimeout(10.0)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        time.sleep(2.0)
        with recording.open("r", encoding="utf-8") as stream:
            for line in stream:
                record = json.loads(line)
                if record.get("schema") != "zed_body38_g1_reference/v1":
                    continue
                sender.sendto(
                    json.dumps(to_live(record), separators=(",", ":")).encode(),
                    ("127.0.0.1", 15050),
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
                    print(
                        "LIVE_BRIDGE_OK "
                        f"sequence={packet['sequence']} "
                        f"targets={packet['valid_targets']}"
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

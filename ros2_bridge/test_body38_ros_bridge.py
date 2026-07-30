"""End-to-end ROS 2 test for the BODY_38 UDP visualization bridge."""

from __future__ import annotations

import json
import socket
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import MarkerArray

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "analysis_panel"))

from g1_skeleton_3d_viewer import demo_packet


def main() -> int:
    port = 15064
    bridge = Path(__file__).with_name("body38_udp_ros2.py")
    process = subprocess.Popen(
        [
            sys.executable,
            str(bridge),
            "--listen-host",
            "127.0.0.1",
            "--listen-port",
            str(port),
            "--frame-id",
            "zed_camera",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    rclpy.init()
    node = rclpy.create_node("body38_bridge_test_receiver")
    received = {"markers": None, "poses": None}
    node.create_subscription(
        MarkerArray,
        "/zed/body38/markers",
        lambda message: received.__setitem__("markers", message),
        1,
    )
    node.create_subscription(
        PoseArray,
        "/zed/body38/keypoints",
        lambda message: received.__setitem__("poses", message),
        qos_profile_sensor_data,
    )
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        time.sleep(0.8)
        deadline = time.monotonic() + 5.0
        sequence = 0
        while time.monotonic() < deadline and (
            received["markers"] is None or received["poses"] is None
        ):
            sequence += 1
            packet = demo_packet()
            packet["sequence"] = sequence
            packet["timestamp_ns"] = time.time_ns()
            sender.sendto(
                json.dumps(packet, separators=(",", ":")).encode("utf-8"),
                ("127.0.0.1", port),
            )
            rclpy.spin_once(node, timeout_sec=0.15)
        assert received["markers"] is not None
        assert received["poses"] is not None
        assert len(received["markers"].markers) >= 2
        assert len(received["poses"].poses) >= 30
        print(
            "BODY38_ROS_BRIDGE_OK "
            f"markers={len(received['markers'].markers)} "
            f"poses={len(received['poses'].poses)}"
        )
    finally:
        sender.close()
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            output, _ = process.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=3)
        if process.returncode not in (0, -15):
            print(output, file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

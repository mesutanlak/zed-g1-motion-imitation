"""Regression test for ZED R/reset propagation through the GMR bridge."""

from __future__ import annotations

import argparse
import json
import socket
import subprocess
import sys
import time
from pathlib import Path

try:
    from isaaclab_bridge.test_live_bridge import to_live
except ModuleNotFoundError:  # Direct execution from this directory.
    from test_live_bridge import to_live


def send_until_matching(
    sender: socket.socket,
    receiver: socket.socket,
    destination: tuple[str, int],
    payload: dict,
    predicate,
    timeout_s: float = 15.0,
) -> dict:
    """Retry startup UDP because the bridge binds after loading GMR assets."""
    deadline = time.monotonic() + timeout_s
    encoded = json.dumps(payload).encode()
    while time.monotonic() < deadline:
        sender.sendto(encoded, destination)
        try:
            receiver.settimeout(0.25)
            packet = json.loads(receiver.recv(65535))
        except socket.timeout:
            continue
        if predicate(packet):
            return packet
    raise TimeoutError("Expected bridge packet was not received after retries")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("recording", type=Path)
    args = parser.parse_args()
    frames = []
    with args.recording.open("r", encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            if record.get("schema") != "zed_body38_g1_reference/v1":
                continue
            if (record.get("calibration") or {}).get("state") != "READY":
                continue
            frames.append(to_live(record))
            if len(frames) >= 12:
                break
    if len(frames) < 12:
        raise RuntimeError("Recording has too few READY BODY_38 frames")

    listen_port, output_port = 15150, 15151
    bridge = Path(__file__).with_name("gmr_live_bridge.py")
    process = subprocess.Popen(
        [
            sys.executable, str(bridge),
            "--listen-host", "127.0.0.1", "--listen-port", str(listen_port),
            "--output-host", "127.0.0.1", "--output-port", str(output_port),
            "--input-fps", "60",
        ]
    )
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", output_port))
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        time.sleep(2.0)
        before = send_until_matching(
            sender, receiver, ("127.0.0.1", listen_port), frames[0],
            lambda packet: packet.get("schema") == "zed_gmr_g1_23dof_live/v1",
        )

        blocked = dict(frames[1])
        blocked["calibration"] = {"state": "COLLECTING", "progress": 0.0}
        reset = send_until_matching(
            sender, receiver, ("127.0.0.1", listen_port), blocked,
            lambda packet: packet.get("status") == "CONTROL_SESSION_RESET",
        )

        after = send_until_matching(
            sender, receiver, ("127.0.0.1", listen_port), frames[2],
            lambda packet: packet.get("schema")
            == "zed_gmr_g1_23dof_live/v1",
        )
        before_id = int(before.get("control_session_id", -1))
        reset_id = int(reset.get("control_session_id", -1))
        after_id = int(after.get("control_session_id", -1))
        assert reset_id == before_id + 1, (before_id, reset_id)
        assert after_id == reset_id, (reset_id, after_id)
        print(
            "CONTROL_SESSION_RESET_OK "
            f"before={before_id} reset={reset_id} after={after_id}"
        )
        return 0
    finally:
        sender.close()
        receiver.close()
        process.terminate()
        process.wait(timeout=5)


if __name__ == "__main__":
    raise SystemExit(main())

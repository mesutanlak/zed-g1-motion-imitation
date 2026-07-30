"""End-to-end UDP recording test for the headless analysis panel."""

from __future__ import annotations

import json
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from g1_skeleton_3d_viewer import demo_packet


def main() -> int:
    viewer = Path(__file__).with_name("g1_skeleton_3d_viewer.py")
    port = 15062
    with tempfile.TemporaryDirectory() as directory:
        process = subprocess.Popen(
            [
                sys.executable,
                str(viewer),
                "--headless",
                "--seconds",
                "2",
                "--record",
                "--record-dir",
                directory,
                "--listen-port",
                str(port),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            time.sleep(0.4)
            for sequence in range(1, 8):
                packet = demo_packet()
                packet["sequence"] = sequence
                packet["timestamp_ns"] = time.time_ns()
                sender.sendto(
                    json.dumps(packet, separators=(",", ":")).encode("utf-8"),
                    ("127.0.0.1", port),
                )
                time.sleep(0.05)
            output, _ = process.communicate(timeout=5)
        finally:
            sender.close()
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=3)
        if process.returncode != 0:
            raise RuntimeError(output)
        json_files = list(Path(directory).glob("*.jsonl"))
        csv_files = list(Path(directory).glob("*.csv"))
        assert len(json_files) == 1 and len(csv_files) == 1
        frames = [
            json.loads(line)
            for line in json_files[0].read_text(encoding="utf-8").splitlines()
            if '"zed_body38_3d_analysis/frame/v1"' in line
        ]
        assert len(frames) == 7
        print(f"UDP_RECORDING_E2E_OK frames={len(frames)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

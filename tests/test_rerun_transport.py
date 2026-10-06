from __future__ import annotations

import json
import socket
import time

from rerun_analysis.transport import LatestUdpReceiver


def test_udp_ingest_journals_every_packet_but_exposes_latest_only(tmp_path):
    receiver = LatestUdpReceiver(
        "127.0.0.1",
        0,
        channel="body",
        journal_path=tmp_path / "body.jsonl",
        allowed_schemas=("zed_body38_live/v1",),
        require_keypoints=True,
    )
    receiver.start()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for sequence in range(30):
            packet = {
                "schema": "zed_body38_live/v1",
                "sequence": sequence,
                "timestamp_ns": sequence + 1,
                "keypoint_names": ["PELVIS"],
            }
            sender.sendto(
                json.dumps(packet).encode(), ("127.0.0.1", receiver.port)
            )
        deadline = time.monotonic() + 2.0
        while receiver.statistics.received < 30 and time.monotonic() < deadline:
            time.sleep(0.01)
        latest = receiver.take_latest(timeout_s=1.0)
    finally:
        sender.close()
        receiver.close()
    assert latest is not None and latest["sequence"] == 29
    lines = (tmp_path / "body.jsonl").read_text().splitlines()
    assert len(lines) == 30
    assert receiver.statistics.journal_dropped == 0
    assert receiver.statistics.sequence_gaps == 0
    assert receiver.statistics.latest_replaced >= 29

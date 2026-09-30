from __future__ import annotations

from motion_pipeline.imu_fusion import (
    ArmImuMeasurement,
    TimeAlignedImuBuffer,
    elbow_interior_deg,
    fuse_elbow_flexion,
    parse_arm_imu_packet,
)
from motion_pipeline.imu_udp import ArmImuUdpReceiver

import json
import socket


def packet(timestamp_ns: int, flexion_deg: float = 90.0, quality: float = 0.95) -> dict:
    return {
        "schema": "arm_imu/v1",
        "sequence": timestamp_ns,
        "timestamp_ns": timestamp_ns,
        "side": "left",
        "valid": True,
        "quality": quality,
        "failure_codes": [],
        "elbow": {
            "flexion_deg": flexion_deg,
            "velocity_deg_s": 0.0,
            "relative_3d_deg": flexion_deg,
            "off_axis_deg": 1.0,
        },
        "relative_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "upper_arm": {
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "segment_axis_world_xyz": [1.0, 0.0, 0.0],
        },
        "forearm": {
            "quaternion_xyzw": [0.0, 0.0, 0.7071068, 0.7071068],
            "segment_axis_world_xyz": [0.0, 1.0, 0.0],
        },
    }


def test_packet_parse_and_timestamp_interpolation() -> None:
    buffer = TimeAlignedImuBuffer()
    buffer.append(parse_arm_imu_packet(packet(1_000_000_000, 80.0)))
    buffer.append(parse_arm_imu_packet(packet(1_100_000_000, 100.0)))
    sample = buffer.align(1_050_000_000, side="left")
    assert sample is not None and sample.interpolated
    assert abs(sample.flexion_deg - 90.0) < 1.0e-9
    assert sample.upper_segment_axis_world_xyz == (1.0, 0.0, 0.0)
    assert sample.forearm_quaternion_xyzw is not None


def test_stale_packet_is_not_aligned() -> None:
    buffer = TimeAlignedImuBuffer()
    buffer.append(parse_arm_imu_packet(packet(1_000_000_000)))
    assert buffer.align(1_300_000_000, side="left", max_age_ms=100.0) is None


def test_coherent_zed_and_imu_are_blended() -> None:
    imu = parse_arm_imu_packet(packet(1_000_000_000, 90.0))
    # A 92 degree human flexion is an 88 degree interior angle.
    result = fuse_elbow_flexion(
        zed_interior_deg=88.0, zed_confidence=90.0, imu=imu
    )
    assert result.usable and result.source == "fused"
    assert 90.0 < result.fused_flexion_deg < 92.0
    assert result.disagreement_deg == 2.0


def test_large_high_quality_conflict_is_exposed() -> None:
    imu = parse_arm_imu_packet(packet(1_000_000_000, 40.0))
    result = fuse_elbow_flexion(
        zed_interior_deg=80.0, zed_confidence=95.0, imu=imu
    )
    assert not result.usable and result.source == "conflict"
    assert "ZED_IMU_ELBOW_CONFLICT" in result.failure_codes


def test_low_confidence_zed_can_fall_back_to_good_imu() -> None:
    imu = parse_arm_imu_packet(packet(1_000_000_000, 95.0))
    result = fuse_elbow_flexion(
        zed_interior_deg=150.0, zed_confidence=20.0, imu=imu
    )
    assert result.usable and result.source == "imu_fallback"
    assert result.fused_flexion_deg == 95.0


def test_body38_geometry_uses_interior_angle() -> None:
    interior = elbow_interior_deg((0, 0, 0), (1, 0, 0), (1, 1, 0))
    assert interior == 90.0


def test_udp_receiver_rejects_bad_packet_and_accepts_valid_packet() -> None:
    with ArmImuUdpReceiver(host="127.0.0.1", port=0) as receiver:
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sender.sendto(b"not-json", receiver.address)
            sender.sendto(json.dumps(packet(1_000_000_000)).encode("utf-8"), receiver.address)
            assert receiver.poll() == 1
            assert receiver.received == 1
            assert receiver.rejected == 1
            assert receiver.align(1_000_000_000, side="left") is not None
        finally:
            sender.close()

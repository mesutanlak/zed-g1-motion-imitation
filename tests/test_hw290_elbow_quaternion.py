from __future__ import annotations

import math
import sys
import types
from pathlib import Path


CAPTURE = Path(__file__).resolve().parents[1] / "imu_capture"
sys.path.insert(0, str(CAPTURE))

# The real hardware module loads Linux libc/I2C at import time.  Quaternion
# tests run on Windows with a stub; Raspberry Pi uses the real module.
hardware = types.ModuleType("dual_hw290_test")
hardware.configure_compass = lambda _fd: None
hardware.configure_mpu6050 = lambda _fd: {}
hardware.read_compass = lambda _fd, _config: None
hardware.read_motion = lambda _fd, _accel, _gyro: ((0.0, 0.0, 9.81), (0.0, 0.0, 0.0))
sys.modules["dual_hw290_test"] = hardware

import hw290_elbow_quaternion as elbow  # noqa: E402
from arm_imu_protocol import build_arm_imu_packet  # noqa: E402


def axis_angle(axis: tuple[float, float, float], angle_deg: float) -> elbow.Quaternion:
    length = math.sqrt(sum(value * value for value in axis))
    half = math.radians(angle_deg) / 2.0
    scale = math.sin(half) / length
    return elbow.q_normalize(
        (math.cos(half), axis[0] * scale, axis[1] * scale, axis[2] * scale)
    )


def measured_delta(
    upper_mount: elbow.Quaternion,
    forearm_mount: elbow.Quaternion,
    common_motion: elbow.Quaternion,
    joint_motion: elbow.Quaternion,
) -> elbow.Quaternion:
    upper_neutral = upper_mount
    forearm_neutral = forearm_mount
    neutral = elbow.relative_orientation(upper_neutral, forearm_neutral)

    upper_now = elbow.q_multiply(common_motion, upper_mount)
    forearm_now = elbow.q_multiply(
        elbow.q_multiply(common_motion, joint_motion), forearm_mount
    )
    current = elbow.relative_orientation(upper_now, forearm_now)
    return elbow.delta_orientation(neutral, current)


def test_total_relative_angle_is_invariant_to_fixed_sensor_mount_rotation() -> None:
    joint = axis_angle((0.2, 0.9, -0.3), 93.0)
    common = axis_angle((0.3, -0.4, 0.8), 47.0)
    mounts = [
        (axis_angle((1.0, 0.0, 0.0), 0.0), axis_angle((0.0, 1.0, 0.0), 0.0)),
        (axis_angle((1.0, 2.0, 3.0), 71.0), axis_angle((-2.0, 1.0, 0.5), 123.0)),
        (axis_angle((0.0, 0.0, 1.0), 180.0), axis_angle((1.0, 0.0, 0.0), 90.0)),
    ]
    for upper_mount, forearm_mount in mounts:
        delta = measured_delta(upper_mount, forearm_mount, common, joint)
        assert math.isclose(elbow.total_angle_deg(delta), 93.0, abs_tol=1.0e-9)


def test_legacy_x_angle_changes_when_sensor_mount_changes() -> None:
    joint = axis_angle((0.0, 1.0, 0.0), 90.0)
    common = axis_angle((1.0, 0.0, 0.0), 25.0)
    first = measured_delta(
        axis_angle((1.0, 0.0, 0.0), 0.0),
        axis_angle((1.0, 0.0, 0.0), 0.0),
        common,
        joint,
    )
    second = measured_delta(
        axis_angle((1.0, 1.0, 0.0), 54.0),
        axis_angle((0.0, 1.0, 1.0), 88.0),
        common,
        joint,
    )
    assert math.isclose(elbow.total_angle_deg(first), 90.0, abs_tol=1.0e-9)
    assert math.isclose(elbow.total_angle_deg(second), 90.0, abs_tol=1.0e-9)
    assert not math.isclose(
        elbow.x_segment_angle_deg(first),
        elbow.x_segment_angle_deg(second),
        abs_tol=1.0,
    )


def test_quaternion_average_handles_antipodal_samples() -> None:
    expected = axis_angle((1.0, 2.0, 3.0), 35.0)
    negative = tuple(-value for value in expected)
    result = elbow.q_average([expected, negative, expected])
    assert elbow.total_angle_deg(elbow.delta_orientation(expected, result)) < 1.0e-9


def test_calibrated_segment_angle_ignores_common_motion_and_forearm_roll() -> None:
    upper_mount = axis_angle((0.2, 1.0, -0.3), 12.0)
    forearm_mount = axis_angle((-0.4, 0.3, 1.0), 17.0)
    upper_axis, forearm_axis = elbow.calibrated_segment_axes(
        upper_mount, forearm_mount, (1.0, 0.0, 0.0)
    )
    common_axis = elbow.q_rotate(upper_mount, upper_axis)
    hinge_axis = elbow.vector_normalize(
        elbow.vector_cross(common_axis, (0.0, 0.0, 1.0))
    )
    common_motion = axis_angle((0.3, -0.6, 0.7), 113.0)
    flexion = axis_angle(hinge_axis, 92.0)
    pronation = axis_angle(common_axis, 76.0)
    upper_now = elbow.q_multiply(common_motion, upper_mount)
    forearm_now = elbow.q_multiply(
        elbow.q_multiply(elbow.q_multiply(common_motion, flexion), pronation),
        forearm_mount,
    )
    measured, _upper_world, _forearm_world = elbow.segment_flexion_deg(
        upper_now, forearm_now, upper_axis, forearm_axis
    )
    assert math.isclose(measured, 92.0, abs_tol=1.0e-9)


def test_accel_axis_calibration_applies_offset_and_scale() -> None:
    corrected = elbow.apply_accel_calibration(
        (3.0, 5.0, 7.0),
        {
            "accel_offset_m_s2": [1.0, 1.0, 1.0],
            "accel_scale": [2.0, 0.5, 1.0],
        },
    )
    assert corrected == (4.0, 2.0, 6.0)


def test_protocol_accepts_hw290_quality_without_bno_calibration_tuple() -> None:
    sensor = {
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        "quaternion_held": False,
        "quaternion_outlier_rejected": False,
    }
    packet = build_arm_imu_packet(
        sequence=1,
        timestamp_ns=1_000_000_000,
        side="left",
        flexion_deg=90.0,
        velocity_deg_s=5.0,
        relative_3d_deg=95.0,
        off_axis_deg=None,
        upper=sensor,
        forearm=sensor,
        read_skew_ms=1.0,
        sample_period_ms=20.0,
        quality_override=0.82,
        extra_failure_codes=("FOREARM_ACCEL_MAGNITUDE_IMPLAUSIBLE",),
    )
    assert packet["valid"]
    assert packet["quality"] == 0.82
    assert "FOREARM_ACCEL_MAGNITUDE_IMPLAUSIBLE" in packet["failure_codes"]

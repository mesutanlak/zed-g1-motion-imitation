"""Build the compact network packet shared by Pi capture and ZED consumers."""

from __future__ import annotations

import math
from typing import Any, Iterable


def calibration_quality(calibration: tuple[int, int, int, int]) -> tuple[float, list[str]]:
    system, gyro, accel, _mag = (int(value) for value in calibration)
    failures: list[str] = []
    if system < 2:
        failures.append("IMU_SYSTEM_CALIBRATION")
    if gyro < 3:
        failures.append("IMU_GYRO_CALIBRATION")
    if accel < 2:
        failures.append("IMU_ACCEL_CALIBRATION")
    # IMUPLUS deliberately does not use the magnetometer.
    score = min(1.0, (system / 3.0) * 0.25 + (gyro / 3.0) * 0.50 + (accel / 3.0) * 0.25)
    return score, failures


def build_arm_imu_packet(
    *,
    sequence: int,
    timestamp_ns: int,
    side: str,
    flexion_deg: float,
    velocity_deg_s: float | None,
    relative_3d_deg: float,
    off_axis_deg: float | None,
    upper: dict[str, Any],
    forearm: dict[str, Any],
    read_skew_ms: float,
    sample_period_ms: float,
    quality_override: float | None = None,
    extra_failure_codes: Iterable[str] = (),
) -> dict[str, Any]:
    failures: list[str] = [str(value) for value in extra_failure_codes]
    if quality_override is None:
        upper_quality, upper_failures = calibration_quality(tuple(upper["calibration"]))
        fore_quality, fore_failures = calibration_quality(tuple(forearm["calibration"]))
        failures.extend(f"UPPER_{item}" for item in upper_failures)
        failures.extend(f"FOREARM_{item}" for item in fore_failures)
        quality = min(upper_quality, fore_quality)
    else:
        # MPU6050/HW-290 has no BNO055-style 0..3 calibration status.  Its
        # capture process supplies quality from gyro noise, acceleration
        # plausibility, timing and saturation diagnostics instead.
        quality = max(0.0, min(1.0, float(quality_override)))
    timing_valid = read_skew_ms <= 15.0 and sample_period_ms <= 75.0
    if read_skew_ms > 15.0:
        failures.append("IMU_READ_SKEW")
    if sample_period_ms > 75.0:
        failures.append("IMU_SAMPLE_LATE")
    speed_valid = velocity_deg_s is None or abs(float(velocity_deg_s)) <= 1200.0
    if not speed_valid:
        failures.append("IMU_HIGH_ANGULAR_SPEED")
    if off_axis_deg is not None and off_axis_deg > 35.0:
        failures.append("IMU_OFF_AXIS_ROTATION")
    elbow_in_range = math.isfinite(flexion_deg) and -15.0 <= flexion_deg <= 165.0
    if not elbow_in_range:
        failures.append("IMU_ELBOW_RANGE")
    quaternion_held = bool(upper.get("quaternion_held") or forearm.get("quaternion_held"))
    quaternion_outlier = bool(
        upper.get("quaternion_outlier_rejected")
        or forearm.get("quaternion_outlier_rejected")
    )
    if quaternion_outlier:
        failures.append("IMU_QUATERNION_OUTLIER_REJECTED")
    if quaternion_held:
        failures.append("IMU_QUATERNION_HELD")
    quality *= max(0.0, 1.0 - max(0.0, read_skew_ms - 5.0) / 50.0)
    if quaternion_held:
        quality *= 0.5
    valid = bool(
        elbow_in_range
        and quality >= 0.55
        and not quaternion_held
        and timing_valid
        and speed_valid
    )
    return {
        "schema": "arm_imu/v1",
        "sequence": int(sequence),
        "timestamp_ns": int(timestamp_ns),
        "side": str(side),
        "valid": valid,
        "quality": round(quality, 4),
        "failure_codes": sorted(set(failures)),
        "frame_convention": {
            "quaternion_order": "xyzw",
            "angular_velocity": "rad_s",
            "linear_acceleration": "m_s2",
            "flexion_zero": "straight_arm",
            "flexion_positive": "elbow_flexion",
        },
        "elbow": {
            "flexion_deg": float(flexion_deg),
            "velocity_deg_s": None if velocity_deg_s is None else float(velocity_deg_s),
            "relative_3d_deg": float(relative_3d_deg),
            "off_axis_deg": None if off_axis_deg is None else float(off_axis_deg),
        },
        "upper_arm": upper,
        "forearm": forearm,
        "timing": {
            "read_skew_ms": float(read_skew_ms),
            "sample_period_ms": float(sample_period_ms),
        },
    }

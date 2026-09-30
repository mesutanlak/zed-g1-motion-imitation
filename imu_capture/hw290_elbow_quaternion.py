"""Two HW-290 boards: wearable elbow capture and optional ZED UDP stream.

Mount both boards so their +X arrows point approximately shoulder-to-hand.
Neutral calibration removes the remaining fixed case alignment error.  The
primary flexion result is the angle between the calibrated upper-arm and
forearm longitudinal axes, so shoulder motion and forearm pronation do not
become elbow flexion.

HW290-1 (/dev/i2c-1) is the upper-arm sensor.
HW290-2 (/dev/i2c-8) is the forearm sensor.
"""

from __future__ import annotations

import argparse
import inspect
import json
import math
from pathlib import Path
import socket
import statistics
import time
from typing import Any

from arm_imu_protocol import build_arm_imu_packet

from dual_hw290_test import (
    configure_compass,
    configure_mpu6050,
    read_compass,
    read_motion,
)


UPPER_BUS = "/dev/i2c-1"
FOREARM_BUS = "/dev/i2c-8"
CALIBRATION_SECONDS = 5.0
NEUTRAL_SECONDS = 2.0
FUSION_HZ = 50.0
PRINT_HZ = 10.0
MADGWICK_BETA_STATIC = 0.14
MADGWICK_BETA_MOVING = 0.02
STANDARD_GRAVITY = 9.80665
DEFAULT_SEGMENT_AXIS = "x"

Quaternion = tuple[float, float, float, float]
Vector3 = tuple[float, float, float]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def vector_norm(vector: Vector3) -> float:
    return math.sqrt(sum(value * value for value in vector))


def vector_dot(a: Vector3, b: Vector3) -> float:
    return sum(a[index] * b[index] for index in range(3))


def vector_cross(a: Vector3, b: Vector3) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def vector_normalize(vector: Vector3) -> Vector3:
    length = vector_norm(vector)
    if length < 1.0e-9:
        raise RuntimeError("Sifir uzunluklu yon vektoru")
    return tuple(value / length for value in vector)  # type: ignore[return-value]


def apply_accel_calibration(
    accel: Vector3, calibration: dict[str, object] | None
) -> Vector3:
    """Apply per-axis six-face offsets/scales when a calibration file exists."""
    if calibration is None:
        return accel
    offsets = calibration.get("accel_offset_m_s2", (0.0, 0.0, 0.0))
    scales = calibration.get("accel_scale", (1.0, 1.0, 1.0))
    if not isinstance(offsets, (list, tuple)) or not isinstance(scales, (list, tuple)):
        raise ValueError("Ivme kalibrasyon dosyasinda offset/scale dizisi gecersiz")
    if len(offsets) != 3 or len(scales) != 3:
        raise ValueError("Ivme kalibrasyon offset/scale tam olarak 3 elemanli olmali")
    return tuple(
        (accel[index] - float(offsets[index])) * float(scales[index])
        for index in range(3)
    )  # type: ignore[return-value]


def load_accel_calibrations(
    path: Path | None,
) -> tuple[dict[str, object] | None, dict[str, object] | None]:
    if path is None:
        return None, None
    payload = json.loads(path.read_text(encoding="utf-8"))
    if int(payload.get("version", 0)) != 1:
        raise ValueError("Desteklenmeyen HW-290 kalibrasyon dosyasi surumu")
    upper = payload.get("upper")
    forearm = payload.get("forearm")
    if not isinstance(upper, dict) or not isinstance(forearm, dict):
        raise ValueError("Kalibrasyon dosyasinda upper ve forearm bolumleri olmali")
    # Validate immediately so a malformed file cannot silently enter fusion.
    apply_accel_calibration((0.0, 0.0, 0.0), upper)
    apply_accel_calibration((0.0, 0.0, 0.0), forearm)
    return upper, forearm


def vector_angle_deg(a: Vector3, b: Vector3) -> float:
    denominator = vector_norm(a) * vector_norm(b)
    if denominator < 1.0e-9:
        return float("nan")
    cosine = vector_dot(a, b) / denominator
    return math.degrees(math.acos(clamp(cosine, -1.0, 1.0)))


def q_normalize(q: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1.0e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(value / norm for value in q)  # type: ignore[return-value]


def q_conjugate(q: Quaternion) -> Quaternion:
    return (q[0], -q[1], -q[2], -q[3])


def q_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def q_same_hemisphere(reference: Quaternion, value: Quaternion) -> Quaternion:
    """Resolve q/-q ambiguity before averaging quaternion samples."""
    dot = sum(reference[index] * value[index] for index in range(4))
    if dot < 0.0:
        return tuple(-part for part in value)  # type: ignore[return-value]
    return value


def q_average(values: list[Quaternion]) -> Quaternion:
    if not values:
        raise RuntimeError("Quaternion ortalamasi icin ornek yok")
    reference = values[0]
    aligned = [q_same_hemisphere(reference, value) for value in values]
    return q_normalize(tuple(
        statistics.fmean(value[index] for value in aligned)
        for index in range(4)
    ))  # type: ignore[arg-type]


def q_rotate(q: Quaternion, vector: Vector3) -> Vector3:
    rotated = q_multiply(q_multiply(q, (0.0, *vector)), q_conjugate(q))
    return (rotated[1], rotated[2], rotated[3])


def calibrated_segment_axes(
    upper_neutral: Quaternion,
    forearm_neutral: Quaternion,
    nominal_axis: Vector3,
) -> tuple[Vector3, Vector3]:
    """Find a fixed local limb axis for each case from the straight-arm pose.

    The two nominal case axes only need to point approximately shoulder-to-hand.
    Both are mapped onto one common neutral direction, which removes fixed print,
    strap and PCB alignment differences without learning a motion axis.
    """
    nominal_axis = vector_normalize(nominal_axis)
    upper_world = vector_normalize(q_rotate(upper_neutral, nominal_axis))
    forearm_world = vector_normalize(q_rotate(forearm_neutral, nominal_axis))
    if vector_dot(upper_world, forearm_world) < 0.0:
        forearm_world = tuple(-value for value in forearm_world)  # type: ignore[assignment]
    common_world = vector_normalize(tuple(
        upper_world[index] + forearm_world[index] for index in range(3)
    ))  # type: ignore[arg-type]
    upper_local = vector_normalize(q_rotate(q_conjugate(upper_neutral), common_world))
    forearm_local = vector_normalize(q_rotate(q_conjugate(forearm_neutral), common_world))
    return upper_local, forearm_local


def segment_flexion_deg(
    upper_q: Quaternion,
    forearm_q: Quaternion,
    upper_axis_local: Vector3,
    forearm_axis_local: Vector3,
) -> tuple[float, Vector3, Vector3]:
    """Return anatomical segment angle, independent of global arm direction."""
    upper_world = vector_normalize(q_rotate(upper_q, upper_axis_local))
    forearm_world = vector_normalize(q_rotate(forearm_q, forearm_axis_local))
    angle = math.degrees(math.acos(clamp(vector_dot(upper_world, forearm_world), -1.0, 1.0)))
    return angle, upper_world, forearm_world


def q_from_accel(accel: Vector3) -> Quaternion:
    """Initial roll/pitch from gravity; yaw deliberately starts at zero."""
    ax, ay, az = accel
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    return q_normalize((cr * cp, sr * cp, cr * sp, -sr * sp))


def madgwick_imu(
    q: Quaternion, gyro_rad_s: Vector3, accel: Vector3, dt: float, beta: float
) -> Quaternion:
    """One 6DOF Madgwick update; quaternion order is w,x,y,z."""
    q0, q1, q2, q3 = q
    gx, gy, gz = gyro_rad_s

    q_dot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
    q_dot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
    q_dot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
    q_dot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

    accel_length = vector_norm(accel)
    if accel_length > 1.0e-9 and beta > 0.0:
        ax, ay, az = (value / accel_length for value in accel)
        q0q0, q1q1, q2q2, q3q3 = q0*q0, q1*q1, q2*q2, q3*q3
        s0 = 4*q0*q2q2 + 2*q2*ax + 4*q0*q1q1 - 2*q1*ay
        s1 = 4*q1*q3q3 - 2*q3*ax + 4*q0q0*q1 - 2*q0*ay - 4*q1 + 8*q1*q1q1 + 8*q1*q2q2 + 4*q1*az
        s2 = 4*q0q0*q2 + 2*q0*ax + 4*q2*q3q3 - 2*q3*ay - 4*q2 + 8*q2*q1q1 + 8*q2*q2q2 + 4*q2*az
        s3 = 4*q1q1*q3 - 2*q1*ax + 4*q2q2*q3 - 2*q2*ay
        step_length = math.sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
        if step_length > 1.0e-12:
            q_dot0 -= beta * s0 / step_length
            q_dot1 -= beta * s1 / step_length
            q_dot2 -= beta * s2 / step_length
            q_dot3 -= beta * s3 / step_length

    return q_normalize((
        q0 + q_dot0 * dt,
        q1 + q_dot1 * dt,
        q2 + q_dot2 * dt,
        q3 + q_dot3 * dt,
    ))


def read_sample(
    fd: int,
    config: dict[str, float | int],
    accel_calibration: dict[str, object] | None = None,
) -> tuple[Vector3, Vector3]:
    accel, gyro = read_motion(
        fd, float(config["accel_scale"]), float(config["gyro_scale"])
    )
    corrected_accel = apply_accel_calibration(accel, accel_calibration)  # type: ignore[arg-type]
    return corrected_accel, gyro  # type: ignore[return-value]


def calibrate(
    fd: int,
    config: dict[str, float | int],
    name: str,
    accel_calibration: dict[str, object] | None = None,
) -> dict[str, object]:
    gyro_samples: list[Vector3] = []
    accel_samples: list[Vector3] = []
    deadline = time.monotonic() + CALIBRATION_SECONDS
    while time.monotonic() < deadline:
        accel, gyro = read_sample(fd, config, accel_calibration)
        accel_samples.append(accel)
        gyro_samples.append(gyro)
        time.sleep(0.01)

    # Median is deliberately used here. A cable touch or one short movement
    # during wearable calibration must not pull the gyro zero away.
    bias = tuple(
        statistics.median(sample[axis] for sample in gyro_samples)
        for axis in range(3)
    )
    gyro_noise = max(
        1.4826
        * statistics.median(
            abs(sample[axis] - bias[axis]) for sample in gyro_samples
        )
        for axis in range(3)
    )
    accel_mean = tuple(
        statistics.fmean(sample[axis] for sample in accel_samples)
        for axis in range(3)
    )
    gravity_norm = statistics.fmean(vector_norm(sample) for sample in accel_samples)
    if gravity_norm < 2.0:
        raise RuntimeError(
            f"{name} ivme verisi gecersiz: |a|={gravity_norm:.2f}. "
            "Kart enerji/lehimi ve SDA-SCL kablolarini kontrol edin; "
            "bu sensor sifir veri uretiyor."
        )
    gravity_plausible = 7.5 <= gravity_norm <= 12.5
    quality = "OK" if gyro_noise <= 3.0 and gravity_plausible else "UYARI"
    print(
        f"{name} kal={quality}  "
        f"gyro_bias=({bias[0]:.2f},{bias[1]:.2f},{bias[2]:.2f})dps "
        f"gyro_gurultu={gyro_noise:.2f}dps  |a|_referans={gravity_norm:.2f} "
        f"ivme_fiziksel={'EVET' if gravity_plausible else 'HAYIR'}",
        flush=True,
    )
    return {
        "gyro_bias": bias,
        "gyro_noise_dps": gyro_noise,
        "gravity_norm": gravity_norm,
        "gravity_plausible": gravity_plausible,
        "six_face_calibrated": accel_calibration is not None,
        "initial_accel": accel_mean,
    }


def corrected_gyro(gyro_dps: Vector3, calibration: dict[str, object]) -> Vector3:
    bias = calibration["gyro_bias"]
    assert isinstance(bias, tuple)
    return tuple(
        math.radians(gyro_dps[index] - bias[index]) for index in range(3)
    )  # type: ignore[return-value]


def update_sensor(
    fd: int,
    config: dict[str, float | int],
    calibration: dict[str, object],
    q: Quaternion,
    dt: float,
    accel_calibration: dict[str, object] | None = None,
) -> tuple[Quaternion, dict[str, object]]:
    monotonic_start = time.monotonic_ns()
    utc_start = time.time_ns()
    accel, gyro_dps = read_sample(fd, config, accel_calibration)
    monotonic_end = time.monotonic_ns()
    utc_end = time.time_ns()
    accel_norm = vector_norm(accel)
    reference_norm = float(calibration["gravity_norm"])
    ratio = accel_norm / reference_norm if reference_norm > 0.0 else 1.0
    physical_ratio = accel_norm / STANDARD_GRAVITY
    gyro_corrected = corrected_gyro(gyro_dps, calibration)
    gyro_speed_dps = math.degrees(vector_norm(gyro_corrected))
    # This HW-290 clone has orientation-dependent accelerometer scale. A broad
    # gate keeps gravity correction active after motion; fast motion uses a
    # lower gain so linear acceleration does not dominate orientation.
    accel_usable = 0.45 <= ratio <= 1.55
    if not accel_usable:
        beta = 0.0
    elif gyro_speed_dps < 15.0:
        beta = MADGWICK_BETA_STATIC
    else:
        beta = MADGWICK_BETA_MOVING
    q = madgwick_imu(q, gyro_corrected, accel, dt, beta)
    return q, {
        "sample_monotonic_ns": (monotonic_start + monotonic_end) // 2,
        "sample_utc_ns": (utc_start + utc_end) // 2,
        "accel_m_s2": accel,
        "accel_ratio": ratio,
        "accel_physical_ratio": physical_ratio,
        "accel_usable": accel_usable,
        "gyro_dps": gyro_dps,
        "gyro_corrected_rad_s": gyro_corrected,
        "gyro_speed_dps": gyro_speed_dps,
        "beta": beta,
    }


def relative_orientation(upper: Quaternion, forearm: Quaternion) -> Quaternion:
    return q_normalize(q_multiply(q_conjugate(upper), forearm))


def delta_orientation(reference: Quaternion, current: Quaternion) -> Quaternion:
    return q_normalize(q_multiply(q_conjugate(reference), current))


def total_angle_deg(delta: Quaternion) -> float:
    return math.degrees(2.0 * math.acos(clamp(abs(delta[0]), 0.0, 1.0)))


def x_segment_angle_deg(delta: Quaternion) -> float:
    moved_x = q_rotate(delta, (1.0, 0.0, 0.0))
    return math.degrees(math.acos(clamp(moved_x[0], -1.0, 1.0)))


def capture_neutral(
    upper_fd: int,
    forearm_fd: int,
    upper_config: dict[str, float | int],
    forearm_config: dict[str, float | int],
    upper_cal: dict[str, object],
    forearm_cal: dict[str, object],
    upper_q: Quaternion,
    forearm_q: Quaternion,
    upper_accel_cal: dict[str, object] | None = None,
    forearm_accel_cal: dict[str, object] | None = None,
) -> tuple[Quaternion, Quaternion, Quaternion, float, Quaternion, Quaternion]:
    """Average a stable straight-arm reference instead of using one sample."""
    print(
        f"NOTR REFERANS: kolu duz ve sabit tutun; {NEUTRAL_SECONDS:.0f} saniye ortalama aliniyor...",
        flush=True,
    )
    relatives: list[Quaternion] = []
    upper_values: list[Quaternion] = []
    forearm_values: list[Quaternion] = []
    relative_rates: list[float] = []
    previous_relative: Quaternion | None = None
    period = 1.0 / FUSION_HZ
    previous_time = time.monotonic()
    deadline = previous_time + NEUTRAL_SECONDS
    while time.monotonic() < deadline:
        tick = time.monotonic()
        dt = clamp(tick - previous_time, 0.001, 0.05)
        previous_time = tick
        upper_q, _ = update_sensor(
            upper_fd, upper_config, upper_cal, upper_q, dt, upper_accel_cal
        )
        forearm_q, _ = update_sensor(
            forearm_fd, forearm_config, forearm_cal, forearm_q, dt, forearm_accel_cal
        )
        relative = relative_orientation(upper_q, forearm_q)
        relatives.append(relative)
        upper_values.append(upper_q)
        forearm_values.append(forearm_q)
        if previous_relative is not None:
            change = delta_orientation(previous_relative, relative)
            relative_rates.append(total_angle_deg(change) / dt)
        previous_relative = relative
        time.sleep(max(0.0, period - (time.monotonic() - tick)))

    neutral = q_average(relatives)
    errors = [total_angle_deg(delta_orientation(neutral, value)) for value in relatives]
    neutral_noise = statistics.median(errors)
    p95_index = max(0, math.ceil(0.95 * len(errors)) - 1)
    neutral_p95 = sorted(errors)[p95_index]
    rate_median = statistics.median(relative_rates) if relative_rates else 0.0
    print(
        f"NOTR ALINDI: medyan oynama={neutral_noise:.2f} derece, "
        f"p95={neutral_p95:.2f} derece, hiz medyani={rate_median:.1f} derece/s",
        flush=True,
    )
    if neutral_p95 > 5.0:
        print(
            "UYARI: Notr referans sirasinda hareket algilandi. "
            "Olcum baslayacak fakat en iyi sonuc icin programi yeniden baslatip "
            "referans boyunca kolu sabit tutun.",
            flush=True,
        )
    return (
        upper_q,
        forearm_q,
        neutral,
        neutral_p95,
        q_average(upper_values),
        q_average(forearm_values),
    )


def sensor_network_record(
    sensor_id: str,
    state: dict[str, object],
    quaternion: Quaternion,
    mag_raw: tuple[float, float, float] | None,
    calibration: dict[str, object],
) -> dict[str, Any]:
    w, x, y, z = quaternion
    return {
        "sensor_id": sensor_id,
        "sample_monotonic_ns": int(state["sample_monotonic_ns"]),
        "sample_utc_ns": int(state["sample_utc_ns"]),
        "quaternion_xyzw": [x, y, z, w],
        "gyro_rad_s": list(state["gyro_corrected_rad_s"]),
        "gyro_raw_dps": list(state["gyro_dps"]),
        "linear_acceleration_m_s2": list(state["accel_m_s2"]),
        "mag_raw": None if mag_raw is None else list(mag_raw),
        "mag_used_in_orientation": False,
        "accel_ratio": float(state["accel_ratio"]),
        "accel_physical_ratio": float(state["accel_physical_ratio"]),
        "accel_correction_used": bool(state["accel_usable"]),
        "gyro_speed_dps": float(state["gyro_speed_dps"]),
        "madgwick_beta": float(state["beta"]),
        "software_calibration": {
            "gyro_bias_dps": list(calibration["gyro_bias"]),
            "gyro_noise_dps": float(calibration["gyro_noise_dps"]),
            "gravity_reference_m_s2": float(calibration["gravity_norm"]),
            "gravity_reference_plausible": bool(calibration["gravity_plausible"]),
            "six_face_accel": bool(calibration["six_face_calibrated"]),
        },
    }


def measurement_quality(
    upper_state: dict[str, object],
    forearm_state: dict[str, object],
    upper_cal: dict[str, object],
    forearm_cal: dict[str, object],
    *,
    dt_gap: bool,
    gyro_near_saturation: bool,
    neutral_p95: float,
) -> tuple[float, list[str]]:
    failures: list[str] = []
    quality = 1.0
    for prefix, state, calibration in (
        ("UPPER", upper_state, upper_cal),
        ("FOREARM", forearm_state, forearm_cal),
    ):
        if not bool(calibration["gravity_plausible"]):
            failures.append(f"{prefix}_ACCEL_REFERENCE_IMPLAUSIBLE")
            quality *= 0.50
        if float(calibration["gyro_noise_dps"]) > 3.0:
            failures.append(f"{prefix}_GYRO_CALIBRATION_MOTION")
            quality *= 0.70
        if not bool(state["accel_usable"]):
            failures.append(f"{prefix}_ACCEL_CORRECTION_REJECTED")
            quality *= 0.70
        physical_ratio = float(state["accel_physical_ratio"])
        if not 0.65 <= physical_ratio <= 1.35:
            failures.append(f"{prefix}_ACCEL_MAGNITUDE_IMPLAUSIBLE")
            quality *= 0.75
    if neutral_p95 > 3.0:
        failures.append("IMU_NEUTRAL_UNSTABLE")
        quality *= 0.75
    if gyro_near_saturation:
        failures.append("IMU_GYRO_NEAR_SATURATION")
        quality *= 0.45
    if dt_gap:
        failures.append("IMU_DT_GAP")
        quality *= 0.60
    return clamp(quality, 0.0, 1.0), sorted(set(failures))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Iki HW-290 ile sol dirsek olcumu, JSONL kaydi ve ZED UDP yayini"
    )
    parser.add_argument("--upper-bus", default=UPPER_BUS)
    parser.add_argument("--forearm-bus", default=FOREARM_BUS)
    parser.add_argument(
        "--swap-sensors",
        action="store_true",
        help=(
            "HW290-2 ust kolda ve HW290-1 on kolda ise buslari ve bunlara "
            "ait alti-yuz kalibrasyonlarini birlikte degistir"
        ),
    )
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument(
        "--segment-axis",
        choices=("x", "y", "z"),
        default=DEFAULT_SEGMENT_AXIS,
        help="Kasada omuzdan ele dogru uzanan yaklasik sensor ekseni",
    )
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--udp-host", help="ZED bilgisayarinin IPv4 adresi")
    parser.add_argument("--udp-port", type=int, default=15060)
    parser.add_argument("--source-id", default=socket.gethostname())
    parser.add_argument("--session-id")
    parser.add_argument(
        "--accel-calibration",
        type=Path,
        help="hw290_six_face_calibration.py tarafindan uretilen JSON",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path.home() / time.strftime("hw290_arm_imu_%Y%m%d_%H%M%S.jsonl"),
    )
    args = parser.parse_args()
    if not 5.0 <= args.hz <= 50.0:
        parser.error("--hz 5 ile 50 arasinda olmali")
    return args


def validate_protocol_api() -> None:
    """Fail before the long calibration if Pi has an older protocol module."""
    parameters = inspect.signature(build_arm_imu_packet).parameters
    required = {"quality_override", "extra_failure_codes"}
    missing = sorted(required.difference(parameters))
    if missing:
        raise RuntimeError(
            "arm_imu_protocol.py eski surum. Eksik parametreler: "
            + ", ".join(missing)
            + ". imu_capture/arm_imu_protocol.py dosyasini Raspberry Pi'ye "
            "yeniden kopyalayin."
        )


def main() -> int:
    args = parse_args()
    validate_protocol_api()
    session_id = args.session_id or time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    nominal_axis = {
        "x": (1.0, 0.0, 0.0),
        "y": (0.0, 1.0, 0.0),
        "z": (0.0, 0.0, 1.0),
    }[args.segment_axis]
    accel_calibration_path = args.accel_calibration
    automatic_calibration = Path.home() / "hw290_accel_calibration.json"
    if accel_calibration_path is None and automatic_calibration.exists():
        accel_calibration_path = automatic_calibration
        print(f"Otomatik ivme kalibrasyonu bulundu: {accel_calibration_path}")
    upper_accel_cal, forearm_accel_cal = load_accel_calibrations(
        accel_calibration_path
    )
    upper_hardware = "HW290-1"
    forearm_hardware = "HW290-2"
    if args.swap_sensors:
        args.upper_bus, args.forearm_bus = args.forearm_bus, args.upper_bus
        upper_accel_cal, forearm_accel_cal = forearm_accel_cal, upper_accel_cal
        upper_hardware, forearm_hardware = forearm_hardware, upper_hardware
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.udp_host else None
    if udp is not None:
        udp.setblocking(False)
    udp_target = (args.udp_host, args.udp_port) if args.udp_host else None
    args.output.parent.mkdir(parents=True, exist_ok=True)

    print(
        f"{upper_hardware}=sol ust kol ({args.upper_bus}), "
        f"{forearm_hardware}=sol on kol ({args.forearm_bus})"
    )
    print(
        f"Kasalarin +{args.segment_axis.upper()} yonu yaklasik omuzdan ele bakmali. "
        "Notr kalibrasyon sabit montaj farkini duzeltecek."
    )
    if accel_calibration_path is None:
        print("UYARI: Alti-yuz ivme kalibrasyonu verilmedi; fiziksel kalite kapisi aktif.")

    try:
        with open(args.upper_bus, "rb+", buffering=0) as upper_bus, open(
            args.forearm_bus, "rb+", buffering=0
        ) as forearm_bus, args.output.open("w", encoding="utf-8") as output:
            upper_fd, forearm_fd = upper_bus.fileno(), forearm_bus.fileno()
            upper_config = configure_mpu6050(upper_fd, gyro_range_dps=500)
            forearm_config = configure_mpu6050(forearm_fd, gyro_range_dps=500)
            upper_compass = configure_compass(upper_fd)
            forearm_compass = configure_compass(forearm_fd)
            print(
                f"Pusula: UST_KOL={upper_compass or 'YOK'}, "
                f"ON_KOL={forearm_compass or 'YOK'}; ham veri kaydedilir, yonelimde kullanilmaz.",
                flush=True,
            )
            print("Kolu duz tutun. Kalibrasyon 5 saniye sonra otomatik baslayacak.")
            for remaining in range(5, 0, -1):
                print(f"Baslamasina {remaining}...", flush=True)
                time.sleep(1.0)
            print("KALIBRASYON BASLADI - iki sensoru 10 saniye kipirdatmayin.")
            upper_cal = calibrate(
                upper_fd, upper_config, "UST_KOL", upper_accel_cal
            )
            forearm_cal = calibrate(
                forearm_fd, forearm_config, "ON_KOL", forearm_accel_cal
            )
            upper_q = q_from_accel(upper_cal["initial_accel"])  # type: ignore[arg-type]
            forearm_q = q_from_accel(forearm_cal["initial_accel"])  # type: ignore[arg-type]

            period = 1.0 / args.hz
            print("Filtre yerlesiyor; kolu 2 saniye daha duz ve sabit tutun...")
            previous = time.monotonic()
            deadline = previous + 2.0
            while time.monotonic() < deadline:
                now = time.monotonic()
                dt = clamp(now - previous, 0.001, 0.05)
                previous = now
                upper_q, _ = update_sensor(
                    upper_fd, upper_config, upper_cal, upper_q, dt, upper_accel_cal
                )
                forearm_q, _ = update_sensor(
                    forearm_fd,
                    forearm_config,
                    forearm_cal,
                    forearm_q,
                    dt,
                    forearm_accel_cal,
                )
                time.sleep(max(0.0, period - (time.monotonic() - now)))

            (
                upper_q,
                forearm_q,
                neutral_relative,
                neutral_p95,
                upper_neutral,
                forearm_neutral,
            ) = capture_neutral(
                upper_fd,
                forearm_fd,
                upper_config,
                forearm_config,
                upper_cal,
                forearm_cal,
                upper_q,
                forearm_q,
                upper_accel_cal,
                forearm_accel_cal,
            )
            upper_axis_local, forearm_axis_local = calibrated_segment_axes(
                upper_neutral, forearm_neutral, nominal_axis
            )
            print("SIFIR ALINDI. Her yone omuz hareketi yapabilirsiniz. Durdurmak: Ctrl+C")
            print("dirsek=iki anatomik kol ekseni arasindaki aci; 3D=toplam bagil donus")
            print(f"Kayit: {args.output}")
            if udp_target is not None:
                print(f"ZED UDP: {udp_target[0]}:{udp_target[1]}")

            previous = time.monotonic()
            next_tick = previous
            previous_flexion: float | None = None
            previous_delta: Quaternion | None = None
            filtered_velocity = 0.0
            filtered_relative_rate = 0.0
            print_every = max(1, round(args.hz / PRINT_HZ))
            sequence = 0
            i2c_errors = 0
            udp_errors = 0
            while True:
                now = time.monotonic()
                raw_dt = now - previous
                dt = clamp(raw_dt, 0.001, 0.05)
                previous = now
                try:
                    upper_q, upper_state = update_sensor(
                        upper_fd,
                        upper_config,
                        upper_cal,
                        upper_q,
                        dt,
                        upper_accel_cal,
                    )
                    forearm_q, forearm_state = update_sensor(
                        forearm_fd,
                        forearm_config,
                        forearm_cal,
                        forearm_q,
                        dt,
                        forearm_accel_cal,
                    )
                    upper_mag = read_compass(upper_fd, upper_compass)
                    forearm_mag = read_compass(forearm_fd, forearm_compass)
                except (OSError, RuntimeError) as exc:
                    i2c_errors += 1
                    print(f"I2C HATA #{i2c_errors}: {exc}", flush=True)
                    sequence += 1
                    next_tick = time.monotonic() + period
                    time.sleep(period)
                    continue

                relative = relative_orientation(upper_q, forearm_q)
                delta = delta_orientation(neutral_relative, relative)
                relative_3d = total_angle_deg(delta)
                flexion, upper_axis_world, forearm_axis_world = segment_flexion_deg(
                    upper_q,
                    forearm_q,
                    upper_axis_local,
                    forearm_axis_local,
                )
                elbow_x_legacy = x_segment_angle_deg(delta)
                if previous_flexion is not None:
                    raw_velocity = (flexion - previous_flexion) / max(dt, 1.0e-3)
                    alpha = 1.0 - math.exp(-2.0 * math.pi * 4.0 * dt)
                    filtered_velocity += alpha * (raw_velocity - filtered_velocity)
                if previous_delta is not None:
                    relative_step = delta_orientation(previous_delta, delta)
                    raw_relative_rate = total_angle_deg(relative_step) / max(dt, 1.0e-3)
                    rate_alpha = 1.0 - math.exp(-2.0 * math.pi * 3.0 * dt)
                    filtered_relative_rate += rate_alpha * (
                        raw_relative_rate - filtered_relative_rate
                    )
                previous_flexion = flexion
                previous_delta = delta

                hinge_world = vector_cross(upper_axis_world, forearm_axis_world)
                gyro_flexion_velocity: float | None = None
                if vector_norm(hinge_world) > 0.05:
                    hinge_world = vector_normalize(hinge_world)
                    upper_gyro_world = q_rotate(
                        upper_q, upper_state["gyro_corrected_rad_s"]  # type: ignore[arg-type]
                    )
                    forearm_gyro_world = q_rotate(
                        forearm_q, forearm_state["gyro_corrected_rad_s"]  # type: ignore[arg-type]
                    )
                    relative_gyro_world = tuple(
                        forearm_gyro_world[index] - upper_gyro_world[index]
                        for index in range(3)
                    )
                    gyro_flexion_velocity = math.degrees(
                        vector_dot(relative_gyro_world, hinge_world)  # type: ignore[arg-type]
                    )

                gyro_ranges = (250.0, 500.0, 1000.0, 2000.0)
                upper_range = gyro_ranges[(int(upper_config["gyro_config"]) >> 3) & 3]
                forearm_range = gyro_ranges[(int(forearm_config["gyro_config"]) >> 3) & 3]
                gyro_near_saturation = (
                    max(abs(float(v)) for v in upper_state["gyro_dps"]) >= 0.90 * upper_range  # type: ignore[union-attr]
                    or max(abs(float(v)) for v in forearm_state["gyro_dps"]) >= 0.90 * forearm_range  # type: ignore[union-attr]
                )
                dt_gap = raw_dt > max(0.040, 2.5 * period)
                quality, failures = measurement_quality(
                    upper_state,
                    forearm_state,
                    upper_cal,
                    forearm_cal,
                    dt_gap=dt_gap,
                    gyro_near_saturation=gyro_near_saturation,
                    neutral_p95=neutral_p95,
                )
                upper_net = sensor_network_record(
                    "upper_arm", upper_state, upper_q, upper_mag, upper_cal
                )
                forearm_net = sensor_network_record(
                    "forearm", forearm_state, forearm_q, forearm_mag, forearm_cal
                )
                upper_net["segment_axis_sensor_xyz"] = list(upper_axis_local)
                upper_net["segment_axis_world_xyz"] = list(upper_axis_world)
                forearm_net["segment_axis_sensor_xyz"] = list(forearm_axis_local)
                forearm_net["segment_axis_world_xyz"] = list(forearm_axis_world)
                timestamp_ns = (
                    int(upper_state["sample_utc_ns"])
                    + int(forearm_state["sample_utc_ns"])
                ) // 2
                read_skew_ms = abs(
                    int(forearm_state["sample_monotonic_ns"])
                    - int(upper_state["sample_monotonic_ns"])
                ) / 1.0e6
                packet = build_arm_imu_packet(
                    sequence=sequence,
                    timestamp_ns=timestamp_ns,
                    side=args.side,
                    flexion_deg=flexion,
                    velocity_deg_s=filtered_velocity,
                    relative_3d_deg=relative_3d,
                    off_axis_deg=None,
                    upper=upper_net,
                    forearm=forearm_net,
                    read_skew_ms=read_skew_ms,
                    sample_period_ms=raw_dt * 1000.0,
                    quality_override=quality,
                    extra_failure_codes=failures,
                )
                packet["elbow"].update({
                    "velocity_gyro_deg_s": gyro_flexion_velocity,
                    "relative_rate_deg_s": filtered_relative_rate,
                    "legacy_x_deg": elbow_x_legacy,
                })
                rw, rx, ry, rz = relative
                dw, dx, dy, dz = delta
                packet["relative_orientation_xyzw"] = [rx, ry, rz, rw]
                packet["relative_delta_xyzw"] = [dx, dy, dz, dw]
                packet["calibration"] = {
                    "neutral_relative_wxyz": list(neutral_relative),
                    "neutral_p95_deg": neutral_p95,
                    "nominal_segment_axis": args.segment_axis,
                    "upper_segment_axis_sensor_xyz": list(upper_axis_local),
                    "forearm_segment_axis_sensor_xyz": list(forearm_axis_local),
                    "six_face_file": None
                    if accel_calibration_path is None
                    else str(accel_calibration_path),
                }
                packet["diagnostics"] = {
                    "i2c_error_count": i2c_errors,
                    "udp_error_count": udp_errors,
                    "gyro_range_dps": [upper_range, forearm_range],
                }
                packet["source_id"] = args.source_id
                packet["session_id"] = session_id
                packet["capture"] = {
                    "implementation": "hw290_elbow_quaternion/v4",
                    "primary_angle": "calibrated_segment_axis_angle",
                    "nominal_rate_hz": args.hz,
                    "upper_bus": args.upper_bus,
                    "forearm_bus": args.forearm_bus,
                    "upper_hardware": upper_hardware,
                    "forearm_hardware": forearm_hardware,
                    "position_source": "ZED_BODY_38",
                    "magnetometer_policy": "logged_not_fused",
                }
                line = json.dumps(
                    packet, ensure_ascii=False, allow_nan=False, separators=(",", ":")
                )
                output.write(line + "\n")
                if sequence % 20 == 0:
                    output.flush()
                if udp is not None and udp_target is not None:
                    try:
                        udp.sendto(line.encode("utf-8"), udp_target)
                    except (BlockingIOError, OSError) as exc:
                        udp_errors += 1
                        if udp_errors <= 3 or udp_errors % 100 == 0:
                            print(f"UDP HATA #{udp_errors}: {exc}")
                if sequence % print_every == 0:
                    flag_text = ",".join(packet["failure_codes"]) or "OK"
                    print(
                        f"dirsek={flexion:6.1f}  3D={relative_3d:6.1f}  "
                        f"hiz={filtered_velocity:7.1f}dps  q={packet['quality']:.2f}  "
                        f"a={float(upper_state['accel_physical_ratio']):.2f}/"
                        f"{float(forearm_state['accel_physical_ratio']):.2f}  {flag_text}"
                    )
                sequence += 1
                next_tick += period
                time.sleep(max(0.0, next_tick - time.monotonic()))
                if next_tick < time.monotonic() - period:
                    next_tick = time.monotonic()
    except KeyboardInterrupt:
        print("\nKayit durduruldu.")
    finally:
        if udp is not None:
            udp.close()
        print(f"Kayit kapatildi: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

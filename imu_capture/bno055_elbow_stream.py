"""Two-bus BNO055 elbow capture, JSONL logger and optional UDP publisher.

Designed for the tested Raspberry Pi layout: both boards keep address 0x28,
with the upper arm on /dev/i2c-1 and forearm on /dev/i2c-8.  No pip package is
required.  The existing bring-up scripts remain unchanged.
"""

from __future__ import annotations

import argparse
import ctypes as c
import json
import math
import os
from pathlib import Path
import socket
import struct
import time
from typing import Any

from arm_imu_protocol import build_arm_imu_packet


ADDRESS = 0x28
DEFAULT_SEGMENT_AXIS = "x"  # Both cases: this arrow must point shoulder -> wrist.


class I2cMessage(c.Structure):
    _fields_ = [
        ("address", c.c_uint16),
        ("flags", c.c_uint16),
        ("length", c.c_uint16),
        ("data", c.POINTER(c.c_uint8)),
    ]


class I2cTransfer(c.Structure):
    _fields_ = [("messages", c.POINTER(I2cMessage)), ("count", c.c_uint32)]


LIBC: Any | None = None
SENSOR_STATE: dict[str, dict[str, Any]] = {}
QUATERNION_OUTLIERS: dict[str, int] = {}


def system_libc() -> Any:
    global LIBC
    if LIBC is None:
        LIBC = c.CDLL(None, use_errno=True)
    return LIBC


def transfer(fd: int, messages: Any) -> None:
    operation = I2cTransfer(messages, len(messages))
    if system_libc().ioctl(fd, 0x0707, c.byref(operation)) < 0:
        error = c.get_errno()
        raise OSError(error, os.strerror(error))


def read_register(fd: int, register: int, count: int) -> bytes:
    command = (c.c_uint8 * 1)(register)
    response = (c.c_uint8 * count)()
    messages = (I2cMessage * 2)(
        I2cMessage(ADDRESS, 0, 1, command),
        I2cMessage(ADDRESS, 1, count, response),
    )
    transfer(fd, messages)
    return bytes(response)


def write_register(fd: int, register: int, value: int) -> None:
    data = (c.c_uint8 * 2)(register, value)
    messages = (I2cMessage * 1)(I2cMessage(ADDRESS, 0, 2, data))
    transfer(fd, messages)


def start_sensor(fd: int, name: str) -> None:
    if read_register(fd, 0x00, 1)[0] != 0xA0:
        raise RuntimeError(f"{name}: BNO055 CHIP_ID okunamadi")
    write_register(fd, 0x3D, 0x00)
    time.sleep(0.03)
    write_register(fd, 0x07, 0x00)
    write_register(fd, 0x3E, 0x00)
    write_register(fd, 0x3B, 0x00)
    write_register(fd, 0x3D, 0x08)  # IMUPLUS: accel + gyro, no magnetometer
    time.sleep(0.7)
    print(f"{name} hazir, CHIP_ID=0xa0")


def dot(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    return sum(first * second for first, second in zip(a, b))


def vector_norm(vector: tuple[float, float, float]) -> float:
    return math.sqrt(dot(vector, vector))


def vector_normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    length = vector_norm(vector)
    if length < 1.0e-9:
        raise RuntimeError("Hareket ekseni bulunamadi")
    return tuple(value / length for value in vector)


def q_normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    length = math.sqrt(sum(value * value for value in q))
    if not 0.5 <= length <= 1.5:
        raise RuntimeError("Gecersiz quaternion")
    return tuple(value / length for value in q)


def q_conjugate(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    w, x, y, z = q
    return w, -x, -y, -z


def q_multiply(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def q_rotate(
    q: tuple[float, float, float, float], vector: tuple[float, float, float]
) -> tuple[float, float, float]:
    result = q_multiply(q_multiply(q, (0.0, *vector)), q_conjugate(q))
    return result[1], result[2], result[3]


def q_step_deg(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> float:
    cosine = min(1.0, abs(dot(first, second)))
    return math.degrees(2.0 * math.acos(cosine))


def read_sensor(fd: int, sensor_id: str) -> dict[str, Any]:
    monotonic_start = time.monotonic_ns()
    utc_start = time.time_ns()
    q = None
    for _attempt in range(5):
        raw_q = tuple(
            value / 16384.0
            for value in struct.unpack("<hhhh", read_register(fd, 0x20, 8))
        )
        try:
            q = q_normalize(raw_q)
            break
        except RuntimeError:
            time.sleep(0.01)
    if q is None:
        raise RuntimeError(f"{sensor_id}: 5 denemede gecerli quaternion okunamadi")
    gyro = tuple(
        math.radians(value / 16.0)
        for value in struct.unpack("<hhh", read_register(fd, 0x14, 6))
    )
    linear_accel = tuple(
        value / 100.0 for value in struct.unpack("<hhh", read_register(fd, 0x28, 6))
    )
    status = read_register(fd, 0x35, 1)[0]
    calibration = (
        (status >> 6) & 3,
        (status >> 4) & 3,
        (status >> 2) & 3,
        status & 3,
    )
    monotonic_end = time.monotonic_ns()
    utc_end = time.time_ns()
    quaternion_outlier_rejected = False
    quaternion_held = False
    previous = SENSOR_STATE.get(sensor_id)
    if previous is not None:
        dt = (monotonic_end - previous["monotonic_ns"]) / 1.0e9
        if 0.0 < dt <= 0.20:
            gyro_speed_deg_s = math.degrees(vector_norm(gyro))
            allowed_step_deg = max(12.0, gyro_speed_deg_s * dt * 2.5 + 5.0)
            if q_step_deg(previous["q"], q) > allowed_step_deg:
                quaternion_outlier_rejected = True
                QUATERNION_OUTLIERS[sensor_id] = QUATERNION_OUTLIERS.get(sensor_id, 0) + 1
                replacement = None
                for _attempt in range(4):
                    time.sleep(0.005)
                    candidate_raw = tuple(
                        value / 16384.0
                        for value in struct.unpack("<hhhh", read_register(fd, 0x20, 8))
                    )
                    try:
                        candidate = q_normalize(candidate_raw)
                    except RuntimeError:
                        continue
                    if q_step_deg(previous["q"], candidate) <= allowed_step_deg:
                        replacement = candidate
                        break
                if replacement is None:
                    q = previous["q"]
                    quaternion_held = True
                else:
                    q = replacement
    SENSOR_STATE[sensor_id] = {"q": q, "monotonic_ns": monotonic_end}
    return {
        "sensor_id": sensor_id,
        "sample_monotonic_ns": (monotonic_start + monotonic_end) // 2,
        "sample_utc_ns": (utc_start + utc_end) // 2,
        "quaternion_wxyz": q,
        "gyro_rad_s": gyro,
        "linear_acceleration_m_s2": linear_accel,
        "calibration": calibration,
        "quaternion_outlier_rejected": quaternion_outlier_rejected,
        "quaternion_held": quaternion_held,
        "quaternion_outlier_count": QUATERNION_OUTLIERS.get(sensor_id, 0),
    }


def relative_orientation(upper: dict[str, Any], forearm: dict[str, Any]) -> tuple[float, float, float, float]:
    return q_normalize(q_multiply(q_conjugate(upper["quaternion_wxyz"]), forearm["quaternion_wxyz"]))


def delta_orientation(
    reference: tuple[float, float, float, float],
    current: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    q = q_normalize(q_multiply(q_conjugate(reference), current))
    return tuple(-value for value in q) if q[0] < 0.0 else q


def total_angle_deg(q: tuple[float, float, float, float]) -> float:
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, abs(q[0])))))


def segment_angle_deg(
    q: tuple[float, float, float, float],
    segment_axis: tuple[float, float, float],
) -> float:
    """Angle between the neutral and current forearm longitudinal axes."""
    current_axis = q_rotate(q, segment_axis)
    cosine = max(-1.0, min(1.0, dot(segment_axis, current_axis)))
    return math.degrees(math.acos(cosine))


def hinge_angle_deg(
    q: tuple[float, float, float, float], axis: tuple[float, float, float]
) -> float:
    w, x, y, z = q
    projection = x * axis[0] + y * axis[1] + z * axis[2]
    angle = math.degrees(2.0 * math.atan2(projection, w))
    return (angle + 180.0) % 360.0 - 180.0


def off_axis_angle_deg(
    q: tuple[float, float, float, float], axis: tuple[float, float, float]
) -> float:
    w, x, y, z = q
    projection = x * axis[0] + y * axis[1] + z * axis[2]
    twist_raw = (w, axis[0] * projection, axis[1] * projection, axis[2] * projection)
    if math.sqrt(sum(value * value for value in twist_raw)) < 1.0e-9:
        return total_angle_deg(q)
    twist_length = math.sqrt(sum(value * value for value in twist_raw))
    twist = tuple(value / twist_length for value in twist_raw)
    swing = q_normalize(q_multiply(q, q_conjugate(twist)))
    return total_angle_deg(swing)


def wrapped_difference(current: float, previous: float) -> float:
    return (current - previous + 180.0) % 360.0 - 180.0


def average_quaternions(
    values: list[tuple[float, float, float, float]],
) -> tuple[float, float, float, float]:
    if not values:
        raise RuntimeError("Quaternion ortalamasi icin ornek yok")
    reference = values[0]
    total = [0.0, 0.0, 0.0, 0.0]
    for value in values:
        aligned = tuple(-part for part in value) if dot(value, reference) < 0.0 else value
        for index in range(4):
            total[index] += aligned[index]
    mean = tuple(value / len(values) for value in total)
    return q_normalize(mean)


def capture_neutral(fd_upper: int, fd_forearm: int, seconds: float = 1.0) -> tuple[float, float, float, float]:
    values: list[tuple[float, float, float, float]] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            upper = read_sensor(fd_upper, "upper_arm")
            forearm = read_sensor(fd_forearm, "forearm")
        except (OSError, RuntimeError):
            time.sleep(0.02)
            continue
        values.append(relative_orientation(upper, forearm))
        time.sleep(0.05)
    if not values:
        raise RuntimeError("Notr poz icin sensor verisi okunamadi")
    return average_quaternions(values)


def learn_axis(
    fd_upper: int,
    fd_forearm: int,
    neutral: tuple[float, float, float, float],
    seconds: float,
) -> tuple[float, float, float]:
    print(f"Dirsegi {seconds:.0f} saniye boyunca 3-4 kez yavasca bukup acin.")
    rotation_vectors: list[tuple[float, float, float]] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            upper = read_sensor(fd_upper, "upper_arm")
            forearm = read_sensor(fd_forearm, "forearm")
        except (OSError, RuntimeError):
            time.sleep(0.02)
            continue
        delta = delta_orientation(neutral, relative_orientation(upper, forearm))
        w, x, y, z = delta
        vector_length = math.sqrt(x * x + y * y + z * z)
        angle = 2.0 * math.atan2(vector_length, w)
        if math.radians(10.0) <= angle <= math.radians(150.0) and vector_length > 1.0e-9:
            rotation_vectors.append((angle * x / vector_length, angle * y / vector_length, angle * z / vector_length))
        time.sleep(0.05)
    if len(rotation_vectors) < 10:
        raise RuntimeError("Yeterli dirsek bukme hareketi algilanmadi")
    reference = rotation_vectors[0]
    total = [0.0, 0.0, 0.0]
    for vector in rotation_vectors:
        aligned = tuple(-value for value in vector) if dot(vector, reference) < 0.0 else vector
        for index in range(3):
            total[index] += aligned[index]
    return vector_normalize(tuple(total))


def network_sensor(sensor: dict[str, Any]) -> dict[str, Any]:
    w, x, y, z = sensor["quaternion_wxyz"]
    return {
        "sensor_id": sensor["sensor_id"],
        "sample_monotonic_ns": sensor["sample_monotonic_ns"],
        "sample_utc_ns": sensor["sample_utc_ns"],
        "quaternion_xyzw": [x, y, z, w],
        "gyro_rad_s": list(sensor["gyro_rad_s"]),
        "linear_acceleration_m_s2": list(sensor["linear_acceleration_m_s2"]),
        "calibration": list(sensor["calibration"]),
        "quaternion_outlier_rejected": sensor["quaternion_outlier_rejected"],
        "quaternion_held": sensor["quaternion_held"],
        "quaternion_outlier_count": sensor["quaternion_outlier_count"],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Iki BNO055 dirsek olcumu ve UDP yayini")
    parser.add_argument("--upper-bus", default="/dev/i2c-1")
    parser.add_argument("--forearm-bus", default="/dev/i2c-8")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument(
        "--angle-mode",
        choices=("segment", "3d", "hinge"),
        default="segment",
        help="segment sabit kol eksenini; 3d toplam donusu; hinge ogrenilen ekseni kullanir",
    )
    parser.add_argument(
        "--segment-axis",
        choices=("x", "y", "z"),
        default=DEFAULT_SEGMENT_AXIS,
        help="Sensor kasasinda omuzdan bilege dogru uzanan sabit eksen",
    )
    parser.add_argument(
        "--flexion-sign",
        type=int,
        choices=(-1, 1),
        default=1,
        help="Bukme eksi yonde cikarsa -1 kullanin",
    )
    parser.add_argument("--source-id", default=socket.gethostname())
    parser.add_argument("--session-id", help="Ayni denemedeki ZED ve IMU kayitlarini eslestiren kimlik")
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--axis-seconds", type=float, default=8.0)
    parser.add_argument("--udp-host", help="ZED bilgisayarinin IP adresi")
    parser.add_argument("--udp-port", type=int, default=15060)
    parser.add_argument("--output", type=Path, default=Path(time.strftime("arm_imu_%Y%m%d_%H%M%S.jsonl")))
    args = parser.parse_args()
    if not 1.0 <= args.hz <= 50.0:
        parser.error("--hz 1 ile 50 arasinda olmali")
    return args


def main() -> int:
    args = parse_args()
    session_id = args.session_id or time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    segment_axis = {
        "x": (1.0, 0.0, 0.0),
        "y": (0.0, 1.0, 0.0),
        "z": (0.0, 0.0, 1.0),
    }[args.segment_axis]
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.udp_host else None
    if udp is not None:
        udp.setblocking(False)
    target = (args.udp_host, args.udp_port) if args.udp_host else None
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.upper_bus, "rb+", buffering=0) as upper_bus, open(
        args.forearm_bus, "rb+", buffering=0
    ) as forearm_bus, args.output.open("w", encoding="utf-8") as output:
        fd_upper, fd_forearm = upper_bus.fileno(), forearm_bus.fileno()
        start_sensor(fd_upper, "BNO1 upper_arm")
        start_sensor(fd_forearm, "BNO2 forearm")
        input("Kolu duz ve sabit tutup Enter'a basin: ")
        neutral = capture_neutral(fd_upper, fd_forearm)
        axis: tuple[float, float, float] | None = None
        if args.angle_mode == "hinge":
            axis = learn_axis(fd_upper, fd_forearm, neutral, args.axis_seconds)
            print("Ogrenilen eksen:", tuple(round(value, 4) for value in axis))
            input("Kolu tekrar duz ve sabit tutup Enter'a basin: ")
            neutral = capture_neutral(fd_upper, fd_forearm)
        elif args.angle_mode == "3d":
            print("3D aci modu: eksen ogrenme atlandi.")
        else:
            print(f"Segment aci modu: sabit {args.segment_axis.upper()} ekseni kullaniliyor.")
        print("Kayit:", args.output.resolve())
        if target:
            print(f"UDP: {target[0]}:{target[1]}")

        period = 1.0 / args.hz
        next_tick = time.monotonic()
        previous_tick_ns: int | None = None
        previous_wrapped: float | None = None
        continuous_flexion = 0.0
        filtered_velocity = 0.0
        sequence = 0
        i2c_errors = 0
        udp_errors = 0
        try:
            while True:
                try:
                    upper = read_sensor(fd_upper, "upper_arm")
                    forearm = read_sensor(fd_forearm, "forearm")
                except (OSError, RuntimeError) as exc:
                    i2c_errors += 1
                    print(f"I2C HATA #{i2c_errors}: {exc}")
                    sequence += 1
                    next_tick += period
                    time.sleep(max(0.0, next_tick - time.monotonic()))
                    continue
                tick_ns = time.monotonic_ns()
                relative = relative_orientation(upper, forearm)
                delta = delta_orientation(neutral, relative)
                relative_3d = total_angle_deg(delta)
                forearm_gyro_upper = q_rotate(relative, forearm["gyro_rad_s"])
                relative_gyro = tuple(
                    forearm_gyro_upper[index] - upper["gyro_rad_s"][index]
                    for index in range(3)
                )
                if args.angle_mode == "segment":
                    flexion = segment_angle_deg(delta, segment_axis)
                    off_axis = None
                    gyro_velocity = None
                elif args.angle_mode == "3d":
                    flexion = relative_3d
                    off_axis = None
                    gyro_velocity = None
                else:
                    flexion = args.flexion_sign * hinge_angle_deg(delta, axis)
                    off_axis = off_axis_angle_deg(delta, axis)
                    gyro_velocity = args.flexion_sign * math.degrees(dot(relative_gyro, axis))

                sample_period_ms = period * 1000.0
                if previous_tick_ns is not None and previous_wrapped is not None:
                    dt = max((tick_ns - previous_tick_ns) / 1.0e9, 1.0e-6)
                    change = wrapped_difference(flexion, previous_wrapped)
                    continuous_flexion += change
                    quaternion_velocity = change / dt
                    alpha = 1.0 - math.exp(-2.0 * math.pi * 4.0 * dt)
                    filtered_velocity += alpha * (quaternion_velocity - filtered_velocity)
                    sample_period_ms = dt * 1000.0
                else:
                    continuous_flexion = flexion

                upper_net = network_sensor(upper)
                forearm_net = network_sensor(forearm)
                timestamp_ns = (upper["sample_utc_ns"] + forearm["sample_utc_ns"]) // 2
                read_skew_ms = abs(forearm["sample_monotonic_ns"] - upper["sample_monotonic_ns"]) / 1.0e6
                packet = build_arm_imu_packet(
                    sequence=sequence,
                    timestamp_ns=timestamp_ns,
                    side=args.side,
                    flexion_deg=flexion,
                    velocity_deg_s=filtered_velocity,
                    relative_3d_deg=relative_3d,
                    off_axis_deg=off_axis,
                    upper=upper_net,
                    forearm=forearm_net,
                    read_skew_ms=read_skew_ms,
                    sample_period_ms=sample_period_ms,
                )
                packet["elbow"]["flexion_unwrapped_deg"] = continuous_flexion
                packet["elbow"]["velocity_gyro_deg_s"] = gyro_velocity
                packet["calibration"] = {
                    "neutral_relative_wxyz": list(neutral),
                    "hinge_axis_upper_xyz": None if axis is None else list(axis),
                    "segment_axis_sensor_xyz": list(segment_axis),
                }
                packet["diagnostics"] = {
                    "i2c_error_count": i2c_errors,
                    "udp_error_count": udp_errors,
                }
                packet["source_id"] = args.source_id
                packet["session_id"] = session_id
                packet["capture"] = {
                    "implementation": "bno055_elbow_stream/v1",
                    "nominal_rate_hz": args.hz,
                    "upper_bus": args.upper_bus,
                    "forearm_bus": args.forearm_bus,
                    "flexion_sign": args.flexion_sign,
                    "angle_mode": args.angle_mode,
                    "segment_axis": args.segment_axis,
                }
                line = json.dumps(packet, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
                output.write(line + "\n")
                if sequence % 20 == 0:
                    output.flush()
                if udp is not None and target is not None:
                    try:
                        udp.sendto(line.encode("utf-8"), target)
                    except (BlockingIOError, OSError) as exc:
                        udp_errors += 1
                        if udp_errors <= 3 or udp_errors % 100 == 0:
                            print(f"UDP HATA #{udp_errors}: {exc}")
                off_axis_text = "  ---" if off_axis is None else f"{off_axis:5.1f}"
                print(
                    f"dirsek={flexion:6.1f}  3D={relative_3d:6.1f}  "
                    f"eksen_disi={off_axis_text}  hiz={filtered_velocity:7.1f}  "
                    f"kal={upper['calibration']}/{forearm['calibration']}  q={packet['quality']:.2f}"
                )
                previous_tick_ns = tick_ns
                previous_wrapped = flexion
                sequence += 1
                next_tick += period
                delay = next_tick - time.monotonic()
                if delay > 0.0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()
        except KeyboardInterrupt:
            print("\nKayit durduruldu.")
        finally:
            if udp is not None:
                udp.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

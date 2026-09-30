"""Offline first check for one or two BNO055 sensors on Raspberry Pi I2C-1.

Runs with the Python standard library only. In Windows Thonny, choose
"Remote Python 3 (SSH)" before pressing F5. The two-sensor angle printed here
is a diagnostic orientation difference, not an anatomical elbow angle.
"""

from __future__ import annotations

import ctypes
import math
import os
import struct
import time


# Start with only one sensor. After it works, power the Pi off, connect the
# second sensor with ADR high (3.3 V), then change this to (0x28, 0x29).
ADDRESSES = (0x28,)
I2C_DEVICE = "/dev/i2c-1"
SAMPLE_HZ = 2.0

I2C_RDWR = 0x0707
I2C_M_RD = 0x0001


class I2CMessage(ctypes.Structure):
    _fields_ = [
        ("addr", ctypes.c_uint16),
        ("flags", ctypes.c_uint16),
        ("len", ctypes.c_uint16),
        ("buf", ctypes.POINTER(ctypes.c_uint8)),
    ]


class I2CTransaction(ctypes.Structure):
    _fields_ = [
        ("msgs", ctypes.POINTER(I2CMessage)),
        ("nmsgs", ctypes.c_uint32),
    ]


LIBC = ctypes.CDLL(None, use_errno=True)
LIBC.ioctl.argtypes = (ctypes.c_int, ctypes.c_ulong, ctypes.c_void_p)
LIBC.ioctl.restype = ctypes.c_int


def transact(fd: int, messages: ctypes.Array[I2CMessage]) -> None:
    request = I2CTransaction(messages, len(messages))
    if LIBC.ioctl(fd, I2C_RDWR, ctypes.byref(request)) < 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))


def read_register(fd: int, address: int, register: int, count: int) -> bytes:
    # Bosch requires a repeated START between register selection and reading.
    command = (ctypes.c_uint8 * 1)(register)
    result = (ctypes.c_uint8 * count)()
    messages = (I2CMessage * 2)(
        I2CMessage(address, 0, 1, command),
        I2CMessage(address, I2C_M_RD, count, result),
    )
    for attempt in range(3):
        try:
            transact(fd, messages)
            return bytes(result)
        except OSError:
            if attempt == 2:
                raise
            time.sleep(0.05)
    raise AssertionError("unreachable")


def write_register(fd: int, address: int, register: int, value: int) -> None:
    data = (ctypes.c_uint8 * 2)(register, value)
    messages = (I2CMessage * 1)(I2CMessage(address, 0, 2, data))
    transact(fd, messages)


def start_sensor(fd: int, address: int) -> None:
    chip_id = read_register(fd, address, 0x00, 1)[0]
    if chip_id != 0xA0:
        raise RuntimeError(f"0x{address:02x}: CHIP_ID 0x{chip_id:02x}, expected 0xa0")
    write_register(fd, address, 0x3D, 0x00)  # CONFIG mode
    time.sleep(0.025)
    write_register(fd, address, 0x07, 0x00)  # register page 0
    write_register(fd, address, 0x3E, 0x00)  # normal power
    write_register(fd, address, 0x3B, 0x00)  # m/s2, degrees/s, degrees
    write_register(fd, address, 0x3D, 0x0C)  # NDOF fusion
    time.sleep(0.7)
    print(f"0x{address:02x}: BNO055 bulundu, CHIP_ID=0xa0, NDOF acik")


def read_sample(fd: int, address: int) -> dict[str, object]:
    raw_q = struct.unpack("<hhhh", read_register(fd, address, 0x20, 8))
    q_wxyz = tuple(value / 16384.0 for value in raw_q)
    q_norm = math.sqrt(sum(value * value for value in q_wxyz))
    q_xyzw = None
    if 0.8 <= q_norm <= 1.2:
        w, x, y, z = q_wxyz
        q_xyzw = (x / q_norm, y / q_norm, z / q_norm, w / q_norm)

    raw_acc = struct.unpack("<hhh", read_register(fd, address, 0x08, 6))
    raw_gyro = struct.unpack("<hhh", read_register(fd, address, 0x14, 6))
    status = read_register(fd, address, 0x35, 1)[0]
    return {
        "address": address,
        "t_monotonic_ns": time.monotonic_ns(),
        "q_xyzw": q_xyzw,
        "q_norm": q_norm,
        "acc_m_s2": tuple(value / 100.0 for value in raw_acc),
        "gyro_rad_s": tuple(math.radians(value / 16.0) for value in raw_gyro),
        "cal": ((status >> 6) & 3, (status >> 4) & 3, (status >> 2) & 3, status & 3),
    }


def angle_between_deg(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    dot = sum(x * y for x, y in zip(a, b))
    return math.degrees(2.0 * math.acos(min(1.0, abs(dot))))


def format_sample(sample: dict[str, object]) -> str:
    q = sample["q_xyzw"]
    q_text = "gecersiz" if q is None else "(" + ",".join(f"{v:+.3f}" for v in q) + ")"
    acc = sample["acc_m_s2"]
    gyro = sample["gyro_rad_s"]
    return (
        f"0x{sample['address']:02x} t={sample['t_monotonic_ns']} "
        f"cal={sample['cal']} q_xyzw={q_text} |q|={sample['q_norm']:.3f} "
        f"acc_m_s2={tuple(round(v, 2) for v in acc)} "
        f"gyro_rad_s={tuple(round(v, 3) for v in gyro)}"
    )


def main() -> None:
    try:
        fd = os.open(I2C_DEVICE, os.O_RDWR)
    except FileNotFoundError as exc:
        raise SystemExit("/dev/i2c-1 yok: Pi'de I2C arayuzunu etkinlestirin ve yeniden baslatin.") from exc
    except PermissionError as exc:
        raise SystemExit("/dev/i2c-1 izin hatasi: 'mesut' kullanicisinin i2c grubunu kontrol edin.") from exc

    try:
        for address in ADDRESSES:
            start_sensor(fd, address)
        print("Kalibrasyon sirasi: sistem, gyro, ivme, manyetometre (her biri 0-3).")
        print("Durdurmak icin Thonny'deki kirmizi Stop dugmesine basin.\n")
        while True:
            started = time.monotonic()
            samples = [read_sample(fd, address) for address in ADDRESSES]
            line = " | ".join(format_sample(sample) for sample in samples)
            if len(samples) == 2:
                q_a, q_b = samples[0]["q_xyzw"], samples[1]["q_xyzw"]
                if q_a is not None and q_b is not None:
                    line += f" | sensorler_arasi_3d_aci={angle_between_deg(q_a, q_b):.1f} deg"
            print(line)
            time.sleep(max(0.0, 1.0 / SAMPLE_HZ - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print("Okuma durduruldu.")
    except OSError as exc:
        raise SystemExit(f"I2C okuma hatasi: {exc}. Kablolari ve 10 kHz ayarini kontrol edin.") from exc
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()

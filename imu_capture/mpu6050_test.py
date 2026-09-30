"""Dependency-free MPU-6050/GY-521 bring-up test for Raspberry Pi Linux I2C."""

from __future__ import annotations

import argparse
import ctypes as c
import math
import os
import struct
import time
from typing import Any


I2C_RDWR = 0x0707
I2C_M_RD = 0x0001


class I2cMessage(c.Structure):
    _fields_ = [
        ("address", c.c_uint16),
        ("flags", c.c_uint16),
        ("length", c.c_uint16),
        ("data", c.POINTER(c.c_uint8)),
    ]


class I2cTransfer(c.Structure):
    _fields_ = [("messages", c.POINTER(I2cMessage)), ("count", c.c_uint32)]


LIBC: Any = c.CDLL(None, use_errno=True)


def transfer(fd: int, messages: Any) -> None:
    operation = I2cTransfer(messages, len(messages))
    if LIBC.ioctl(fd, I2C_RDWR, c.byref(operation)) < 0:
        error = c.get_errno()
        raise OSError(error, os.strerror(error))


def read_registers(fd: int, address: int, register: int, count: int) -> bytes:
    command = (c.c_uint8 * 1)(register)
    response = (c.c_uint8 * count)()
    messages = (I2cMessage * 2)(
        I2cMessage(address, 0, 1, command),
        I2cMessage(address, I2C_M_RD, count, response),
    )
    transfer(fd, messages)
    return bytes(response)


def write_register(fd: int, address: int, register: int, value: int) -> None:
    data = (c.c_uint8 * 2)(register, value)
    messages = (I2cMessage * 1)(I2cMessage(address, 0, 2, data))
    transfer(fd, messages)


def configure(fd: int, address: int) -> None:
    who_am_i = read_registers(fd, address, 0x75, 1)[0]
    if who_am_i != 0x68:
        raise RuntimeError(
            f"WHO_AM_I=0x{who_am_i:02x}; beklenen MPU-6050 degeri 0x68"
        )
    write_register(fd, address, 0x6B, 0x01)  # Wake, PLL with X gyro clock.
    time.sleep(0.10)
    write_register(fd, address, 0x1A, 0x03)  # DLPF about 44 Hz accel / 42 Hz gyro.
    write_register(fd, address, 0x19, 0x09)  # 1 kHz / (1 + 9) = 100 Hz.
    write_register(fd, address, 0x1B, 0x08)  # Gyro +/-500 deg/s, 65.5 LSB/(deg/s).
    write_register(fd, address, 0x1C, 0x08)  # Accel +/-4 g, 8192 LSB/g.
    print(f"MPU6050 bulundu: adres=0x{address:02x}, WHO_AM_I=0x{who_am_i:02x}")


def read_sample(fd: int, address: int) -> dict[str, tuple[float, float, float] | float]:
    ax, ay, az, raw_temp, gx, gy, gz = struct.unpack(
        ">hhhhhhh", read_registers(fd, address, 0x3B, 14)
    )
    gravity = 9.80665
    accel = tuple(raw / 8192.0 * gravity for raw in (ax, ay, az))
    gyro = tuple(raw / 65.5 for raw in (gx, gy, gz))
    temperature = raw_temp / 340.0 + 36.53
    return {"accel_m_s2": accel, "gyro_deg_s": gyro, "temperature_c": temperature}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MPU6050 Raspberry Pi ilk test")
    parser.add_argument("--bus", default="/dev/i2c-1")
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x68)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--seconds", type=float, default=30.0)
    args = parser.parse_args()
    if args.address not in (0x68, 0x69):
        parser.error("--address 0x68 veya 0x69 olmali")
    if not 1.0 <= args.hz <= 100.0:
        parser.error("--hz 1 ile 100 arasinda olmali")
    return args


def main() -> int:
    args = parse_args()
    period = 1.0 / args.hz
    deadline = math.inf if args.seconds <= 0.0 else time.monotonic() + args.seconds
    with open(args.bus, "rb+", buffering=0) as bus:
        fd = bus.fileno()
        configure(fd, args.address)
        print("Kart sabitken |ivme| yaklasik 9.81 m/s2, gyro eksenleri 0 deg/s olmali.")
        try:
            while time.monotonic() < deadline:
                started = time.monotonic()
                sample = read_sample(fd, args.address)
                ax, ay, az = sample["accel_m_s2"]
                gx, gy, gz = sample["gyro_deg_s"]
                accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
                print(
                    f"acc=({ax:7.3f},{ay:7.3f},{az:7.3f}) m/s2 "
                    f"|a|={accel_norm:6.3f}  "
                    f"gyro=({gx:7.2f},{gy:7.2f},{gz:7.2f}) deg/s  "
                    f"T={sample['temperature_c']:5.1f} C"
                )
                time.sleep(max(0.0, period - (time.monotonic() - started)))
        except KeyboardInterrupt:
            print("\nTest durduruldu.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

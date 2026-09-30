"""Identify and smoke-test the HW-290 MPU6050 + compass + BMP180 board."""

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


def is_present(fd: int, address: int, register: int = 0x00) -> bool:
    try:
        read_registers(fd, address, register, 1)
        return True
    except OSError:
        return False


def configure_mpu6050(fd: int, address: int) -> None:
    write_register(fd, address, 0x6B, 0x01)  # Wake, PLL with X gyro clock.
    time.sleep(0.10)
    write_register(fd, address, 0x1A, 0x03)  # DLPF.
    write_register(fd, address, 0x19, 0x09)  # 100 Hz internal sample rate.
    write_register(fd, address, 0x1B, 0x08)  # Gyro +/-500 deg/s.
    write_register(fd, address, 0x1C, 0x08)  # Accel +/-4 g.
    write_register(fd, address, 0x6A, 0x00)  # Disable auxiliary I2C master.
    write_register(fd, address, 0x37, 0x02)  # Expose compass in bypass mode.
    time.sleep(0.05)


def read_motion(fd: int, address: int) -> tuple[tuple[float, ...], tuple[float, ...]]:
    ax, ay, az, _temperature, gx, gy, gz = struct.unpack(
        ">hhhhhhh", read_registers(fd, address, 0x3B, 14)
    )
    accel = tuple(raw / 8192.0 * 9.80665 for raw in (ax, ay, az))
    gyro = tuple(raw / 65.5 for raw in (gx, gy, gz))
    return accel, gyro


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="HW-290 10DOF kimlik ve hareket testi")
    parser.add_argument("--bus", default="/dev/i2c-1")
    parser.add_argument("--seconds", type=float, default=15.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    with open(args.bus, "rb+", buffering=0) as bus:
        fd = bus.fileno()
        if not is_present(fd, 0x68, 0x75):
            raise RuntimeError("0x68 adresinde MPU6050 bulunamadi")
        mpu_id = read_registers(fd, 0x68, 0x75, 1)[0]
        print(f"Hareket yongasi: MPU6050, adres=0x68, WHO_AM_I=0x{mpu_id:02x}")
        configure_mpu6050(fd, 0x68)

        if is_present(fd, 0x1E, 0x0A):
            identity = read_registers(fd, 0x1E, 0x0A, 3)
            print(
                "Manyetometre: HMC5883L adayi, adres=0x1e, "
                f"ID={identity.hex()} (beklenen 483433)"
            )
        elif is_present(fd, 0x0D, 0x0D):
            chip_id = read_registers(fd, 0x0D, 0x0D, 1)[0]
            print(
                "Manyetometre: QMC5883 adayi, adres=0x0d, "
                f"CHIP_ID=0x{chip_id:02x}"
            )
        else:
            print("Manyetometre: 0x1e veya 0x0d adresinde bulunamadi")

        if is_present(fd, 0x77, 0xD0):
            pressure_id = read_registers(fd, 0x77, 0xD0, 1)[0]
            print(
                "Basinc yongasi: BMP180 adayi, adres=0x77, "
                f"CHIP_ID=0x{pressure_id:02x} (beklenen 0x55)"
            )
        else:
            print("Basinc yongasi: 0x77 adresinde bulunamadi")

        print("15 saniyelik MPU6050 ivme/gyro testi basliyor.")
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            accel, gyro = read_motion(fd, 0x68)
            accel_norm = math.sqrt(sum(value * value for value in accel))
            print(
                f"acc=({accel[0]:7.3f},{accel[1]:7.3f},{accel[2]:7.3f}) "
                f"|a|={accel_norm:6.3f} m/s2  "
                f"gyro=({gyro[0]:7.2f},{gyro[1]:7.2f},{gyro[2]:7.2f}) deg/s"
            )
            time.sleep(0.10)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

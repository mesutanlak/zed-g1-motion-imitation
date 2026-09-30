"""Simultaneous two-bus test for two HW-290 10DOF boards on Raspberry Pi."""

from __future__ import annotations

import ctypes as c
from contextlib import ExitStack
import math
import os
import struct
import time
from typing import Any


BOARDS = (("HW290-1", "/dev/i2c-1"), ("HW290-2", "/dev/i2c-8"))
TEST_SECONDS = 30.0
PRINT_HZ = 5.0
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


LIBC: Any | None = None


def system_libc() -> Any:
    global LIBC
    if LIBC is None:
        LIBC = c.CDLL(None, use_errno=True)
    return LIBC


def transfer(fd: int, messages: Any) -> None:
    operation = I2cTransfer(messages, len(messages))
    if system_libc().ioctl(fd, I2C_RDWR, c.byref(operation)) < 0:
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


def write_register_verified(
    fd: int,
    address: int,
    register: int,
    value: int,
    *,
    mask: int = 0xFF,
    retries: int = 5,
) -> int:
    """Write a configuration byte and verify it, retrying flaky clone boards."""
    last_error: Exception | None = None
    for _attempt in range(retries):
        try:
            write_register(fd, address, register, value)
            time.sleep(0.03)
            actual = read_registers(fd, address, register, 1)[0]
            if actual & mask == value & mask:
                return actual
            last_error = RuntimeError(
                f"register 0x{register:02x}: yazilan 0x{value:02x}, "
                f"okunan 0x{actual:02x}"
            )
        except OSError as exc:
            last_error = exc
        time.sleep(0.05)
    raise RuntimeError(
        f"MPU6050 register 0x{register:02x} {retries} denemede "
        f"dogrulanamadi: {last_error}"
    )


def present(fd: int, address: int, register: int = 0x00) -> bool:
    try:
        read_registers(fd, address, register, 1)
        return True
    except OSError:
        return False


def configure_mpu6050(
    fd: int, *, gyro_range_dps: int = 500
) -> dict[str, float | int]:
    who_am_i = read_registers(fd, 0x68, 0x75, 1)[0]
    if who_am_i != 0x68:
        raise RuntimeError(f"MPU6050 WHO_AM_I=0x{who_am_i:02x}; beklenen 0x68")
    # Reset may self-clear before it can be read back. Confirm communication
    # after reset, then verify every persistent configuration write.
    reset_error: Exception | None = None
    for _attempt in range(5):
        try:
            write_register(fd, 0x68, 0x6B, 0x80)
            time.sleep(0.15)
            if read_registers(fd, 0x68, 0x75, 1)[0] == 0x68:
                reset_error = None
                break
        except OSError as exc:
            reset_error = exc
        time.sleep(0.05)
    else:
        raise RuntimeError(f"MPU6050 reset sonrasi cevap vermedi: {reset_error}")

    write_register_verified(fd, 0x68, 0x6B, 0x01, mask=0x7F)  # Wake, PLL.
    write_register_verified(fd, 0x68, 0x6C, 0x00)  # Enable all axes.
    write_register_verified(fd, 0x68, 0x1A, 0x03, mask=0x07)  # DLPF.
    write_register_verified(fd, 0x68, 0x19, 0x09)  # 100 Hz internal rate.
    # Keep the accelerometer at its power-on range because one tested HW-290
    # clone reports +/-4 g while continuing to use +/-2 g scaling.  Arm motion
    # in the BODY_38 trials can exceed 250 deg/s, so use a wider gyro range and
    # verify the value by reading the register back below.
    gyro_range_register = {250: 0x00, 500: 0x08, 1000: 0x10, 2000: 0x18}
    if gyro_range_dps not in gyro_range_register:
        raise ValueError("gyro_range_dps 250, 500, 1000 veya 2000 olmali")
    write_register_verified(
        fd, 0x68, 0x1B, gyro_range_register[gyro_range_dps], mask=0x18
    )
    write_register_verified(fd, 0x68, 0x1C, 0x00, mask=0x18)  # Accel +/-2 g.
    write_register_verified(
        fd, 0x68, 0x6A, 0x00, mask=0x20
    )  # Disable auxiliary I2C master.
    write_register_verified(
        fd, 0x68, 0x37, 0x02, mask=0x02
    )  # Bypass: expose compass to Pi bus.
    time.sleep(0.05)

    # Read the settings back. Some MPU6050-compatible clones power up with a
    # different range or ignore the first configuration write.
    gyro_config = read_registers(fd, 0x68, 0x1B, 1)[0]
    accel_config = read_registers(fd, 0x68, 0x1C, 1)[0]
    power_mgmt_1 = read_registers(fd, 0x68, 0x6B, 1)[0]
    power_mgmt_2 = read_registers(fd, 0x68, 0x6C, 1)[0]
    if power_mgmt_1 & 0x40:
        raise RuntimeError(
            f"MPU6050 uyku modundan cikmadi: PWR_MGMT_1=0x{power_mgmt_1:02x}"
        )
    gyro_fs = (gyro_config >> 3) & 0x03
    accel_fs = (accel_config >> 3) & 0x03
    gyro_scales = (131.0, 65.5, 32.8, 16.4)
    accel_scales = (16384.0, 8192.0, 4096.0, 2048.0)
    return {
        "who_am_i": who_am_i,
        "gyro_config": gyro_config,
        "accel_config": accel_config,
        "power_mgmt_1": power_mgmt_1,
        "power_mgmt_2": power_mgmt_2,
        "gyro_scale": gyro_scales[gyro_fs],
        "accel_scale": accel_scales[accel_fs],
    }


def configure_compass(fd: int) -> str | None:
    # New HW-290/GY-87 variants often use HP5883/QMC5883P at 0x2C.
    if present(fd, 0x2C, 0x00):
        chip_id = read_registers(fd, 0x2C, 0x00, 1)[0]
        if chip_id == 0x80:
            write_register(fd, 0x2C, 0x0B, 0x01)  # Soft reset.
            time.sleep(0.02)
            write_register(fd, 0x2C, 0x0A, 0xCD)  # Continuous mode.
            time.sleep(0.02)
            return f"hp:{chip_id:02x}"
    if present(fd, 0x1E, 0x0A):
        identity = read_registers(fd, 0x1E, 0x0A, 3)
        write_register(fd, 0x1E, 0x00, 0x70)
        write_register(fd, 0x1E, 0x01, 0x20)
        write_register(fd, 0x1E, 0x02, 0x00)
        return f"hmc:{identity.hex()}"
    if present(fd, 0x0D, 0x0D):
        chip_id = read_registers(fd, 0x0D, 0x0D, 1)[0]
        write_register(fd, 0x0D, 0x0B, 0x01)
        write_register(fd, 0x0D, 0x09, 0x1D)
        return f"qmc:{chip_id:02x}"
    return None


def read_compass(fd: int, model: str | None) -> tuple[float, float, float] | None:
    if model is None:
        return None
    if model.startswith("hmc:"):
        x, z, y = struct.unpack(">hhh", read_registers(fd, 0x1E, 0x03, 6))
        return x * 0.092, y * 0.092, z * 0.092
    if model.startswith("hp:"):
        # HP5883 clean register map: XYZ little-endian from 0x01.
        x, y, z = struct.unpack("<hhh", read_registers(fd, 0x2C, 0x01, 6))
        return float(x), float(y), float(z)
    x, y, z = struct.unpack("<hhh", read_registers(fd, 0x0D, 0x00, 6))
    return x / 30.0, y / 30.0, z / 30.0


def read_motion(
    fd: int, accel_scale: float, gyro_scale: float
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    ax, ay, az, _temperature, gx, gy, gz = struct.unpack(
        ">hhhhhhh", read_registers(fd, 0x68, 0x3B, 14)
    )
    accel = tuple(raw / accel_scale * 9.80665 for raw in (ax, ay, az))
    gyro = tuple(raw / gyro_scale for raw in (gx, gy, gz))
    return accel, gyro


def read_bmp_calibration(fd: int) -> tuple[int, ...] | None:
    if not present(fd, 0x77, 0xD0):
        return None
    chip_id = read_registers(fd, 0x77, 0xD0, 1)[0]
    if chip_id != 0x55:
        return None
    return struct.unpack(">hhhHHHhhhhh", read_registers(fd, 0x77, 0xAA, 22))


def read_bmp180(fd: int, cal: tuple[int, ...] | None) -> tuple[float, float] | None:
    if cal is None:
        return None
    ac1, ac2, ac3, ac4, ac5, ac6, b1, b2, _mb, mc, md = cal
    write_register(fd, 0x77, 0xF4, 0x2E)
    time.sleep(0.005)
    ut = struct.unpack(">H", read_registers(fd, 0x77, 0xF6, 2))[0]
    x1 = ((ut - ac6) * ac5) >> 15
    denominator = x1 + md
    if denominator == 0:
        return None
    x2 = (mc << 11) // denominator
    b5 = x1 + x2
    temperature_c = ((b5 + 8) >> 4) / 10.0

    write_register(fd, 0x77, 0xF4, 0x34)  # OSS=0.
    time.sleep(0.005)
    raw = read_registers(fd, 0x77, 0xF6, 3)
    up = ((raw[0] << 16) | (raw[1] << 8) | raw[2]) >> 8
    b6 = b5 - 4000
    x1 = (b2 * ((b6 * b6) >> 12)) >> 11
    x2 = (ac2 * b6) >> 11
    x3 = x1 + x2
    b3 = (((ac1 * 4 + x3) + 2) >> 2)
    x1 = (ac3 * b6) >> 13
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16
    x3 = ((x1 + x2) + 2) >> 2
    b4 = (ac4 * (x3 + 32768)) >> 15
    if b4 == 0:
        return None
    b7 = (up - b3) * 50000
    pressure = (b7 * 2) // b4 if b7 < 0x80000000 else (b7 // b4) * 2
    x1 = (pressure >> 8) * (pressure >> 8)
    x1 = (x1 * 3038) >> 16
    x2 = (-7357 * pressure) >> 16
    pressure += (x1 + x2 + 3791) >> 4
    return temperature_c, pressure / 100.0


def initialize_board(name: str, path: str, fd: int) -> dict[str, Any]:
    mpu = configure_mpu6050(fd)
    compass = configure_compass(fd)
    bmp_cal = read_bmp_calibration(fd)
    compass_text = compass or "YOK"
    bmp_text = "BMP180/0x55" if bmp_cal is not None else "YOK"
    print(
        f"{name} {path}: MPU6050=0x{int(mpu['who_am_i']):02x}, "
        f"ACCEL_CONFIG=0x{int(mpu['accel_config']):02x} "
        f"({mpu['accel_scale']:.0f} LSB/g), "
        f"GYRO_CONFIG=0x{int(mpu['gyro_config']):02x} "
        f"({mpu['gyro_scale']:.1f} LSB/dps), "
        f"pusula={compass_text}, basinc={bmp_text}"
    )
    return {
        "name": name,
        "path": path,
        "fd": fd,
        "compass": compass,
        "bmp_cal": bmp_cal,
        "accel_scale": mpu["accel_scale"],
        "gyro_scale": mpu["gyro_scale"],
        "errors": 0,
    }


def format_sample(board: dict[str, Any]) -> str:
    try:
        accel, gyro = read_motion(
            board["fd"], board["accel_scale"], board["gyro_scale"]
        )
        compass = read_compass(board["fd"], board["compass"])
        bmp = read_bmp180(board["fd"], board["bmp_cal"])
        accel_norm = math.sqrt(sum(value * value for value in accel))
        if compass is None:
            mag_text = "YOK"
        else:
            mag_unit = "raw" if board["compass"].startswith("hp:") else "uT"
            mag_text = (
                f"({compass[0]:.1f},{compass[1]:.1f},{compass[2]:.1f}){mag_unit}"
            )
        bmp_text = "YOK" if bmp is None else f"{bmp[1]:.1f}hPa/{bmp[0]:.1f}C"
        return (
            f"{board['name']}: |a|={accel_norm:5.2f} "
            f"g=({gyro[0]:6.1f},{gyro[1]:6.1f},{gyro[2]:6.1f})dps "
            f"mag={mag_text} bmp={bmp_text}"
        )
    except (OSError, RuntimeError, ZeroDivisionError) as exc:
        board["errors"] += 1
        return f"{board['name']}: HATA #{board['errors']} {exc}"


def main() -> int:
    print("Iki karti sabit tutun. Test 30 saniye sonra otomatik bitecek.")
    with ExitStack() as stack:
        boards = []
        for name, path in BOARDS:
            bus = stack.enter_context(open(path, "rb+", buffering=0))
            try:
                boards.append(initialize_board(name, path, bus.fileno()))
            except (OSError, RuntimeError) as exc:
                print(f"{name} {path}: BASLATILAMADI -> {exc}")
        if not boards:
            print("Hicbir HW-290 baslatilamadi.")
            return 1
        deadline = time.monotonic() + TEST_SECONDS
        period = 1.0 / PRINT_HZ
        try:
            while time.monotonic() < deadline:
                started = time.monotonic()
                print(" | ".join(format_sample(board) for board in boards))
                time.sleep(max(0.0, period - (time.monotonic() - started)))
        except KeyboardInterrupt:
            print("\nTest durduruldu.")
        print("Test tamamlandi:", ", ".join(f"{b['name']} hata={b['errors']}" for b in boards))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

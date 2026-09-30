"""Read two Adafruit BNO055 boards on one Raspberry Pi I2C bus.

This is a hardware bring-up tool. Its relative rotation is a diagnostic, not
an anatomically calibrated elbow flexion angle or a robot control signal.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any


def normalize_xyzw(values: tuple[float, float, float, float]) -> tuple[float, float, float, float] | None:
    if not all(math.isfinite(float(value)) for value in values):
        return None
    length = math.sqrt(sum(float(value) ** 2 for value in values))
    if not 0.5 <= length <= 1.5:
        return None
    return tuple(float(value) / length for value in values)


def conjugate_xyzw(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = q
    return -x, -y, -z, w


def multiply_xyzw(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def relative_change_deg(
    reference: tuple[float, float, float, float],
    current: tuple[float, float, float, float],
) -> float:
    change = multiply_xyzw(conjugate_xyzw(reference), current)
    return math.degrees(2.0 * math.acos(min(1.0, abs(change[3]))))


def finite_vector(values: Any) -> list[float] | None:
    if values is None or any(value is None for value in values):
        return None
    result = [float(value) for value in values]
    return result if all(math.isfinite(value) for value in result) else None


def read_one(sensor: Any, name: str, address: int) -> dict[str, Any]:
    sample: dict[str, Any] = {
        "sensor_id": name,
        "address": f"0x{address:02x}",
        "pi_monotonic_ns": time.monotonic_ns(),
        "pi_utc_ns": time.time_ns(),
        "quaternion_xyzw": None,
        "gyro_rad_s": None,
        "accel_m_s2": None,
        "calibration": None,
        "valid": False,
        "orientation_ready": False,
    }
    try:
        # Adafruit's BNO055 quaternion register order is w, x, y, z.
        w, x, y, z = sensor.quaternion
        sample["pi_monotonic_ns"] = time.monotonic_ns()
        sample["pi_utc_ns"] = time.time_ns()
        if None not in (w, x, y, z):
            q = normalize_xyzw((x, y, z, w))
            if q is not None:
                sample["quaternion_xyzw"] = q
                sample["valid"] = True
        system, gyro, accel, mag = sensor.calibration_status
        sample["calibration"] = {
            "system": system,
            "gyro": gyro,
            "accel": accel,
            "mag": mag,
        }
        sample["orientation_ready"] = sample["valid"] and system > 0
        sample["gyro_rad_s"] = finite_vector(sensor.gyro)
        sample["accel_m_s2"] = finite_vector(sensor.acceleration)
    except (OSError, RuntimeError, TypeError, ValueError) as exc:
        sample["valid"] = False
        sample["orientation_ready"] = False
        sample["error"] = f"{type(exc).__name__}: {exc}"
    return sample


def describe(sample: dict[str, Any]) -> str:
    if "error" in sample:
        return f"{sample['sensor_id']} HATA={sample['error']}"
    calibration = sample["calibration"] or {}
    levels = "/".join(str(calibration.get(key, "?")) for key in ("system", "gyro", "accel", "mag"))
    quaternion = sample["quaternion_xyzw"]
    q_text = "yok" if quaternion is None else "(" + ", ".join(f"{part:+.3f}" for part in quaternion) + ")"
    return f"{sample['sensor_id']} kal={levels} hazir={sample['orientation_ready']} q_xyzw={q_text}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Raspberry Pi'de iki BNO055 ilk okuma testi")
    parser.add_argument("--upper-address", type=lambda value: int(value, 0), default=0x28)
    parser.add_argument("--forearm-address", type=lambda value: int(value, 0), default=0x29)
    parser.add_argument("--hz", type=float, default=5.0, help="Ilk test icin 5 Hz onerilir")
    parser.add_argument("--duration", type=float, default=0.0, help="Saniye; 0 ise Ctrl+C'ye kadar")
    parser.add_argument("--output", type=Path, help="Istege bagli JSONL kaydi")
    args = parser.parse_args()
    if args.upper_address == args.forearm_address:
        parser.error("Iki BNO055 farkli I2C adreslerinde olmali.")
    if not 0x03 <= args.upper_address <= 0x77 or not 0x03 <= args.forearm_address <= 0x77:
        parser.error("Gecersiz 7-bit I2C adresi.")
    if not 0 < args.hz <= 50:
        parser.error("--hz 0 ile 50 arasinda olmali.")
    if args.duration < 0:
        parser.error("--duration negatif olamaz.")
    return args


def main() -> int:
    args = parse_args()
    try:
        import board
        import adafruit_bno055
    except ImportError as exc:
        raise SystemExit("Pi ortaminda 'python -m pip install -r imu_capture/requirements-pi.txt' calistirin.") from exc

    i2c = board.I2C()
    while not i2c.try_lock():
        time.sleep(0.01)
    try:
        found = set(i2c.scan())
    finally:
        i2c.unlock()
    expected = {args.upper_address, args.forearm_address}
    if not expected.issubset(found):
        actual = ", ".join(f"0x{address:02x}" for address in sorted(found)) or "hicbiri"
        raise SystemExit(f"BNO055 adresleri bulunamadi. Beklenen {sorted(expected)}, gorulen {actual}. ADR ve kablolari kontrol edin.")

    upper = adafruit_bno055.BNO055_I2C(i2c, address=args.upper_address)
    forearm = adafruit_bno055.BNO055_I2C(i2c, address=args.forearm_address)
    print("Iki BNO055 acildi. Kalibrasyon sirasi: sistem/jyro/ivme/manyetometre (0-3).")
    print("Iki sensor hazir olunca kol duz pozda Enter'a basin: bu poz sifir kabul edilir.")
    print("Goreli donus toplam 3B degisimdir; dirsek bukulme acisi degildir.")

    file = None
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        file = args.output.open("w", encoding="utf-8")
        print(f"Kayit: {args.output}")

    period = 1.0 / args.hz
    started = time.monotonic()
    next_tick = started
    neutral = None
    can_prompt_for_zero = True
    sequence = 0
    try:
        while args.duration == 0 or time.monotonic() - started < args.duration:
            upper_sample = read_one(upper, "upper_arm", args.upper_address)
            forearm_sample = read_one(forearm, "forearm", args.forearm_address)
            upper_q = upper_sample["quaternion_xyzw"] if upper_sample["orientation_ready"] else None
            forearm_q = forearm_sample["quaternion_xyzw"] if forearm_sample["orientation_ready"] else None
            relative = None
            if upper_q is not None and forearm_q is not None:
                relative = multiply_xyzw(conjugate_xyzw(upper_q), forearm_q)
            if neutral is None and relative is not None and can_prompt_for_zero:
                try:
                    input("Iki sensor hazir. Kolu duz ve sabit tutup Enter'a basin: ")
                except EOFError:
                    can_prompt_for_zero = False
                else:
                    # Read again after Enter so the saved zero uses the actual pose.
                    upper_sample = read_one(upper, "upper_arm", args.upper_address)
                    forearm_sample = read_one(forearm, "forearm", args.forearm_address)
                    if upper_sample["orientation_ready"] and forearm_sample["orientation_ready"]:
                        relative = multiply_xyzw(
                            conjugate_xyzw(upper_sample["quaternion_xyzw"]),
                            forearm_sample["quaternion_xyzw"],
                        )
                        neutral = relative
                        print("Bu duz kol pozu sifir referansi olarak alindi.")
                    else:
                        relative = None
                        print("Sifir alinamadi. Iki sensorun yeniden hazir olmasini bekleyin.")
            change = None
            if neutral is not None and relative is not None:
                change = relative_change_deg(neutral, relative)
            packet = {
                "schema": "arm_imu_pair_probe/v1",
                "sequence": sequence,
                "upper_arm": upper_sample,
                "forearm": forearm_sample,
                "relative_rotation_from_start_deg": change,
            }
            if file is not None:
                file.write(json.dumps(packet, ensure_ascii=False, allow_nan=False) + "\n")
                file.flush()
            change_text = "yok" if change is None else f"{change:.1f} derece"
            print(f"{sequence:05d} | {describe(upper_sample)} | {describe(forearm_sample)} | goreli={change_text}")
            sequence += 1
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        print("\nKullanici durdurdu.")
    finally:
        if file is not None:
            file.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Interactive six-face accelerometer calibration for the two HW-290 boards."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import time

from dual_hw290_test import configure_mpu6050, read_motion


GRAVITY = 9.80665
POSES = (
    ("x_pos", "+X oku dik olarak YUKARI"),
    ("x_neg", "+X oku dik olarak ASAGI"),
    ("y_pos", "+Y yonu dik olarak YUKARI"),
    ("y_neg", "+Y yonu dik olarak ASAGI"),
    ("z_pos", "kartin +Z yuzu YUKARI"),
    ("z_neg", "kartin +Z yuzu ASAGI"),
)


def capture_pose(
    fd: int, config: dict[str, float | int], seconds: float
) -> tuple[float, float, float]:
    values: list[tuple[float, float, float]] = []
    io_errors = 0
    invalid_samples = 0
    consecutive_errors = 0
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            accel, _gyro = read_motion(
                fd, float(config["accel_scale"]), float(config["gyro_scale"])
            )
        except OSError as exc:
            io_errors += 1
            consecutive_errors += 1
            if consecutive_errors >= 3:
                try:
                    # Kisa bir kablo/bus kesintisinden sonra MPU'yu tekrar
                    # uyandir. Yeni okunan olcekleri de kullan.
                    config.update(configure_mpu6050(fd, gyro_range_dps=500))
                    consecutive_errors = 0
                except (OSError, RuntimeError):
                    pass
            if io_errors >= 8:
                raise RuntimeError(
                    "I2C arka arkaya okunamadi. VCC/GND/SDA/SCL, breadboard "
                    "temasi ve kablo uzunlugunu kontrol edin. "
                    f"Son hata: {exc}"
                ) from exc
            time.sleep(0.04)
            continue

        consecutive_errors = 0
        accel_norm = math.sqrt(sum(float(value) ** 2 for value in accel))
        if not all(math.isfinite(float(value)) for value in accel) or accel_norm < 2.0:
            invalid_samples += 1
            if invalid_samples >= 20 and not values:
                raise RuntimeError(
                    "MPU6050 sifir/gecersiz ivme veriyor. Bu kalibrasyon "
                    "yapilamaz; once I2C ve besleme baglantisini duzeltin."
                )
            time.sleep(0.01)
            continue
        values.append(tuple(float(value) for value in accel))
        time.sleep(0.01)

    minimum_samples = max(20, int(seconds * 30))
    if len(values) < minimum_samples:
        raise RuntimeError(
            f"Yalniz {len(values)} gecerli ornek alindi; en az "
            f"{minimum_samples} gerekli. I2C baglantisini kontrol edin."
        )
    medians = tuple(
        statistics.median(value[axis] for value in values) for axis in range(3)
    )
    median_norm = math.sqrt(sum(value * value for value in medians))
    # Bazi HW-290 klonlarinda bir ivme ekseninin kazanci ciddi bicimde
    # dusuk olabiliyor. Altı-yuz olcumu tam olarak bunu tespit etmek icin
    # yapildigindan, sifir olmayan dusuk bir pozu burada hemen reddetme.
    # Nihai karar asagida pozitif/negatif eksen araligindan verilir.
    if not 2.0 <= median_norm <= 25.0:
        raise RuntimeError(
            f"Gecersiz sabit ivme buyuklugu: {median_norm:.2f} m/s2. "
            f"Medyan={tuple(round(value, 3) for value in medians)}. "
            "Sensor, besleme veya olcek ayari kontrol edilmeli."
        )
    if io_errors or invalid_samples:
        print(
            f"Not: {io_errors} I2C hatasi ve {invalid_samples} gecersiz "
            "ornek atlandi.",
            flush=True,
        )
    return medians  # type: ignore[return-value]


def preflight_sensor(
    name: str, fd: int, config: dict[str, float | int]
) -> None:
    """Kalibrasyon sorularindan once sensorun gercek ivme verdigini dogrula."""
    values: list[float] = []
    for _ in range(20):
        try:
            accel, _gyro = read_motion(
                fd, float(config["accel_scale"]), float(config["gyro_scale"])
            )
        except OSError as exc:
            raise RuntimeError(
                f"{name} I2C on kontrolden gecemedi: {exc}. "
                "Kablo ve bus adresini kontrol edin."
            ) from exc
        values.append(math.sqrt(sum(float(value) ** 2 for value in accel)))
        time.sleep(0.02)
    median_norm = statistics.median(values)
    if median_norm < 2.0:
        raise RuntimeError(
            f"{name} ivme verisi sifir: |a|={median_norm:.2f} m/s2. "
            "Kalibrasyona baslanmadi."
        )
    print(f"{name} on kontrol OK: |a|={median_norm:.2f} m/s2")


def calibrate_sensor(
    name: str,
    fd: int,
    config: dict[str, float | int],
    seconds: float,
) -> dict[str, object]:
    readings: dict[str, tuple[float, float, float]] = {}
    print(f"\n{name} ALTI-YUZ KALIBRASYONU")
    print("Her konumda karti sert bir yuzeye koyun ve tamamen sabit tutun.")
    for key, description in POSES:
        input(f"{description}. Hazirsa Enter: ")
        print(f"{seconds:.1f} saniye olculuyor...", flush=True)
        readings[key] = capture_pose(fd, config, seconds)
        print("medyan:", tuple(round(value, 3) for value in readings[key]))

    offsets: list[float] = []
    scales: list[float] = []
    spans: list[float] = []
    for axis, label in enumerate(("x", "y", "z")):
        positive = readings[f"{label}_pos"][axis]
        negative = readings[f"{label}_neg"][axis]
        if positive <= negative:
            raise RuntimeError(
                f"{name} {label.upper()} pozlari ters veya hatali: "
                f"pozitif={positive:.2f}, negatif={negative:.2f} m/s2."
            )
        span = abs(positive - negative)
        if span < 8.0:
            raise RuntimeError(
                f"{name} {label.upper()} ekseni araligi yalniz {span:.2f} m/s2. "
                "Beklenen yaklasik 19.61 m/s2'dir. Bu kadar dusuk aralik "
                "gurultuyu fazla buyutur; pozu, baglantiyi ve sensoru kontrol edin."
            )
        offsets.append((positive + negative) / 2.0)
        scales.append((2.0 * GRAVITY) / span)
        spans.append(span)
    result = {
        "accel_offset_m_s2": offsets,
        "accel_scale": scales,
        "axis_span_m_s2": spans,
        "pose_medians_m_s2": {key: list(value) for key, value in readings.items()},
    }
    print(
        "Sonuc: offset=",
        tuple(round(value, 4) for value in offsets),
        "scale=",
        tuple(round(value, 4) for value in scales),
        "span=",
        tuple(round(value, 3) for value in spans),
    )
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Iki HW-290 alti-yuz ivme kalibrasyonu")
    parser.add_argument("--upper-bus", default="/dev/i2c-1")
    parser.add_argument("--forearm-bus", default="/dev/i2c-8")
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument(
        "--sensor",
        choices=("both", "upper", "forearm"),
        default="both",
        help="Kalibre edilecek kart: both, upper veya forearm",
    )
    parser.add_argument(
        "--output", type=Path, default=Path.home() / "hw290_accel_calibration.json"
    )
    args = parser.parse_args()
    if not 1.0 <= args.seconds <= 10.0:
        parser.error("--seconds 1 ile 10 arasinda olmali")
    return args


def partial_path(output: Path) -> Path:
    return output.with_name(f"{output.stem}.partial{output.suffix}")


def load_previous(output: Path) -> dict[str, object]:
    """Tam veya yarim kalmis onceki sonucu yukle."""
    # Yeni yarim sonuc varsa eski tamamlanmis dosyadan once onu kullan.
    for path in (partial_path(output), output):
        if not path.exists():
            continue
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"Kalibrasyon dosyasi okunamadi: {path}: {exc}") from exc
        if payload.get("version") != 1:
            raise RuntimeError(f"Desteklenmeyen kalibrasyon surumu: {path}")
        print(f"Onceki sonuc yuklendi: {path}")
        return payload
    return {"version": 1}


def write_json_atomic(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def save_progress(args: argparse.Namespace, payload: dict[str, object]) -> None:
    payload.update(
        {
            "version": 1,
            "created_utc_ns": time.time_ns(),
            "upper_bus": args.upper_bus,
            "forearm_bus": args.forearm_bus,
        }
    )
    progress = partial_path(args.output)
    write_json_atomic(progress, payload)
    print(f"Bu kartin sonucu gecici olarak kaydedildi: {progress}")


def main() -> int:
    args = parse_args()
    payload = load_previous(args.output)

    if args.sensor in ("both", "upper"):
        with open(args.upper_bus, "rb+", buffering=0) as upper_bus:
            upper_config = configure_mpu6050(
                upper_bus.fileno(), gyro_range_dps=500
            )
            preflight_sensor(
                f"HW290-1 {args.upper_bus}", upper_bus.fileno(), upper_config
            )
            payload["upper"] = calibrate_sensor(
                "HW290-1 UST KOL", upper_bus.fileno(), upper_config, args.seconds
            )
        save_progress(args, payload)

    if args.sensor in ("both", "forearm"):
        with open(args.forearm_bus, "rb+", buffering=0) as forearm_bus:
            forearm_config = configure_mpu6050(
                forearm_bus.fileno(), gyro_range_dps=500
            )
            preflight_sensor(
                f"HW290-2 {args.forearm_bus}",
                forearm_bus.fileno(),
                forearm_config,
            )
            payload["forearm"] = calibrate_sensor(
                "HW290-2 ON KOL",
                forearm_bus.fileno(),
                forearm_config,
                args.seconds,
            )
        save_progress(args, payload)

    if not isinstance(payload.get("upper"), dict) or not isinstance(
        payload.get("forearm"), dict
    ):
        missing = []
        if not isinstance(payload.get("upper"), dict):
            missing.append("upper")
        if not isinstance(payload.get("forearm"), dict):
            missing.append("forearm")
        print(
            "Kalibrasyon henuz tamamlanmadi. Eksik kart: " + ", ".join(missing)
        )
        print("Dirsek programinda kullanmadan once iki karti da tamamlayin.")
        return 0

    write_json_atomic(args.output, payload)
    progress = partial_path(args.output)
    if progress.exists():
        progress.unlink()
    print(f"\nKalibrasyon kaydedildi: {args.output}")
    print("Dirsek kodunu --accel-calibration ile bu dosyaya baglayin.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

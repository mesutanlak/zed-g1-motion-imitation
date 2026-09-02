#!/usr/bin/env python3
"""Validate the runtime contract of a four-ZED BODY_38 extrinsic file."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np


ZED360_METHOD = "zed_sdk_read_fusion_configuration_file_from_zed360"
BODY38_METHOD = "trimmed_kabsch_from_synchronized_body38_core_joints"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Dort-ZED BODY_38 runtime extrinsic kalite kapisi"
    )
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--expected-serials", required=True)
    parser.add_argument("--reference-serial", type=int, required=True)
    return parser.parse_args()


def finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} gercek bir JSON sayisi olmali.")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} sonlu olmali.")
    return result


def validate_document(
    document: dict[str, Any],
    *,
    expected_serials: set[int],
    reference_serial: int,
) -> None:
    if document.get("schema") != "zed_body38_distributed_extrinsics/v1":
        raise ValueError("Yanlis kalibrasyon semasi.")
    if document.get("coordinate_system") != "RIGHT_HANDED_Z_UP_X_FWD":
        raise ValueError("Koordinat sistemi RIGHT_HANDED_Z_UP_X_FWD olmali.")
    if document.get("units") != "meter":
        raise ValueError("Kalibrasyon birimi meter olmali.")
    try:
        declared_reference = int(document["reference_world_serial"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("reference_world_serial gecersiz.") from exc
    if declared_reference != reference_serial:
        raise ValueError(
            f"Referans kamera uyusmuyor: beklenen={reference_serial}, "
            f"dosya={declared_reference}."
        )

    cameras = document.get("cameras")
    if not isinstance(cameras, dict):
        raise ValueError("cameras nesnesi eksik.")
    try:
        actual_serials = {int(value) for value in cameras}
    except (TypeError, ValueError) as exc:
        raise ValueError("cameras seri anahtarlari sayisal olmali.") from exc
    if actual_serials != expected_serials:
        raise ValueError(
            f"Kalibrasyon seri seti yanlis: beklenen={sorted(expected_serials)}, "
            f"dosya={sorted(actual_serials)}."
        )

    method = str(document.get("method", ""))
    if method not in {ZED360_METHOD, BODY38_METHOD}:
        raise ValueError(f"Desteklenmeyen kalibrasyon methodu: {method or 'MISSING'}")

    for serial in sorted(expected_serials):
        camera = cameras.get(str(serial))
        if not isinstance(camera, dict):
            raise ValueError(f"ZED {serial} kalibrasyon nesnesi eksik.")
        try:
            rotation = np.asarray(camera["rotation_camera_to_world"], dtype=np.float64)
            translation = np.asarray(
                camera["translation_camera_to_world_m"], dtype=np.float64
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"ZED {serial} rotation/translation sayisal degil.") from exc
        if (
            rotation.shape != (3, 3)
            or translation.shape != (3,)
            or not np.isfinite(rotation).all()
            or not np.isfinite(translation).all()
        ):
            raise ValueError(f"ZED {serial} rotation 3x3 ve translation 3 olmali.")
        if (
            not np.isclose(np.linalg.det(rotation), 1.0, atol=0.03)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=0.03)
        ):
            raise ValueError(f"ZED {serial} rotation proper/ortonormal degil.")

        if serial == reference_serial:
            if not (
                np.allclose(rotation, np.eye(3), atol=0.03)
                and np.allclose(translation, np.zeros(3), atol=0.03)
            ):
                raise ValueError(
                    "Referans ZED pozu identity degil; pozlar referans kamera "
                    "frame'ine yeniden tabanlanmamis."
                )
            continue

        fit = camera.get("fit")
        if not isinstance(fit, dict) or fit.get("quality_gate_passed") is not True:
            raise ValueError(f"ZED {serial} kalite kapisini gecmemis.")
        if method == ZED360_METHOD:
            if fit.get("source") != "zed360_sdk_configuration":
                raise ValueError(f"ZED {serial} ZED360 kaynak etiketi gecersiz.")
            baseline_m = float(np.linalg.norm(translation))
            if not 0.05 <= baseline_m <= 20.0:
                raise ValueError(
                    f"ZED {serial} ZED360 referans baseline'i gecersiz: "
                    f"{baseline_m:.3f}m."
                )
            continue

        capture_samples = finite_number(
            fit.get("capture_samples"), f"ZED {serial} capture_samples"
        )
        paired = finite_number(
            fit.get("paired_keypoints"), f"ZED {serial} paired_keypoints"
        )
        candidates = finite_number(
            fit.get("candidate_keypoints"), f"ZED {serial} candidate_keypoints"
        )
        inlier_ratio = finite_number(
            fit.get("inlier_ratio"), f"ZED {serial} inlier_ratio"
        )
        rms_m = finite_number(fit.get("rms_m"), f"ZED {serial} rms_m")
        pelvis_p95_m = finite_number(
            fit.get("pelvis_p95_m"), f"ZED {serial} pelvis_p95_m"
        )
        trajectory_fields = (
            "pelvis_reference_motion_m",
            "pelvis_motion_scale_ratio",
            "pelvis_trajectory_correlation",
        )
        if any(name not in fit for name in trajectory_fields):
            raise ValueError(
                f"ZED {serial} kalibrasyonu eski: ayni-operator pelvis hareket "
                "dogrulamasi yok. Yeni calibrate_distributed_body38.py ile "
                "yeniden kalibre edin."
            )
        reference_motion_m = finite_number(
            fit.get("pelvis_reference_motion_m"),
            f"ZED {serial} pelvis_reference_motion_m",
        )
        motion_ratio = finite_number(
            fit.get("pelvis_motion_scale_ratio"),
            f"ZED {serial} pelvis_motion_scale_ratio",
        )
        trajectory_correlation = finite_number(
            fit.get("pelvis_trajectory_correlation"),
            f"ZED {serial} pelvis_trajectory_correlation",
        )
        if candidates <= 0.0 or abs(inlier_ratio - paired / candidates) > 0.01:
            raise ValueError(f"ZED {serial} inlier orani tutarsiz.")
        if capture_samples < 60 or paired < 300 or inlier_ratio < 0.25:
            raise ValueError(
                f"ZED {serial} ornek/inlier yetersiz: kare={capture_samples:g}, "
                f"nokta={paired:g}, oran={inlier_ratio:.3f}."
            )
        if not 0.0 <= rms_m <= 0.16:
            raise ValueError(f"ZED {serial} RMS={rms_m:.3f}m > 0.16m.")
        if not 0.0 <= pelvis_p95_m <= 0.25:
            raise ValueError(
                f"ZED {serial} pelvis p95={pelvis_p95_m:.3f}m > 0.25m."
            )
        if reference_motion_m < 0.10:
            raise ValueError(
                f"ZED {serial} referans pelvis hareketi "
                f"{reference_motion_m:.3f}m < 0.10m; kalibrasyonda ortak "
                "hacimde daha fazla yuruyun."
            )
        if not 0.60 <= motion_ratio <= 1.0:
            raise ValueError(
                f"ZED {serial} pelvis hareket orani={motion_ratio:.3f} < 0.60; "
                "kamera muhtemelen farkli/sabit bir kisiye kilitlendi."
            )
        if not 0.75 <= trajectory_correlation <= 1.0:
            raise ValueError(
                f"ZED {serial} pelvis yol korelasyonu="
                f"{trajectory_correlation:.3f} < 0.75; dort kamera ayni "
                "operatoru takip etmemis."
            )


def main() -> int:
    args = parse_args()
    path = args.input.expanduser().resolve()
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        expected = {
            int(value.strip())
            for value in args.expected_serials.split(",")
            if value.strip()
        }
        if len(expected) != 4:
            raise ValueError("Tam dort benzersiz beklenen seri gerekli.")
        validate_document(
            document,
            expected_serials=expected,
            reference_serial=args.reference_serial,
        )
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        print(f"HATA: 4-ZED kalibrasyon gecersiz: {exc}", file=sys.stderr)
        return 3
    print(
        f"4-ZED KALIBRASYON GECERLI | method={document['method']} "
        f"| reference={document['reference_world_serial']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

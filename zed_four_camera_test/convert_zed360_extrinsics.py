#!/usr/bin/env python3
"""Convert a saved ZED360 configuration to the project's BODY_38 world frame.

The ZED SDK parser performs the coordinate-system conversion.  Reading the
JSON through ``sl.read_fusion_configuration_file`` is safer than interpreting
ZED360's serialized rotation convention by hand.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np
import pyzed.sl as sl

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from zed_four_camera_test.calibrate_distributed_body38 import write_world_poses_jsonl
from zed_four_camera_test.fusion_config_io import read_fusion_configuration_file


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ZED360 Fusion JSON pozlarini BODY_38 kamera-dunya extrinsiclerine cevirir"
    )
    parser.add_argument("--input", type=Path, required=True, help="ZED360'da Finish Calibration ile kaydedilen JSON")
    parser.add_argument("--output", type=Path, required=True, help="zed_body38_distributed_extrinsics/v1 JSON")
    parser.add_argument("--world-poses-jsonl", type=Path, required=True)
    parser.add_argument("--reference-serial", type=int, required=True)
    return parser.parse_args()


def convert_configuration(path: Path, *, reference_serial: int) -> tuple[dict[str, Any], dict[int, int]]:
    configurations = read_fusion_configuration_file(
        path,
        sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD,
        sl.UNIT.METER,
    )
    if len(configurations) != 4:
        raise ValueError(f"ZED360 dosyasi tam 4 kamera icermeli; bulunan={len(configurations)}")
    serials = [int(item.serial_number) for item in configurations]
    if len(set(serials)) != 4 or reference_serial not in serials:
        raise ValueError("Dort benzersiz seri ve listede bulunan reference-serial gerekli.")

    cameras: dict[str, Any] = {}
    source_ports: dict[int, int] = {}
    for item in configurations:
        serial = int(item.serial_number)
        transform = np.asarray(item.pose.m, dtype=np.float64)
        if transform.shape != (4, 4) or not np.isfinite(transform).all():
            raise ValueError(f"ZED {serial} pozu gecersiz.")
        rotation = transform[:3, :3]
        translation = transform[:3, 3]
        if not np.isclose(np.linalg.det(rotation), 1.0, atol=0.03):
            raise ValueError(f"ZED {serial} rotation matrisi proper rotation degil.")
        cameras[str(serial)] = {
            "rotation_camera_to_world": rotation.tolist(),
            "translation_camera_to_world_m": translation.tolist(),
            "fit": {
                "source": "zed360_sdk_configuration",
                "reference": serial == reference_serial,
            },
        }
        source_ports[serial] = int(item.communication_parameters.port)

    origins = np.asarray([
        cameras[str(serial)]["translation_camera_to_world_m"] for serial in serials
    ], dtype=np.float64)
    maximum_baseline = max(
        float(np.linalg.norm(origins[first] - origins[second]))
        for first in range(len(origins))
        for second in range(first + 1, len(origins))
    )
    if maximum_baseline < 0.05:
        raise ValueError(
            "Tum kameralar ayni dunya noktasinda. Bu bir kalibrasyon seed dosyasi; "
            "ZED360 Finish Calibration ile kaydedilen sonucu secin."
        )

    document = {
        "schema": "zed_body38_distributed_extrinsics/v1",
        "created_unix_ns": time.time_ns(),
        "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
        "units": "meter",
        "reference_world_serial": reference_serial,
        "source_capture": str(path),
        "method": "zed_sdk_read_fusion_configuration_file_from_zed360",
        "cameras": cameras,
        "warning": "Tripodlar veya kameralar hareket ederse bu dosya gecersizdir; yeniden kalibre edin.",
    }
    return document, source_ports


def main() -> int:
    args = parse_args()
    source = args.input.expanduser().resolve()
    if not source.is_file():
        print(f"HATA: ZED360 JSON bulunamadi: {source}", file=sys.stderr)
        return 2
    try:
        document, source_ports = convert_configuration(
            source,
            reference_serial=args.reference_serial,
        )
    except (RuntimeError, ValueError) as exc:
        print(f"HATA: {exc}", file=sys.stderr)
        return 3

    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    poses = args.world_poses_jsonl.expanduser().resolve()
    write_world_poses_jsonl(poses, output=document, source_ports=source_ports)
    print(f"BODY_38 EXTRINSIC HAZIR: {output}")
    print(f"DUNYA POZLARI JSONL HAZIR: {poses}")
    for serial_text, camera in document["cameras"].items():
        print(f"ZED {serial_text} | origin_world_m={camera['translation_camera_to_world_m']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

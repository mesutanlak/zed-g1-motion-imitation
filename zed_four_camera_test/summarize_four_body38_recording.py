#!/usr/bin/env python3
"""Print acceptance metrics for a recorded distributed four-ZED session."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import statistics
from typing import Any


def finite_number(value: Any) -> float | None:
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except (TypeError, ValueError):
        return None


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def format_metric(value: float | None, suffix: str = "") -> str:
    return "n/a" if value is None else f"{value:.2f}{suffix}"


def main() -> int:
    parser = argparse.ArgumentParser(description="4-ZED BODY_38 fusion JSONL kabul ozeti")
    parser.add_argument("--input", type=Path, required=True)
    args = parser.parse_args()
    path = args.input.expanduser().resolve()

    packets: list[dict[str, Any]] = []
    metadata: dict[str, Any] | None = None
    invalid = 0
    with path.open("r", encoding="utf-8") as source:
        for line in source:
            if not line.strip():
                continue
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                invalid += 1
                continue
            schema = str(item.get("schema", ""))
            if schema.endswith("/metadata/v1"):
                metadata = item
            elif schema == "zed_body38_live/v1":
                packets.append(item)

    if not packets:
        print(f"HATA: fusion paketi bulunamadi: {path}")
        return 2

    timestamps = [int(packet.get("timestamp_ns", 0) or 0) for packet in packets]
    timestamps = [value for value in timestamps if value > 0]
    duration_s = (max(timestamps) - min(timestamps)) / 1.0e9 if len(timestamps) >= 2 else 0.0
    measured_hz = (len(timestamps) - 1) / duration_s if duration_s > 0.0 else 0.0
    view_counts: Counter[int] = Counter()
    serial_contributions: Counter[int] = Counter()
    source_fps: dict[int, list[float]] = defaultdict(list)
    sync_ms: list[float] = []
    mpjpe_m: list[float] = []
    p95_error_m: list[float] = []
    capture_send_ms: list[float] = []
    record_drops: list[float] = []

    for packet in packets:
        multi = packet.get("multi_camera") or {}
        serials = [int(value) for value in multi.get("contributing_serials") or []]
        view_counts[len(serials)] += 1
        serial_contributions.update(serials)
        value = finite_number(multi.get("camera_timestamp_delta_ms"))
        if value is not None:
            sync_ms.append(value)
        agreement = multi.get("cross_view_agreement") or {}
        value = finite_number(agreement.get("mpjpe_m"))
        if value is not None:
            mpjpe_m.append(value)
        value = finite_number(agreement.get("p95_error_m"))
        if value is not None:
            p95_error_m.append(value)
        for serial_text, metric in ((multi.get("fusion_metrics") or {}).get("per_camera") or {}).items():
            value = finite_number((metric or {}).get("body_fps"))
            if value is not None:
                source_fps[int(serial_text)].append(value)
        transport = packet.get("transport_metrics") or {}
        value = finite_number(transport.get("capture_to_send_ms"))
        if value is not None:
            capture_send_ms.append(value)
        value = finite_number(transport.get("record_dropped"))
        if value is not None:
            record_drops.append(value)

    total = len(packets)
    configured = sorted(
        int(value)
        for value in (
            ((packets[-1].get("multi_camera") or {}).get("configured_serials"))
            or (metadata or {}).get("serial_numbers")
            or serial_contributions
        )
    )
    print(f"DOSYA: {path}")
    print(f"FUSION: kare={total} sure={duration_s:.2f}s gercek_hiz={measured_hz:.2f} fps bozuk_json={invalid}")
    for views in (4, 3, 2):
        count = view_counts[views]
        print(f"KATKI {views}/4: {count} kare ({100.0 * count / total:.1f}%)")
    for serial in configured:
        values = source_fps.get(serial, [])
        mean_fps = statistics.fmean(values) if values else None
        print(
            f"ZED {serial}: katki={serial_contributions[serial]}/{total} "
            f"({100.0 * serial_contributions[serial] / total:.1f}%) "
            f"ortalama_BODY_fps={format_metric(mean_fps)}"
        )
    print(
        "SENKRON: "
        f"p50={format_metric(percentile(sync_ms, 0.50), 'ms')} "
        f"p95={format_metric(percentile(sync_ms, 0.95), 'ms')}"
    )
    print(
        "CROSS_VIEW: "
        f"MPJPE_p50={format_metric(percentile(mpjpe_m, 0.50), 'm')} "
        f"MPJPE_p95={format_metric(percentile(mpjpe_m, 0.95), 'm')} "
        f"joint_p95_p50={format_metric(percentile(p95_error_m, 0.50), 'm')}"
    )
    print(
        "AKTARIM: "
        f"capture_to_send_p50={format_metric(percentile(capture_send_ms, 0.50), 'ms')} "
        f"p95={format_metric(percentile(capture_send_ms, 0.95), 'ms')} "
        f"record_drop_max={int(max(record_drops, default=0.0))}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

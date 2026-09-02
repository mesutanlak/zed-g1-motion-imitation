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
    intervals_s = [
        (current - previous) / 1.0e9
        for previous, current in zip(timestamps, timestamps[1:])
        if current > previous
    ]
    active_intervals = [value for value in intervals_s if value < 1.0]
    active_duration_s = sum(active_intervals)
    active_hz = len(active_intervals) / active_duration_s if active_duration_s > 0.0 else 0.0
    segments = 1 + sum(value >= 1.0 for value in intervals_s)
    view_counts: Counter[int] = Counter()
    serial_contributions: Counter[int] = Counter()
    source_fps: dict[int, list[float]] = defaultdict(list)
    source_latency: dict[int, list[float]] = defaultdict(list)
    source_raw_clock_mixed: dict[int, list[float]] = defaultdict(list)
    source_clock_offset: dict[int, list[float]] = defaultdict(list)
    source_network_queue: dict[int, list[float]] = defaultdict(list)
    source_prediction: dict[int, list[float]] = defaultdict(list)
    sync_ms: list[float] = []
    selection_sync_ms: list[float] = []
    mpjpe_m: list[float] = []
    p95_error_m: list[float] = []
    aligned_mpjpe_m: list[float] = []
    capture_send_ms: list[float] = []
    record_drops: list[float] = []
    left_clear_views: list[float] = []
    right_clear_views: list[float] = []
    workspace_excluded_frames = 0
    workspace_excluded_serials: Counter[int] = Counter()

    for packet in packets:
        multi = packet.get("multi_camera") or {}
        serials = [int(value) for value in multi.get("contributing_serials") or []]
        view_counts[len(serials)] += 1
        serial_contributions.update(serials)
        value = finite_number(multi.get("camera_timestamp_delta_ms"))
        if value is not None:
            sync_ms.append(value)
        fusion_detail = packet.get("fusion") or {}
        value = finite_number(fusion_detail.get("selection_capture_spread_ms"))
        if value is not None:
            selection_sync_ms.append(value)
        excluded_workspace = [
            int(value)
            for value in multi.get("workspace_excluded_serials") or []
        ]
        if excluded_workspace:
            workspace_excluded_frames += 1
            workspace_excluded_serials.update(excluded_workspace)
        agreement = multi.get("cross_view_agreement") or {}
        value = finite_number(agreement.get("mpjpe_m"))
        if value is not None:
            mpjpe_m.append(value)
        value = finite_number(agreement.get("p95_error_m"))
        if value is not None:
            p95_error_m.append(value)
        aligned_agreement = multi.get("post_alignment_agreement") or {}
        value = finite_number(aligned_agreement.get("mpjpe_m"))
        if value is not None:
            aligned_mpjpe_m.append(value)
        for serial_text, metric in ((multi.get("fusion_metrics") or {}).get("per_camera") or {}).items():
            metric = metric or {}
            serial = int(serial_text)
            value = finite_number(metric.get("body_fps"))
            if value is not None:
                source_fps[serial].append(value)
            for field, target in (
                ("corrected_capture_to_receive_ms", source_latency),
                ("raw_clock_mixed_capture_to_receive_ms", source_raw_clock_mixed),
                ("clock_offset_estimate_ms", source_clock_offset),
                ("network_queue_ms", source_network_queue),
                ("temporal_prediction_ms", source_prediction),
            ):
                value = finite_number(metric.get(field))
                if value is not None:
                    target[serial].append(value)
        arm_evidence = multi.get("arm_evidence") or {}
        for side, target in (("left", left_clear_views), ("right", right_clear_views)):
            value = finite_number((arm_evidence.get(side) or {}).get("reliable_clear_views"))
            if value is not None:
                target.append(value)
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
    if metadata:
        print(f"KALIBRASYON: {metadata.get('extrinsics_path') or 'metadata yok'}")
    print(
        f"FUSION: kare={total} duvar_suresi={duration_s:.2f}s "
        f"duvar_hizi={measured_hz:.2f} fps aktif_sure={active_duration_s:.2f}s "
        f"aktif_hiz={active_hz:.2f} fps kesintisiz_parca={segments} bozuk_json={invalid}"
    )
    for views in (4, 3, 2):
        count = view_counts[views]
        print(f"KATKI {views}/4: {count} kare ({100.0 * count / total:.1f}%)")
    for serial in configured:
        values = source_fps.get(serial, [])
        mean_fps = statistics.fmean(values) if values else None
        print(
            f"ZED {serial}: katki={serial_contributions[serial]}/{total} "
            f"({100.0 * serial_contributions[serial] / total:.1f}%) "
            f"ortalama_BODY_fps={format_metric(mean_fps)} "
            f"duzeltilmis_gecikme_p50={format_metric(percentile(source_latency[serial], 0.50), 'ms')} "
            f"ag_kuyruk_p50={format_metric(percentile(source_network_queue[serial], 0.50), 'ms')} "
            f"clock_offset_p50={format_metric(percentile(source_clock_offset[serial], 0.50), 'ms')} "
            f"tahmin_p50={format_metric(percentile(source_prediction[serial], 0.50), 'ms')}"
        )
    print(
        "SENKRON: "
        f"katkici_p50={format_metric(percentile(sync_ms, 0.50), 'ms')} "
        f"katkici_p95={format_metric(percentile(sync_ms, 0.95), 'ms')} "
        f"secim_p95={format_metric(percentile(selection_sync_ms, 0.95), 'ms')}"
    )
    print(
        "CALISMA_ALANI: "
        f"dislanan_kare={workspace_excluded_frames}/{total} "
        f"({100.0 * workspace_excluded_frames / total:.1f}%) "
        f"kamera_sayaci={dict(sorted(workspace_excluded_serials.items())) or '{}'}"
    )
    print(
        "CROSS_VIEW: "
        f"MPJPE_p50={format_metric(percentile(mpjpe_m, 0.50), 'm')} "
        f"MPJPE_p95={format_metric(percentile(mpjpe_m, 0.95), 'm')} "
        f"joint_p95_p50={format_metric(percentile(p95_error_m, 0.50), 'm')}"
    )
    if aligned_mpjpe_m:
        print(
            "HIZALANMIS_FUSION: "
            f"MPJPE_p50={format_metric(percentile(aligned_mpjpe_m, 0.50), 'm')} "
            f"MPJPE_p95={format_metric(percentile(aligned_mpjpe_m, 0.95), 'm')}"
        )
    print(
        "AKTARIM: "
        f"capture_to_send_p50={format_metric(percentile(capture_send_ms, 0.50), 'ms')} "
        f"p95={format_metric(percentile(capture_send_ms, 0.95), 'ms')} "
        f"record_drop_max={int(max(record_drops, default=0.0))}"
    )
    if left_clear_views or right_clear_views:
        print(
            "KOL_GORUSU: "
            f"sol_temiz_kamera_p50={format_metric(percentile(left_clear_views, 0.50))} "
            f"sag_temiz_kamera_p50={format_metric(percentile(right_clear_views, 0.50))}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Application-level multi-host BODY_38 fusion for the G1 perception path.

This is intentionally *not* a wrapper around ``sl.Fusion`` or ZED360.  It is
the fallback transport for the two-host rig when the SDK Network Fusion path
rejects otherwise valid BODY_38 data.  Each physical ZED is opened by
``zed_g1_skeleton.py`` and sends its normal ``zed_body38_live/v1`` UDP packet
to this process.  After an explicit static extrinsic calibration, this process
transforms and confidence-fuses keypoints, then emits the same downstream
``zed_body38_live/v1`` contract.  It never sends robot commands.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import selectors
import socket
import sys
import time
from typing import Any

import numpy as np


BODY38_NAMES = (
    "PELVIS", "SPINE_1", "SPINE_2", "SPINE_3", "NECK", "NOSE",
    "LEFT_EYE", "RIGHT_EYE", "LEFT_EAR", "RIGHT_EAR",
    "LEFT_CLAVICLE", "RIGHT_CLAVICLE", "LEFT_SHOULDER", "RIGHT_SHOULDER",
    "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST",
    "LEFT_HIP", "RIGHT_HIP", "LEFT_KNEE", "RIGHT_KNEE", "LEFT_ANKLE",
    "RIGHT_ANKLE", "LEFT_BIG_TOE", "RIGHT_BIG_TOE", "LEFT_SMALL_TOE",
    "RIGHT_SMALL_TOE", "LEFT_HEEL", "RIGHT_HEEL", "LEFT_HAND_THUMB_4",
    "RIGHT_HAND_THUMB_4", "LEFT_HAND_INDEX_1", "RIGHT_HAND_INDEX_1",
    "LEFT_HAND_MIDDLE_4", "RIGHT_HAND_MIDDLE_4", "LEFT_HAND_PINKY_1",
    "RIGHT_HAND_PINKY_1",
)
IDX = {name: index for index, name in enumerate(BODY38_NAMES)}


@dataclass(frozen=True)
class InputEndpoint:
    serial: int
    port: int


@dataclass
class Sample:
    serial: int
    packet: dict[str, Any]
    received_ns: int
    sequence: int


@dataclass(frozen=True)
class Extrinsic:
    rotation: np.ndarray
    translation: np.ndarray


def sanitize(value: Any) -> Any:
    """Make NumPy and non-finite values valid JSON without changing packets."""
    if isinstance(value, np.ndarray):
        return sanitize(value.tolist())
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, dict):
        return {str(key): sanitize(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [sanitize(item) for item in value]
    return value


def parse_endpoint(value: str) -> InputEndpoint:
    try:
        serial_text, port_text = value.strip().split(":", 1)
        serial, port = int(serial_text), int(port_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Kaynak SERIAL:PORT biciminde olmali.") from exc
    if serial <= 0 or not (1024 <= port <= 65535):
        raise argparse.ArgumentTypeError("Gecerli SERIAL ve 1024-65535 UDP portu verin.")
    return InputEndpoint(serial, port)


def vector(value: Any, length: int) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
        if result.shape == (length,):
            return result
    except (TypeError, ValueError):
        pass
    return np.full(length, np.nan, dtype=np.float64)


def points(value: Any) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
        if result.shape == (38, 3):
            return result
    except (TypeError, ValueError):
        pass
    return np.full((38, 3), np.nan, dtype=np.float64)


def confidence(value: Any) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
        if result.shape == (38,):
            return result
    except (TypeError, ValueError):
        pass
    return np.zeros(38, dtype=np.float64)


def quaternion_xyzw_to_matrix(value: Any) -> np.ndarray:
    x, y, z, w = vector(value, 4)
    norm = float(math.sqrt(x * x + y * y + z * z + w * w))
    if not math.isfinite(norm) or norm < 1.0e-8:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def matrix_to_quaternion_xyzw(matrix: np.ndarray) -> np.ndarray:
    value = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(value))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quat = np.array([
            (value[2, 1] - value[1, 2]) / scale,
            (value[0, 2] - value[2, 0]) / scale,
            (value[1, 0] - value[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(value)))
        if index == 0:
            scale = math.sqrt(max(1.0e-12, 1.0 + value[0, 0] - value[1, 1] - value[2, 2])) * 2.0
            quat = np.array([0.25 * scale, (value[0, 1] + value[1, 0]) / scale,
                             (value[0, 2] + value[2, 0]) / scale, (value[2, 1] - value[1, 2]) / scale])
        elif index == 1:
            scale = math.sqrt(max(1.0e-12, 1.0 + value[1, 1] - value[0, 0] - value[2, 2])) * 2.0
            quat = np.array([(value[0, 1] + value[1, 0]) / scale, 0.25 * scale,
                             (value[1, 2] + value[2, 1]) / scale, (value[0, 2] - value[2, 0]) / scale])
        else:
            scale = math.sqrt(max(1.0e-12, 1.0 + value[2, 2] - value[0, 0] - value[1, 1])) * 2.0
            quat = np.array([(value[0, 2] + value[2, 0]) / scale, (value[1, 2] + value[2, 1]) / scale,
                             0.25 * scale, (value[1, 0] - value[0, 1]) / scale])
    return quat / max(float(np.linalg.norm(quat)), 1.0e-12)


def load_extrinsics(path: Path | None, endpoints: list[InputEndpoint]) -> dict[int, Extrinsic]:
    if path is None:
        return {}
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"Extrinsic dosyasi okunamadi: {path}: {exc}") from exc
    if document.get("schema") != "zed_body38_distributed_extrinsics/v1":
        raise ValueError("Extrinsic schema zed_body38_distributed_extrinsics/v1 olmali.")
    if document.get("coordinate_system") != "RIGHT_HANDED_Z_UP_X_FWD":
        raise ValueError("Extrinsic koordinat sistemi RIGHT_HANDED_Z_UP_X_FWD olmali.")
    cameras = document.get("cameras")
    if not isinstance(cameras, dict):
        raise ValueError("Extrinsic dosyasinda cameras nesnesi yok.")
    result: dict[int, Extrinsic] = {}
    for endpoint in endpoints:
        item = cameras.get(str(endpoint.serial))
        if not isinstance(item, dict):
            raise ValueError(f"Extrinsic dosyasinda ZED {endpoint.serial} yok.")
        rotation = np.asarray(item.get("rotation_camera_to_world"), dtype=np.float64)
        translation = np.asarray(item.get("translation_camera_to_world_m"), dtype=np.float64)
        if rotation.shape != (3, 3) or translation.shape != (3,) or not np.isfinite(rotation).all() or not np.isfinite(translation).all():
            raise ValueError(f"ZED {endpoint.serial} extrinsic matrisi gecersiz.")
        if not np.isclose(np.linalg.det(rotation), 1.0, atol=0.03):
            raise ValueError(f"ZED {endpoint.serial} rotation matrisi proper rotation degil.")
        result[endpoint.serial] = Extrinsic(rotation, translation)
    return result


def sample_quality(packet: dict[str, Any], minimum_confidence: float) -> float:
    xyz = points(packet.get("keypoints_3d_m"))
    conf = confidence(packet.get("keypoint_confidence"))
    visible = np.isfinite(xyz).all(axis=1) & (conf >= minimum_confidence)
    upper = [IDX[name] for name in ("SPINE_3", "NECK", "LEFT_SHOULDER", "RIGHT_SHOULDER", "LEFT_ELBOW", "RIGHT_ELBOW", "LEFT_WRIST", "RIGHT_WRIST")]
    upper_visible = float(np.mean(visible[upper]))
    overall_visible = float(np.mean(visible))
    mean_conf = float(np.mean(conf[visible])) / 100.0 if np.any(visible) else 0.0
    body_conf = float(packet.get("body_confidence", 0.0) or 0.0) / 100.0
    return float(np.clip(0.50 * upper_visible + 0.25 * overall_visible + 0.15 * mean_conf + 0.10 * body_conf, 0.0, 1.0))


def transformed_points(packet: dict[str, Any], extrinsic: Extrinsic) -> np.ndarray:
    value = points(packet.get("keypoints_3d_m"))
    valid = np.isfinite(value).all(axis=1)
    result = value.copy()
    result[valid] = value[valid] @ extrinsic.rotation.T + extrinsic.translation
    return result


def fuse_keypoints(
    views: list[tuple[Sample, Extrinsic]],
    *,
    minimum_confidence: float,
    maximum_spread_m: float,
) -> tuple[np.ndarray, np.ndarray, list[int]]:
    """Robustly average joints after static camera-to-world transforms.

    Per-joint medians remove a single bad/occluded view before confidence
    weighting.  A joint visible in only one calibrated camera is retained,
    which is important for wrists and the BODY_38 hand proxy joints.
    """
    result = np.full((38, 3), np.nan, dtype=np.float64)
    result_confidence = np.zeros(38, dtype=np.float64)
    contribution_count: list[int] = []
    prepared = []
    for sample, extrinsic in views:
        packet_conf = confidence(sample.packet.get("keypoint_confidence"))
        prepared.append((transformed_points(sample.packet, extrinsic), packet_conf, sample_quality(sample.packet, minimum_confidence)))
    for joint in range(38):
        candidates: list[tuple[np.ndarray, float, float]] = []
        for xyz, conf, quality in prepared:
            if np.isfinite(xyz[joint]).all() and conf[joint] >= minimum_confidence:
                candidates.append((xyz[joint], float(conf[joint]), quality))
        if not candidates:
            contribution_count.append(0)
            continue
        median = np.median(np.asarray([item[0] for item in candidates]), axis=0)
        inliers = [item for item in candidates if float(np.linalg.norm(item[0] - median)) <= maximum_spread_m]
        if not inliers:
            inliers = [max(candidates, key=lambda item: item[1] * item[2])]
        weights = np.asarray([max(1.0e-4, (item[1] / 100.0) ** 2 * max(item[2], 0.05)) for item in inliers])
        joint_values = np.asarray([item[0] for item in inliers])
        result[joint] = np.average(joint_values, axis=0, weights=weights)
        result_confidence[joint] = float(np.average(np.asarray([item[1] for item in inliers]), weights=weights))
        contribution_count.append(len(inliers))
    return result, result_confidence, contribution_count


def compact_sample(sample: Sample) -> dict[str, Any]:
    packet = sample.packet
    return sanitize({
        "serial": sample.serial,
        "source_timestamp_ns": int(packet.get("timestamp_ns", 0) or 0),
        "receiver_timestamp_ns": sample.received_ns,
        "sequence": sample.sequence,
        "body_confidence": packet.get("body_confidence", 0.0),
        "keypoint_names": packet.get("keypoint_names", BODY38_NAMES),
        "keypoints_3d_m": packet.get("keypoints_3d_m"),
        "keypoint_confidence": packet.get("keypoint_confidence"),
    })


def make_output_packet(
    views: list[tuple[Sample, Extrinsic]],
    *,
    minimum_confidence: float,
    maximum_spread_m: float,
    output_sequence: int,
) -> dict[str, Any]:
    best_sample, best_extrinsic = max(
        views,
        key=lambda item: sample_quality(item[0].packet, minimum_confidence),
    )
    packet = dict(best_sample.packet)
    fused, fused_confidence, contributions = fuse_keypoints(
        views,
        minimum_confidence=minimum_confidence,
        maximum_spread_m=maximum_spread_m,
    )
    pelvis = fused[IDX["PELVIS"]]
    source_orientation = quaternion_xyzw_to_matrix(packet.get("global_root_orientation_xyzw"))
    packet.update(sanitize({
        "schema": "zed_body38_live/v1",
        "source_serial": 0,
        "sequence": output_sequence,
        "timestamp_ns": time.time_ns(),
        "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
        "units": "meter",
        "root_position_m": pelvis,
        "global_root_orientation_xyzw": matrix_to_quaternion_xyzw(best_extrinsic.rotation @ source_orientation),
        "keypoint_names": BODY38_NAMES,
        "keypoints_3d_raw_m": fused,
        "keypoints_3d_m": fused,
        "keypoint_confidence": fused_confidence,
        "root_relative_keypoints_m": fused - pelvis if np.isfinite(pelvis).all() else np.full_like(fused, np.nan),
        "fusion": {
            "implementation": "application_level_weighted_body38/v1",
            "reference_world_serial": None,
            "contributing_serials": [sample.serial for sample, _ in views],
            "best_orientation_serial": best_sample.serial,
            "per_joint_contributions": contributions,
            "maximum_joint_spread_m": maximum_spread_m,
        },
        "latency_trace_ns": {
            "t0_capture_ns": int(best_sample.packet.get("timestamp_ns", 0) or 0),
            "t2_windows_udp_receive_ns": best_sample.received_ns,
            "t3_application_fused_ns": time.time_ns(),
        },
    }))
    return packet


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Iki-host, dort-ZED application BODY_38 fusion alicisi")
    parser.add_argument("--source", type=parse_endpoint, action="append", required=True, help="Her kamera icin SERIAL:UDP_PORT; dort kez verin.")
    parser.add_argument("--bind", default="0.0.0.0", help="Dinlenecek PC IPv4 adresi (varsayilan tum arayuzler).")
    parser.add_argument("--extrinsics", type=Path, default=None, help="Kalibre edilmis zed_body38_distributed_extrinsics/v1 JSON dosyasi.")
    parser.add_argument("--output-host", default="", help="Birlesik BODY_38 UDP hedefi; bos ise cikis yayini kapali.")
    parser.add_argument("--output-port", type=int, default=15050)
    parser.add_argument("--output-max-hz", type=float, default=15.0)
    parser.add_argument("--confidence", type=float, default=45.0)
    parser.add_argument("--max-sync-ms", type=float, default=110.0, help="Ana PC'ye varis zamanina gore azami dortlu paket yayilimi.")
    parser.add_argument("--max-joint-spread-m", type=float, default=0.30)
    parser.add_argument("--calibration-record", type=Path, default=None, help="Dortlu senkron ham BODY_38 karelerini JSONL olarak kaydet.")
    parser.add_argument("--calibration-max-hz", type=float, default=12.0)
    parser.add_argument("--duration", type=float, default=0.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    endpoints: list[InputEndpoint] = args.source
    if len(endpoints) != 4 or len({item.serial for item in endpoints}) != 4 or len({item.port for item in endpoints}) != 4:
        print("HATA: tam dort farkli SERIAL:PORT kaynagi gerekli.", file=sys.stderr)
        return 2
    if not 0.0 <= args.confidence <= 100.0 or args.max_sync_ms <= 0.0 or args.max_joint_spread_m <= 0.0:
        print("HATA: confidence, max-sync-ms ve max-joint-spread-m gecersiz.", file=sys.stderr)
        return 2
    try:
        extrinsics = load_extrinsics(args.extrinsics.resolve() if args.extrinsics else None, endpoints)
    except ValueError as exc:
        print(f"HATA: {exc}", file=sys.stderr)
        return 2
    if not extrinsics:
        print("UYARI: extrinsic yok. Kayit yapilabilir ancak guvenli olarak sadece en iyi tek kamera paketlenecek.")

    selector = selectors.DefaultSelector()
    sockets: list[socket.socket] = []
    port_to_serial = {endpoint.port: endpoint.serial for endpoint in endpoints}
    try:
        for endpoint in endpoints:
            receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            receiver.bind((args.bind, endpoint.port))
            receiver.setblocking(False)
            selector.register(receiver, selectors.EVENT_READ, data=endpoint)
            sockets.append(receiver)
            print(f"DINLE | ZED {endpoint.serial} | {args.bind}:{endpoint.port}")
    except OSError as exc:
        print(f"HATA: UDP portu acilamadi: {exc}", file=sys.stderr)
        for item in sockets:
            item.close()
        return 3

    output_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.output_host else None
    record_file = None
    if args.calibration_record:
        path = args.calibration_record.expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        record_file = path.open("w", encoding="utf-8", buffering=1)
        record_file.write(json.dumps(sanitize({
            "schema": "zed_body38_multihost_calibration_metadata/v1",
            "created_unix_ns": time.time_ns(),
            "coordinate_system": "RIGHT_HANDED_Z_UP_X_FWD",
            "units": "meter",
            "sources": [{"serial": item.serial, "port": item.port} for item in endpoints],
            "maximum_arrival_spread_ms": args.max_sync_ms,
            "instruction": "Tripodlar sabitken ortak gorus alaninda 20-30 saniye T-pozda sakin durun.",
        }), ensure_ascii=False, allow_nan=False) + "\n")
        print(f"HAM KALIBRASYON KAYDI: {path}")

    latest: dict[int, Sample] = {}
    last_bundle: tuple[tuple[int, int], ...] | None = None
    last_output_at = 0.0
    last_record_at = 0.0
    output_sequence = 0
    input_packets = 0
    invalid_packets = 0
    fused_packets = 0
    raw_records = 0
    started = time.monotonic()
    last_status = started
    try:
        while True:
            events = selector.select(timeout=0.20)
            for key, _ in events:
                endpoint: InputEndpoint = key.data
                while True:
                    try:
                        payload, _sender = key.fileobj.recvfrom(65535)
                    except BlockingIOError:
                        break
                    except OSError:
                        break
                    try:
                        document = json.loads(payload.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        invalid_packets += 1
                        continue
                    if not isinstance(document, dict) or document.get("schema") != "zed_body38_live/v1":
                        invalid_packets += 1
                        continue
                    declared_serial = int(document.get("source_serial", endpoint.serial) or endpoint.serial)
                    if declared_serial not in (0, endpoint.serial):
                        invalid_packets += 1
                        print(f"UYARI: port {endpoint.port} ZED {endpoint.serial} beklerken {declared_serial} paketi geldi; atlandi.", file=sys.stderr)
                        continue
                    if points(document.get("keypoints_3d_m")).shape != (38, 3):
                        invalid_packets += 1
                        continue
                    latest[endpoint.serial] = Sample(
                        serial=endpoint.serial,
                        packet=document,
                        received_ns=time.time_ns(),
                        sequence=int(document.get("sequence", 0) or 0),
                    )
                    input_packets += 1

            required = [item.serial for item in endpoints]
            if all(serial in latest for serial in required):
                samples = [latest[serial] for serial in required]
                arrivals = [sample.received_ns for sample in samples]
                arrival_spread_ms = (max(arrivals) - min(arrivals)) / 1.0e6
                marker = tuple(sorted((sample.serial, sample.sequence) for sample in samples))
                synchronized = arrival_spread_ms <= args.max_sync_ms
                is_new = marker != last_bundle
                now = time.monotonic()
                if synchronized and is_new:
                    last_bundle = marker
                    if record_file is not None and now - last_record_at >= 0.98 / args.calibration_max_hz:
                        record_file.write(json.dumps(sanitize({
                            "schema": "zed_body38_multihost_calibration_sample/v1",
                            "recorded_unix_ns": time.time_ns(),
                            "arrival_spread_ms": arrival_spread_ms,
                            "sources": {str(sample.serial): compact_sample(sample) for sample in samples},
                        }), ensure_ascii=False, allow_nan=False) + "\n")
                        raw_records += 1
                        last_record_at = now
                    if extrinsics and now - last_output_at >= 0.98 / args.output_max_hz:
                        views = [(sample, extrinsics[sample.serial]) for sample in samples]
                        packet = make_output_packet(
                            views,
                            minimum_confidence=args.confidence,
                            maximum_spread_m=args.max_joint_spread_m,
                            output_sequence=output_sequence,
                        )
                        packet["fusion"]["reference_world_serial"] = min(extrinsics)
                        encoded = json.dumps(packet, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
                        if len(encoded) <= 60000:
                            output_sequence += 1
                            fused_packets += 1
                            last_output_at = now
                            if output_socket is not None:
                                output_socket.sendto(encoded, (args.output_host, args.output_port))
                        else:
                            print(f"UYARI: birlesik UDP paketi cok buyuk ({len(encoded)} byte).", file=sys.stderr)

            now = time.monotonic()
            if now - last_status >= 1.0:
                online = ", ".join(str(serial) for serial in sorted(latest)) or "yok"
                print(
                    f"DURUM | kaynak={len(latest)}/4 [{online}] | input={input_packets} "
                    f"| ham_kayit={raw_records} | fusion_cikis={fused_packets} | gecersiz={invalid_packets}",
                    flush=True,
                )
                last_status = now
            if args.duration > 0.0 and now - started >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if record_file is not None:
            record_file.close()
        if output_socket is not None:
            output_socket.close()
        for item in sockets:
            try:
                selector.unregister(item)
            except Exception:
                pass
            item.close()
        selector.close()
    print(f"Bitti | input={input_packets} | ham_kayit={raw_records} | fusion_cikis={fused_packets}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Time alignment and guarded elbow fusion for external arm IMUs.

The ZED capture processes do not import this module.  A consumer can attach an
``arm_imu/v1`` stream at the retargeting boundary, which keeps camera capture
and multi-camera BODY_38 fusion independent from Raspberry Pi availability.

Angles use the human convention used by the BNO055 prototype: a straight arm
is zero degrees of flexion.  ZED's geometric interior elbow angle is converted
with ``flexion = 180 - interior`` before comparison.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Any, Iterable


def _finite(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _finite_tuple(value: Any, length: int) -> tuple[float, ...] | None:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        return None
    converted = tuple(_finite(part) for part in value)
    if any(part is None for part in converted):
        return None
    return tuple(float(part) for part in converted if part is not None)


@dataclass(frozen=True)
class ArmImuMeasurement:
    timestamp_ns: int
    sequence: int
    side: str
    flexion_deg: float
    velocity_deg_s: float | None
    relative_3d_deg: float | None
    off_axis_deg: float | None
    quality: float
    valid: bool
    upper_quaternion_xyzw: tuple[float, ...] | None = None
    forearm_quaternion_xyzw: tuple[float, ...] | None = None
    relative_orientation_xyzw: tuple[float, ...] | None = None
    upper_segment_axis_world_xyz: tuple[float, ...] | None = None
    forearm_segment_axis_world_xyz: tuple[float, ...] | None = None
    failure_codes: tuple[str, ...] = ()
    interpolated: bool = False


@dataclass(frozen=True)
class ElbowFusionResult:
    fused_flexion_deg: float | None
    zed_flexion_deg: float | None
    imu_flexion_deg: float | None
    disagreement_deg: float | None
    zed_weight: float
    imu_weight: float
    source: str
    usable: bool
    failure_codes: tuple[str, ...]

    def as_dict(self) -> dict[str, Any]:
        return {
            "fused_flexion_deg": self.fused_flexion_deg,
            "zed_flexion_deg": self.zed_flexion_deg,
            "imu_flexion_deg": self.imu_flexion_deg,
            "disagreement_deg": self.disagreement_deg,
            "zed_weight": self.zed_weight,
            "imu_weight": self.imu_weight,
            "source": self.source,
            "usable": self.usable,
            "failure_codes": list(self.failure_codes),
        }


def parse_arm_imu_packet(packet: dict[str, Any]) -> ArmImuMeasurement:
    """Validate the compact ``arm_imu/v1`` network contract."""
    if packet.get("schema") != "arm_imu/v1":
        raise ValueError("unsupported IMU schema")
    side = str(packet.get("side", "")).lower()
    if side not in {"left", "right"}:
        raise ValueError("IMU side must be left or right")
    timestamp_ns = int(packet.get("timestamp_ns") or 0)
    if timestamp_ns <= 0:
        raise ValueError("IMU timestamp_ns must be positive")
    elbow = packet.get("elbow") or {}
    flexion = _finite(elbow.get("flexion_deg"))
    if flexion is None:
        raise ValueError("IMU flexion_deg must be finite")
    failures = {str(item) for item in packet.get("failure_codes", [])}
    valid = bool(packet.get("valid", True))
    # Human elbow tolerance includes a small hyperextension allowance.  Values
    # outside this range remain logged but cannot enter control.
    if not -15.0 <= flexion <= 165.0:
        valid = False
        failures.add("IMU_ELBOW_RANGE")
    quality = _finite(packet.get("quality"))
    quality = _clamp(quality if quality is not None else 0.0, 0.0, 1.0)
    if quality < 0.55:
        failures.add("IMU_LOW_QUALITY")
    upper = packet.get("upper_arm") or {}
    forearm = packet.get("forearm") or {}
    return ArmImuMeasurement(
        timestamp_ns=timestamp_ns,
        sequence=int(packet.get("sequence") or 0),
        side=side,
        flexion_deg=flexion,
        velocity_deg_s=_finite(elbow.get("velocity_deg_s")),
        relative_3d_deg=_finite(elbow.get("relative_3d_deg")),
        off_axis_deg=_finite(elbow.get("off_axis_deg")),
        quality=quality,
        valid=valid,
        upper_quaternion_xyzw=_finite_tuple(upper.get("quaternion_xyzw"), 4),
        forearm_quaternion_xyzw=_finite_tuple(forearm.get("quaternion_xyzw"), 4),
        relative_orientation_xyzw=_finite_tuple(
            packet.get("relative_orientation_xyzw"), 4
        ),
        upper_segment_axis_world_xyz=_finite_tuple(
            upper.get("segment_axis_world_xyz"), 3
        ),
        forearm_segment_axis_world_xyz=_finite_tuple(
            forearm.get("segment_axis_world_xyz"), 3
        ),
        failure_codes=tuple(sorted(failures)),
    )


class TimeAlignedImuBuffer:
    """Small timestamp ordered IMU buffer for a 30/60 Hz BODY_38 consumer."""

    def __init__(self, capacity: int = 256) -> None:
        if capacity < 2:
            raise ValueError("capacity must be at least two")
        self._samples: deque[ArmImuMeasurement] = deque(maxlen=capacity)

    def append(self, sample: ArmImuMeasurement) -> None:
        if self._samples and sample.timestamp_ns <= self._samples[-1].timestamp_ns:
            # UDP can reorder.  Keep a short sorted history rather than making
            # sequence order a hidden timing assumption.
            values = list(self._samples)
            values.append(sample)
            values.sort(key=lambda item: item.timestamp_ns)
            self._samples = deque(values[-self._samples.maxlen :], maxlen=self._samples.maxlen)
            return
        self._samples.append(sample)

    def align(
        self,
        timestamp_ns: int,
        *,
        side: str,
        max_age_ms: float = 100.0,
    ) -> ArmImuMeasurement | None:
        candidates = [sample for sample in self._samples if sample.side == side]
        if not candidates:
            return None
        before = [sample for sample in candidates if sample.timestamp_ns <= timestamp_ns]
        after = [sample for sample in candidates if sample.timestamp_ns >= timestamp_ns]
        left = before[-1] if before else None
        right = after[0] if after else None
        max_age_ns = int(max_age_ms * 1.0e6)
        if left is not None and right is not None and left.timestamp_ns != right.timestamp_ns:
            if timestamp_ns - left.timestamp_ns <= max_age_ns and right.timestamp_ns - timestamp_ns <= max_age_ns:
                alpha = (timestamp_ns - left.timestamp_ns) / (right.timestamp_ns - left.timestamp_ns)

                def blend(first: float | None, second: float | None) -> float | None:
                    if first is None or second is None:
                        return first if abs(timestamp_ns - left.timestamp_ns) <= abs(right.timestamp_ns - timestamp_ns) else second
                    return first + alpha * (second - first)

                failures = tuple(sorted(set(left.failure_codes) | set(right.failure_codes)))
                nearest = (
                    left
                    if abs(timestamp_ns - left.timestamp_ns)
                    <= abs(right.timestamp_ns - timestamp_ns)
                    else right
                )
                return ArmImuMeasurement(
                    timestamp_ns=timestamp_ns,
                    sequence=max(left.sequence, right.sequence),
                    side=side,
                    flexion_deg=blend(left.flexion_deg, right.flexion_deg),
                    velocity_deg_s=blend(left.velocity_deg_s, right.velocity_deg_s),
                    relative_3d_deg=blend(left.relative_3d_deg, right.relative_3d_deg),
                    off_axis_deg=blend(left.off_axis_deg, right.off_axis_deg),
                    quality=min(left.quality, right.quality),
                    valid=left.valid and right.valid,
                    # Orientation needs SLERP.  At 50 Hz, selecting the nearer
                    # normalized quaternion avoids invalid linear interpolation
                    # and keeps the maximum temporal error below one IMU tick.
                    upper_quaternion_xyzw=nearest.upper_quaternion_xyzw,
                    forearm_quaternion_xyzw=nearest.forearm_quaternion_xyzw,
                    relative_orientation_xyzw=nearest.relative_orientation_xyzw,
                    upper_segment_axis_world_xyz=nearest.upper_segment_axis_world_xyz,
                    forearm_segment_axis_world_xyz=nearest.forearm_segment_axis_world_xyz,
                    failure_codes=failures,
                    interpolated=True,
                )
        nearest = min(candidates, key=lambda item: abs(item.timestamp_ns - timestamp_ns))
        if abs(nearest.timestamp_ns - timestamp_ns) > max_age_ns:
            return None
        return nearest


def fuse_elbow_flexion(
    *,
    zed_interior_deg: float | None,
    zed_confidence: float,
    imu: ArmImuMeasurement | None,
    agreement_deg: float = 12.0,
    reject_deg: float = 25.0,
) -> ElbowFusionResult:
    """Fuse only coherent sources; expose conflicts instead of hiding them."""
    failures: set[str] = set()
    interior = _finite(zed_interior_deg)
    zed_flexion = None if interior is None else _clamp(180.0 - interior, -15.0, 165.0)
    zed_quality = _clamp(float(zed_confidence) / 100.0, 0.0, 1.0) if zed_flexion is not None else 0.0
    imu_ready = imu is not None and imu.valid and imu.quality >= 0.55
    if imu is not None:
        failures.update(imu.failure_codes)
    if not imu_ready and zed_flexion is None:
        failures.add("NO_VALID_ELBOW_SOURCE")
        return ElbowFusionResult(None, None, None, None, 0.0, 0.0, "none", False, tuple(sorted(failures)))
    if not imu_ready:
        failures.add("IMU_UNAVAILABLE")
        return ElbowFusionResult(zed_flexion, zed_flexion, None, None, zed_quality, 0.0, "zed", zed_quality >= 0.25, tuple(sorted(failures)))
    if zed_flexion is None:
        failures.add("ZED_ELBOW_UNAVAILABLE")
        return ElbowFusionResult(imu.flexion_deg, None, imu.flexion_deg, None, 0.0, imu.quality, "imu", True, tuple(sorted(failures)))

    disagreement = abs(zed_flexion - imu.flexion_deg)
    if disagreement >= reject_deg:
        failures.add("ZED_IMU_ELBOW_CONFLICT")
        if zed_quality < 0.40 and imu.quality >= 0.75:
            return ElbowFusionResult(imu.flexion_deg, zed_flexion, imu.flexion_deg, disagreement, 0.0, imu.quality, "imu_fallback", True, tuple(sorted(failures)))
        if imu.quality < 0.65 and zed_quality >= 0.70:
            return ElbowFusionResult(zed_flexion, zed_flexion, imu.flexion_deg, disagreement, zed_quality, 0.0, "zed_fallback", True, tuple(sorted(failures)))
        return ElbowFusionResult(None, zed_flexion, imu.flexion_deg, disagreement, zed_quality, imu.quality, "conflict", False, tuple(sorted(failures)))

    # Reduce the weaker source smoothly between the agreement and rejection
    # thresholds.  Squared quality makes a well calibrated IMU useful during
    # wrist/elbow occlusion without allowing it to dominate clear ZED data.
    conflict_scale = 1.0
    if disagreement > agreement_deg:
        conflict_scale = (reject_deg - disagreement) / (reject_deg - agreement_deg)
        failures.add("ZED_IMU_ELBOW_DISAGREEMENT")
    zed_weight = zed_quality * zed_quality
    imu_weight = imu.quality * imu.quality * conflict_scale
    total = zed_weight + imu_weight
    fused = (zed_weight * zed_flexion + imu_weight * imu.flexion_deg) / total
    return ElbowFusionResult(
        fused_flexion_deg=fused,
        zed_flexion_deg=zed_flexion,
        imu_flexion_deg=imu.flexion_deg,
        disagreement_deg=disagreement,
        zed_weight=zed_weight / total,
        imu_weight=imu_weight / total,
        source="fused",
        usable=True,
        failure_codes=tuple(sorted(failures)),
    )


def elbow_interior_deg(
    shoulder: Iterable[float], elbow: Iterable[float], wrist: Iterable[float]
) -> float | None:
    first = [float(value) for value in shoulder]
    center = [float(value) for value in elbow]
    second = [float(value) for value in wrist]
    if not all(math.isfinite(value) for value in first + center + second):
        return None
    upper = [first[index] - center[index] for index in range(3)]
    fore = [second[index] - center[index] for index in range(3)]
    upper_norm = math.sqrt(sum(value * value for value in upper))
    fore_norm = math.sqrt(sum(value * value for value in fore))
    if upper_norm < 1.0e-6 or fore_norm < 1.0e-6:
        return None
    cosine = sum(a * b for a, b in zip(upper, fore)) / (upper_norm * fore_norm)
    return math.degrees(math.acos(_clamp(cosine, -1.0, 1.0)))

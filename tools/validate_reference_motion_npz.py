"""Validate the numerical contract of a G1 23-DOF reference-motion NPZ."""

from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def joint_limits(xml_path: Path) -> dict[str, tuple[float, float]]:
    root = ET.parse(xml_path).getroot()
    result = {}
    for node in root.findall(".//joint"):
        name, limits = node.get("name"), node.get("range")
        if name and limits:
            low, high = (float(item) for item in limits.split())
            result[name] = (low, high)
    return result


def percentile(values: np.ndarray, value: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=np.float64), value))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("motion_npz", type=Path)
    parser.add_argument(
        "--robot-xml",
        type=Path,
        default=Path(r"C:\g1il\repos\unitree_ros\robots\g1_description\g1_23dof.xml"),
    )
    parser.add_argument("--max-velocity", type=float, default=5.05)
    parser.add_argument("--max-acceleration", type=float, default=31.0)
    args = parser.parse_args()

    data = np.load(args.motion_npz, allow_pickle=False)
    required = {
        "fps", "joint_names", "body_names", "joint_pos", "joint_vel",
        "body_pos_w", "body_quat_w", "body_lin_vel_w", "body_ang_vel_w",
        "reference_confidence", "phase", "foot_contact",
    }
    missing = sorted(required.difference(data.files))
    if missing:
        raise ValueError(f"missing NPZ keys: {missing}")
    fps = float(np.asarray(data["fps"]).reshape(-1)[0])
    names = [str(value) for value in np.asarray(data["joint_names"]).reshape(-1)]
    q = np.asarray(data["joint_pos"], dtype=np.float64)
    qd = np.asarray(data["joint_vel"], dtype=np.float64)
    confidence = np.asarray(data["reference_confidence"], dtype=np.float64).reshape(-1)
    if q.ndim != 2 or q.shape[1] != 23 or len(names) != 23:
        raise ValueError("joint_pos and joint_names must use exactly 23 joints")
    if qd.shape != q.shape or len(confidence) != len(q):
        raise ValueError("joint velocity/confidence frame counts do not match joint positions")
    for key in required:
        value = np.asarray(data[key])
        if value.dtype.kind in "fci" and not np.isfinite(value).all():
            raise ValueError(f"non-finite values in {key}")

    dt = 1.0 / fps
    acceleration = np.gradient(qd, dt, axis=0)
    limits = joint_limits(args.robot_xml)
    violations = []
    for index, name in enumerate(names):
        if name not in limits:
            violations.append(f"missing XML limit: {name}")
            continue
        low, high = limits[name]
        if q[:, index].min() < low - 1.0e-4 or q[:, index].max() > high + 1.0e-4:
            violations.append(name)
    quat_norm_error = np.max(np.abs(np.linalg.norm(data["body_quat_w"], axis=-1) - 1.0))
    report = {
        "schema": "g1_reference_motion_validation/v1",
        "file": str(args.motion_npz.resolve()),
        "frames": len(q),
        "duration_s": (len(q) - 1) / fps,
        "fps": fps,
        "confidence": {
            "mean": float(confidence.mean()),
            "p05": percentile(confidence, 5),
            "minimum": float(confidence.min()),
        },
        "joint_speed_rad_s": {
            "p95": percentile(np.abs(qd), 95),
            "maximum": float(np.abs(qd).max()),
        },
        "joint_acceleration_rad_s2": {
            "p95": percentile(np.abs(acceleration), 95),
            "maximum": float(np.abs(acceleration).max()),
        },
        "quaternion_norm_error_max": float(quat_norm_error),
        "joint_limit_violations": violations,
    }
    report["accepted"] = bool(
        fps == 50.0
        and not violations
        and report["joint_speed_rad_s"]["maximum"] <= args.max_velocity
        and report["joint_acceleration_rad_s2"]["maximum"] <= args.max_acceleration
        and quat_norm_error < 1.0e-3
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["accepted"] else 2


if __name__ == "__main__":
    raise SystemExit(main())

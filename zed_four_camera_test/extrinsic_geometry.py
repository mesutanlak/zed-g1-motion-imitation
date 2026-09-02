"""Pure geometry helpers shared by ZED360 conversion and unit tests."""

from __future__ import annotations

import numpy as np


def rebase_camera_transforms(
    transforms_world_from_camera: dict[int, np.ndarray],
    reference_serial: int,
) -> dict[int, np.ndarray]:
    """Express all camera poses in the selected reference camera frame."""
    if reference_serial not in transforms_world_from_camera:
        raise ValueError("Reference camera is missing from ZED360 transforms.")
    reference = np.asarray(
        transforms_world_from_camera[reference_serial], dtype=np.float64
    )
    if reference.shape != (4, 4) or not np.isfinite(reference).all():
        raise ValueError("Reference camera transform is invalid.")
    reference_from_world = np.linalg.inv(reference)
    rebased: dict[int, np.ndarray] = {}
    for serial, value in transforms_world_from_camera.items():
        transform = np.asarray(value, dtype=np.float64)
        if transform.shape != (4, 4) or not np.isfinite(transform).all():
            raise ValueError(f"ZED {serial} pose is invalid.")
        candidate = reference_from_world @ transform
        if serial == reference_serial:
            candidate = np.eye(4, dtype=np.float64)
        rebased[serial] = candidate
    return rebased

"""Regression test for Stereolabs BODY_38 local-to-global rotations."""

from __future__ import annotations

import math

import numpy as np

from body38_to_gmr import Body38ToGMR, _quat_xyzw_to_matrix


def z_quaternion_xyzw(degrees: float) -> list[float]:
    half = math.radians(degrees) * 0.5
    return [0.0, 0.0, math.sin(half), math.cos(half)]


def main() -> int:
    names = [
        "PELVIS",
        "SPINE_1",
        "SPINE_2",
        "SPINE_3",
        "LEFT_CLAVICLE",
        "LEFT_SHOULDER",
        "LEFT_ELBOW",
        "LEFT_WRIST",
    ]
    local = [[0.0, 0.0, 0.0, 1.0] for _ in names]
    local[names.index("LEFT_SHOULDER")] = z_quaternion_xyzw(90.0)
    local[names.index("LEFT_ELBOW")] = z_quaternion_xyzw(30.0)
    frame = {
        "keypoint_names": names,
        "global_root_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "local_orientation_per_joint_xyzw": local,
        "pelvis_frame": {"rotation_camera_from_pelvis": np.eye(3).tolist()},
    }
    result = Body38ToGMR._global_orientations(frame)
    shoulder = _quat_xyzw_to_matrix(
        [result["left_shoulder"][1], result["left_shoulder"][2],
         result["left_shoulder"][3], result["left_shoulder"][0]]
    )
    elbow = _quat_xyzw_to_matrix(
        [result["left_elbow"][1], result["left_elbow"][2],
         result["left_elbow"][3], result["left_elbow"][0]]
    )
    expected_shoulder = _quat_xyzw_to_matrix(z_quaternion_xyzw(90.0))
    expected_elbow = _quat_xyzw_to_matrix(z_quaternion_xyzw(120.0))
    assert shoulder is not None and np.allclose(shoulder, expected_shoulder, atol=1e-7)
    assert elbow is not None and np.allclose(elbow, expected_elbow, atol=1e-7)
    print("BODY38_ORIENTATION_OK shoulder=90deg elbow_global=120deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

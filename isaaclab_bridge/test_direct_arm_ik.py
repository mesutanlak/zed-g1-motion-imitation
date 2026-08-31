"""Regression tests for deterministic G1 23-DOF arm position IK."""

from __future__ import annotations

from types import SimpleNamespace

import mujoco
import numpy as np

from g1_dof_projection import (
    ArmPositionIKRefiner,
    SimpleArmPositionIK,
    official_g1_23dof_xml,
)


def _retargeter():
    model = mujoco.MjModel.from_xml_path(str(official_g1_23dof_xml()))
    data = mujoco.MjData(model)
    return SimpleNamespace(
        model=model,
        configuration=SimpleNamespace(data=data),
    )


def _joint_qpos_id(model, name: str) -> int:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return int(model.jnt_qposadr[joint_id])


def _body_position(model, data, name: str) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    return data.xpos[body_id].copy()


def _targets_from_current_pose(retargeter) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    model = retargeter.model
    data = retargeter.configuration.data
    result = {}
    identity = np.array([1.0, 0.0, 0.0, 0.0])
    for side in ("left", "right"):
        result[f"{side}_shoulder"] = (
            _body_position(model, data, f"{side}_shoulder_pitch_link"), identity
        )
        result[f"{side}_elbow"] = (
            _body_position(model, data, f"{side}_elbow_link"), identity
        )
        result[f"{side}_wrist"] = (
            _body_position(model, data, f"{side}_wrist_roll_rubber_hand"), identity
        )
    return result


def test_hanging_arm_targets_reject_ninety_degree_gmr_branch() -> None:
    retargeter = _retargeter()
    model = retargeter.model
    data = retargeter.configuration.data
    mujoco.mj_forward(model, data)
    hanging_targets = _targets_from_current_pose(retargeter)

    adversarial = data.qpos.copy()
    adversarial[_joint_qpos_id(model, "left_shoulder_roll_joint")] = 1.55
    adversarial[_joint_qpos_id(model, "left_shoulder_yaw_joint")] = 1.55
    adversarial[_joint_qpos_id(model, "right_shoulder_roll_joint")] = -1.55
    adversarial[_joint_qpos_id(model, "right_shoulder_yaw_joint")] = -1.55

    refiner = ArmPositionIKRefiner(retargeter)
    solved = refiner.update(adversarial, hanging_targets)

    for side in ("left", "right"):
        for joint in ("shoulder_roll_joint", "shoulder_yaw_joint"):
            value = solved[_joint_qpos_id(model, f"{side}_{joint}")]
            assert abs(float(value)) < 0.05
        assert refiner.last_error_m[side] < 1.0e-4


def test_front_arm_target_converges_continuously() -> None:
    retargeter = _retargeter()
    model = retargeter.model
    data = retargeter.configuration.data
    target_qpos = data.qpos.copy()
    target_values = {
        "left_shoulder_pitch_joint": -0.70,
        "left_shoulder_roll_joint": 0.35,
        "left_shoulder_yaw_joint": 0.40,
        "left_elbow_joint": 1.00,
        "right_shoulder_pitch_joint": -0.70,
        "right_shoulder_roll_joint": -0.35,
        "right_shoulder_yaw_joint": -0.40,
        "right_elbow_joint": 1.00,
    }
    for name, value in target_values.items():
        target_qpos[_joint_qpos_id(model, name)] = value
    data.qpos[:] = target_qpos
    mujoco.mj_forward(model, data)
    front_targets = _targets_from_current_pose(retargeter)

    refiner = ArmPositionIKRefiner(retargeter)
    current = np.zeros_like(target_qpos)
    deltas = []
    for _ in range(24):
        solved = refiner.update(current, front_targets)
        deltas.append(float(np.linalg.norm(solved - current)))
        current = solved

    assert max(deltas) <= 2.0 * refiner.max_frame_delta_rad + 1.0e-6
    assert refiner.last_error_m["left"] < 0.012
    assert refiner.last_error_m["right"] < 0.012


def test_front_torso_multistart_uses_gmr_candidate_without_hyperextension() -> None:
    retargeter = _retargeter()
    model = retargeter.model
    data = retargeter.configuration.data
    target_qpos = data.qpos.copy()
    for side, sign in (("left", 1.0), ("right", -1.0)):
        values = {
            f"{side}_shoulder_pitch_joint": -0.85,
            f"{side}_shoulder_roll_joint": 0.25 * sign,
            f"{side}_shoulder_yaw_joint": 0.35 * sign,
            f"{side}_elbow_joint": 1.10,
        }
        for name, value in values.items():
            target_qpos[_joint_qpos_id(model, name)] = value
    data.qpos[:] = target_qpos
    mujoco.mj_forward(model, data)
    targets = _targets_from_current_pose(retargeter)

    refiner = ArmPositionIKRefiner(retargeter)
    previous = np.zeros_like(target_qpos)
    refiner.reset(previous)
    solved = refiner.update(target_qpos, targets)
    for side in ("left", "right"):
        assert refiner.last_candidate_count[side] >= 2
        elbow = solved[_joint_qpos_id(model, f"{side}_elbow_joint")]
        assert elbow <= float(refiner.chains[side]["straight_elbow_rad"]) + 1e-9


def test_simple_arm_ik_anchors_floating_root_for_fixed_base() -> None:
    retargeter = _retargeter()
    model = retargeter.model
    data = retargeter.configuration.data
    target_qpos = data.qpos.copy()
    target_qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    for side, sign in (("left", 1.0), ("right", -1.0)):
        target_qpos[
            _joint_qpos_id(model, f"{side}_shoulder_roll_joint")
        ] = 0.85 * sign
        target_qpos[
            _joint_qpos_id(model, f"{side}_elbow_joint")
        ] = 1.15
    data.qpos[:] = target_qpos
    mujoco.mj_forward(model, data)
    targets = _targets_from_current_pose(retargeter)

    refiner = SimpleArmPositionIK(retargeter)
    current = data.qpos.copy()
    # This is close to the erroneous GMR root yaw measured in the recording.
    yaw = np.radians(147.0)
    rotated_root = np.array([
        np.cos(0.5 * yaw), 0.0, 0.0, np.sin(0.5 * yaw)
    ])
    for _ in range(24):
        current[3:7] = rotated_root
        current = refiner.update(current, targets)

    np.testing.assert_allclose(current[3:7], (1.0, 0.0, 0.0, 0.0))
    for side in ("left", "right"):
        assert refiner.last_direction_error_deg[side]["upper"] < 5.0
        assert refiner.last_direction_error_deg[side]["forearm"] < 8.0


def test_simple_arm_ik_recovers_hanging_arm_from_stale_raised_branch() -> None:
    retargeter = _retargeter()
    model = retargeter.model
    data = retargeter.configuration.data
    hanging_qpos = data.qpos.copy()
    for side in ("left", "right"):
        hanging_qpos[
            _joint_qpos_id(model, f"{side}_elbow_joint")
        ] = 1.3826
    data.qpos[:] = hanging_qpos
    mujoco.mj_forward(model, data)
    targets = _targets_from_current_pose(retargeter)

    stale = data.qpos.copy()
    stale[_joint_qpos_id(model, "left_shoulder_roll_joint")] = 1.45
    stale[_joint_qpos_id(model, "left_shoulder_yaw_joint")] = 1.20
    stale[_joint_qpos_id(model, "right_shoulder_roll_joint")] = -1.45
    stale[_joint_qpos_id(model, "right_shoulder_yaw_joint")] = -1.20
    stale[_joint_qpos_id(model, "left_elbow_joint")] = 0.20
    stale[_joint_qpos_id(model, "right_elbow_joint")] = 0.20

    refiner = SimpleArmPositionIK(retargeter)
    refiner.reset(stale)
    current = stale
    for _ in range(18):
        # Keep presenting a bad GMR proposal: recovery must come from the
        # deterministic hanging-arm seed, not a lucky whole-body restart.
        current = refiner.update(stale, targets)

    for side in ("left", "right"):
        assert refiner.last_direction_error_deg[side]["upper"] < 8.0
        assert refiner.last_direction_error_deg[side]["forearm"] < 12.0
        assert refiner.last_error_m[side] < 0.012


def test_simple_arm_ik_preserves_lateral_straight_elbows() -> None:
    """A BODY_38 T-pose must not be accepted as a bent G1 arm branch."""

    retargeter = _retargeter()
    model = retargeter.model
    data = retargeter.configuration.data
    refiner = SimpleArmPositionIK(retargeter)
    target_qpos = data.qpos.copy()
    for side, sign in (("left", 1.0), ("right", -1.0)):
        target_qpos[
            _joint_qpos_id(model, f"{side}_shoulder_pitch_joint")
        ] = 0.0
        target_qpos[
            _joint_qpos_id(model, f"{side}_shoulder_roll_joint")
        ] = 1.25 * sign
        target_qpos[
            _joint_qpos_id(model, f"{side}_shoulder_yaw_joint")
        ] = 0.0
        target_qpos[
            _joint_qpos_id(model, f"{side}_elbow_joint")
        ] = float(refiner.chains[side]["straight_elbow_rad"])
    data.qpos[:] = target_qpos
    mujoco.mj_forward(model, data)
    lateral_targets = _targets_from_current_pose(retargeter)

    stale = target_qpos.copy()
    for side in ("left", "right"):
        stale[_joint_qpos_id(model, f"{side}_elbow_joint")] = 0.20
    refiner.reset(stale)
    current = stale
    for _ in range(5):
        # Continue presenting a bent GMR proposal. The arm refiner must use
        # observed upper/forearm collinearity to recover the straight branch.
        current = refiner.update(stale, lateral_targets)

    for side in ("left", "right"):
        elbow = current[_joint_qpos_id(model, f"{side}_elbow_joint")]
        straight = float(refiner.chains[side]["straight_elbow_rad"])
        assert abs(float(elbow) - straight) < 0.12
        assert refiner.last_direction_error_deg[side]["forearm"] < 10.0

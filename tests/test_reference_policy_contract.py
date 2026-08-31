from __future__ import annotations

import numpy as np
import pytest

from motion_pipeline.reference_policy import (
    ACTION_SLEW_PER_STEP,
    ACTION_DIM,
    FIXED_DOUBLE_SUPPORT_CONTACT_STATE,
    OBSERVATION_DIM,
    TRACKED_BODIES,
    UPPER_POLICY_JOINTS,
    assemble_observation,
    policy_metadata,
    policy_collision_action_scale_numpy,
    policy_collision_action_scale_torch,
    postprocess_action_numpy,
    postprocess_action_torch,
    residual_scale_vector,
)


def _observation(**overrides):
    values = {
        "projected_gravity": [0.0, 0.0, -1.0],
        "base_ang_vel": [0.0, 0.0, 0.0],
        "q_minus_q_default": np.zeros(ACTION_DIM),
        "qd": np.zeros(ACTION_DIM),
        "q_ref_minus_q": np.zeros(ACTION_DIM),
        "qd_ref_minus_qd": np.zeros(ACTION_DIM),
        "body_reference_error": np.zeros(3 * len(TRACKED_BODIES)),
        "foot_contact_state": [1.0, 1.0],
        "previous_action": np.zeros(ACTION_DIM),
        "reference_confidence": 1.0,
    }
    values.update(overrides)
    return assemble_observation(**values)


def _safe_robot_positions():
    return {
        "pelvis": [0.0, 0.0, 0.0],
        "torso_link": [0.0, 0.0, 0.40],
        "left_shoulder_pitch_link": [0.0, 0.24, 0.42],
        "left_elbow_link": [0.0, 0.38, 0.30],
        "left_wrist_roll_rubber_hand": [0.0, 0.55, 0.22],
        "right_shoulder_pitch_link": [0.0, -0.24, 0.42],
        "right_elbow_link": [0.0, -0.38, 0.30],
        "right_wrist_roll_rubber_hand": [0.0, -0.55, 0.22],
    }


def test_upper_policy_contract_is_waist_and_arms_only():
    assert ACTION_DIM == 11
    assert UPPER_POLICY_JOINTS[0] == "waist_yaw_joint"
    assert not any("hip" in name or "ankle" in name for name in UPPER_POLICY_JOINTS)
    assert _observation().shape == (OBSERVATION_DIM,)
    assert OBSERVATION_DIM == 88


def test_observation_scales_match_deployment_contract():
    obs = _observation(base_ang_vel=[4.0, 0.0, 0.0], qd=np.ones(ACTION_DIM))
    assert obs[3] == pytest.approx(1.0)
    qd_start = 6 + ACTION_DIM
    assert np.allclose(obs[qd_start : qd_start + ACTION_DIM], 0.05)


def test_observation_rejects_wrong_shape_and_nonfinite_values():
    with pytest.raises(ValueError):
        _observation(previous_action=np.zeros(ACTION_DIM - 1))
    bad = np.zeros(ACTION_DIM)
    bad[2] = np.nan
    with pytest.raises(ValueError):
        _observation(q_ref_minus_q=bad)


def test_metadata_and_array_contract_agree():
    metadata = policy_metadata()
    assert metadata["observation_dim"] == _observation().size
    assert metadata["action_dim"] == len(metadata["joint_names"])
    assert metadata["fixed_double_support_contact_state"] == list(FIXED_DOUBLE_SUPPORT_CONTACT_STATE)
    assert residual_scale_vector().shape == (ACTION_DIM,)


def test_live_action_wrapper_logs_raw_and_bounds_applied_residual():
    raw = np.full(ACTION_DIM, 3.0, dtype=np.float32)
    result = postprocess_action_numpy(
        raw,
        np.zeros(ACTION_DIM, dtype=np.float32),
        reference_confidence=1.0,
    )
    assert np.all(result["raw"] == 3.0)
    assert np.all(result["saturation_mask"])
    assert np.allclose(result["slewed"], ACTION_SLEW_PER_STEP)
    assert np.max(np.abs(result["correction_rad"])) <= 0.11 * ACTION_SLEW_PER_STEP + 1e-7


def test_confidence_dropout_removes_policy_correction():
    result = postprocess_action_numpy(
        np.ones(ACTION_DIM, dtype=np.float32),
        np.ones(ACTION_DIM, dtype=np.float32),
        reference_confidence=0.0,
    )
    assert np.allclose(result["applied"], 0.0)
    assert np.allclose(result["correction_rad"], 0.0)


def test_collision_guard_disables_only_the_at_risk_policy_arm():
    safe = _safe_robot_positions()
    safe["right_elbow_link"] = [0.0, -0.20, 0.28]
    safe["right_wrist_roll_rubber_hand"] = [0.0, 0.02, 0.24]
    guard = policy_collision_action_scale_numpy(safe, safe)
    assert guard["active"] is True
    assert np.allclose(guard["action_scale"][6:11], 0.0)
    assert np.allclose(guard["action_scale"][1:6], 1.0)
    assert guard["action_scale"][0] == pytest.approx(1.0)


def test_collision_guard_detects_forearm_segment_crossing_torso():
    safe = _safe_robot_positions()
    # Endpoints can both look plausible while the connecting segment crosses
    # the trunk; this is the failure mode a point-origin guard missed.
    safe["left_elbow_link"] = [0.0, 0.25, 0.30]
    safe["left_wrist_roll_rubber_hand"] = [0.0, -0.25, 0.30]
    guard = policy_collision_action_scale_numpy(safe, safe)
    assert guard["left_arm_scale"] == pytest.approx(0.0)
    assert "left_forearm_torso_capsule" in guard["reasons"]
    assert guard["clearances"]["left_forearm_torso_capsule"]["safe_m"] < 0.0


def test_numpy_and_torch_collision_guards_match():
    torch = pytest.importorskip("torch")
    safe = _safe_robot_positions()
    safe["left_elbow_link"] = [0.0, 0.25, 0.30]
    safe["left_wrist_roll_rubber_hand"] = [0.0, -0.25, 0.30]
    names = list(TRACKED_BODIES)
    tensor = torch.tensor([[safe[name] for name in names]], dtype=torch.float32)
    numpy_scale = policy_collision_action_scale_numpy(safe, safe)["action_scale"]
    torch_scale = policy_collision_action_scale_torch(tensor, tensor, names)[0]
    assert np.allclose(numpy_scale, torch_scale.numpy(), atol=1.0e-6)


def test_collision_guard_clears_carried_residual_without_touching_ik():
    action_scale = np.ones(ACTION_DIM, dtype=np.float32)
    action_scale[1:6] = 0.0
    result = postprocess_action_numpy(
        np.ones(ACTION_DIM, dtype=np.float32),
        np.ones(ACTION_DIM, dtype=np.float32),
        reference_confidence=1.0,
        safety_scale=action_scale,
    )
    assert np.allclose(result["slewed"][1:6], 0.0)
    assert np.allclose(result["correction_rad"][1:6], 0.0)
    assert np.all(result["correction_rad"][6:11] > 0.0)


def test_supportive_residual_cannot_push_away_from_normal_ik():
    error = np.full(ACTION_DIM, 0.10, dtype=np.float32)
    result = postprocess_action_numpy(
        -np.ones(ACTION_DIM, dtype=np.float32),
        np.zeros(ACTION_DIM, dtype=np.float32),
        reference_confidence=1.0,
        support_error_rad=error,
    )
    assert np.allclose(result["correction_rad"], 0.0)
    assert np.allclose(result["applied"], 0.0)


def test_supportive_residual_is_zero_when_ik_is_already_tracked():
    result = postprocess_action_numpy(
        np.ones(ACTION_DIM, dtype=np.float32),
        np.zeros(ACTION_DIM, dtype=np.float32),
        reference_confidence=1.0,
        support_error_rad=np.full(ACTION_DIM, 0.005, dtype=np.float32),
    )
    assert np.allclose(result["support_gain"], 0.0)
    assert np.allclose(result["correction_rad"], 0.0)


def test_numpy_and_training_supportive_wrappers_are_equivalent():
    torch = pytest.importorskip("torch")
    rng = np.random.default_rng(7)
    raw = rng.normal(size=(4, ACTION_DIM)).astype(np.float32)
    previous = rng.uniform(-0.5, 0.5, size=raw.shape).astype(np.float32)
    confidence = rng.uniform(0.2, 1.0, size=(4, 1)).astype(np.float32)
    safety = rng.uniform(0.0, 1.0, size=raw.shape).astype(np.float32)
    error = rng.normal(scale=0.1, size=raw.shape).astype(np.float32)
    scale = np.broadcast_to(residual_scale_vector(), raw.shape).copy()
    numpy_result = postprocess_action_numpy(
        raw,
        previous,
        confidence,
        safety_scale=safety,
        residual_scale_rad=scale,
        support_error_rad=error,
    )
    torch_result = postprocess_action_torch(
        torch.from_numpy(raw),
        torch.from_numpy(previous),
        torch.from_numpy(confidence),
        safety_scale=torch.from_numpy(safety),
        residual_scale_rad=torch.from_numpy(scale),
        support_error_rad=torch.from_numpy(error),
    )
    for name in ("slewed", "applied", "correction_rad", "support_gain"):
        assert np.allclose(
            numpy_result[name], torch_result[name].cpu().numpy(), atol=1.0e-7
        )

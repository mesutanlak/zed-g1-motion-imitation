from reference_policy.evaluation_report import build_evaluation_report, report_markdown


def _metrics(scale=1.0):
    return {
        "joint_rmse_rad": 0.10 * scale,
        "hand_position_rmse_m": 0.05 * scale,
        "elbow_position_rmse_m": 0.04 * scale,
        "limb_direction_error": 0.03 * scale,
        "orientation_error_rad": 0.08 * scale,
        "reward_mean": 1.0 / scale,
        "action_rms": 0.1,
        "action_abs_max": 0.8,
        "action_rate_rms": 0.1,
        "joint_limit_saturation_rate": 0.0,
        "self_collision_proxy": 0.0,
        "joint_acceleration_rms_rad_s2": 1.0,
        "torque_rms_nm": 1.0,
        "foot_slip_speed_m_s": 0.0,
    }


def test_better_policy_is_accepted_and_documented():
    report = build_evaluation_report(
        manifest_path="clips_manifest.json",
        train_clip_ids=["a", "b"],
        validation_clip_ids=["held_out"],
        baseline=_metrics(1.0),
        policy=_metrics(0.8),
        steps=500,
        num_envs=64,
        training_run="run",
    )
    assert report["accepted"] is True
    assert report["policy_strength_percent"] > 0
    assert "ACCEPTED" in report_markdown(report)


def test_tracking_regression_is_not_published():
    report = build_evaluation_report(
        manifest_path="clips_manifest.json",
        train_clip_ids=["a"],
        validation_clip_ids=["held_out"],
        baseline=_metrics(1.0),
        policy=_metrics(1.2),
        steps=100,
        num_envs=8,
        training_run="run",
    )
    assert report["accepted"] is False


def test_baseline_relative_stationary_and_lower_body_gates_allow_no_regression():
    baseline = _metrics(1.0)
    policy = _metrics(0.9)
    baseline.update(
        {
            "stationary_joint_velocity_rms_rad_s": 0.138,
            "stationary_action_rate_rms": 0.0,
            "lower_body_position_rmse_rad": 0.039,
            "lower_body_velocity_rms_rad_s": 0.818,
        }
    )
    policy.update(
        {
            "stationary_joint_velocity_rms_rad_s": 0.136,
            "stationary_action_rate_rms": 0.02,
            "lower_body_position_rmse_rad": 0.039,
            "lower_body_velocity_rms_rad_s": 0.818,
            "policy_collision_guard_demand_rms": 0.073,
        }
    )
    report = build_evaluation_report(
        manifest_path="clips_manifest.json",
        train_clip_ids=["a"],
        validation_clip_ids=["held_out"],
        baseline=baseline,
        policy=policy,
        steps=500,
        num_envs=64,
        training_run="run",
    )
    assert report["accepted"] is True
    assert report["acceptance_gate_version"] == "g1_reference_policy_gates/v4_robot_body_barrier"


def test_collision_guard_demand_above_relaxed_ceiling_is_rejected():
    baseline = _metrics(1.0)
    policy = _metrics(0.9)
    policy["policy_collision_guard_demand_rms"] = 0.081
    report = build_evaluation_report(
        manifest_path="clips_manifest.json",
        train_clip_ids=["a"],
        validation_clip_ids=["held_out"],
        baseline=baseline,
        policy=policy,
        steps=500,
        num_envs=64,
        training_run="run",
    )
    assert report["accepted"] is False

"""Pure helpers for held-out policy acceptance, Pareto selection and reports."""

from __future__ import annotations

import math
from datetime import datetime, timezone


TRACKING_WEIGHTS = {
    "joint_rmse_rad": 1.0,
    "hand_position_rmse_m": 2.0,
    "elbow_position_rmse_m": 1.5,
    "limb_direction_error": 0.5,
    "orientation_error_rad": 0.25,
}


# Deployment gates are intentionally expressed relative to normal IK whenever
# the metric is also affected by the simulator/reset harness.  A residual
# policy must not be rejected for motion already present in the baseline, but
# it still has a hard ceiling so a genuinely unhealthy rollout cannot pass.
ACCEPTANCE_GATE_VERSION = "g1_reference_policy_gates/v4_robot_body_barrier"
STATIONARY_VELOCITY_HARD_MAX_RAD_S = 0.15
STATIONARY_VELOCITY_RELATIVE_LIMIT = 1.05
STATIONARY_ACTION_RATE_MAX = 0.12
COLLISION_GUARD_DEMAND_RMS_MAX = 0.08
WRAPPER_REJECTED_DEMAND_RMS_MAX = 0.08
BODY_BARRIER_HARD_VIOLATION_RATE_MAX = 0.005
LOWER_BODY_POSITION_HARD_MAX_RAD = 0.05
LOWER_BODY_POSITION_DELTA_MAX_RAD = 0.002
LOWER_BODY_VELOCITY_HARD_MAX_RAD_S = 1.0
LOWER_BODY_VELOCITY_RELATIVE_LIMIT = 1.05
LOWER_BODY_VELOCITY_DELTA_MAX_RAD_S = 0.02


def composite_tracking_error(metrics: dict[str, float]) -> float:
    return float(sum(TRACKING_WEIGHTS[name] * float(metrics[name]) for name in TRACKING_WEIGHTS))


def _finite(metrics: dict[str, float]) -> bool:
    return all(math.isfinite(float(value)) for value in metrics.values())


def build_evaluation_report(
    *,
    manifest_path: str,
    train_clip_ids: list[str],
    validation_clip_ids: list[str],
    baseline: dict[str, float],
    policy: dict[str, float],
    steps: int,
    num_envs: int,
    training_run: str,
    checkpoint_name: str | None = None,
    checkpoint_iteration: int | None = None,
    train_session_ids: list[str] | None = None,
    validation_session_ids: list[str] | None = None,
    stress: dict[str, float] | None = None,
    contract: dict[str, float | bool | str] | None = None,
    action_diagnostics: dict | None = None,
) -> dict:
    """Create the deployment gate used for both candidate and live policy."""

    stress = dict(stress or {})
    contract = dict(contract or {})
    baseline_composite = composite_tracking_error(baseline)
    policy_composite = composite_tracking_error(policy)
    denominator = max(baseline_composite, 1.0e-9)
    strength_percent = 100.0 * (baseline_composite - policy_composite) / denominator
    deltas = {
        name: float(policy[name]) - float(baseline[name])
        for name in sorted(set(baseline).intersection(policy))
    }
    train_sessions = set(train_session_ids or [])
    validation_sessions = set(validation_session_ids or [])
    session_disjoint = not train_sessions or not validation_sessions or train_sessions.isdisjoint(validation_sessions)
    stress_composite = float(stress.get("composite_tracking_error", policy_composite))
    baseline_stationary_velocity = float(
        baseline.get("stationary_joint_velocity_rms_rad_s", 0.0)
    )
    policy_stationary_velocity = float(policy.get("stationary_joint_velocity_rms_rad_s", 0.0))
    stationary_velocity_limit = min(
        STATIONARY_VELOCITY_HARD_MAX_RAD_S,
        baseline_stationary_velocity * STATIONARY_VELOCITY_RELATIVE_LIMIT + 1.0e-6,
    )
    baseline_lower_position = float(baseline.get("lower_body_position_rmse_rad", 0.0))
    policy_lower_position = float(policy.get("lower_body_position_rmse_rad", 0.0))
    lower_position_limit = min(
        LOWER_BODY_POSITION_HARD_MAX_RAD,
        baseline_lower_position + LOWER_BODY_POSITION_DELTA_MAX_RAD,
    )
    baseline_lower_velocity = float(baseline.get("lower_body_velocity_rms_rad_s", 0.0))
    policy_lower_velocity = float(policy.get("lower_body_velocity_rms_rad_s", 0.0))
    lower_velocity_limit = min(
        LOWER_BODY_VELOCITY_HARD_MAX_RAD_S,
        baseline_lower_velocity * LOWER_BODY_VELOCITY_RELATIVE_LIMIT
        + LOWER_BODY_VELOCITY_DELTA_MAX_RAD_S,
    )
    criteria = {
        "tracking_not_degraded_over_0_5_percent": policy_composite <= baseline_composite * 1.005,
        "joint_acceleration_not_degraded_over_15_percent": (
            float(policy.get("joint_acceleration_rms_rad_s2", 0.0))
            <= float(baseline.get("joint_acceleration_rms_rad_s2", 0.0)) * 1.15 + 0.25
        ),
        "torque_not_degraded_over_15_percent": (
            float(policy.get("torque_rms_nm", 0.0))
            <= float(baseline.get("torque_rms_nm", 0.0)) * 1.15 + 0.10
        ),
        "joint_jerk_not_degraded_over_20_percent": (
            float(policy.get("joint_jerk_rms_rad_s3", 0.0))
            <= float(baseline.get("joint_jerk_rms_rad_s3", 0.0)) * 1.20 + 2.0
        ),
        "action_rate_bounded": float(policy.get("action_rate_rms", 0.0)) <= 0.35,
        "stationary_drift_bounded": (
            policy_stationary_velocity <= stationary_velocity_limit
            and float(policy.get("stationary_action_rate_rms", 0.0))
            <= STATIONARY_ACTION_RATE_MAX
        ),
        "raw_action_saturation_below_2_percent": (
            float(policy.get("raw_action_saturation_rate", 0.0)) <= 0.02
        ),
        "observation_ood_below_1_percent": (
            float(policy.get("observation_ood_fraction", 0.0)) <= 0.01
        ),
        "live_wrapper_equivalent": float(contract.get("wrapper_max_abs_error", 0.0)) <= 1.0e-6,
        "confidence_dropout_safe": float(stress.get("dropout_correction_abs_max_rad", 0.0)) <= 1.0e-6,
        "delayed_noisy_reference_robust": stress_composite <= policy_composite * 1.30 + 1.0e-6,
        "cross_session_validation": session_disjoint,
        "joint_limit_saturation_not_increased": (
            float(policy["joint_limit_saturation_rate"])
            <= float(baseline["joint_limit_saturation_rate"]) + 1.0e-3
        ),
        "self_collision_proxy_not_increased": (
            float(policy["self_collision_proxy"])
            <= float(baseline["self_collision_proxy"]) + 1.0e-4
        ),
        "policy_collision_guard_demand_below_0_08_rms": (
            float(policy.get("policy_collision_guard_demand_rms", 0.0))
            <= COLLISION_GUARD_DEMAND_RMS_MAX
        ),
        "policy_wrapper_rejected_demand_below_0_08_rms": (
            float(policy.get("policy_wrapper_rejected_demand_rms", 0.0))
            <= WRAPPER_REJECTED_DEMAND_RMS_MAX
        ),
        "robot_body_barrier_proximity_not_increased": (
            float(policy.get("robot_body_barrier_proximity_rms", 0.0))
            <= float(baseline.get("robot_body_barrier_proximity_rms", 0.0))
            + 1.0e-3
        ),
        "robot_body_barrier_hard_violation_bounded": (
            float(policy.get("robot_body_barrier_hard_violation_rate", 0.0))
            <= min(
                BODY_BARRIER_HARD_VIOLATION_RATE_MAX,
                float(baseline.get("robot_body_barrier_hard_violation_rate", 0.0))
                + 1.0e-3,
            )
        ),
        "lower_body_held_nominal": (
            policy_lower_position <= lower_position_limit
            and policy_lower_velocity <= lower_velocity_limit
        ),
        "all_metrics_finite": _finite(baseline) and _finite(policy) and _finite(stress),
    }
    accepted = all(criteria.values())
    return {
        "schema": "g1_reference_policy_evaluation/v2",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "training_run": training_run,
        "checkpoint": {"name": checkpoint_name, "iteration": checkpoint_iteration},
        "dataset_manifest": manifest_path,
        "train_clip_ids": list(train_clip_ids),
        "validation_clip_ids": list(validation_clip_ids),
        "train_session_ids": sorted(train_sessions),
        "validation_session_ids": sorted(validation_sessions),
        "evaluation_steps": int(steps),
        "evaluation_environments": int(num_envs),
        "baseline_mode": "normal_ik_zero_residual",
        "candidate_mode": "normal_ik_plus_live_bounded_residual_wrapper",
        "baseline": {key: float(value) for key, value in baseline.items()},
        "policy": {key: float(value) for key, value in policy.items()},
        "stress_tests": {key: float(value) for key, value in stress.items()},
        "live_contract": contract,
        "action_diagnostics": action_diagnostics or {},
        "delta_policy_minus_baseline": deltas,
        "composite_tracking_error": {"baseline": baseline_composite, "policy": policy_composite},
        "policy_strength_percent": strength_percent,
        "acceptance_gate_version": ACCEPTANCE_GATE_VERSION,
        "acceptance_limits": {
            "stationary_joint_velocity_rms_rad_s": stationary_velocity_limit,
            "stationary_action_rate_rms": STATIONARY_ACTION_RATE_MAX,
            "policy_collision_guard_demand_rms": COLLISION_GUARD_DEMAND_RMS_MAX,
            "policy_wrapper_rejected_demand_rms": WRAPPER_REJECTED_DEMAND_RMS_MAX,
            "robot_body_barrier_hard_violation_rate": BODY_BARRIER_HARD_VIOLATION_RATE_MAX,
            "lower_body_position_rmse_rad": lower_position_limit,
            "lower_body_velocity_rms_rad_s": lower_velocity_limit,
        },
        "acceptance_criteria": criteria,
        "accepted": accepted,
    }


def pareto_score(report: dict) -> float:
    """Lower is better; tracking dominates, physical regressions break ties."""

    baseline = report["baseline"]
    policy = report["policy"]

    def ratio(name: str) -> float:
        return float(policy.get(name, 0.0)) / max(float(baseline.get(name, 0.0)), 1.0e-6)

    tracking = float(report["composite_tracking_error"]["policy"])
    physical = (
        0.20 * ratio("joint_acceleration_rms_rad_s2")
        + 0.15 * ratio("torque_rms_nm")
        + 0.15 * ratio("joint_jerk_rms_rad_s3")
        + 0.10 * float(policy.get("raw_action_saturation_rate", 0.0))
        + 0.05 * float(policy.get("stationary_joint_velocity_rms_rad_s", 0.0))
        + 0.20 * float(policy.get("policy_collision_guard_demand_rms", 0.0))
    )
    rejection = 0.0 if report.get("accepted") else 1000.0
    return rejection + tracking + physical


def select_pareto_report(reports: list[dict]) -> dict:
    if not reports:
        raise ValueError("checkpoint sweep produced no reports")
    selected = min(reports, key=pareto_score)
    for report in reports:
        report["pareto_score"] = pareto_score(report)
        report["selected"] = report is selected
    return selected


def report_markdown(report: dict) -> str:
    rows = []
    baseline = report["baseline"]
    policy = report["policy"]
    for name in sorted(baseline):
        rows.append(
            f"| `{name}` | {baseline[name]:.6g} | {policy[name]:.6g} | "
            f"{policy[name] - baseline[name]:+.6g} |"
        )
    criteria = "\n".join(
        f"- {'PASS' if passed else 'FAIL'} — `{name}`"
        for name, passed in report["acceptance_criteria"].items()
    )
    checkpoint = report.get("checkpoint", {})
    return (
        "# G1 Reference Policy Validation\n\n"
        f"- Result: **{'ACCEPTED' if report['accepted'] else 'REJECTED'}**\n"
        f"- Selected checkpoint: **{checkpoint.get('name') or 'unknown'}** "
        f"(iteration {checkpoint.get('iteration')})\n"
        f"- Policy contribution: **{report['policy_strength_percent']:+.3f}%** "
        "(positive means lower held-out tracking error)\n"
        f"- Acceptance gate version: **{report.get('acceptance_gate_version', 'legacy')}**\n"
        f"- Train clips: {len(report['train_clip_ids'])}\n"
        f"- Held-out validation clips: {', '.join(report['validation_clip_ids'])}\n"
        f"- Evaluation: {report['evaluation_environments']} environments × "
        f"{report['evaluation_steps']} steps\n\n"
        "## Metrics\n\n"
        "| Metric | Normal IK | Policy + IK | Delta |\n"
        "|---|---:|---:|---:|\n"
        + "\n".join(rows)
        + "\n\n## Acceptance gates\n\n"
        + criteria
        + "\n"
    )

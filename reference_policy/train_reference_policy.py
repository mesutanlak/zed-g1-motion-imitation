"""RSL-RL trainer for the installed G1 reference-motion task.

Unitree RL Lab's pinned ``train.py`` discovers tasks before it launches Isaac
Sim.  That ordering does not work with this pip-based Isaac Sim 5 installation
(``omni.physics`` is unavailable before AppLauncher).  This launcher keeps the
same RSL-RL workflow but imports task packages after Isaac starts.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import os
import pathlib
import shutil
import subprocess
import sys
import uuid
from datetime import datetime


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

install_root = pathlib.Path(os.environ.get("G1_ISAAC_INSTALL_ROOT", r"C:\g1il"))
rsl_script_dir = install_root / "repos" / "unitree_rl_lab" / "scripts" / "rsl_rl"
sys.path.insert(0, str(rsl_script_dir))
import cli_args  # noqa: E402

from isaaclab.app import AppLauncher  # noqa: E402


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--video", action="store_true", default=False)
parser.add_argument("--video_length", type=int, default=400)
parser.add_argument("--video_interval", type=int, default=5000)
parser.add_argument("--num_envs", type=int, default=4096)
parser.add_argument("--seed", type=int, default=None)
parser.add_argument("--max_iterations", type=int, default=20000)
parser.add_argument(
    "--resume-checkpoint",
    type=pathlib.Path,
    default=None,
    help="Continue from this accepted checkpoint instead of forgetting prior sessions",
)
parser.add_argument(
    "--validation-steps",
    type=int,
    default=500,
    help="Held-out rollout steps for normal-IK versus policy+IK comparison",
)
parser.add_argument(
    "--policy-output-dir",
    type=pathlib.Path,
    default=PROJECT_ROOT / "policies" / "g1_reference_upper_body",
    help="Stable live-deployment directory for policy.onnx and metadata",
)
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
if args_cli.video:
    args_cli.enable_cameras = True
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

import isaaclab_tasks  # noqa: F401,E402
import unitree_rl_lab.tasks  # noqa: F401,E402
from isaaclab.envs import ManagerBasedRLEnvCfg  # noqa: E402
from isaaclab.utils.dict import print_dict  # noqa: E402
from isaaclab.utils.io import dump_yaml  # noqa: E402
from isaaclab_rl.rsl_rl import (  # noqa: E402
    RslRlOnPolicyRunnerCfg,
    RslRlVecEnvWrapper,
    export_policy_as_onnx,
)
from isaaclab_tasks.utils import get_checkpoint_path  # noqa: E402
from isaaclab_tasks.utils.hydra import hydra_task_config  # noqa: E402
from unitree_rl_lab.utils.export_deploy_cfg import export_deploy_cfg  # noqa: E402
from isaaclab.utils.math import quat_error_magnitude  # noqa: E402

from motion_pipeline.reference_policy import (  # noqa: E402
    FIXED_DOUBLE_SUPPORT_CONTACT_STATE,
    INFERENCE_WRAPPER_VERSION,
    LOWER_POLICY_JOINTS,
    UPPER_POLICY_JOINTS,
    policy_collision_action_scale_torch,
    policy_metadata,
    postprocess_action_numpy,
    postprocess_action_torch,
    residual_scale_vector,
)
from reference_policy.evaluation_report import (  # noqa: E402
    build_evaluation_report,
    composite_tracking_error,
    report_markdown,
    select_pareto_report,
)


TASK_ID = "Unitree-G1-23dof-Reference-Motion"
torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
torch.backends.cudnn.deterministic = False
torch.backends.cudnn.benchmark = False


def _tracking_metrics(
    env,
    actions: torch.Tensor,
    previous_actions: torch.Tensor,
    reward: torch.Tensor,
    previous_joint_acceleration: torch.Tensor,
) -> dict[str, float]:
    base = env.unwrapped
    command = base.command_manager.get_term("motion")
    robot = base.scene["robot"]
    upper_names = list(policy_metadata()["joint_names"])
    upper_ids = torch.as_tensor(
        [robot.joint_names.index(name) for name in upper_names],
        dtype=torch.long,
        device=robot.device,
    )
    lower_ids = torch.as_tensor(
        [robot.joint_names.index(name) for name in LOWER_POLICY_JOINTS],
        dtype=torch.long,
        device=robot.device,
    )
    joint_error = command.joint_pos[:, upper_ids] - robot.data.joint_pos[:, upper_ids]

    def body_ids(names: list[str]) -> list[int]:
        return [command.cfg.body_names.index(name) for name in names]

    hand_ids = body_ids(["left_wrist_roll_rubber_hand", "right_wrist_roll_rubber_hand"])
    elbow_ids = body_ids(["left_elbow_link", "right_elbow_link"])
    tracked_error = command.body_pos_relative_w - command.robot_body_pos_w
    hand_rmse = torch.sqrt(torch.mean(torch.square(tracked_error[:, hand_ids])))
    elbow_rmse = torch.sqrt(torch.mean(torch.square(tracked_error[:, elbow_ids])))

    limb_errors = []
    for start_name, end_name in (
        ("left_shoulder_pitch_link", "left_elbow_link"),
        ("left_elbow_link", "left_wrist_roll_rubber_hand"),
        ("right_shoulder_pitch_link", "right_elbow_link"),
        ("right_elbow_link", "right_wrist_roll_rubber_hand"),
    ):
        start, end = body_ids([start_name, end_name])
        target = torch.nn.functional.normalize(
            command.body_pos_relative_w[:, end] - command.body_pos_relative_w[:, start], dim=-1, eps=1.0e-6
        )
        actual = torch.nn.functional.normalize(
            command.robot_body_pos_w[:, end] - command.robot_body_pos_w[:, start], dim=-1, eps=1.0e-6
        )
        limb_errors.append(1.0 - torch.sum(target * actual, dim=-1).clamp(-1.0, 1.0))
    orientation_ids = body_ids(["pelvis", "torso_link", "left_elbow_link", "right_elbow_link"])
    orientation_error = quat_error_magnitude(
        command.body_quat_relative_w[:, orientation_ids],
        command.robot_body_quat_w[:, orientation_ids],
    ).mean()

    limits = robot.data.soft_joint_pos_limits[:, upper_ids]
    positions = robot.data.joint_pos[:, upper_ids]
    margin = torch.minimum(positions - limits[..., 0], limits[..., 1] - positions)
    saturation = (margin < 0.01).to(torch.float32).mean()

    collision_proxy = torch.zeros(base.num_envs, device=robot.device)
    for first, second, margin_m in (
        ("left_wrist_roll_rubber_hand", "right_wrist_roll_rubber_hand", 0.12),
        ("left_elbow_link", "right_elbow_link", 0.16),
        ("left_wrist_roll_rubber_hand", "torso_link", 0.15),
        ("right_wrist_roll_rubber_hand", "torso_link", 0.15),
        ("left_elbow_link", "torso_link", 0.14),
        ("right_elbow_link", "torso_link", 0.14),
    ):
        first_id = robot.body_names.index(first)
        second_id = robot.body_names.index(second)
        distance = torch.linalg.vector_norm(
            robot.data.body_pos_w[:, first_id] - robot.data.body_pos_w[:, second_id], dim=-1
        )
        collision_proxy += torch.square(
            torch.clamp((margin_m - distance) / margin_m, min=0.0)
        )

    action_term = base.action_manager.get_term("residual_joint_position")
    guard_demand = action_term.clipped_actions * (1.0 - action_term.safety_scale)
    wrapper_rejected = action_term.pre_safety_slewed - action_term.bounded_actions
    body_barrier_scale = policy_collision_action_scale_torch(
        command.robot_body_pos_w,
        command.robot_body_pos_w,
        list(command.cfg.body_names),
    )
    arm_barrier_scale = torch.stack(
        (body_barrier_scale[:, 1], body_barrier_scale[:, 6]), dim=-1
    )

    foot_ids = [robot.body_names.index("left_ankle_roll_link"), robot.body_names.index("right_ankle_roll_link")]
    foot_slip = torch.linalg.vector_norm(robot.data.body_lin_vel_w[:, foot_ids, :2], dim=-1).mean()
    joint_acceleration = robot.data.joint_acc[:, upper_ids]
    joint_jerk = (joint_acceleration - previous_joint_acceleration) / float(base.step_dt)
    stationary = torch.sqrt(torch.mean(torch.square(command.joint_vel[:, upper_ids]), dim=-1)) < 0.05
    if torch.any(stationary):
        stationary_velocity = torch.sqrt(
            torch.mean(torch.square(robot.data.joint_vel[stationary][:, upper_ids]))
        )
        stationary_action_rate = torch.sqrt(
            torch.mean(torch.square((actions - previous_actions)[stationary]))
        )
    else:
        stationary_velocity = torch.zeros((), device=robot.device)
        stationary_action_rate = torch.zeros((), device=robot.device)
    return {
        "joint_rmse_rad": float(torch.sqrt(torch.mean(torch.square(joint_error))).item()),
        "hand_position_rmse_m": float(hand_rmse.item()),
        "elbow_position_rmse_m": float(elbow_rmse.item()),
        "limb_direction_error": float(torch.stack(limb_errors).mean().item()),
        "orientation_error_rad": float(orientation_error.item()),
        "reward_mean": float(reward.mean().item()),
        "action_rms": float(torch.sqrt(torch.mean(torch.square(actions))).item()),
        "action_abs_max": float(torch.max(torch.abs(actions)).item()),
        "action_rate_rms": float(torch.sqrt(torch.mean(torch.square(actions - previous_actions))).item()),
        "joint_limit_saturation_rate": float(saturation.item()),
        "self_collision_proxy": float(collision_proxy.mean().item()),
        "policy_collision_guard_demand_rms": float(
            torch.sqrt(torch.mean(torch.square(guard_demand))).item()
        ),
        "policy_wrapper_rejected_demand_rms": float(
            torch.sqrt(torch.mean(torch.square(wrapper_rejected))).item()
        ),
        "robot_body_barrier_proximity_rms": float(
            torch.sqrt(torch.mean(torch.square(1.0 - arm_barrier_scale))).item()
        ),
        "robot_body_barrier_hard_violation_rate": float(
            (arm_barrier_scale <= 1.0e-6).to(torch.float32).mean().item()
        ),
        "joint_acceleration_rms_rad_s2": float(torch.sqrt(torch.mean(torch.square(joint_acceleration))).item()),
        "joint_jerk_rms_rad_s3": float(torch.sqrt(torch.mean(torch.square(joint_jerk))).item()),
        "torque_rms_nm": float(
            torch.sqrt(torch.mean(torch.square(robot.data.applied_torque[:, upper_ids]))).item()
        ),
        "foot_slip_speed_m_s": float(foot_slip.item()),
        "stationary_joint_velocity_rms_rad_s": float(stationary_velocity.item()),
        "stationary_action_rate_rms": float(stationary_action_rate.item()),
        "stationary_sample_fraction": float(stationary.to(torch.float32).mean().item()),
        "lower_body_position_rmse_rad": float(
            torch.sqrt(
                torch.mean(
                    torch.square(
                        robot.data.joint_pos[:, lower_ids]
                        - robot.data.default_joint_pos[:, lower_ids]
                    )
                )
            ).item()
        ),
        "lower_body_velocity_rms_rad_s": float(
            torch.sqrt(torch.mean(torch.square(robot.data.joint_vel[:, lower_ids]))).item()
        ),
    }


def _validation_rollout(
    env,
    policy,
    steps: int,
    seed: int,
    *,
    normalizer=None,
    observation_delay_steps: int = 0,
    observation_noise_std: float = 0.0,
) -> tuple[dict[str, float], dict]:
    command = env.unwrapped.command_manager.get_term("motion")
    action_term = env.unwrapped.action_manager.get_term("residual_joint_position")
    robot = env.unwrapped.scene["robot"]
    upper_ids = torch.as_tensor(
        [robot.joint_names.index(name) for name in UPPER_POLICY_JOINTS],
        dtype=torch.long,
        device=robot.device,
    )
    totals: dict[str, float] = {}
    maxima: dict[str, float] = {"action_abs_max": 0.0}
    raw_abs_chunks: list[np.ndarray] = []
    saturation_by_joint = np.zeros(len(UPPER_POLICY_JOINTS), dtype=np.float64)
    sample_count = 0
    clip_l1_sum = 0.0
    ood_count = 0
    ood_total = 0
    first_second_applied_delta_max = 0.0
    first_second_target_delta_max_rad = 0.0
    # RSL-RL collects the final training rollout under inference mode, so some
    # Isaac manager metric buffers are inference tensors.  Keep reset and the
    # complete validation replay in the same mode instead of mutating those
    # buffers after leaving it.
    with torch.inference_mode():
        command.set_active_split("validation")
        torch.manual_seed(seed)
        obs, _ = env.reset()
        previous = torch.zeros((env.num_envs, env.num_actions), dtype=torch.float32, device=env.device)
        previous_acceleration = robot.data.joint_acc[:, upper_ids].clone()
        previous_target = action_term.processed_actions.clone()
        observation_history = [obs.clone() for _ in range(observation_delay_steps + 1)]
        for step_index in range(steps):
            policy_obs = observation_history[0]
            if observation_noise_std > 0.0:
                policy_obs = policy_obs + observation_noise_std * torch.randn_like(policy_obs)
            actions = torch.zeros_like(previous) if policy is None else policy(policy_obs)
            obs, reward, _, _ = env.step(actions)
            observation_history.append(obs.clone())
            observation_history.pop(0)
            bounded = action_term.bounded_actions.clone()
            current = _tracking_metrics(
                env, bounded, previous, reward, previous_acceleration
            )
            for name, value in current.items():
                if name in maxima:
                    maxima[name] = max(maxima[name], value)
                else:
                    totals[name] = totals.get(name, 0.0) + value
            raw_abs = torch.abs(actions).detach().cpu().numpy()
            raw_abs_chunks.append(raw_abs.reshape(-1))
            saturation = (raw_abs > 1.0)
            saturation_by_joint += saturation.sum(axis=0)
            sample_count += raw_abs.shape[0]
            clip_l1_sum += float(torch.abs(action_term.clip_delta).sum().item())
            if normalizer is not None and hasattr(normalizer, "mean"):
                z = torch.abs((obs - normalizer.mean) / (normalizer.std + 1.0e-2))
                ood_count += int((z > 6.0).sum().item())
                ood_total += int(z.numel())
            if step_index < round(1.0 / float(env.unwrapped.step_dt)):
                first_second_applied_delta_max = max(
                    first_second_applied_delta_max,
                    float(torch.max(torch.abs(bounded - previous)).item()),
                )
                first_second_target_delta_max_rad = max(
                    first_second_target_delta_max_rad,
                    float(torch.max(torch.abs(action_term.processed_actions - previous_target)).item()),
                )
            previous = bounded
            previous_acceleration = robot.data.joint_acc[:, upper_ids].clone()
            previous_target = action_term.processed_actions.clone()
    result = {name: value / steps for name, value in totals.items()}
    result.update(maxima)
    all_raw_abs = np.concatenate(raw_abs_chunks) if raw_abs_chunks else np.zeros(1)
    result.update(
        {
            "raw_action_p50": float(np.quantile(all_raw_abs, 0.50)),
            "raw_action_p90": float(np.quantile(all_raw_abs, 0.90)),
            "raw_action_p99": float(np.quantile(all_raw_abs, 0.99)),
            "raw_action_abs_max": float(np.max(all_raw_abs)),
            "raw_action_saturation_rate": float(np.mean(all_raw_abs > 1.0)),
            "clipped_unclipped_action_l1_mean": float(
                clip_l1_sum / max(sample_count * len(UPPER_POLICY_JOINTS), 1)
            ),
            "observation_ood_fraction": float(ood_count / max(ood_total, 1)),
            "first_second_applied_action_delta_max": first_second_applied_delta_max,
            "first_second_q_target_delta_max_rad": first_second_target_delta_max_rad,
        }
    )
    diagnostics = {
        "joint_names": list(UPPER_POLICY_JOINTS),
        "raw_action_saturation_rate_by_joint": (
            saturation_by_joint / max(sample_count, 1)
        ).astype(float).tolist(),
        "raw_action_quantiles_abs": {
            "p50": result["raw_action_p50"],
            "p90": result["raw_action_p90"],
            "p99": result["raw_action_p99"],
            "max": result["raw_action_abs_max"],
        },
        "clipped_unclipped_l1_mean": result["clipped_unclipped_action_l1_mean"],
    }
    return result, diagnostics


def _wrapper_contract_check(seed: int) -> dict[str, float | str]:
    rng = np.random.default_rng(seed)
    raw = rng.normal(size=(32, len(UPPER_POLICY_JOINTS))).astype(np.float32)
    previous = rng.uniform(-0.5, 0.5, size=raw.shape).astype(np.float32)
    confidence = rng.uniform(0.0, 1.0, size=(raw.shape[0], 1)).astype(np.float32)
    safety_scale = rng.uniform(0.0, 1.0, size=raw.shape).astype(np.float32)
    support_error = rng.normal(scale=0.12, size=raw.shape).astype(np.float32)
    residual_scale = np.broadcast_to(residual_scale_vector(), raw.shape).copy()
    numpy_result = postprocess_action_numpy(
        raw,
        previous,
        confidence,
        safety_scale=safety_scale,
        residual_scale_rad=residual_scale,
        support_error_rad=support_error,
    )
    torch_result = postprocess_action_torch(
        torch.from_numpy(raw),
        torch.from_numpy(previous),
        torch.from_numpy(confidence),
        safety_scale=torch.from_numpy(safety_scale),
        residual_scale_rad=torch.from_numpy(residual_scale),
        support_error_rad=torch.from_numpy(support_error),
    )
    error = max(
        float(np.max(np.abs(numpy_result[name] - torch_result[name].cpu().numpy())))
        for name in (
            "clipped", "slewed", "applied", "clip_delta",
            "correction_rad", "support_gain",
        )
    )
    dropout = postprocess_action_numpy(raw, previous, np.zeros((raw.shape[0], 1), np.float32))
    return {
        "inference_wrapper_version": INFERENCE_WRAPPER_VERSION,
        "wrapper_max_abs_error": error,
        "fixed_double_support_contact_left": float(FIXED_DOUBLE_SUPPORT_CONTACT_STATE[0]),
        "fixed_double_support_contact_right": float(FIXED_DOUBLE_SUPPORT_CONTACT_STATE[1]),
        "dropout_correction_abs_max_rad": float(
            np.max(np.abs(dropout["correction_rad"]))
        ),
    }


def _atomic_copy(source: pathlib.Path, destination: pathlib.Path) -> None:
    temporary = destination.with_name(destination.name + f".{uuid.uuid4().hex}.tmp")
    shutil.copy2(source, temporary)
    os.replace(temporary, destination)


def _sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_provenance() -> dict:
    def run(*arguments: str) -> str:
        completed = subprocess.run(
            ["git", *arguments],
            cwd=PROJECT_ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
        return completed.stdout.strip()

    return {
        "commit": run("rev-parse", "HEAD") or None,
        "branch": run("branch", "--show-current") or None,
        "dirty_worktree": bool(run("status", "--porcelain")),
    }


def _package_version(distribution: str) -> str | None:
    try:
        return importlib.metadata.version(distribution)
    except importlib.metadata.PackageNotFoundError:
        return None


def _checkpoint_iteration(path: pathlib.Path) -> int:
    try:
        return int(path.stem.split("_")[-1])
    except ValueError:
        return -1


def _checkpoint_candidates(log_dir: pathlib.Path, maximum: int = 8) -> list[pathlib.Path]:
    checkpoints = sorted(log_dir.glob("model_*.pt"), key=_checkpoint_iteration)
    if not checkpoints:
        raise RuntimeError(f"training produced no checkpoints in {log_dir}")
    preferred_iterations = (1000, 1200, 1400, 1600, 1800, 2000, 2500, 2999)
    by_iteration = {_checkpoint_iteration(path): path for path in checkpoints}
    selected = [by_iteration[value] for value in preferred_iterations if value in by_iteration]
    selected.append(checkpoints[-1])
    # Resumed runs often start above the canonical sweep.  Add evenly spaced
    # checkpoints from this run without evaluating hundreds of near-duplicates.
    if len(set(selected)) < min(4, len(checkpoints)):
        indexes = np.linspace(0, len(checkpoints) - 1, num=min(maximum, len(checkpoints)), dtype=int)
        selected.extend(checkpoints[int(index)] for index in indexes)
    return sorted(set(selected), key=_checkpoint_iteration)[-maximum:]


def _load_validation_checkpoint(runner, checkpoint: pathlib.Path) -> None:
    """Load after RSL-RL's inference-mode final rollout without tensor errors."""

    with torch.inference_mode():
        runner.load(str(checkpoint), load_optimizer=False)


def _normalizer_summary(normalizer) -> dict:
    if not hasattr(normalizer, "mean"):
        return {"enabled": False}
    mean = normalizer.mean.detach().cpu().numpy().astype(np.float64)
    std = normalizer.std.detach().cpu().numpy().astype(np.float64)
    return {
        "enabled": True,
        "count": int(getattr(normalizer, "count", torch.tensor(0)).item()),
        "mean_min": float(np.min(mean)),
        "mean_max": float(np.max(mean)),
        "std_min": float(np.min(std)),
        "std_p50": float(np.median(std)),
        "std_max": float(np.max(std)),
        # Indices 74:76 are the fixed contact contract in the 88-D schema.
        "foot_contact_mean": mean[74:76].astype(float).tolist(),
        "foot_contact_std": std[74:76].astype(float).tolist(),
    }


def _reward_weights(env_cfg) -> dict[str, float]:
    result = {}
    for name, value in vars(env_cfg.rewards).items():
        if not name.startswith("_") and hasattr(value, "weight"):
            result[name] = float(value.weight)
    return result


@hydra_task_config(TASK_ID, "rsl_rl_cfg_entry_point")
def train(env_cfg: ManagerBasedRLEnvCfg, agent_cfg: RslRlOnPolicyRunnerCfg):
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs
    agent_cfg.max_iterations = args_cli.max_iterations
    if args_cli.seed is not None:
        agent_cfg.seed = args_cli.seed
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    log_root = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
    log_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if agent_cfg.run_name:
        log_name += f"_{agent_cfg.run_name}"
    log_dir = os.path.join(log_root, log_name)
    print(f"[INFO] Logging experiment in: {log_dir}")

    resume_path = None
    if args_cli.resume_checkpoint is not None:
        resume_path = str(args_cli.resume_checkpoint.expanduser().resolve())
        if not pathlib.Path(resume_path).is_file():
            raise FileNotFoundError(f"resume checkpoint not found: {resume_path}")
    elif agent_cfg.resume:
        resume_path = get_checkpoint_path(log_root, agent_cfg.load_run, agent_cfg.load_checkpoint)

    env = gym.make(TASK_ID, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "train"),
            "step_trigger": lambda step: step % args_cli.video_interval == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
    runner.add_git_repo_to_log(__file__)
    if resume_path:
        print(f"[CONTINUAL TRAINING] Loading prior accepted knowledge: {resume_path}", flush=True)
        runner.load(resume_path)

    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)
    export_deploy_cfg(env.unwrapped, log_dir)
    cfg_source = pathlib.Path(sys.modules[env_cfg.__class__.__module__].__file__)
    shutil.copy(cfg_source, os.path.join(log_dir, "params", cfg_source.name))
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

    manifest_path = os.environ.get("G1_REFERENCE_MANIFEST")
    evaluation = None
    selected_checkpoint = _checkpoint_candidates(pathlib.Path(log_dir))[-1]
    checkpoint_reports: list[dict] = []
    run_export_dir = pathlib.Path(log_dir) / "exported"
    run_export_dir.mkdir(parents=True, exist_ok=True)
    if manifest_path:
        manifest = json.loads(pathlib.Path(manifest_path).read_text(encoding="utf-8-sig"))
        evaluation_seed = int(agent_cfg.seed if agent_cfg.seed is not None else 42) + 1701
        validation_steps = max(50, int(args_cli.validation_steps))
        baseline_metrics, _ = _validation_rollout(
            env, None, max(50, int(args_cli.validation_steps)), evaluation_seed
        )
        by_id = {str(clip["id"]): clip for clip in manifest["clips"]}
        train_session_ids = sorted(
            {str(by_id[clip_id].get("session_id", "single_session")) for clip_id in manifest["splits"]["train"]}
        )
        validation_session_ids = sorted(
            {str(by_id[clip_id].get("session_id", "single_session")) for clip_id in manifest["splits"]["validation"]}
        )
        contract = _wrapper_contract_check(evaluation_seed)
        nominal_by_checkpoint: dict[str, tuple[dict[str, float], dict]] = {}
        for checkpoint in _checkpoint_candidates(pathlib.Path(log_dir)):
            _load_validation_checkpoint(runner, checkpoint)
            inference_policy = runner.get_inference_policy(device=env.device)
            policy_metrics, action_diagnostics = _validation_rollout(
                env,
                inference_policy,
                validation_steps,
                evaluation_seed,
                normalizer=runner.obs_normalizer,
            )
            nominal_by_checkpoint[checkpoint.name] = (policy_metrics, action_diagnostics)
            report = build_evaluation_report(
                manifest_path=str(pathlib.Path(manifest_path).resolve()),
                train_clip_ids=list(manifest["splits"]["train"]),
                validation_clip_ids=list(manifest["splits"]["validation"]),
                train_session_ids=train_session_ids,
                validation_session_ids=validation_session_ids,
                baseline=baseline_metrics,
                policy=policy_metrics,
                steps=validation_steps,
                num_envs=env.num_envs,
                training_run=str(pathlib.Path(log_dir).resolve()),
                checkpoint_name=checkpoint.name,
                checkpoint_iteration=_checkpoint_iteration(checkpoint),
                stress={"dropout_correction_abs_max_rad": float(contract["dropout_correction_abs_max_rad"])},
                contract=contract,
                action_diagnostics=action_diagnostics,
            )
            checkpoint_reports.append(report)
            print(
                f"[CHECKPOINT] {checkpoint.name} accepted={report['accepted']} "
                f"tracking={report['composite_tracking_error']['policy']:.5f} "
                f"acc={policy_metrics['joint_acceleration_rms_rad_s2']:.3f} "
                f"torque={policy_metrics['torque_rms_nm']:.3f} "
                f"sat={100.0 * policy_metrics['raw_action_saturation_rate']:.2f}%",
                flush=True,
            )
        selected_summary = select_pareto_report(checkpoint_reports)
        selected_checkpoint = pathlib.Path(log_dir) / selected_summary["checkpoint"]["name"]
        _load_validation_checkpoint(runner, selected_checkpoint)
        selected_policy = runner.get_inference_policy(device=env.device)
        selected_metrics, selected_diagnostics = nominal_by_checkpoint[selected_checkpoint.name]
        stress_metrics, _ = _validation_rollout(
            env,
            selected_policy,
            validation_steps,
            evaluation_seed,
            normalizer=runner.obs_normalizer,
            observation_delay_steps=2,
            observation_noise_std=0.01,
        )
        stress = {
            "composite_tracking_error": composite_tracking_error(stress_metrics),
            "delayed_noisy_joint_rmse_rad": stress_metrics["joint_rmse_rad"],
            "delayed_noisy_action_saturation_rate": stress_metrics["raw_action_saturation_rate"],
            "dropout_correction_abs_max_rad": float(contract["dropout_correction_abs_max_rad"]),
        }
        evaluation = build_evaluation_report(
            manifest_path=str(pathlib.Path(manifest_path).resolve()),
            train_clip_ids=list(manifest["splits"]["train"]),
            validation_clip_ids=list(manifest["splits"]["validation"]),
            train_session_ids=train_session_ids,
            validation_session_ids=validation_session_ids,
            baseline=baseline_metrics,
            policy=selected_metrics,
            steps=validation_steps,
            num_envs=env.num_envs,
            training_run=str(pathlib.Path(log_dir).resolve()),
            checkpoint_name=selected_checkpoint.name,
            checkpoint_iteration=_checkpoint_iteration(selected_checkpoint),
            stress=stress,
            contract=contract,
            action_diagnostics=selected_diagnostics,
        )
        # Keep the complete sweep; the final checkpoint is no longer silently
        # assumed to be the best deployment model.
        (run_export_dir / "checkpoint_sweep.json").write_text(
            json.dumps(
                {
                    "schema": "g1_reference_policy_checkpoint_sweep/v1",
                    "selected_checkpoint": selected_checkpoint.name,
                    "reports": checkpoint_reports,
                },
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        evaluation_json = run_export_dir / "policy_evaluation.json"
        evaluation_md = run_export_dir / "policy_evaluation.md"
        evaluation_json.write_text(
            json.dumps(evaluation, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        evaluation_md.write_text(report_markdown(evaluation), encoding="utf-8")
        print(
            f"[VALIDATION] accepted={evaluation['accepted']} "
            f"contribution={evaluation['policy_strength_percent']:+.3f}% "
            f"report={evaluation_md}",
            flush=True,
        )

    # Export only the selected Pareto checkpoint together with its empirical
    # normalizer.  This is the exact raw actor consumed by the shared live
    # bounded-residual wrapper.
    _load_validation_checkpoint(runner, selected_checkpoint)
    export_policy_as_onnx(
        runner.alg.policy,
        str(run_export_dir),
        normalizer=runner.obs_normalizer,
        filename="policy.onnx",
    )
    metadata = policy_metadata()
    manifest_file = pathlib.Path(manifest_path).resolve() if manifest_path else None
    manifest = (
        json.loads(manifest_file.read_text(encoding="utf-8-sig")) if manifest_file else None
    )
    train_clip_ids = list(manifest["splits"]["train"]) if manifest else []
    by_id = {str(clip["id"]): clip for clip in manifest["clips"]} if manifest else {}
    training_motions = [str(pathlib.Path(by_id[clip_id]["reference_npz"]).resolve()) for clip_id in train_clip_ids]
    onnx_path = run_export_dir / "policy.onnx"
    metadata.update(
        {
            "training_motion": training_motions[0] if training_motions else str(pathlib.Path(os.environ["G1_REFERENCE_MOTION"]).resolve()),
            "training_motions": training_motions,
            "training_clip_ids": train_clip_ids,
            "training_manifest": str(manifest_file) if manifest_file else None,
            "training_run": str(pathlib.Path(log_dir).resolve()),
            "additional_iterations": int(agent_cfg.max_iterations),
            "resume_checkpoint": str(pathlib.Path(resume_path).resolve()) if resume_path else None,
            "selected_checkpoint": selected_checkpoint.name,
            "selected_checkpoint_iteration": _checkpoint_iteration(selected_checkpoint),
            "deployment_id": uuid.uuid4().hex,
            "provenance": {
                "onnx_sha256": _sha256(onnx_path),
                "dataset_manifest_sha256": _sha256(manifest_file) if manifest_file else None,
                "checkpoint_sha256": _sha256(selected_checkpoint),
                "resume_checkpoint_sha256": _sha256(pathlib.Path(resume_path)) if resume_path else None,
                "git": _git_provenance(),
                "versions": {
                    "isaac_sim": _package_version("isaacsim"),
                    "isaac_lab": _package_version("isaaclab"),
                    "rsl_rl": _package_version("rsl-rl-lib"),
                    "torch": torch.__version__,
                },
                "random_seed": int(agent_cfg.seed),
                "reward_weights": _reward_weights(env_cfg),
                "domain_randomization": str(env_cfg.events),
                "observation_normalizer": _normalizer_summary(runner.obs_normalizer),
                "policy_inference_wrapper_version": INFERENCE_WRAPPER_VERSION,
            },
            "validation": (
                {
                    "accepted": bool(evaluation["accepted"]),
                    "policy_strength_percent": float(evaluation["policy_strength_percent"]),
                    "report": "policy_evaluation.json",
                    "validation_clip_ids": list(evaluation["validation_clip_ids"]),
                    "validation_session_ids": list(evaluation["validation_session_ids"]),
                    "checkpoint_sweep": "checkpoint_sweep.json",
                }
                if evaluation is not None
                else {"accepted": False, "reason": "single_clip_has_no_held_out_validation"}
            ),
        }
    )
    run_metadata = run_export_dir / "policy_metadata.json"
    run_metadata.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    stable_dir = args_cli.policy_output_dir.expanduser().resolve()
    stable_dir.mkdir(parents=True, exist_ok=True)
    if evaluation is not None and evaluation["accepted"]:
        _atomic_copy(run_export_dir / "policy_evaluation.json", stable_dir / "policy_evaluation.json")
        _atomic_copy(run_export_dir / "policy_evaluation.md", stable_dir / "policy_evaluation.md")
        _atomic_copy(run_export_dir / "policy.onnx", stable_dir / "policy.onnx")
        _atomic_copy(run_export_dir / "checkpoint_sweep.json", stable_dir / "checkpoint_sweep.json")
        _atomic_copy(selected_checkpoint, stable_dir / "policy_checkpoint.pt")
        # Metadata is replaced last: live P reload sees a complete deployment.
        _atomic_copy(run_metadata, stable_dir / "policy_metadata.json")
        print(f"[POLICY READY] ONNX: {stable_dir / 'policy.onnx'}", flush=True)
        print(f"[POLICY READY] Metadata: {stable_dir / 'policy_metadata.json'}", flush=True)
        print(f"[POLICY READY] Validation: {stable_dir / 'policy_evaluation.md'}", flush=True)
    else:
        print(
            "[POLICY REJECTED] Held-out validation did not pass; the existing live P policy was not replaced. "
            f"Candidate remains in {run_export_dir}",
            flush=True,
        )
    env.close()


if __name__ == "__main__":
    try:
        train()
    finally:
        simulation_app.close()

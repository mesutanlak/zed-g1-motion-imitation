"""Tracking rewards and physical-feasibility penalties for G1."""

from __future__ import annotations

import torch

from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor
from isaaclab.utils.math import quat_error_magnitude

from .commands import ReferenceMotionCommand
from motion_pipeline.reference_policy import ACTION_CLIP, policy_collision_action_scale_torch


def _command(env, command_name: str) -> ReferenceMotionCommand:
    return env.command_manager.get_term(command_name)


def _confidence(command: ReferenceMotionCommand, floor: float = 0.0) -> torch.Tensor:
    return command.reference_confidence.clamp(min=floor, max=1.0)


def _body_ids(command: ReferenceMotionCommand, body_names: list[str]) -> list[int]:
    missing = [name for name in body_names if name not in command.cfg.body_names]
    if missing:
        raise ValueError(f"reward refers to unknown motion bodies: {missing}")
    return [command.cfg.body_names.index(name) for name in body_names]


def joint_tracking_exp(
    env,
    command_name: str,
    std: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    command = _command(env, command_name)
    robot = env.scene[asset_cfg.name]
    error = torch.mean(
        torch.square(command.joint_pos[:, asset_cfg.joint_ids] - robot.data.joint_pos[:, asset_cfg.joint_ids]), dim=-1
    )
    return _confidence(command) * torch.exp(-error / std**2)


def body_position_tracking_exp(
    env,
    command_name: str,
    body_names: list[str],
    std: float,
) -> torch.Tensor:
    command = _command(env, command_name)
    ids = _body_ids(command, body_names)
    error = torch.square(command.body_pos_relative_w[:, ids] - command.robot_body_pos_w[:, ids]).sum(dim=-1).mean(dim=-1)
    return _confidence(command) * torch.exp(-error / std**2)


def body_orientation_tracking_exp(
    env,
    command_name: str,
    body_names: list[str],
    std: float,
) -> torch.Tensor:
    command = _command(env, command_name)
    ids = _body_ids(command, body_names)
    error = torch.square(quat_error_magnitude(command.body_quat_relative_w[:, ids], command.robot_body_quat_w[:, ids])).mean(dim=-1)
    return _confidence(command) * torch.exp(-error / std**2)


def limb_direction_tracking_exp(
    env,
    command_name: str,
    segments: list[tuple[str, str]],
    std: float,
) -> torch.Tensor:
    """Track normalized limb directions, independent of human/robot limb length."""
    command = _command(env, command_name)
    errors = []
    for start_name, end_name in segments:
        start, end = _body_ids(command, [start_name, end_name])
        target = command.body_pos_relative_w[:, end] - command.body_pos_relative_w[:, start]
        actual = command.robot_body_pos_w[:, end] - command.robot_body_pos_w[:, start]
        target = torch.nn.functional.normalize(target, dim=-1, eps=1.0e-6)
        actual = torch.nn.functional.normalize(actual, dim=-1, eps=1.0e-6)
        # 1-dot is smooth and avoids acos instability near straight limbs.
        errors.append((1.0 - torch.sum(target * actual, dim=-1).clamp(-1.0, 1.0)) * 2.0)
    error = torch.stack(errors, dim=-1).mean(dim=-1)
    return _confidence(command) * torch.exp(-error / std**2)


def foot_slip_penalty(
    env,
    asset_cfg: SceneEntityCfg,
    sensor_cfg: SceneEntityCfg,
    threshold: float,
) -> torch.Tensor:
    robot = env.scene[asset_cfg.name]
    sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    forces = sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids]
    contact = torch.linalg.vector_norm(forces, dim=-1).amax(dim=1) > threshold
    planar_speed = torch.linalg.vector_norm(robot.data.body_lin_vel_w[:, asset_cfg.body_ids, :2], dim=-1)
    return torch.sum(contact * torch.square(planar_speed), dim=-1)


def self_collision_proximity(
    env,
    asset_cfg: SceneEntityCfg,
    body_pairs: list[tuple[str, str, float]],
) -> torch.Tensor:
    """GPU-friendly proxy for self collision using named link-origin margins.

    PhysX self-contact events are not exposed by the ordinary external contact
    sensor on every Isaac version. This dense margin penalty prevents arm-arm
    and arm-torso penetration before a hard self-contact occurs.
    """
    robot = env.scene[asset_cfg.name]
    result = torch.zeros(env.num_envs, device=robot.device)
    for first, second, margin in body_pairs:
        first_id = robot.body_names.index(first)
        second_id = robot.body_names.index(second)
        distance = torch.linalg.vector_norm(
            robot.data.body_pos_w[:, first_id] - robot.data.body_pos_w[:, second_id], dim=-1
        )
        # Normalize by the requested clearance.  Squared metres made a 6 cm
        # torso penetration almost invisible beside the tracking rewards.
        violation = torch.clamp(
            (float(margin) - distance) / max(float(margin), 1.0e-6), min=0.0
        )
        result += torch.square(violation)
    return result


def policy_safety_guard_activation(
    env,
    action_name: str = "residual_joint_position",
) -> torch.Tensor:
    """Penalize action demand that the deployment collision guard must block."""

    action = env.action_manager.get_term(action_name)
    blocked_demand = action.clipped_actions * (1.0 - action.safety_scale)
    return torch.mean(torch.square(blocked_demand), dim=-1)


def policy_wrapper_rejected_demand(
    env,
    action_name: str = "residual_joint_position",
) -> torch.Tensor:
    """Penalize residual demand discarded by the live supportive wrapper.

    This includes collision projection, low-confidence gating and commands
    that would push away from the normal IK reference.  The actor therefore
    learns to emit useful corrections instead of relying on deployment clips.
    """

    action = env.action_manager.get_term(action_name)
    rejected = action.pre_safety_slewed - action.bounded_actions
    return torch.mean(torch.square(rejected), dim=-1)


def raw_action_saturation_penalty(
    env,
    action_name: str = "residual_joint_position",
) -> torch.Tensor:
    """Expose unclipped ONNX saturation to PPO instead of hiding it."""

    action = env.action_manager.get_term(action_name)
    excess = torch.relu(torch.abs(action.raw_actions) - float(ACTION_CLIP))
    return torch.mean(torch.square(excess), dim=-1)


def robot_body_barrier_violation(
    env,
    command_name: str,
) -> torch.Tensor:
    """Dense capsule penalty for forearm/hand proximity to the G1 trunk."""

    command = _command(env, command_name)
    scale = policy_collision_action_scale_torch(
        command.robot_body_pos_w,
        command.robot_body_pos_w,
        list(command.cfg.body_names),
    )
    # Every action in one arm receives the same minimum geometry scale.
    left = scale[:, 1]
    right = scale[:, 6]
    return torch.square(1.0 - left) + torch.square(1.0 - right)

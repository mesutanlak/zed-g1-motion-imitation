"""Policy observations for confidence-aware reference tracking."""

from __future__ import annotations

import math
import torch

from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor
from isaaclab.utils.math import quat_apply, quat_inv

from .commands import ReferenceMotionCommand


def _command(env, command_name: str) -> ReferenceMotionCommand:
    return env.command_manager.get_term(command_name)


def q_ref_minus_q(env, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    robot = env.scene[asset_cfg.name]
    command = _command(env, command_name)
    return command.joint_pos[:, asset_cfg.joint_ids] - robot.data.joint_pos[:, asset_cfg.joint_ids]


def qd_ref_minus_qd(env, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    robot = env.scene[asset_cfg.name]
    command = _command(env, command_name)
    return command.joint_vel[:, asset_cfg.joint_ids] - robot.data.joint_vel[:, asset_cfg.joint_ids]


def body_reference_error(env, command_name: str, body_names: list[str] | None = None) -> torch.Tensor:
    """Selected target-minus-robot body positions expressed in the robot anchor frame."""
    command = _command(env, command_name)
    ids = [
        index
        for index, name in enumerate(command.cfg.body_names)
        if body_names is None or name in body_names
    ]
    error_w = command.body_pos_relative_w[:, ids] - command.robot_body_pos_w[:, ids]
    anchor_inv = quat_inv(command.robot_anchor_quat_w)[:, None, :].expand(-1, len(ids), -1)
    return quat_apply(anchor_inv, error_w).reshape(env.num_envs, -1)


def foot_contact_state(
    env,
    sensor_cfg: SceneEntityCfg,
    threshold: float = 10.0,
) -> torch.Tensor:
    sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    forces = sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids]
    contact = torch.linalg.vector_norm(forces, dim=-1).amax(dim=1) > threshold
    return contact.to(dtype=torch.float32)


def fixed_double_support_contact_state(env) -> torch.Tensor:
    """Stable contact encoding shared with fixed-root live deployment.

    See ``motion_pipeline.reference_policy.FIXED_DOUBLE_SUPPORT_CONTACT_STATE``.
    The current schema intentionally preserves the zero encoding learned by
    existing checkpoints; changing it requires an observation schema bump.
    """

    return torch.zeros((env.num_envs, 2), dtype=torch.float32, device=env.device)


def bounded_previous_action(env, action_name: str = "residual_joint_position") -> torch.Tensor:
    """Action that actually reached the residual wrapper, not raw actor output."""

    return env.action_manager.get_term(action_name).bounded_actions


def reference_phase(env, command_name: str) -> torch.Tensor:
    """Return sin/cos phase to avoid a discontinuity at a cyclic clip boundary."""
    phase = _command(env, command_name).phase
    angle = (2.0 * math.pi) * phase
    return torch.stack((torch.sin(angle), torch.cos(angle)), dim=-1)


def reference_confidence(env, command_name: str) -> torch.Tensor:
    return _command(env, command_name).reference_confidence.unsqueeze(-1)


def reference_foot_contact(env, command_name: str) -> torch.Tensor:
    """Privileged/reference contact state saved in the offline motion clip."""
    return _command(env, command_name).reference_foot_contact

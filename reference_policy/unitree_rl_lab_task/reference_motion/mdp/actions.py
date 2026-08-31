"""Residual joint-position action centered on the current reference pose."""

from __future__ import annotations

import torch

from motion_pipeline.reference_policy import (
    ACTION_CLIP,
    ACTION_SLEW_PER_STEP,
    FULL_TRACKING_CONFIDENCE,
    LOWER_POLICY_JOINTS,
    policy_collision_action_scale_torch,
    postprocess_action_torch,
)

from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg
from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction
from isaaclab.utils import configclass

from .commands import ReferenceMotionCommand


class ResidualReferenceJointPositionAction(JointPositionAction):
    """Apply a small policy residual around ``q_ref`` instead of the default pose."""

    cfg: "ResidualReferenceJointPositionActionCfg"

    def __init__(self, cfg: "ResidualReferenceJointPositionActionCfg", env):
        super().__init__(cfg, env)
        self._command: ReferenceMotionCommand = env.command_manager.get_term(cfg.command_name)
        self._last_safe_reference = self._asset.data.default_joint_pos[:, self._joint_ids].clone()
        self._slewed_actions = torch.zeros_like(self._raw_actions)
        self._bounded_actions = torch.zeros_like(self._raw_actions)
        self._clipped_actions = torch.zeros_like(self._raw_actions)
        self._clip_delta = torch.zeros_like(self._raw_actions)
        self._saturation_mask = torch.zeros_like(self._raw_actions, dtype=torch.bool)
        self._safety_scale = torch.ones_like(self._raw_actions)
        self._support_gain = torch.ones_like(self._raw_actions)
        self._pre_safety_slewed = torch.zeros_like(self._raw_actions)
        self._fixed_joint_ids, self._fixed_joint_names = self._asset.find_joints(
            list(cfg.fixed_joint_names), preserve_order=True
        )
        self._fixed_joint_target = self._asset.data.default_joint_pos[
            :, self._fixed_joint_ids
        ].clone()
        self._fixed_joint_velocity = torch.zeros_like(self._fixed_joint_target)

    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        reference = self._command.joint_pos[:, self._joint_ids]
        confidence = self._command.reference_confidence.unsqueeze(-1)
        safe = confidence >= self.cfg.hold_below_confidence
        # RSL-RL may collect rollouts under torch.inference_mode().  Rebinding
        # this persistent state to the result of torch.where would turn it into
        # an inference tensor, which then cannot be reset outside that context
        # during held-out validation.  Keep the original normal tensor alive.
        self._last_safe_reference.copy_(
            torch.where(safe, reference, self._last_safe_reference)
        )

        safety_scale = policy_collision_action_scale_torch(
            self._command.body_pos_relative_w,
            self._command.robot_body_pos_w,
            list(self._command.cfg.body_names),
        )
        wrapped = postprocess_action_torch(
            self._raw_actions,
            self._slewed_actions,
            confidence,
            action_clip=self.cfg.action_clip,
            slew_per_step=self.cfg.action_slew_per_step,
            full_tracking_confidence=self.cfg.full_tracking_confidence,
            hold_below_confidence=self.cfg.hold_below_confidence,
            safety_scale=safety_scale,
            residual_scale_rad=self._scale,
            support_error_rad=(
                self._last_safe_reference
                - self._asset.data.joint_pos[:, self._joint_ids]
            ),
        )
        self._slewed_actions.copy_(wrapped["slewed"])
        self._bounded_actions.copy_(wrapped["applied"])
        self._clipped_actions.copy_(wrapped["clipped"])
        self._clip_delta.copy_(wrapped["clip_delta"])
        self._saturation_mask.copy_(wrapped["saturation_mask"])
        self._safety_scale.copy_(wrapped["safety_scale"])
        self._support_gain.copy_(wrapped["support_gain"])
        self._pre_safety_slewed.copy_(wrapped["pre_safety_slewed"])

        target = self._last_safe_reference + wrapped["correction_rad"]
        limits = self._asset.data.soft_joint_pos_limits[:, self._joint_ids]
        self._processed_actions = torch.clamp(target, min=limits[..., 0], max=limits[..., 1])

    def apply_actions(self):
        # Apply the learned upper-body target, then explicitly refresh the
        # nominal lower-body target at every physics step. This adds no actor
        # dimensions and cannot teach or deploy leg motion.
        super().apply_actions()
        self._asset.set_joint_position_target(
            self._fixed_joint_target, joint_ids=self._fixed_joint_ids
        )
        self._asset.set_joint_velocity_target(
            self._fixed_joint_velocity, joint_ids=self._fixed_joint_ids
        )

    @property
    def bounded_actions(self) -> torch.Tensor:
        return self._bounded_actions

    @property
    def clipped_actions(self) -> torch.Tensor:
        return self._clipped_actions

    @property
    def clip_delta(self) -> torch.Tensor:
        return self._clip_delta

    @property
    def saturation_mask(self) -> torch.Tensor:
        return self._saturation_mask

    @property
    def safety_scale(self) -> torch.Tensor:
        return self._safety_scale

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def support_gain(self) -> torch.Tensor:
        return self._support_gain

    @property
    def pre_safety_slewed(self) -> torch.Tensor:
        return self._pre_safety_slewed

    def reset(self, env_ids=None):
        if env_ids is None:
            env_ids = slice(None)
        super().reset(env_ids)
        self._last_safe_reference[env_ids] = self._asset.data.default_joint_pos[env_ids][:, self._joint_ids]
        self._slewed_actions[env_ids] = 0.0
        self._bounded_actions[env_ids] = 0.0
        self._clipped_actions[env_ids] = 0.0
        self._clip_delta[env_ids] = 0.0
        self._saturation_mask[env_ids] = False
        self._safety_scale[env_ids] = 1.0
        self._support_gain[env_ids] = 1.0
        self._pre_safety_slewed[env_ids] = 0.0
        self._fixed_joint_target[env_ids] = self._asset.data.default_joint_pos[
            env_ids
        ][:, self._fixed_joint_ids]
        self._fixed_joint_velocity[env_ids] = 0.0


@configclass
class ResidualReferenceJointPositionActionCfg(JointPositionActionCfg):
    class_type: type = ResidualReferenceJointPositionAction
    command_name: str = "motion"
    full_tracking_confidence: float = FULL_TRACKING_CONFIDENCE
    hold_below_confidence: float = 0.25
    action_clip: float = ACTION_CLIP
    action_slew_per_step: float = ACTION_SLEW_PER_STEP
    use_default_offset: bool = False
    fixed_joint_names: tuple[str, ...] = LOWER_POLICY_JOINTS

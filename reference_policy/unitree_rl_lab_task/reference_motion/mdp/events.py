"""Event terms specific to the reference-motion action contract."""

from __future__ import annotations

from typing import TYPE_CHECKING, Literal

import torch

from isaaclab.assets import Articulation
from isaaclab.envs.mdp.events import _randomize_prop_by_op
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


def randomize_joint_default_pos_for_action(
    env: "ManagerBasedEnv",
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    action_name: str,
    pos_distribution_params: tuple[float, float] | None = None,
    operation: Literal["add", "scale", "abs"] = "abs",
    distribution: Literal["uniform", "log_uniform", "gaussian"] = "uniform",
) -> None:
    """Randomize defaults and update the explicitly named joint action term.

    Unitree's generic helper hard-codes ``JointPositionAction``.  This task
    intentionally names its reference-centered action
    ``residual_joint_position``, so the action term must be selected by config.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    asset.data.default_joint_pos_nominal = torch.clone(asset.data.default_joint_pos[0])

    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=asset.device)
    joint_ids = (
        slice(None)
        if asset_cfg.joint_ids == slice(None)
        else torch.tensor(asset_cfg.joint_ids, dtype=torch.int, device=asset.device)
    )
    if pos_distribution_params is None:
        return

    positions = _randomize_prop_by_op(
        asset.data.default_joint_pos.to(asset.device).clone(),
        pos_distribution_params,
        env_ids,
        joint_ids,
        operation=operation,
        distribution=distribution,
    )[env_ids][:, joint_ids]

    indexed_env_ids = env_ids
    if env_ids != slice(None) and joint_ids != slice(None):
        indexed_env_ids = env_ids[:, None]
    asset.data.default_joint_pos[indexed_env_ids, joint_ids] = positions

    action = env.action_manager.get_term(action_name)
    # Reference-centered actions deliberately keep ``use_default_offset``
    # disabled, in which case Isaac Lab stores the unused offset as a scalar.
    if isinstance(getattr(action, "_offset", None), torch.Tensor):
        action._offset[indexed_env_ids, joint_ids] = positions

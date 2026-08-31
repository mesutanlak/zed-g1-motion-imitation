"""Gym registration for the G1 23-DOF reference-motion task."""

import gymnasium as gym


gym.register(
    id="Unitree-G1-23dof-Reference-Motion",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.tracking_env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{__name__}.tracking_env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.rsl_rl_ppo_cfg:ReferenceMotionPPORunnerCfg",
    },
)

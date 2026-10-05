"""Unitree G1 asset configurations for Isaac Lab 3.0 and Isaac Sim 6.1."""

from __future__ import annotations

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg


def _rigid_props(*, retain_accelerations: bool = False):
    return sim_utils.RigidBodyPropertiesCfg(
        disable_gravity=False,
        retain_accelerations=retain_accelerations,
        linear_damping=0.0,
        angular_damping=0.0,
        max_linear_velocity=1000.0,
        max_angular_velocity=1000.0,
        max_depenetration_velocity=1.0,
    )


def _articulation_props(*, self_collisions: bool = True):
    return sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=self_collisions,
        solver_position_iteration_count=8,
        solver_velocity_iteration_count=4,
    )


def UnitreeUsdFileCfg(*, usd_path: str):
    """Build the G1 USD spawner used by the native Ubuntu path."""

    return sim_utils.UsdFileCfg(
        usd_path=usd_path,
        activate_contact_sensors=True,
        rigid_props=_rigid_props(),
        articulation_props=_articulation_props(),
    )


def UnitreeUrdfFileCfg(
    *, asset_path: str, usd_dir: str, usd_file_name: str
):
    """Build an Isaac Lab 3.0 URDF importer configuration for G1."""

    return sim_utils.UrdfFileCfg(
        asset_path=asset_path,
        usd_dir=usd_dir,
        usd_file_name=usd_file_name,
        fix_base=False,
        activate_contact_sensors=True,
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=0.0,
                damping=0.0,
            )
        ),
        rigid_props=_rigid_props(),
        articulation_props=_articulation_props(),
    )


def g1_23dof_cfg() -> ArticulationCfg:
    """Return the official Unitree G1 23-DOF simulation configuration."""

    return ArticulationCfg(
        prim_path="/World/G1",
        # design_scene always replaces this placeholder with the selected
        # cached USD or the official Unitree URDF importer configuration.
        spawn=UnitreeUsdFileCfg(usd_path="unused.usd"),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.8),
            joint_pos={
                ".*_hip_pitch_joint": -0.1,
                ".*_knee_joint": 0.3,
                ".*_ankle_pitch_joint": -0.2,
                ".*_shoulder_pitch_joint": 0.3,
                "left_shoulder_roll_joint": 0.25,
                "right_shoulder_roll_joint": -0.25,
                ".*_elbow_joint": 0.97,
                "left_wrist_roll_joint": 0.15,
                "right_wrist_roll_joint": -0.15,
            },
            joint_vel={".*": 0.0},
        ),
        soft_joint_pos_limit_factor=0.9,
        actuators={
            "N7520-14.3": ImplicitActuatorCfg(
                joint_names_expr=[
                    ".*_hip_pitch_.*",
                    ".*_hip_yaw_.*",
                    "waist_yaw_joint",
                ],
                joint_effort_limit=88.0,
                joint_velocity_limit=32.0,
                stiffness={".*_hip_.*": 100.0, "waist_yaw_joint": 200.0},
                damping={".*_hip_.*": 2.0, "waist_yaw_joint": 5.0},
                armature=0.01,
            ),
            "N7520-22.5": ImplicitActuatorCfg(
                joint_names_expr=[".*_hip_roll_.*", ".*_knee_.*"],
                joint_effort_limit=139.0,
                joint_velocity_limit=20.0,
                stiffness={".*_hip_roll_.*": 100.0, ".*_knee_.*": 150.0},
                damping={".*_hip_roll_.*": 2.0, ".*_knee_.*": 4.0},
                armature=0.01,
            ),
            "N5020-16": ImplicitActuatorCfg(
                joint_names_expr=[
                    ".*_shoulder_.*",
                    ".*_elbow_.*",
                    ".*_wrist_roll_.*",
                ],
                joint_effort_limit=25.0,
                joint_velocity_limit=37.0,
                stiffness=40.0,
                damping=1.0,
                armature=0.01,
            ),
            "N5020-16-parallel": ImplicitActuatorCfg(
                joint_names_expr=[".*ankle.*"],
                joint_effort_limit=35.0,
                joint_velocity_limit=30.0,
                stiffness=40.0,
                damping=2.0,
                armature=0.01,
            ),
        },
    )


def g1_29dof_dex3_cfg(asset: Path) -> ArticulationCfg:
    """Return an Isaac Lab 3.0 configuration for Unitree's official Dex3 USD."""

    if not asset.is_file() or asset.stat().st_size < 1024:
        raise FileNotFoundError(f"Official Unitree G1-29 + Dex3 USD missing: {asset}")
    return ArticulationCfg(
        prim_path="/World/G1",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(asset),
            activate_contact_sensors=True,
            rigid_props=_rigid_props(retain_accelerations=True),
            articulation_props=_articulation_props(self_collisions=False),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.75),
            joint_pos={
                ".*_hip_yaw_joint": 0.0,
                ".*_hip_roll_joint": 0.0,
                ".*_hip_pitch_joint": -0.05,
                ".*_knee_joint": 0.2,
                ".*_ankle_pitch_joint": -0.15,
                ".*_ankle_roll_joint": 0.0,
                "waist_.*_joint": 0.0,
                ".*_shoulder_.*_joint": 0.0,
                ".*_elbow_joint": 0.0,
                ".*_wrist_.*_joint": 0.0,
                ".*_hand_.*_joint": 0.0,
            },
            joint_vel={".*": 0.0},
        ),
        soft_joint_pos_limit_factor=0.9,
        actuators={
            "body": ImplicitActuatorCfg(
                joint_names_expr=[
                    ".*_hip_.*_joint",
                    ".*_knee_joint",
                    ".*_ankle_.*_joint",
                    "waist_.*_joint",
                    ".*_shoulder_.*_joint",
                    ".*_elbow_joint",
                    ".*_wrist_.*_joint",
                ],
                joint_effort_limit=300.0,
                joint_velocity_limit=100.0,
                stiffness=40.0,
                damping=2.0,
                armature=0.01,
            ),
            "hands": ImplicitActuatorCfg(
                joint_names_expr=[".*_hand_.*_joint"],
                joint_effort_limit=300.0,
                joint_velocity_limit=100.0,
                stiffness=100.0,
                damping=10.0,
                armature=0.1,
            ),
        },
    )


UNITREE_G1_23DOF_CFG = g1_23dof_cfg()

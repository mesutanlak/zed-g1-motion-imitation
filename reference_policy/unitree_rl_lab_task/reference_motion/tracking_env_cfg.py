"""Isaac Lab environment for physically feasible G1 23-DOF motion tracking."""

from __future__ import annotations

import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from unitree_rl_lab.assets.robots.unitree import UNITREE_G1_23DOF_CFG

from motion_pipeline.reference_policy import (
    ACTION_CLIP,
    ACTION_SLEW_PER_STEP,
    FULL_TRACKING_CONFIDENCE,
    OBSERVATION_SCALES,
    RESIDUAL_SCALE_RAD_BY_JOINT,
    TRACKED_BODIES as DEPLOYMENT_TRACKED_BODIES,
    UPPER_POLICY_JOINTS,
)

from . import mdp


MOTION_FILE = os.environ.get("G1_REFERENCE_MOTION", str(Path(__file__).with_name("reference_motion.npz")))
MOTION_MANIFEST = os.environ.get("G1_REFERENCE_MANIFEST")
USD_FILE = os.environ.get("G1_23DOF_USD", r"C:\g1il\cache\g1_23dof\g1_23dof_rev_1_0.usd")

TRACKED_BODIES = list(DEPLOYMENT_TRACKED_BODIES)

ARM_SEGMENTS = [
    ("left_shoulder_pitch_link", "left_elbow_link"),
    ("left_elbow_link", "left_wrist_roll_rubber_hand"),
    ("right_shoulder_pitch_link", "right_elbow_link"),
    ("right_elbow_link", "right_wrist_roll_rubber_hand"),
]

ROBOT_CFG = UNITREE_G1_23DOF_CFG.copy()
ROBOT_CFG.prim_path = "{ENV_REGEX_NS}/Robot"
ROBOT_CFG.spawn.usd_path = USD_FILE
ROBOT_CFG.spawn.articulation_props.fix_root_link = True
ROBOT_CFG.soft_joint_pos_limit_factor = 0.9


@configclass
class RobotSceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=0.9,
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path="{NVIDIA_NUCLEUS_DIR}/Materials/Base/Architecture/Shingles_01.mdl",
            project_uvw=True,
        ),
    )
    robot: ArticulationCfg = ROBOT_CFG
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(color=(0.13, 0.13, 0.13), intensity=1000.0),
    )
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*",
        history_length=3,
        track_air_time=True,
        force_threshold=10.0,
        debug_vis=False,
    )


@configclass
class CommandsCfg:
    motion = mdp.ReferenceMotionCommandCfg(
        asset_name="robot",
        motion_file=MOTION_FILE,
        motion_manifest=MOTION_MANIFEST,
        active_split="train",
        anchor_body_name="pelvis",
        body_names=TRACKED_BODIES,
        resampling_time_range=(1.0e9, 1.0e9),
        debug_vis=False,
        # The operator stands in place and the live lower body is nominal.
        # Root disturbances would make the policy spend capacity on balance.
        pose_range={key: (0.0, 0.0) for key in ("x", "y", "z", "roll", "pitch", "yaw")},
        velocity_range={key: (0.0, 0.0) for key in ("x", "y", "z", "roll", "pitch", "yaw")},
        joint_position_range=(0.0, 0.0),
        adaptive_kernel_size=3,
        adaptive_uniform_ratio=0.15,
    )


@configclass
class ActionsCfg:
    residual_joint_position = mdp.ResidualReferenceJointPositionActionCfg(
        asset_name="robot",
        joint_names=list(UPPER_POLICY_JOINTS),
        preserve_order=True,
        scale=dict(RESIDUAL_SCALE_RAD_BY_JOINT),
        use_default_offset=False,
        command_name="motion",
        full_tracking_confidence=FULL_TRACKING_CONFIDENCE,
        hold_below_confidence=0.25,
        action_clip=ACTION_CLIP,
        action_slew_per_step=ACTION_SLEW_PER_STEP,
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        # The order below is the public deployment contract.
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.02, n_max=0.02))
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            scale=OBSERVATION_SCALES["base_ang_vel"],
            noise=Unoise(n_min=-0.1, n_max=0.1),
        )
        q_minus_q_default = ObsTerm(
            func=mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot", joint_names=list(UPPER_POLICY_JOINTS), preserve_order=True
                )
            },
            noise=Unoise(n_min=-0.005, n_max=0.005),
        )
        qd = ObsTerm(
            func=mdp.joint_vel_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot", joint_names=list(UPPER_POLICY_JOINTS), preserve_order=True
                )
            },
            scale=OBSERVATION_SCALES["qd"],
            noise=Unoise(n_min=-0.2, n_max=0.2),
        )
        q_ref_minus_q = ObsTerm(
            func=mdp.q_ref_minus_q,
            params={
                "command_name": "motion",
                "asset_cfg": SceneEntityCfg(
                    "robot", joint_names=list(UPPER_POLICY_JOINTS), preserve_order=True
                ),
            },
        )
        qd_ref_minus_qd = ObsTerm(
            func=mdp.qd_ref_minus_qd,
            params={
                "command_name": "motion",
                "asset_cfg": SceneEntityCfg(
                    "robot", joint_names=list(UPPER_POLICY_JOINTS), preserve_order=True
                ),
            },
            scale=OBSERVATION_SCALES["qd_ref_minus_qd"],
        )
        body_reference_error = ObsTerm(
            func=mdp.body_reference_error,
            params={"command_name": "motion", "body_names": TRACKED_BODIES},
            scale=OBSERVATION_SCALES["body_reference_error"],
        )
        foot_contact_state = ObsTerm(
            func=mdp.fixed_double_support_contact_state,
        )
        previous_action = ObsTerm(
            func=mdp.bounded_previous_action,
            params={"action_name": "residual_joint_position"},
        )
        reference_confidence = ObsTerm(func=mdp.reference_confidence, params={"command_name": "motion"})

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(PolicyCfg):
        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class EventCfg:
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.6, 1.4),
            "dynamic_friction_range": (0.5, 1.2),
            "restitution_range": (0.0, 0.1),
            "num_buckets": 64,
        },
    )
    add_joint_default_pos = EventTerm(
        func=mdp.randomize_joint_default_pos_for_action,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", joint_names=list(UPPER_POLICY_JOINTS), preserve_order=True
            ),
            "action_name": "residual_joint_position",
            "pos_distribution_params": (-0.01, 0.01),
            "operation": "add",
        },
    )


@configclass
class RewardsCfg:
    joint_tracking = RewTerm(
        func=mdp.joint_tracking_exp,
        weight=5.0,
        params={
            "command_name": "motion",
            "std": 0.20,
            "asset_cfg": SceneEntityCfg(
                "robot", joint_names=list(UPPER_POLICY_JOINTS), preserve_order=True
            ),
        },
    )
    hand_position = RewTerm(
        func=mdp.body_position_tracking_exp,
        weight=2.0,
        params={
            "command_name": "motion",
            "body_names": ["left_wrist_roll_rubber_hand", "right_wrist_roll_rubber_hand"],
            "std": 0.08,
        },
    )
    elbow_position = RewTerm(
        func=mdp.body_position_tracking_exp,
        weight=1.5,
        params={
            "command_name": "motion",
            "body_names": ["left_elbow_link", "right_elbow_link"],
            "std": 0.07,
        },
    )
    limb_direction = RewTerm(
        func=mdp.limb_direction_tracking_exp,
        weight=2.5,
        params={"command_name": "motion", "segments": ARM_SEGMENTS, "std": 0.20},
    )
    orientation = RewTerm(
        func=mdp.body_orientation_tracking_exp,
        weight=1.0,
        params={
            "command_name": "motion",
            "body_names": ["pelvis", "torso_link", "left_elbow_link", "right_elbow_link"],
            "std": 0.30,
        },
    )

    torque = RewTerm(
        func=mdp.joint_torques_l2,
        weight=-2.0e-5,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", joint_names=list(UPPER_POLICY_JOINTS), preserve_order=True
            )
        },
    )
    joint_acceleration = RewTerm(
        func=mdp.joint_acc_l2,
        weight=-2.5e-7,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", joint_names=list(UPPER_POLICY_JOINTS), preserve_order=True
            )
        },
    )
    # Residual is assistance, not a second pose generator. Prefer zero action
    # unless a small lead demonstrably improves tracking of the normal IK.
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.040)
    action_magnitude = RewTerm(func=mdp.action_l2, weight=-0.080)
    joint_limit = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-3.0,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", joint_names=list(UPPER_POLICY_JOINTS), preserve_order=True
            )
        },
    )
    self_collision = RewTerm(
        func=mdp.self_collision_proximity,
        weight=-12.0,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "body_pairs": [
                ("left_wrist_roll_rubber_hand", "right_wrist_roll_rubber_hand", 0.12),
                ("left_elbow_link", "right_elbow_link", 0.16),
                ("left_wrist_roll_rubber_hand", "torso_link", 0.15),
                ("right_wrist_roll_rubber_hand", "torso_link", 0.15),
            ],
        },
    )
    policy_collision_guard = RewTerm(
        func=mdp.policy_safety_guard_activation,
        weight=-6.0,
        params={"action_name": "residual_joint_position"},
    )
    policy_wrapper_rejection = RewTerm(
        func=mdp.policy_wrapper_rejected_demand,
        weight=-3.0,
        params={"action_name": "residual_joint_position"},
    )
    raw_action_saturation = RewTerm(
        func=mdp.raw_action_saturation_penalty,
        weight=-0.5,
        params={"action_name": "residual_joint_position"},
    )
    robot_body_barrier = RewTerm(
        func=mdp.robot_body_barrier_violation,
        weight=-10.0,
        params={"command_name": "motion"},
    )


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class RobotEnvCfg(ManagerBasedRLEnvCfg):
    scene: RobotSceneCfg = RobotSceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum = None

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 20.0
        self.sim.dt = 0.005  # 200 Hz physics, 50 Hz policy.
        self.sim.render_interval = self.decimation
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15


class RobotPlayEnvCfg(RobotEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 1
        self.episode_length_s = 1.0e9
        self.observations.policy.enable_corruption = False
        self.commands.motion.debug_vis = True

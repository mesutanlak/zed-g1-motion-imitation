"""Dynamic G1 23-DOF Isaac Lab scene driven by GMR reference packets.

The script uses Unitree RL Lab's official G1 23-DOF actuator configuration and
the official Unitree ROS URDF.  It does not open Unitree DDS channels.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import socket
import sys
import time
import traceback
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# Load CPU ONNX Runtime before Isaac Sim modifies the process DLL search path.
# Loading it after SimulationApp on Windows can bind against Isaac's CUDA DLLs.
import numpy as np
import onnxruntime as ort
import yaml

from motion_pipeline.metrics import PacketMetrics, latency_breakdown_ms
from motion_pipeline.reference_motion import LowLatencyReferenceMotion
from motion_pipeline.reference_policy import (
    ACTION_DIM as REFERENCE_POLICY_ACTION_DIM,
    FIXED_DOUBLE_SUPPORT_CONTACT_STATE,
    INFERENCE_WRAPPER_VERSION,
    OBSERVATION_DIM as REFERENCE_POLICY_OBSERVATION_DIM,
    POLICY_STEP_DT as REFERENCE_POLICY_STEP_DT,
    SCHEMA as REFERENCE_POLICY_SCHEMA,
    TRACKED_BODIES as REFERENCE_POLICY_TRACKED_BODIES,
    UPPER_POLICY_JOINTS,
    assemble_observation as assemble_reference_policy_observation,
    policy_collision_action_scale_numpy,
    postprocess_action_numpy,
    residual_scale_vector,
)
from isaaclab_bridge.dex3_simulation import (
    Dex3SimulationController, assert_neutral_only_joints_present,
)

G1_RUBBER_HAND_ENDPOINT_OFFSET_LOCAL_M = {
    "left": (0.1079465665, 0.00163511945, 0.00202244863),
    "right": (0.1079465665, -0.00163511945, 0.00202244863),
}

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser()
parser.add_argument("--listen-host", default="0.0.0.0")
parser.add_argument("--listen-port", type=int, default=15051)
parser.add_argument("--telemetry-host", default="127.0.0.1")
parser.add_argument("--telemetry-port", type=int, default=15053)
parser.add_argument(
    "--urdf",
    type=Path,
    default=None,
    help="Official Unitree g1_23dof_rev_1_0.urdf path",
)
parser.add_argument(
    "--usd",
    type=Path,
    default=None,
    help="Pre-converted official G1 23-DOF USD cache path",
)
parser.add_argument(
    "--asset-profile",
    choices=("g1_23dof", "g1_29dof_dex3"),
    default="g1_23dof",
    help="Use the existing official 23-DOF body or Unitree's official G1-29 + Dex3 asset",
)
parser.add_argument(
    "--unitree-sim-root",
    type=Path,
    default=None,
    help="Official unitreerobotics/unitree_sim_isaaclab checkout",
)
parser.add_argument(
    "--mode",
    choices=("upper_body", "whole_body"),
    default="upper_body",
    help="upper_body preserves the trained/nominal standing legs",
)
parser.add_argument(
    "--imitation-mode",
    choices=("dynamic", "kinematic_debug"),
    default="kinematic_debug",
    help="PhysX/PD imitation or fixed-base direct q_ref mapping validation",
)
parser.add_argument("--stale-after", type=float, default=0.35)
parser.add_argument(
    "--input-fps", type=float, default=15.0,
    help="Accepted GMR reference rate used by the 200 Hz cubic resampler",
)
parser.add_argument(
    "--reference-tracking-mode",
    choices=("low_latency", "smooth_bounded"),
    default="low_latency",
    help="Live velocity-feedforward tracking or deliberately slow jerk-bounded tracking",
)
parser.add_argument(
    "--reference-response-hz", type=float, default=5.5,
    help="Critically damped upper-body reference response bandwidth",
)
parser.add_argument(
    "--reference-max-velocity", type=float, default=0.85,
    help="Maximum applied upper-body reference velocity in rad/s",
)
parser.add_argument(
    "--reference-max-acceleration", type=float, default=4.0,
    help="Maximum applied upper-body reference acceleration in rad/s^2",
)
parser.add_argument(
    "--render-interval", type=int, default=8,
    help="Render one frame per N physics steps (8 = 25 Hz at 200 Hz physics)",
)
parser.add_argument(
    "--reference-max-jerk", type=float, default=35.0,
    help="Maximum applied upper-body reference jerk in rad/s^3",
)
parser.add_argument(
    "--stale-return-delay",
    type=float,
    default=0.25,
    help="Seconds after input becomes stale before returning arms to nominal",
)
parser.add_argument(
    "--stale-return-tau",
    type=float,
    default=0.60,
    help="Time constant for the smooth stale-input return to nominal pose",
)
parser.add_argument("--reset-height", type=float, default=0.42)
parser.add_argument(
    "--balance-policy",
    type=Path,
    default=Path(__file__).resolve().parents[1]
    / "policies"
    / "g1_23dof_velocity"
    / "policy.onnx",
    help="Completed Unitree RL MjLab G1-23DOF velocity policy",
)
parser.add_argument(
    "--balance-config",
    type=Path,
    default=Path(__file__).resolve().parents[1]
    / "policies"
    / "g1_23dof_velocity"
    / "deploy.yaml",
)
parser.add_argument(
    "--reference-policy",
    type=Path,
    default=PROJECT_ROOT / "policies" / "g1_reference_upper_body" / "policy.onnx",
    help="Upper-body IK residual ONNX policy (optional in normal IK mode)",
)
parser.add_argument(
    "--reference-policy-metadata",
    type=Path,
    default=PROJECT_ROOT
    / "policies"
    / "g1_reference_upper_body"
    / "policy_metadata.json",
)
parser.add_argument(
    "--reference-policy-start-enabled",
    action="store_true",
    help="Start in IK+policy mode; GUI users can toggle the same mode with P",
)
parser.add_argument(
    "--reference-policy-blend",
    type=float,
    default=1.0,
    help="Blend of the bounded learned correction over the normal IK target",
)
parser.add_argument(
    "--mimic-blend",
    type=float,
    default=1.0,
    help="Upper-body GMR contribution (1.0 tracks the newest accepted target)",
)
parser.add_argument(
    "--stance-mode",
    choices=("fixed_double_support", "balance_policy"),
    default="fixed_double_support",
    help="Keep both feet in the official nominal stance, or allow policy leg corrections",
)
parser.add_argument(
    "--min-upper-targets",
    type=int,
    default=8,
    help="Minimum coherent BODY_38/GMR targets required for an upper-body update",
)
parser.add_argument(
    "--upper-stiffness-scale",
    type=float,
    default=2.0,
    help="Simulation-only upper-body PD stiffness multiplier",
)
parser.add_argument(
    "--upper-damping-scale",
    type=float,
    default=2.0,
    help="Simulation-only upper-body PD damping multiplier",
)
parser.add_argument(
    "--fall-arrest",
    action=argparse.BooleanOptionalAction,
    default=True,
    help="Physics-based virtual safety harness; does not fix or teleport the base",
)
parser.add_argument(
    "--max-steps",
    type=int,
    default=0,
    help="Exit after N physics steps (0 keeps the interactive simulation running)",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import carb
import omni.kit.app
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext
from isaaclab.utils import math as math_utils
from unitree_rl_lab.assets.robots.unitree import (
    UNITREE_G1_23DOF_CFG,
    UnitreeUrdfFileCfg,
    UnitreeUsdFileCfg,
)


INSTALL_ROOT = Path(os.environ.get("G1IL_ROOT", r"C:\g1il"))
if os.name == "nt":
    DEFAULT_URDF = Path(
        INSTALL_ROOT / "repos" / "unitree_ros" / "robots"
        / "g1_description" / "g1_23dof_rev_1_0.urdf"
    )
else:
    DEFAULT_URDF = Path(
        Path.home() / "g1_isaaclab_project" / "repos" / "unitree_ros"
        / "robots" / "g1_description" / "g1_23dof_rev_1_0.urdf"
    )
DEFAULT_USD = INSTALL_ROOT / "cache" / "g1_23dof" / "g1_23dof_rev_1_0.usd"
DEFAULT_UNITREE_SIM_ROOT = INSTALL_ROOT / "repos" / "unitree_sim_isaaclab"
LOWER_BODY = {
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
}
POLICY_ORDER = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
)
UPPER_BODY = set(POLICY_ORDER) - LOWER_BODY
if tuple(name for name in POLICY_ORDER if name in UPPER_BODY) != UPPER_POLICY_JOINTS:
    raise RuntimeError("Live upper-body order differs from the reference-policy contract")


def configure_fabric_gpu_viewport() -> None:
    """Keep the viewport on the same GPU/Fabric state used by Isaac Lab.

    Isaac Lab reads and writes physics buffers through Fabric.  If the
    Simulation Output window is left on USD, the simulation continues to run
    but the viewport can display the unchanged authoring pose.  Configure the
    official PhysX Fabric output explicitly so live GMR motion is visible.
    """

    extension_manager = omni.kit.app.get_app().get_extension_manager()
    if not extension_manager.is_extension_enabled("omni.physx.fabric"):
        extension_manager.set_extension_enabled_immediate("omni.physx.fabric", True)
    settings = carb.settings.get_settings()
    settings.set_bool("/physics/updateToUsd", False)
    settings.set_bool("physics/fabricEnabled", True)
    settings.set_bool("/physics/fabricUpdateTransformations", True)
    settings.set_bool("physics/fabricUpdatePoints", True)
    settings.set_bool("physics/fabricUseGPUInterop", True)
    print(
        "Simulation Output: Fabric GPU "
        "(USD authoring view disabled for live physics visualization)",
        flush=True,
    )


class BalancePolicy:
    """Run the completed Unitree G1-23DOF policy with its deployment contract."""

    def __init__(
        self,
        policy_path: Path,
        config_path: Path,
        robot: Articulation,
        joint_index: dict[str, int],
    ) -> None:
        if not policy_path.is_file():
            raise FileNotFoundError(f"Balance policy not found: {policy_path}")
        if not config_path.is_file():
            raise FileNotFoundError(f"Balance config not found: {config_path}")
        with config_path.open("r", encoding="utf-8") as stream:
            cfg = yaml.safe_load(stream)
        self.session = ort.InferenceSession(
            str(policy_path), providers=["CPUExecutionProvider"]
        )
        policy_input = self.session.get_inputs()[0]
        policy_output = self.session.get_outputs()[0]
        if policy_input.shape != [1, 80] or policy_output.shape != [1, 23]:
            raise RuntimeError(
                f"Unexpected policy contract: {policy_input.shape} -> "
                f"{policy_output.shape}"
            )
        self.input_name = policy_input.name
        self.output_name = policy_output.name
        self.indices = torch.tensor(
            [joint_index[name] for name in POLICY_ORDER],
            dtype=torch.long,
            device=robot.device,
        )
        self.default = torch.tensor(
            cfg["default_joint_pos"], dtype=torch.float32, device=robot.device
        )
        self.scale = torch.tensor(
            cfg["actions"]["JointPositionAction"]["scale"],
            dtype=torch.float32,
            device=robot.device,
        )
        self.stiffness = torch.tensor(
            cfg["stiffness"], dtype=torch.float32, device=robot.device
        )
        self.damping = torch.tensor(
            cfg["damping"], dtype=torch.float32, device=robot.device
        )
        self.step_dt = float(cfg["step_dt"])
        self.last_action = np.zeros((1, 23), dtype=np.float32)
        self.target = self.default.clone()

    def apply_deployment_gains(self, robot: Articulation) -> None:
        robot.write_joint_stiffness_to_sim(
            self.stiffness.unsqueeze(0), joint_ids=self.indices
        )
        robot.write_joint_damping_to_sim(
            self.damping.unsqueeze(0), joint_ids=self.indices
        )

    def apply_fixed_stance_gains(
        self, robot: Articulation, lower_policy_ids: list[int]
    ) -> None:
        """Increase damping for a quiet nominal double-support stance."""
        selected = torch.tensor(
            lower_policy_ids, dtype=torch.long, device=robot.device
        )
        robot.write_joint_stiffness_to_sim(
            (1.25 * self.stiffness[selected]).unsqueeze(0),
            joint_ids=self.indices[selected],
        )
        robot.write_joint_damping_to_sim(
            (2.0 * self.damping[selected]).unsqueeze(0),
            joint_ids=self.indices[selected],
        )

    def apply_upper_tracking_gains(
        self,
        robot: Articulation,
        upper_policy_ids: list[int],
        stiffness_scale: float,
        damping_scale: float,
    ) -> None:
        """Tighten simulation tracking without changing the trained leg gains."""
        selected = torch.tensor(
            upper_policy_ids, dtype=torch.long, device=robot.device
        )
        robot.write_joint_stiffness_to_sim(
            (stiffness_scale * self.stiffness[selected]).unsqueeze(0),
            joint_ids=self.indices[selected],
        )
        robot.write_joint_damping_to_sim(
            (damping_scale * self.damping[selected]).unsqueeze(0),
            joint_ids=self.indices[selected],
        )

    def infer(self, robot: Articulation) -> torch.Tensor:
        q = robot.data.joint_pos[0, self.indices]
        qd = robot.data.joint_vel[0, self.indices]
        obs = torch.cat(
            (
                robot.data.root_link_ang_vel_b[0],
                robot.data.projected_gravity_b[0],
                torch.zeros(3, device=robot.device),
                torch.zeros(2, device=robot.device),  # zero command => zero gait phase
                q - self.default,
                qd,
                torch.as_tensor(
                    self.last_action[0], dtype=torch.float32, device=robot.device
                ),
            )
        )
        if obs.numel() != 80 or not torch.isfinite(obs).all():
            raise RuntimeError("Invalid 80-element balance-policy observation")
        action = self.session.run(
            [self.output_name],
            {self.input_name: obs.detach().cpu().numpy()[None].astype(np.float32)},
        )[0]
        action = np.clip(action, -10.0, 10.0).astype(np.float32, copy=False)
        self.last_action = action
        action_t = torch.as_tensor(action[0], device=robot.device)
        self.target = self.default + self.scale * action_t
        return self.target

    def reset(self) -> None:
        self.last_action.fill(0.0)
        self.target = self.default.clone()


class ReferenceUpperBodyPolicy:
    """Bounded ONNX residual that augments, but never replaces, live GMR IK."""

    def __init__(
        self,
        policy_path: Path,
        metadata_path: Path,
        robot: Articulation,
        joint_index: dict[str, int],
        blend: float,
    ) -> None:
        if not policy_path.is_file():
            raise FileNotFoundError(f"Reference policy not found: {policy_path}")
        if not metadata_path.is_file():
            raise FileNotFoundError(
                f"Reference policy metadata not found: {metadata_path}"
            )
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        validation = metadata.get("validation")
        if isinstance(validation, dict) and validation.get("accepted") is not True:
            raise RuntimeError(
                "Reference policy was not accepted by held-out validation: "
                f"{validation}"
            )
        expected = {
            "schema": REFERENCE_POLICY_SCHEMA,
            "observation_dim": REFERENCE_POLICY_OBSERVATION_DIM,
            "action_dim": REFERENCE_POLICY_ACTION_DIM,
            "joint_names": list(UPPER_POLICY_JOINTS),
            "tracked_body_names": list(REFERENCE_POLICY_TRACKED_BODIES),
        }
        mismatches = {
            key: (metadata.get(key), value)
            for key, value in expected.items()
            if metadata.get(key) != value
        }
        if mismatches:
            raise RuntimeError(f"Reference policy metadata mismatch: {mismatches}")
        self.session = ort.InferenceSession(
            str(policy_path), providers=["CPUExecutionProvider"]
        )
        policy_input = self.session.get_inputs()[0]
        policy_output = self.session.get_outputs()[0]
        if list(policy_input.shape) != [1, REFERENCE_POLICY_OBSERVATION_DIM]:
            raise RuntimeError(
                f"Reference policy input must be [1, {REFERENCE_POLICY_OBSERVATION_DIM}], "
                f"got {policy_input.shape}"
            )
        if list(policy_output.shape) != [1, REFERENCE_POLICY_ACTION_DIM]:
            raise RuntimeError(
                f"Reference policy output must be [1, {REFERENCE_POLICY_ACTION_DIM}], "
                f"got {policy_output.shape}"
            )
        self.input_name = policy_input.name
        self.output_name = policy_output.name
        self.indices = torch.tensor(
            [joint_index[name] for name in UPPER_POLICY_JOINTS],
            dtype=torch.long,
            device=robot.device,
        )
        scale_by_joint = metadata.get("residual_scale_rad_by_joint")
        if isinstance(scale_by_joint, dict):
            self.residual_scale = np.asarray(
                [float(scale_by_joint[name]) for name in UPPER_POLICY_JOINTS],
                dtype=np.float32,
            )
        else:
            self.residual_scale = np.full(
                REFERENCE_POLICY_ACTION_DIM,
                float(metadata.get("residual_scale_rad", 0.15)),
                dtype=np.float32,
            )
        if self.residual_scale.shape != residual_scale_vector().shape:
            raise RuntimeError("Reference policy residual scale has the wrong shape")
        self.step_dt = float(metadata.get("policy_step_dt", REFERENCE_POLICY_STEP_DT))
        self.deployment_id = str(metadata.get("deployment_id", "legacy"))
        self.model_wrapper_version = str(
            metadata.get("inference_wrapper_version", INFERENCE_WRAPPER_VERSION)
        )
        # The deployed ONNX network is compatible with newer bounded wrappers.
        # Report the wrapper actually enforcing the live command, not the one
        # that happened to be current when the checkpoint was exported.
        self.wrapper_version = INFERENCE_WRAPPER_VERSION
        self.blend = float(np.clip(blend, 0.0, 1.0))
        self.last_slewed_action = np.zeros(
            (1, REFERENCE_POLICY_ACTION_DIM), dtype=np.float32
        )
        self.last_action = np.zeros((1, REFERENCE_POLICY_ACTION_DIM), dtype=np.float32)
        self.last_raw_action = np.zeros(
            (1, REFERENCE_POLICY_ACTION_DIM), dtype=np.float32
        )
        self.last_clipped_action = np.zeros_like(self.last_raw_action)
        self.last_clip_delta = np.zeros_like(self.last_raw_action)
        self.last_saturation_mask = np.zeros_like(self.last_raw_action, dtype=bool)
        self.last_safety_scale = np.ones_like(self.last_raw_action)
        self.last_support_gain = np.zeros_like(self.last_raw_action)
        self.last_safety_reasons: list[str] = []
        self.last_safety_clearances: dict = {}
        self.last_correction = torch.zeros(
            REFERENCE_POLICY_ACTION_DIM, dtype=torch.float32, device=robot.device
        )

    def infer(
        self,
        robot: Articulation,
        reference_position: torch.Tensor,
        reference_velocity: torch.Tensor,
        body_reference_error: np.ndarray,
        reference_confidence: float,
        safety_guard: dict | None = None,
    ) -> torch.Tensor:
        q = robot.data.joint_pos[0, self.indices]
        qd = robot.data.joint_vel[0, self.indices]
        default = robot.data.default_joint_pos[0, self.indices]
        obs = assemble_reference_policy_observation(
            projected_gravity=robot.data.projected_gravity_b[0].detach().cpu().numpy(),
            base_ang_vel=robot.data.root_link_ang_vel_b[0].detach().cpu().numpy(),
            q_minus_q_default=(q - default).detach().cpu().numpy(),
            qd=qd.detach().cpu().numpy(),
            q_ref_minus_q=(reference_position - q).detach().cpu().numpy(),
            qd_ref_minus_qd=(reference_velocity - qd).detach().cpu().numpy(),
            body_reference_error=body_reference_error,
            # Training and deployment both use nominal fixed double support.
            foot_contact_state=FIXED_DOUBLE_SUPPORT_CONTACT_STATE,
            previous_action=self.last_action[0],
            reference_confidence=reference_confidence,
        )
        action = self.session.run(
            [self.output_name], {self.input_name: obs[None]}
        )[0]
        if action.shape != (1, REFERENCE_POLICY_ACTION_DIM) or not np.isfinite(action).all():
            raise RuntimeError("Reference policy produced an invalid action")
        wrapped = postprocess_action_numpy(
            action[0],
            self.last_slewed_action[0],
            reference_confidence,
            residual_scale_rad=self.residual_scale,
            safety_scale=(safety_guard or {}).get("action_scale"),
            support_error_rad=(reference_position - q).detach().cpu().numpy(),
        )
        self.last_raw_action[0] = wrapped["raw"]
        self.last_clipped_action[0] = wrapped["clipped"]
        self.last_slewed_action[0] = wrapped["slewed"]
        self.last_action[0] = wrapped["applied"]
        self.last_clip_delta[0] = wrapped["clip_delta"]
        self.last_saturation_mask[0] = wrapped["saturation_mask"]
        self.last_safety_scale[0] = wrapped["safety_scale"]
        self.last_support_gain[0] = wrapped["support_gain"]
        self.last_safety_reasons = list((safety_guard or {}).get("reasons", []))
        self.last_safety_clearances = dict(
            (safety_guard or {}).get("clearances", {})
        )
        correction = self.blend * wrapped["correction_rad"]
        self.last_correction = torch.as_tensor(
            correction, dtype=torch.float32, device=robot.device
        )
        return self.last_correction

    def reset(self) -> None:
        self.last_slewed_action.fill(0.0)
        self.last_action.fill(0.0)
        self.last_raw_action.fill(0.0)
        self.last_clipped_action.fill(0.0)
        self.last_clip_delta.fill(0.0)
        self.last_saturation_mask.fill(False)
        self.last_safety_scale.fill(1.0)
        self.last_support_gain.fill(0.0)
        self.last_safety_reasons = []
        self.last_safety_clearances = {}
        self.last_correction.zero_()


def packet_reference_confidence(packet: dict | None) -> float:
    """Extract the upper-body confidence shared by policy and telemetry."""

    if not packet:
        return 0.0
    human_state = packet.get("source_human_state") or {}
    segment_quality = human_state.get("segment_quality") or {}
    terms = [
        float(segment_quality[name])
        for name in ("torso", "left_arm", "right_arm")
        if isinstance(segment_quality.get(name), (int, float))
        and np.isfinite(segment_quality[name])
    ]
    if terms:
        return float(np.clip(np.mean(terms), 0.0, 1.0))
    return float(
        np.clip(float(packet.get("body_confidence", 0.0)) / 100.0, 0.0, 1.0)
    )


def live_body_reference_error(
    robot: Articulation,
    body_ids: dict[str, int],
    packet: dict | None,
) -> tuple[np.ndarray, float]:
    """Return pelvis-frame target-minus-actual upper-body errors and coverage."""

    errors = np.zeros(3 * len(REFERENCE_POLICY_TRACKED_BODIES), dtype=np.float32)
    safe = ((packet or {}).get("g1_skeleton") or {}).get("safe_positions_m") or {}
    if "pelvis" not in safe or "pelvis" not in body_ids:
        return errors, 0.0
    reference_pelvis = np.asarray(safe["pelvis"], dtype=np.float32)
    if reference_pelvis.shape != (3,) or not np.isfinite(reference_pelvis).all():
        return errors, 0.0
    actual_pelvis = robot.data.body_pos_w[0, body_ids["pelvis"]]
    error_vectors = []
    valid_indices = []
    for body_index, body_name in enumerate(REFERENCE_POLICY_TRACKED_BODIES):
        if body_name not in safe or body_name not in body_ids:
            continue
        reference = np.asarray(safe[body_name], dtype=np.float32)
        if reference.shape != (3,) or not np.isfinite(reference).all():
            continue
        reference_local = torch.as_tensor(
            reference - reference_pelvis, device=robot.device
        )
        actual_local = robot.data.body_pos_w[0, body_ids[body_name]] - actual_pelvis
        error_vectors.append(reference_local - actual_local)
        valid_indices.append(body_index)
    if not error_vectors:
        return errors, 0.0
    error_w = torch.stack(error_vectors)
    pelvis_quat = robot.data.body_quat_w[0, body_ids["pelvis"]]
    pelvis_inv = math_utils.quat_inv(pelvis_quat.unsqueeze(0)).expand(
        len(error_vectors), -1
    )
    error_b = math_utils.quat_apply(pelvis_inv, error_w).detach().cpu().numpy()
    for body_index, value in zip(valid_indices, error_b):
        errors[3 * body_index : 3 * body_index + 3] = value
    return errors, len(valid_indices) / len(REFERENCE_POLICY_TRACKED_BODIES)


def live_policy_collision_guard(
    robot: Articulation,
    body_ids: dict[str, int],
    packet: dict | None,
) -> dict:
    """Build the policy-only clearance gate from GMR-safe and Isaac links."""

    safe = ((packet or {}).get("g1_skeleton") or {}).get("safe_positions_m") or {}
    actual = {
        name: robot.data.body_pos_w[0, body_id].detach().cpu().numpy()
        for name, body_id in body_ids.items()
    }
    return policy_collision_action_scale_numpy(safe, actual)


def apply_fall_arrest(robot: Articulation, pelvis_body_id: int) -> None:
    """Apply a compliant safety tether while keeping gravity and foot contacts active."""
    pos = robot.data.root_link_pos_w[0]
    vel = robot.data.root_link_lin_vel_w[0]
    up_w = math_utils.quat_apply(
        robot.data.root_link_quat_w[0:1],
        torch.tensor([[0.0, 0.0, 1.0]], device=robot.device),
    )[0]
    world_up = torch.tensor([0.0, 0.0, 1.0], device=robot.device)
    tilt_axis = torch.linalg.cross(up_w, world_up)
    ang_vel = robot.data.root_link_ang_vel_w[0]
    total_mass = torch.sum(robot.data.default_mass[0])

    force = torch.zeros((1, 1, 3), device=robot.device)
    force[0, 0, 0:2] = -180.0 * pos[0:2] - 35.0 * vel[0:2]
    force[0, 0, 2] = (
        0.55 * total_mass * 9.81
        + 500.0 * (0.80 - pos[2])
        - 70.0 * vel[2]
    )
    force[0, 0, 2] = torch.clamp(force[0, 0, 2], -150.0, 320.0)
    force[0, 0, 0:2] = torch.clamp(force[0, 0, 0:2], -120.0, 120.0)

    torque = torch.zeros((1, 1, 3), device=robot.device)
    torque[0, 0] = 110.0 * tilt_axis - 16.0 * ang_vel
    torque = torch.clamp(torque, -90.0, 90.0)
    robot.set_external_force_and_torque(
        force, torque, body_ids=[pelvis_body_id], is_global=True
    )


def _official_unitree_dex3_config(unitree_sim_root: Path):
    source = unitree_sim_root / "robots" / "unitree.py"
    asset = (
        unitree_sim_root / "assets" / "robots" / "g1-29dof-dex3-base-fix-usd"
        / "g1_29dof_with_dex3_base_fix.usd"
    )
    if not source.is_file() or not asset.is_file() or asset.stat().st_size < 1024:
        raise FileNotFoundError(
            "Official Unitree G1-29 + Dex3 source/asset missing. Run "
            "install/install_unitree_dex3_sim.ps1 first."
        )
    previous_root = os.environ.get("PROJECT_ROOT")
    os.environ["PROJECT_ROOT"] = str(unitree_sim_root)
    try:
        spec = importlib.util.spec_from_file_location(
            "unitree_official_robot_config", source
        )
        if spec is None or spec.loader is None:
            raise RuntimeError(f"Cannot import official Unitree config: {source}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cfg = module.G129_CFG_WITH_DEX3_BASE_FIX.copy()
    finally:
        if previous_root is None:
            os.environ.pop("PROJECT_ROOT", None)
        else:
            os.environ["PROJECT_ROOT"] = previous_root
    return cfg


def design_scene(
    urdf_path: Path,
    usd_path: Path | None,
    *,
    asset_profile: str,
    unitree_sim_root: Path,
) -> Articulation:
    ground = sim_utils.GroundPlaneCfg()
    ground.func("/World/Ground", ground)
    light = sim_utils.DomeLightCfg(intensity=2500.0, color=(0.8, 0.8, 0.8))
    light.func("/World/Light", light)

    if asset_profile == "g1_29dof_dex3":
        cfg = _official_unitree_dex3_config(unitree_sim_root)
    else:
        cfg = UNITREE_G1_23DOF_CFG.copy()
    cfg.prim_path = "/World/G1"
    if asset_profile == "g1_29dof_dex3":
        print(
            "Using official unitreerobotics G1-29DOF + Dex3 USD (DDS disabled)",
            flush=True,
        )
    elif usd_path is not None and usd_path.is_file():
        print(f"Using cached G1 USD: {usd_path}", flush=True)
        cfg.spawn = UnitreeUsdFileCfg(usd_path=str(usd_path))
    else:
        if os.name == "nt":
            usd_cache = INSTALL_ROOT / "cache" / "g1_23dof"
        else:
            usd_cache = Path("/tmp/IsaacLab/g1_23dof")
        usd_cache.mkdir(parents=True, exist_ok=True)
        print(
            "Cached USD not found; converting the official URDF. "
            "The first conversion can take several minutes.",
            flush=True,
        )
        cfg.spawn = UnitreeUrdfFileCfg(
            asset_path=str(urdf_path),
            usd_dir=str(usd_cache),
            usd_file_name="g1_23dof_rev_1_0.usd",
        )
    return Articulation(cfg)


def main() -> None:
    urdf_path = (args_cli.urdf or DEFAULT_URDF).expanduser().resolve()
    usd_path = (args_cli.usd or DEFAULT_USD).expanduser().resolve()
    unitree_sim_root = (
        args_cli.unitree_sim_root or DEFAULT_UNITREE_SIM_ROOT
    ).expanduser().resolve()
    if args_cli.asset_profile == "g1_23dof" and not urdf_path.is_file():
        raise FileNotFoundError(f"Official G1 23-DOF URDF not found: {urdf_path}")

    configure_fabric_gpu_viewport()
    print("Creating Isaac Lab SimulationContext...", flush=True)
    sim = SimulationContext(
        sim_utils.SimulationCfg(
            device=args_cli.device,
            dt=0.005,
            render_interval=max(1, int(args_cli.render_interval)),
            use_fabric=True,
        )
    )
    sim.set_camera_view([2.6, 2.2, 1.7], [0.0, 0.0, 0.8])
    print(f"Spawning official Unitree asset: {args_cli.asset_profile}", flush=True)
    robot = design_scene(
        urdf_path, usd_path,
        asset_profile=args_cli.asset_profile,
        unitree_sim_root=unitree_sim_root,
    )
    print("Resetting the scene and initializing physics...", flush=True)
    sim.reset()
    print("G1 scene initialization complete.", flush=True)

    names = list(robot.joint_names)
    index = {name: i for i, name in enumerate(names)}
    dex3_controller = None
    dex3_state = None
    if args_cli.asset_profile == "g1_29dof_dex3":
        assert_neutral_only_joints_present(names)
        dex3_controller = Dex3SimulationController(names)
        print(
            "Dex3 local articulation controller ready: 14 joints, no DDS",
            flush=True,
        )
    nominal = robot.data.default_joint_pos.clone()
    desired = nominal.clone()
    balance = BalancePolicy(
        args_cli.balance_policy.expanduser().resolve(),
        args_cli.balance_config.expanduser().resolve(),
        robot,
        index,
    )
    balance.apply_deployment_gains(robot)
    lower_policy_ids = [
        policy_id
        for policy_id, name in enumerate(POLICY_ORDER)
        if name in LOWER_BODY
    ]
    upper_policy_ids = [
        policy_id
        for policy_id, name in enumerate(POLICY_ORDER)
        if name in UPPER_BODY
    ]
    lower_policy_ids_t = torch.tensor(
        lower_policy_ids, dtype=torch.long, device=robot.device
    )
    if args_cli.mode == "upper_body" and args_cli.stance_mode == "fixed_double_support":
        balance.apply_fixed_stance_gains(robot, lower_policy_ids)
    if args_cli.mode == "upper_body":
        balance.apply_upper_tracking_gains(
            robot,
            upper_policy_ids,
            max(0.1, args_cli.upper_stiffness_scale),
            max(0.1, args_cli.upper_damping_scale),
        )
    if args_cli.mode == "upper_body":
        # Do not move the arms before the first camera packet. The official
        # Isaac asset and locomotion-policy neutral poses differ slightly
        # (for example elbow 0.97 vs 0.87 rad); overwriting all 23 joints here
        # caused the visible open/close twitch at startup. Only the fixed
        # double-support legs use the balance-policy neutral in upper-body
        # teleoperation. Upper joints remain exactly at the spawned asset pose.
        desired[
            0, balance.indices[lower_policy_ids_t]
        ] = balance.default[lower_policy_ids_t]
    else:
        desired[0, balance.indices] = balance.default
    pelvis_ids, pelvis_names = robot.find_bodies("pelvis")
    if len(pelvis_ids) != 1:
        raise RuntimeError(f"Expected one pelvis body, found: {pelvis_names}")
    pelvis_body_id = pelvis_ids[0]
    left_foot_ids, _ = robot.find_bodies("left_ankle_roll_link")
    right_foot_ids, _ = robot.find_bodies("right_ankle_roll_link")
    if len(left_foot_ids) != 1 or len(right_foot_ids) != 1:
        raise RuntimeError("Expected one left and one right ankle-roll body")
    left_foot_id = left_foot_ids[0]
    right_foot_id = right_foot_ids[0]
    # GMR and Isaac both use the exact official Unitree 23-DOF body names.
    tracking_body_aliases = {
        name: name for name in REFERENCE_POLICY_TRACKED_BODIES
    }
    tracking_body_ids: dict[str, int] = {}
    for reference_name, asset_name in tracking_body_aliases.items():
        try:
            body_ids, _ = robot.find_bodies(asset_name)
        except ValueError:
            # Metrics must never abort the simulator if an upstream USD asset
            # changes an optional task-frame name.
            print(
                f"Warning: telemetry body '{asset_name}' not present; "
                f"'{reference_name}' tracking metric disabled."
            )
            continue
        if len(body_ids) == 1:
            tracking_body_ids[reference_name] = body_ids[0]
    balance_policy_interval = max(
        1, int(round(balance.step_dt / sim.get_physics_dt()))
    )
    reference_policy: ReferenceUpperBodyPolicy | None = None
    reference_policy_path = args_cli.reference_policy.expanduser().resolve()
    reference_policy_metadata = (
        args_cli.reference_policy_metadata.expanduser().resolve()
    )
    if reference_policy_path.is_file() and reference_policy_metadata.is_file():
        reference_policy = ReferenceUpperBodyPolicy(
            reference_policy_path,
            reference_policy_metadata,
            robot,
            index,
            args_cli.reference_policy_blend,
        )
        print(
            "Reference policy ready: IK + bounded upper-body residual "
            f"({reference_policy_path})",
            flush=True,
        )
    elif args_cli.reference_policy_start_enabled:
        raise FileNotFoundError(
            "Policy-powered mode requested but policy files are missing: "
            f"{reference_policy_path}, {reference_policy_metadata}"
        )
    else:
        print(
            "Reference policy not trained yet; normal IK remains available. "
            "P becomes active after policy.onnx is exported.",
            flush=True,
        )
    reference_policy_interval = max(
        1,
        int(
            round(
                (reference_policy.step_dt if reference_policy else REFERENCE_POLICY_STEP_DT)
                / sim.get_physics_dt()
            )
        ),
    )
    reference_policy_signature = (
        (
            reference_policy_path.stat().st_mtime_ns,
            reference_policy_path.stat().st_size,
            reference_policy_metadata.stat().st_mtime_ns,
            reference_policy_metadata.stat().st_size,
        )
        if reference_policy_path.is_file() and reference_policy_metadata.is_file()
        else None
    )

    def reload_reference_policy_if_changed() -> bool:
        """Hot-load the validated deployment produced after Isaac startup."""

        nonlocal reference_policy, reference_policy_interval, reference_policy_signature
        if not reference_policy_path.is_file() or not reference_policy_metadata.is_file():
            return reference_policy is not None
        signature = (
            reference_policy_path.stat().st_mtime_ns,
            reference_policy_path.stat().st_size,
            reference_policy_metadata.stat().st_mtime_ns,
            reference_policy_metadata.stat().st_size,
        )
        if reference_policy is not None and signature == reference_policy_signature:
            return True
        try:
            candidate = ReferenceUpperBodyPolicy(
                reference_policy_path,
                reference_policy_metadata,
                robot,
                index,
                args_cli.reference_policy_blend,
            )
        except Exception as exc:
            print(f"POLICY HOT-RELOAD REJECTED: {exc}", flush=True)
            return reference_policy is not None
        reference_policy = candidate
        reference_policy_interval = max(
            1, int(round(reference_policy.step_dt / sim.get_physics_dt()))
        )
        reference_policy_signature = signature
        print(
            "POLICY HOT-RELOAD READY: validated deployment "
            f"id={reference_policy.deployment_id}",
            flush=True,
        )
        return True
    control_mode = {
        "policy_powered": bool(
            args_cli.reference_policy_start_enabled and reference_policy is not None
        )
    }
    input_interface = None
    keyboard = None
    keyboard_subscription = None

    def on_keyboard_event(event, *_) -> bool:
        if (
            event.type == carb.input.KeyboardEventType.KEY_PRESS
            and str(event.input.name).upper() == "P"
        ):
            if not reload_reference_policy_if_changed():
                print(
                    "POLICY MODE UNAVAILABLE: first train/export "
                    "policies/g1_reference_upper_body/policy.onnx",
                    flush=True,
                )
            else:
                control_mode["policy_powered"] = not control_mode["policy_powered"]
                reference_policy.reset()
                label = (
                    "POLICY POWERED (BODY_38/GMR IK + learned residual)"
                    if control_mode["policy_powered"]
                    else "NORMAL IK"
                )
                print(f"CONTROL MODE: {label}", flush=True)
        return True

    if not args_cli.headless:
        import omni.appwindow as omni_appwindow

        input_interface = carb.input.acquire_input_interface()
        keyboard = omni_appwindow.get_default_app_window().get_keyboard()
        keyboard_subscription = input_interface.subscribe_to_keyboard_events(
            keyboard, on_keyboard_event
        )
        print(
            "P: NORMAL IK <-> POLICY POWERED (IK + upper-body residual)",
            flush=True,
        )
    print(
        "CONTROL MODE: "
        + (
            "POLICY POWERED (BODY_38/GMR IK + learned residual)"
            if control_mode["policy_powered"]
            else "NORMAL IK"
        ),
        flush=True,
    )
    gmr_desired = desired.clone()
    gmr_interpolated = desired.clone()
    trajectory_start = desired.clone()
    trajectory_start_velocity = torch.zeros_like(desired)
    gmr_interpolated_velocity = torch.zeros_like(desired)
    trajectory_started = time.monotonic()
    trajectory_duration = 1.0 / max(float(args_cli.input_fps), 1.0)
    gmr_valid_targets = 0
    mimic_blend = float(np.clip(args_cli.mimic_blend, 0.0, 1.0))

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args_cli.listen_host, args_cli.listen_port))
    sock.setblocking(False)
    telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    telemetry_destination = (args_cli.telemetry_host, args_cli.telemetry_port)
    last_packet = time.monotonic()
    last_print = last_packet
    last_print_step_count = 0
    physics_wall_hz = 0.0
    packet_count = 0
    resets = 0
    step_count = 0
    upper_indices = torch.tensor(
        [index[name] for name in POLICY_ORDER if name in UPPER_BODY],
        dtype=torch.long,
        device=robot.device,
    )
    reference_position = desired.clone()
    reference_velocity = torch.zeros_like(desired)
    reference_acceleration = torch.zeros_like(desired)
    reference_lower_np = (
        robot.data.soft_joint_pos_limits[0, upper_indices, 0]
        .detach().cpu().numpy().astype(np.float64)
    )
    reference_upper_np = (
        robot.data.soft_joint_pos_limits[0, upper_indices, 1]
        .detach().cpu().numpy().astype(np.float64)
    )
    # Unitree's official G1 high-level example limits joint interpolation to
    # 0.5 rad/s at a 20 ms command period.  Kinematic debug runs at the GUI's
    # wall-clock rate rather than the nominal 200 Hz physics rate, therefore
    # it needs a wall-clock governor instead of direct q_ref assignment.
    kinematic_reference = LowLatencyReferenceMotion(
        reference_position[0, upper_indices].detach().cpu().numpy(),
        response_hz=float(args_cli.reference_response_hz),
        max_velocity=float(args_cli.reference_max_velocity),
        max_acceleration=float(args_cli.reference_max_acceleration),
        max_jerk=float(args_cli.reference_max_jerk),
    )
    last_reference_wall_time = time.monotonic()
    max_command_delta = 0.0
    max_actual_delta = 0.0
    command_delta = 0.0
    pending_telemetry: dict | None = None
    latest_reference_packet: dict | None = None
    last_remote_control_request: tuple[int, int] | None = None
    isaac_receive_timestamp_ns = 0
    system_packet_metrics = PacketMetrics()
    stale_watchdog_active = False
    last_control_session_id: int | None = None
    previous_desired = desired.clone()
    previous_desired_velocity = torch.zeros_like(desired)
    previous_desired_acceleration = torch.zeros_like(desired)
    target_jerk_rms_rad_s3 = 0.0
    target_jerk_max_rad_s3 = 0.0

    print(
        f"Isaac G1-23DOF mode={args_cli.mode} "
        f"input={args_cli.listen_host}:{args_cli.listen_port} "
        f"urdf={urdf_path}"
    )
    try:
        while simulation_app.is_running():
            newest = None
            while True:
                try:
                    newest = json.loads(sock.recv(65535))
                except BlockingIOError:
                    break
            now = time.monotonic()
            if newest and newest.get("schema") == "zed_gmr_g1_23dof_live/status/v1":
                if newest.get("status") == "CONTROL_SESSION_RESET":
                    # R in the ZED UI invalidates the complete causal chain,
                    # not only person selection.  Drop the old arm command
                    # and enter the existing smooth stale-return path.
                    gmr_desired[0, upper_indices] = nominal[0, upper_indices]
                    gmr_interpolated[0, upper_indices] = nominal[0, upper_indices]
                    trajectory_start[0, upper_indices] = nominal[0, upper_indices]
                    trajectory_start_velocity[0, upper_indices] = 0.0
                    gmr_interpolated_velocity[0, upper_indices] = 0.0
                    gmr_valid_targets = 0
                    pending_telemetry = None
                    latest_reference_packet = None
                    if reference_policy is not None:
                        reference_policy.reset()
                    last_control_session_id = int(
                        newest.get("control_session_id", 0)
                    )
                    last_packet = (
                        now
                        - args_cli.stale_after
                        - max(0.0, args_cli.stale_return_delay)
                        - sim.get_physics_dt()
                    )
                    print(
                        "CONTROL_SESSION_RESET "
                        f"id={last_control_session_id} "
                        f"reason={newest.get('reason', 'unknown')}",
                        flush=True,
                    )
            if newest and newest.get("schema") == "zed_gmr_g1_23dof_live/v1":
                isaac_receive_timestamp_ns = time.time_ns()
                remote_request = newest.get("source_control_mode_request") or {}
                try:
                    remote_request_id = (
                        int(remote_request["session_ns"]),
                        int(remote_request["revision"]),
                    )
                except (KeyError, TypeError, ValueError):
                    remote_request_id = None
                if (
                    remote_request_id is not None
                    and remote_request_id != last_remote_control_request
                ):
                    last_remote_control_request = remote_request_id
                    requested_policy = bool(
                        remote_request.get("policy_powered", False)
                    )
                    if requested_policy:
                        reload_reference_policy_if_changed()
                    if requested_policy and reference_policy is None:
                        control_mode["policy_powered"] = False
                        print(
                            "REMOTE POLICY REQUEST REJECTED: trained policy "
                            "files are not available; NORMAL IK continues.",
                            flush=True,
                        )
                    else:
                        control_mode["policy_powered"] = requested_policy
                        if reference_policy is not None:
                            reference_policy.reset()
                        print(
                            "CONTROL MODE (ZED P): "
                            + (
                                "POLICY POWERED (BODY_38/GMR IK + learned residual)"
                                if requested_policy
                                else "NORMAL IK"
                            ),
                            flush=True,
                        )
                packet_session_id = int(newest.get("control_session_id", 0))
                if (
                    last_control_session_id is not None
                    and packet_session_id != last_control_session_id
                ):
                    # This also covers the rare case where UDP queue draining
                    # superseded the explicit reset-status packet.
                    gmr_desired[0, upper_indices] = nominal[0, upper_indices]
                    gmr_valid_targets = 0
                    pending_telemetry = None
                    latest_reference_packet = None
                    if reference_policy is not None:
                        reference_policy.reset()
                    print(
                        "CONTROL_SESSION_CHANGED "
                        f"old={last_control_session_id} new={packet_session_id}",
                        flush=True,
                    )
                last_control_session_id = packet_session_id
                packet_names = newest["joint_names"]
                packet_values = newest["joint_position_rad"]
                packet_array = np.asarray(packet_values, dtype=np.float64)
                if (
                    packet_array.shape != (len(packet_names),)
                    or not np.isfinite(packet_array).all()
                ):
                    print("REJECTED_NONFINITE_GMR_PACKET", flush=True)
                else:
                    if dex3_controller is not None:
                        dex3_controller.submit(newest.get("dex3_control"), now)
                    trajectory_start.copy_(gmr_interpolated)
                    trajectory_start_velocity.copy_(gmr_interpolated_velocity)
                    trajectory_started = now
                    for name, value in zip(packet_names, packet_values):
                        if name not in index:
                            continue
                        if args_cli.mode == "upper_body" and name in LOWER_BODY:
                            continue
                        gmr_desired[0, index[name]] = float(value)
                    last_packet = now
                    packet_count += 1
                    gmr_valid_targets = int(newest.get("valid_targets", 0))
                    command_delta = float(
                        torch.max(
                            torch.abs(
                                gmr_desired[0, upper_indices]
                                - nominal[0, upper_indices]
                            )
                        )
                    )
                    max_command_delta = max(max_command_delta, command_delta)
                    pending_telemetry = newest
                    latest_reference_packet = newest

            # Resample the latest 30 Hz feasible GMR reference at the physics
            # rate.  In upper-body imitation the GMR bridge has already checked
            # the straight joint-space segment against the robot-body barrier.
            # A Hermite start-velocity term can leave that checked segment and
            # overshoot into the torso, so upper-body mode deliberately follows
            # the same segment with a scalar smoothstep.  Other modes retain the
            # velocity-continuous Hermite trajectory.  The UDP queue is still
            # drained to newest-only, so old motion is never replayed later.
            phase = float(np.clip(
                (now - trajectory_started) / max(trajectory_duration, 1.0e-4),
                0.0, 1.0,
            ))
            phase2 = phase * phase
            phase3 = phase2 * phase
            h00 = 2.0 * phase3 - 3.0 * phase2 + 1.0
            h10 = phase3 - 2.0 * phase2 + phase
            h01 = -2.0 * phase3 + 3.0 * phase2
            dh00 = (6.0 * phase2 - 6.0 * phase) / trajectory_duration
            dh10 = 3.0 * phase2 - 4.0 * phase + 1.0
            dh01 = (-6.0 * phase2 + 6.0 * phase) / trajectory_duration
            if args_cli.mode == "upper_body":
                safe_segment_delta = gmr_desired - trajectory_start
                gmr_interpolated = trajectory_start + h01 * safe_segment_delta
                gmr_interpolated_velocity = dh01 * safe_segment_delta
            else:
                gmr_interpolated = (
                    h00 * trajectory_start
                    + h10 * trajectory_duration * trajectory_start_velocity
                    + h01 * gmr_desired
                )
                gmr_interpolated_velocity = (
                    dh00 * trajectory_start
                    + dh10 * trajectory_start_velocity
                    + dh01 * gmr_desired
                )

            if (
                step_count % balance_policy_interval == 0
                and not (
                    args_cli.mode == "upper_body"
                    and args_cli.stance_mode == "fixed_double_support"
                )
            ):
                policy_target = balance.infer(robot)
                desired[0, balance.indices] = policy_target
            elif args_cli.mode == "upper_body":
                # The camera never commands the lower body in this phase.
                # Re-assert the official nominal G1 double-support pose every
                # physics step so a locomotion policy cannot lift either foot.
                desired[
                    0, balance.indices[lower_policy_ids_t]
                ] = balance.default[lower_policy_ids_t]

            stale_age = now - last_packet
            input_fresh = stale_age <= args_cli.stale_after
            if input_fresh:
                stale_watchdog_active = False
            elif not stale_watchdog_active:
                system_packet_metrics.watchdog_triggers += 1
                stale_watchdog_active = True
            upper_state = "LIVE" if input_fresh else "HOLD"
            upper_reference_target = reference_position[0, upper_indices].clone()
            upper_reference_velocity_target = torch.zeros_like(
                upper_reference_target
            )
            if input_fresh and gmr_valid_targets >= args_cli.min_upper_targets:
                upper_reference_target = (
                    (1.0 - mimic_blend) * nominal[0, upper_indices]
                    + mimic_blend * gmr_interpolated[0, upper_indices]
                )
                upper_reference_velocity_target = (
                    mimic_blend * gmr_interpolated_velocity[0, upper_indices]
                )
            elif packet_count == 0:
                # Explicit cold-start state: until the first valid camera/GMR
                # packet arrives, hold the exact spawned nominal pose. This is
                # distinct from RETURN, which is used only after a live stream
                # has gone stale.
                upper_state = "ZERO"
                upper_reference_target = nominal[0, upper_indices]
            elif (
                args_cli.mode == "upper_body"
                and stale_age
                > args_cli.stale_after + max(0.0, args_cli.stale_return_delay)
            ):
                upper_state = "RETURN"
                upper_reference_target = nominal[0, upper_indices]

            normal_ik_target = upper_reference_target.clone()
            normal_ik_velocity_target = upper_reference_velocity_target.clone()
            policy_reference_confidence = packet_reference_confidence(
                latest_reference_packet
            )
            policy_body_error, policy_body_coverage = live_body_reference_error(
                robot, tracking_body_ids, latest_reference_packet
            )
            policy_collision_guard = live_policy_collision_guard(
                robot, tracking_body_ids, latest_reference_packet
            )
            policy_input_valid = bool(
                input_fresh
                and gmr_valid_targets >= args_cli.min_upper_targets
                and policy_body_coverage >= 0.75
                and policy_reference_confidence >= 0.25
            )
            if reference_policy is not None:
                if control_mode["policy_powered"] and policy_input_valid:
                    if step_count % reference_policy_interval == 0:
                        reference_policy.infer(
                            robot,
                            upper_reference_target,
                            upper_reference_velocity_target,
                            policy_body_error,
                            policy_reference_confidence * policy_body_coverage,
                            safety_guard=policy_collision_guard,
                        )
                    # The learned term is a small residual around the current
                    # GMR/IK target; it cannot command legs or replace IK.
                    upper_reference_target = (
                        upper_reference_target + reference_policy.last_correction
                    )
                elif bool(
                    torch.count_nonzero(reference_policy.last_correction).item()
                ):
                    reference_policy.reset()

            # GMR has already projected q onto the official joint, velocity,
            # acceleration and collision constraints.  In live mode we must
            # not add another low-bandwidth position filter, which previously
            # introduced >1.5 s of visible phase lag.  The default path uses
            # the resampled target velocity as feed-forward and only limits
            # acceleration. The old deliberately slow jerk-bounded path is
            # retained as an explicit diagnostic option.
            physics_dt = sim.get_physics_dt()
            reference_wall_dt = float(np.clip(
                now - last_reference_wall_time, 1.0e-4, 0.05
            ))
            last_reference_wall_time = now
            reference_step_dt = physics_dt
            ref_q = reference_position[0, upper_indices]
            ref_qd = reference_velocity[0, upper_indices]
            ref_qdd = reference_acceleration[0, upper_indices]
            if args_cli.imitation_mode == "kinematic_debug":
                # Kinematic debug is the causal live-mapping view: GMR has
                # already enforced joint limits, anatomical continuity and
                # collision feasibility, while the wall-clock cubic resampler
                # above bridges sparse camera packets.  Never assign the new
                # pose directly: doing that produced 6+ rad/s references and
                # one-frame shoulder/elbow jumps.  Advance by measured wall
                # time so render load cannot make the cap too fast or too slow.
                reference_step_dt = reference_wall_dt
                sample = kinematic_reference.update(
                    upper_reference_target.detach().cpu().numpy(),
                    upper_reference_velocity_target.detach().cpu().numpy(),
                    reference_wall_dt,
                    lower=reference_lower_np,
                    upper=reference_upper_np,
                )
                ref_q = torch.as_tensor(
                    sample.position, dtype=desired.dtype, device=robot.device
                )
                ref_qd = torch.as_tensor(
                    sample.velocity, dtype=desired.dtype, device=robot.device
                )
                ref_qdd = torch.as_tensor(
                    sample.acceleration, dtype=desired.dtype, device=robot.device
                )
            elif args_cli.reference_tracking_mode == "low_latency":
                error = upper_reference_target - ref_q
                tau = 1.0 / (
                    2.0 * np.pi
                    * max(0.1, float(args_cli.reference_response_hz))
                )
                requested_velocity = upper_reference_velocity_target + error / tau
                max_acceleration = max(
                    0.1, float(args_cli.reference_max_acceleration)
                )
                braking_velocity = torch.sqrt(
                    torch.clamp(
                        2.0 * max_acceleration * torch.abs(error), min=0.0
                    )
                )
                relative_velocity = torch.clamp(
                    requested_velocity - upper_reference_velocity_target,
                    min=-braking_velocity,
                    max=braking_velocity,
                )
                requested_velocity = torch.clamp(
                    upper_reference_velocity_target + relative_velocity,
                    min=-max(0.1, float(args_cli.reference_max_velocity)),
                    max=max(0.1, float(args_cli.reference_max_velocity)),
                )
                requested_acceleration = torch.clamp(
                    (requested_velocity - ref_qd) / physics_dt,
                    min=-max_acceleration,
                    max=max_acceleration,
                )
                max_jerk_step = (
                    max(0.1, float(args_cli.reference_max_jerk)) * physics_dt
                )
                ref_qdd = ref_qdd + torch.clamp(
                    requested_acceleration - ref_qdd,
                    min=-max_jerk_step,
                    max=max_jerk_step,
                )
                ref_qdd = torch.clamp(
                    ref_qdd, min=-max_acceleration, max=max_acceleration
                )
                ref_qd = torch.clamp(
                    ref_qd + ref_qdd * physics_dt,
                    min=-max(0.1, float(args_cli.reference_max_velocity)),
                    max=max(0.1, float(args_cli.reference_max_velocity)),
                )
            else:
                omega = 2.0 * np.pi * max(
                    0.1, float(args_cli.reference_response_hz)
                )
                requested_acceleration = (
                    omega * omega * (upper_reference_target - ref_q)
                    - 2.0 * omega * ref_qd
                )
                max_jerk_step = (
                    max(0.1, float(args_cli.reference_max_jerk)) * physics_dt
                )
                ref_qdd = ref_qdd + torch.clamp(
                    requested_acceleration - ref_qdd,
                    min=-max_jerk_step,
                    max=max_jerk_step,
                )
                ref_qdd = torch.clamp(
                    ref_qdd,
                    min=-max(0.1, float(args_cli.reference_max_acceleration)),
                    max=max(0.1, float(args_cli.reference_max_acceleration)),
                )
                ref_qd = torch.clamp(
                    ref_qd + ref_qdd * physics_dt,
                    min=-max(0.1, float(args_cli.reference_max_velocity)),
                    max=max(0.1, float(args_cli.reference_max_velocity)),
                )
            if args_cli.imitation_mode != "kinematic_debug":
                ref_q = ref_q + ref_qd * physics_dt
            reference_low = robot.data.soft_joint_pos_limits[0, upper_indices, 0]
            reference_high = robot.data.soft_joint_pos_limits[0, upper_indices, 1]
            clipped_ref_q = torch.clamp(ref_q, reference_low, reference_high)
            clipped = torch.abs(clipped_ref_q - ref_q) > 1.0e-8
            ref_q = clipped_ref_q
            ref_qd = torch.where(clipped, torch.zeros_like(ref_qd), ref_qd)
            ref_qdd = torch.where(clipped, torch.zeros_like(ref_qdd), ref_qdd)
            reference_target_error = upper_reference_target - ref_q
            reference_target_error_rms_rad = float(
                torch.sqrt(torch.mean(reference_target_error ** 2))
            )
            reference_target_error_max_rad = float(
                torch.max(torch.abs(reference_target_error))
            )
            reference_position[0, upper_indices] = ref_q
            reference_velocity[0, upper_indices] = ref_qd
            reference_acceleration[0, upper_indices] = ref_qdd
            desired[0, upper_indices] = ref_q

            pelvis_z = float(robot.data.root_pos_w[0, 2])
            if pelvis_z < args_cli.reset_height:
                root = robot.data.default_root_state.clone()
                robot.write_root_pose_to_sim(root[:, :7])
                robot.write_root_velocity_to_sim(root[:, 7:])
                robot.write_joint_state_to_sim(
                    robot.data.default_joint_pos, robot.data.default_joint_vel
                )
                desired = nominal.clone()
                desired[0, balance.indices] = balance.default
                gmr_desired = desired.clone()
                gmr_interpolated = desired.clone()
                trajectory_start = desired.clone()
                trajectory_start_velocity.zero_()
                gmr_interpolated_velocity.zero_()
                trajectory_started = now
                reference_position.copy_(desired)
                reference_velocity.zero_()
                reference_acceleration.zero_()
                kinematic_reference.reset(
                    desired[0, upper_indices].detach().cpu().numpy()
                )
                last_reference_wall_time = now
                previous_desired.copy_(desired)
                previous_desired_velocity.zero_()
                previous_desired_acceleration.zero_()
                balance.reset()
                if dex3_controller is not None:
                    dex3_controller.reset()
                if reference_policy is not None:
                    reference_policy.reset()
                robot.reset()
                balance.apply_deployment_gains(robot)
                if (
                    args_cli.mode == "upper_body"
                    and args_cli.stance_mode == "fixed_double_support"
                ):
                    balance.apply_fixed_stance_gains(robot, lower_policy_ids)
                if args_cli.mode == "upper_body":
                    balance.apply_upper_tracking_gains(
                        robot,
                        upper_policy_ids,
                        max(0.1, args_cli.upper_stiffness_scale),
                        max(0.1, args_cli.upper_damping_scale),
                    )
                resets += 1

            if dex3_controller is not None:
                # Only the 14 official hand joint indices are written. The
                # 23 BODY_38/GMR targets above and six unsupported G1-29 body
                # axes remain unchanged/neutral.
                dex3_state = dex3_controller.write_targets(desired, now)
            desired = torch.max(
                torch.min(desired, robot.data.soft_joint_pos_limits[:, :, 1]),
                robot.data.soft_joint_pos_limits[:, :, 0],
            )
            desired_velocity = (desired - previous_desired) / reference_step_dt
            desired_acceleration = (
                desired_velocity - previous_desired_velocity
            ) / reference_step_dt
            desired_jerk = (
                desired_acceleration - previous_desired_acceleration
            ) / reference_step_dt
            upper_jerk = desired_jerk[0, upper_indices]
            target_jerk_rms_rad_s3 = float(torch.sqrt(torch.mean(upper_jerk ** 2)))
            target_jerk_max_rad_s3 = float(torch.max(torch.abs(upper_jerk)))
            previous_desired.copy_(desired)
            previous_desired_velocity.copy_(desired_velocity)
            previous_desired_acceleration.copy_(desired_acceleration)
            if args_cli.fall_arrest and args_cli.imitation_mode == "dynamic":
                apply_fall_arrest(robot, pelvis_body_id)
            if args_cli.imitation_mode == "kinematic_debug":
                # Deterministic mapping check: remove balance/contact effects
                # and display the exact feasible q_ref on a fixed base. Keep
                # the PhysX drive target equal to the teleported state too;
                # otherwise the drive advances toward its stale/default
                # target during sim.step(), creating a visible startup twitch
                # and roughly 0.15 rad of false tracking error with no camera.
                root = robot.data.default_root_state.clone()
                robot.write_root_pose_to_sim(root[:, :7])
                robot.write_root_velocity_to_sim(torch.zeros_like(root[:, 7:]))
                robot.write_joint_state_to_sim(
                    desired, torch.zeros_like(robot.data.joint_vel)
                )
                robot.set_joint_position_target(desired)
                robot.write_data_to_sim()
            else:
                robot.set_joint_position_target(desired)
                robot.write_data_to_sim()
            command_applied_timestamp_ns = time.time_ns()
            sim.step()
            robot.update(sim.get_physics_dt())
            control_observed_timestamp_ns = time.time_ns()
            step_count += 1
            if pending_telemetry is not None:
                actual = robot.data.joint_pos[0]
                joint_rmse = float(
                    torch.sqrt(torch.mean((actual - desired[0]) ** 2))
                )
                actual_velocity = robot.data.joint_vel[0]
                default_position = robot.data.default_joint_pos[0]
                human_state = pending_telemetry.get("source_human_state") or {}
                segment_quality = {
                    str(name): float(value)
                    for name, value in (human_state.get("segment_quality") or {}).items()
                    if isinstance(value, (int, float)) and np.isfinite(value)
                }
                reference_confidence = packet_reference_confidence(
                    pending_telemetry
                )
                root_quat = robot.data.root_quat_w[0].detach().cpu().numpy()
                w, x, y, z = [float(item) for item in root_quat]
                roll = float(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
                pitch = float(np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0)))
                applied_torque = getattr(robot.data, "applied_torque", None)
                torque_rms = None
                energy = None
                if applied_torque is not None:
                    torque = applied_torque[0]
                    torque_rms = float(torch.sqrt(torch.mean(torque ** 2)))
                    energy = float(
                        torch.sum(torch.abs(torque * robot.data.joint_vel[0]))
                        * sim.get_physics_dt()
                    )
                left_velocity = robot.data.body_lin_vel_w[0, left_foot_id, :2]
                right_velocity = robot.data.body_lin_vel_w[0, right_foot_id, :2]
                foot_slip = float(
                    0.5 * (torch.linalg.norm(left_velocity) + torch.linalg.norm(right_velocity))
                )
                body_mpjpe = None
                body_position_errors_m = {}
                actual_positions_m = {}
                safe_reference = (
                    (pending_telemetry.get("g1_skeleton") or {}).get(
                        "safe_positions_m", {}
                    )
                )
                if "pelvis" in safe_reference and "pelvis" in tracking_body_ids:
                    reference_pelvis = np.asarray(safe_reference["pelvis"], dtype=float)
                    actual_pelvis = robot.data.body_pos_w[
                        0, tracking_body_ids["pelvis"]
                    ].detach().cpu().numpy()
                    body_errors = []
                    for body_name, body_id in tracking_body_ids.items():
                        if body_name not in safe_reference:
                            continue
                        reference_local = (
                            np.asarray(safe_reference[body_name], dtype=float)
                            - reference_pelvis
                        )
                        actual_local = (
                            robot.data.body_pos_w[0, body_id].detach().cpu().numpy()
                            - actual_pelvis
                        )
                        actual_positions_m[body_name] = (
                            reference_pelvis + actual_local
                        ).astype(float).tolist()
                        body_error = float(np.linalg.norm(actual_local - reference_local))
                        body_errors.append(body_error)
                        body_position_errors_m[body_name] = body_error
                    for side in ("left", "right"):
                        wrist_name = f"{side}_wrist_roll_rubber_hand"
                        endpoint_name = f"{side}_hand_endpoint"
                        if (
                            wrist_name not in tracking_body_ids
                            or endpoint_name not in safe_reference
                        ):
                            continue
                        wrist_id = tracking_body_ids[wrist_name]
                        local_offset = torch.tensor(
                            [G1_RUBBER_HAND_ENDPOINT_OFFSET_LOCAL_M[side]],
                            dtype=robot.data.body_pos_w.dtype,
                            device=robot.data.body_pos_w.device,
                        )
                        endpoint_world = (
                            robot.data.body_pos_w[0, wrist_id]
                            + math_utils.quat_apply(
                                robot.data.body_quat_w[0, wrist_id].unsqueeze(0),
                                local_offset,
                            )[0]
                        )
                        endpoint_local = (
                            endpoint_world.detach().cpu().numpy() - actual_pelvis
                        )
                        actual_positions_m[endpoint_name] = (
                            reference_pelvis + endpoint_local
                        ).astype(float).tolist()
                        reference_local = (
                            np.asarray(safe_reference[endpoint_name], dtype=float)
                            - reference_pelvis
                        )
                        endpoint_error = float(
                            np.linalg.norm(endpoint_local - reference_local)
                        )
                        body_errors.append(endpoint_error)
                        body_position_errors_m[endpoint_name] = endpoint_error
                    if body_errors:
                        body_mpjpe = float(np.mean(body_errors))
                telemetry = dict(pending_telemetry)
                trace = dict(telemetry.get("latency_trace_ns") or {})
                trace.update(
                    {
                        "t6_isaac_receive_ns": isaac_receive_timestamp_ns,
                        "t7_isaac_command_applied_ns": command_applied_timestamp_ns,
                        "t8_control_observed_ns": control_observed_timestamp_ns,
                    }
                )
                telemetry["latency_trace_ns"] = trace
                latency = latency_breakdown_ms(trace)
                system_packet_metrics.observe(
                    int(telemetry.get("sequence", packet_count)),
                    isaac_receive_timestamp_ns,
                    latency.get("total_control_ms"),
                )
                telemetry["isaac_metrics"] = {
                    "asset_profile": args_cli.asset_profile,
                    "dex3_dds_enabled": False,
                    "dex3_source_timestamp_ns": (
                        dex3_state.source_timestamp_ns if dex3_state is not None else None
                    ),
                    "dex3_packet_age_ms": (
                        dex3_state.packet_age_ms if dex3_state is not None else None
                    ),
                    "dex3_watchdog_left": (
                        dex3_state.watchdog_left if dex3_state is not None else "DISABLED"
                    ),
                    "dex3_watchdog_right": (
                        dex3_state.watchdog_right if dex3_state is not None else "DISABLED"
                    ),
                    "joint_tracking_rmse_rad": joint_rmse,
                    "body_tracking_mpjpe_m": body_mpjpe,
                    "body_position_errors_m": body_position_errors_m,
                    "target_jerk_rms_rad_s3": target_jerk_rms_rad_s3,
                    "target_jerk_max_rad_s3": target_jerk_max_rad_s3,
                    "imitation_mode": args_cli.imitation_mode,
                    "base_roll_rad": roll,
                    "base_pitch_rad": pitch,
                    "foot_slip_m_s": foot_slip,
                    "torque_rms_nm": torque_rms,
                    "step_energy_j": energy,
                    "fall_count": resets,
                    "fall_rate_per_step": resets / max(1, step_count),
                    "episode_success": bool(resets == 0 and pelvis_z >= args_cli.reset_height),
                    "joint_names": list(robot.joint_names),
                    "actual_joint_position_rad": actual.detach().cpu().numpy().astype(float).tolist(),
                    "desired_joint_position_rad": desired[0].detach().cpu().numpy().astype(float).tolist(),
                    "reference_joint_position_rad": reference_position[0].detach().cpu().numpy().astype(float).tolist(),
                    "reference_joint_velocity_rad_s": reference_velocity[0].detach().cpu().numpy().astype(float).tolist(),
                    "reference_joint_acceleration_rad_s2": reference_acceleration[0].detach().cpu().numpy().astype(float).tolist(),
                    "reference_target_error_rms_rad": reference_target_error_rms_rad,
                    "reference_target_error_max_rad": reference_target_error_max_rad,
                    "reference_response_hz": float(args_cli.reference_response_hz),
                    "reference_tracking_mode": args_cli.reference_tracking_mode,
                    "reference_max_velocity_rad_s": float(args_cli.reference_max_velocity),
                    "reference_max_acceleration_rad_s2": float(args_cli.reference_max_acceleration),
                    "reference_max_jerk_rad_s3": float(
                        args_cli.reference_max_jerk
                    ),
                    "reference_step_dt_s": float(reference_step_dt),
                    "reference_limits_basis": (
                        "unitree_g1_upper_body_balanced_live_profile"
                    ),
                    "control_mode": (
                        "policy_powered"
                        if control_mode["policy_powered"]
                        else "normal_ik"
                    ),
                    "reference_policy_available": reference_policy is not None,
                    "reference_policy_body_coverage": policy_body_coverage,
                    "reference_policy_residual_rms_rad": (
                        float(torch.sqrt(torch.mean(reference_policy.last_correction ** 2)))
                        if reference_policy is not None
                        else 0.0
                    ),
                    "reference_policy_residual_max_rad": (
                        float(torch.max(torch.abs(reference_policy.last_correction)))
                        if reference_policy is not None
                        else 0.0
                    ),
                    "reference_policy_collision_guard_active": bool(
                        reference_policy is not None
                        and reference_policy.last_safety_reasons
                    ),
                    "reference_policy_left_arm_scale": (
                        float(np.min(reference_policy.last_safety_scale[0, 1:6]))
                        if reference_policy is not None
                        else 1.0
                    ),
                    "reference_policy_right_arm_scale": (
                        float(np.min(reference_policy.last_safety_scale[0, 6:11]))
                        if reference_policy is not None
                        else 1.0
                    ),
                }
                # This is the clean data contract for the future
                # reference-motion policy. Live teleoperation has no cyclic
                # phase, and real contact state must come from an Isaac Lab
                # contact sensor during policy training; neither is invented
                # from camera data here.
                telemetry["reference_motion"] = {
                    "schema": "g1_reference_motion/v1",
                    "joint_names": list(robot.joint_names),
                    "source_timestamp_ns": int(
                        pending_telemetry.get("source_timestamp_ns", 0) or 0
                    ),
                    "position_rad": reference_position[0].detach().cpu().numpy().astype(float).tolist(),
                    "velocity_rad_s": reference_velocity[0].detach().cpu().numpy().astype(float).tolist(),
                    "acceleration_rad_s2": reference_acceleration[0].detach().cpu().numpy().astype(float).tolist(),
                    "phase": None,
                    "confidence": reference_confidence,
                    "segment_confidence": segment_quality,
                    "fusion_state": str(
                        (pending_telemetry.get("source_multi_camera") or {}).get(
                            "fusion_state", "UNKNOWN"
                        )
                    ),
                }
                policy_indices = (
                    reference_policy.indices
                    if reference_policy is not None
                    else upper_indices
                )
                previous_policy_action = (
                    reference_policy.last_action[0]
                    if reference_policy is not None
                    else np.zeros(REFERENCE_POLICY_ACTION_DIM, dtype=np.float32)
                )
                deployment_observation = assemble_reference_policy_observation(
                    projected_gravity=robot.data.projected_gravity_b[0].detach().cpu().numpy(),
                    base_ang_vel=robot.data.root_link_ang_vel_b[0].detach().cpu().numpy(),
                    q_minus_q_default=(
                        actual[policy_indices] - default_position[policy_indices]
                    ).detach().cpu().numpy(),
                    qd=actual_velocity[policy_indices].detach().cpu().numpy(),
                    q_ref_minus_q=(
                        normal_ik_target - actual[policy_indices]
                    ).detach().cpu().numpy(),
                    qd_ref_minus_qd=(
                        normal_ik_velocity_target - actual_velocity[policy_indices]
                    ).detach().cpu().numpy(),
                    body_reference_error=policy_body_error,
                    foot_contact_state=FIXED_DOUBLE_SUPPORT_CONTACT_STATE,
                    previous_action=previous_policy_action,
                    reference_confidence=reference_confidence,
                )
                telemetry["policy_observation_v1"] = {
                    "schema": "g1_reference_policy_observation/v1",
                    "policy_schema": REFERENCE_POLICY_SCHEMA,
                    "joint_names": list(UPPER_POLICY_JOINTS),
                    "tracked_body_names": list(REFERENCE_POLICY_TRACKED_BODIES),
                    "observation": deployment_observation.astype(float).tolist(),
                    "previous_action": previous_policy_action.astype(float).tolist(),
                    "residual_rad": (
                        reference_policy.last_correction.detach().cpu().numpy().astype(float).tolist()
                        if reference_policy is not None
                        else [0.0] * REFERENCE_POLICY_ACTION_DIM
                    ),
                    "inference_wrapper_version": (
                        reference_policy.wrapper_version
                        if reference_policy is not None
                        else INFERENCE_WRAPPER_VERSION
                    ),
                    "raw_action": (
                        reference_policy.last_raw_action[0].astype(float).tolist()
                        if reference_policy is not None
                        else [0.0] * REFERENCE_POLICY_ACTION_DIM
                    ),
                    "clipped_action": (
                        reference_policy.last_clipped_action[0].astype(float).tolist()
                        if reference_policy is not None
                        else [0.0] * REFERENCE_POLICY_ACTION_DIM
                    ),
                    "raw_action_saturation": (
                        reference_policy.last_saturation_mask[0].astype(bool).tolist()
                        if reference_policy is not None
                        else [False] * REFERENCE_POLICY_ACTION_DIM
                    ),
                    "raw_action_saturation_rate": (
                        float(np.mean(reference_policy.last_saturation_mask[0]))
                        if reference_policy is not None
                        else 0.0
                    ),
                    "collision_guard_action_scale": (
                        reference_policy.last_safety_scale[0].astype(float).tolist()
                        if reference_policy is not None
                        else [1.0] * REFERENCE_POLICY_ACTION_DIM
                    ),
                    "supportive_residual_gain": (
                        reference_policy.last_support_gain[0].astype(float).tolist()
                        if reference_policy is not None
                        else [0.0] * REFERENCE_POLICY_ACTION_DIM
                    ),
                    "collision_guard_reasons": (
                        list(reference_policy.last_safety_reasons)
                        if reference_policy is not None
                        else []
                    ),
                    "collision_guard_clearances": (
                        dict(reference_policy.last_safety_clearances)
                        if reference_policy is not None
                        else {}
                    ),
                    "model_training_wrapper_version": (
                        reference_policy.model_wrapper_version
                        if reference_policy is not None
                        else None
                    ),
                    "clipped_unclipped_l1": (
                        float(np.mean(np.abs(reference_policy.last_clip_delta[0])))
                        if reference_policy is not None
                        else 0.0
                    ),
                    "control_mode": (
                        "policy_powered"
                        if control_mode["policy_powered"]
                        else "normal_ik"
                    ),
                    "reference_confidence": reference_confidence,
                    "body_coverage": policy_body_coverage,
                }
                telemetry.setdefault("g1_skeleton", {})[
                    "actual_positions_m"
                ] = actual_positions_m
                telemetry["latency_breakdown_ms"] = latency
                telemetry["system_metrics"] = system_packet_metrics.snapshot()
                telemetry["system_metrics"]["physics_wall_hz"] = physics_wall_hz
                telemetry["system_metrics"]["render_interval"] = int(
                    args_cli.render_interval
                )
                telemetry["schema"] = "zed_gmr_g1_23dof_isaac_telemetry/v1"
                telemetry_sock.sendto(
                    json.dumps(telemetry, separators=(",", ":")).encode(),
                    telemetry_destination,
                )
                pending_telemetry = None
            if now - last_print >= 1.0:
                physics_wall_hz = (
                    (step_count - last_print_step_count)
                    / max(now - last_print, 1.0e-6)
                )
                left_foot_z = float(robot.data.body_pos_w[0, left_foot_id, 2])
                right_foot_z = float(robot.data.body_pos_w[0, right_foot_id, 2])
                actual_delta = float(
                    torch.max(
                        torch.abs(
                            robot.data.joint_pos[0, upper_indices]
                            - nominal[0, upper_indices]
                        )
                    )
                )
                tracking_error = float(
                    torch.max(
                        torch.abs(
                            robot.data.joint_pos[0, upper_indices]
                            - desired[0, upper_indices]
                        )
                    )
                )
                max_actual_delta = max(max_actual_delta, actual_delta)
                print(
                    f"packets={packet_count} input_age={now-last_packet:.2f}s "
                    f"targets={gmr_valid_targets} pelvis_z={pelvis_z:.3f} "
                    f"foot_z=({left_foot_z:.3f},{right_foot_z:.3f}) "
                    f"foot_delta={abs(left_foot_z-right_foot_z):.3f} "
                    f"imitation={args_cli.imitation_mode} stance={args_cli.stance_mode} "
                    f"control={'POLICY' if control_mode['policy_powered'] else 'IK'} "
                    f"upper_state={upper_state} "
                    f"fall_arrest={'ON' if args_cli.fall_arrest and args_cli.imitation_mode == 'dynamic' else 'OFF'} "
                    f"cmd_delta={command_delta if packet_count else 0.0:.3f}rad "
                    f"actual_delta={actual_delta:.3f}rad "
                    f"tracking_err={tracking_error:.3f}rad "
                    f"physics_wall={physics_wall_hz:.0f}Hz "
                    f"max_cmd={max_command_delta:.3f}rad "
                    f"max_actual={max_actual_delta:.3f}rad "
                    f"resets={resets}"
                )
                last_print_step_count = step_count
                last_print = now
            if args_cli.max_steps and step_count >= args_cli.max_steps:
                print(f"TEST_COMPLETE steps={step_count} packets={packet_count}")
                break
    finally:
        if (
            input_interface is not None
            and keyboard is not None
            and keyboard_subscription is not None
        ):
            input_interface.unsubscribe_to_keyboard_events(
                keyboard, keyboard_subscription
            )
        sock.close()
        telemetry_sock.close()


if __name__ == "__main__":
    exit_code = 0
    try:
        main()
    except KeyboardInterrupt:
        print("Isaac G1 canlı simülasyonu kullanıcı tarafından durduruldu.")
    except BaseException:
        traceback.print_exc()
        exit_code = 1
    finally:
        # With the unvalidated 610.x Windows driver, Isaac Sim 5.0 can hang in
        # SimulationApp.close() even though the simulation itself is healthy.
        # No recorder or physical-robot channel is open here, so an OS-level
        # process exit is the deterministic and data-safe shutdown path.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)

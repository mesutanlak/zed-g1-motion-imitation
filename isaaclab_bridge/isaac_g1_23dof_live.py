"""Dynamic G1 23-DOF Isaac Lab scene driven by GMR reference packets.

The script uses Unitree RL Lab's official G1 23-DOF actuator configuration and
the official Unitree ROS URDF.  It does not open Unitree DDS channels.
"""

from __future__ import annotations

import argparse
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
    "--mode",
    choices=("upper_body", "whole_body"),
    default="upper_body",
    help="upper_body preserves the trained/nominal standing legs",
)
parser.add_argument("--stale-after", type=float, default=0.35)
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
    "waist_yaw_joint",
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


def design_scene(urdf_path: Path, usd_path: Path | None) -> Articulation:
    ground = sim_utils.GroundPlaneCfg()
    ground.func("/World/Ground", ground)
    light = sim_utils.DomeLightCfg(intensity=2500.0, color=(0.8, 0.8, 0.8))
    light.func("/World/Light", light)

    cfg = UNITREE_G1_23DOF_CFG.copy()
    cfg.prim_path = "/World/G1"
    if usd_path is not None and usd_path.is_file():
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
    if not urdf_path.is_file():
        raise FileNotFoundError(f"Official G1 23-DOF URDF not found: {urdf_path}")

    configure_fabric_gpu_viewport()
    print("Creating Isaac Lab SimulationContext...", flush=True)
    sim = SimulationContext(
        sim_utils.SimulationCfg(device=args_cli.device, dt=0.005, use_fabric=True)
    )
    sim.set_camera_view([2.6, 2.2, 1.7], [0.0, 0.0, 0.8])
    print("Spawning the official Unitree G1 23-DOF asset...", flush=True)
    robot = design_scene(urdf_path, usd_path)
    print("Resetting the scene and initializing physics...", flush=True)
    sim.reset()
    print("G1 scene initialization complete.", flush=True)

    names = list(robot.joint_names)
    index = {name: i for i, name in enumerate(names)}
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
        "pelvis": "pelvis",
        "torso_link": "torso_link",
        "left_shoulder_pitch_link": "left_shoulder_pitch_link",
        "left_elbow_link": "left_elbow_link",
        "left_wrist_roll_rubber_hand": "left_wrist_roll_rubber_hand",
        "right_shoulder_pitch_link": "right_shoulder_pitch_link",
        "right_elbow_link": "right_elbow_link",
        "right_wrist_roll_rubber_hand": "right_wrist_roll_rubber_hand",
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
    policy_interval = max(1, int(round(balance.step_dt / sim.get_physics_dt())))
    gmr_desired = desired.clone()
    gmr_valid_targets = 0
    mimic_blend = float(np.clip(args_cli.mimic_blend, 0.0, 1.0))

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args_cli.listen_host, args_cli.listen_port))
    sock.setblocking(False)
    telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    telemetry_destination = (args_cli.telemetry_host, args_cli.telemetry_port)
    last_packet = time.monotonic()
    last_print = last_packet
    packet_count = 0
    resets = 0
    step_count = 0
    upper_indices = torch.tensor(
        [index[name] for name in POLICY_ORDER if name in UPPER_BODY],
        dtype=torch.long,
        device=robot.device,
    )
    max_command_delta = 0.0
    max_actual_delta = 0.0
    command_delta = 0.0
    pending_telemetry: dict | None = None
    isaac_receive_timestamp_ns = 0
    system_packet_metrics = PacketMetrics()
    stale_watchdog_active = False
    last_control_session_id: int | None = None

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
                    gmr_valid_targets = 0
                    pending_telemetry = None
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

            if (
                step_count % policy_interval == 0
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
            if input_fresh and gmr_valid_targets >= args_cli.min_upper_targets:
                for name in UPPER_BODY:
                    joint_id = index[name]
                    desired[0, joint_id] = (
                        (1.0 - mimic_blend) * desired[0, joint_id]
                        + mimic_blend * gmr_desired[0, joint_id]
                    )
            elif (
                args_cli.mode == "upper_body"
                and stale_age
                > args_cli.stale_after + max(0.0, args_cli.stale_return_delay)
            ):
                upper_state = "RETURN"
                # A long occlusion must not hold a potentially ambiguous arm
                # pose forever. Return only the upper body to the official
                # nominal stance; feet and physics remain fully active.
                tau = max(0.05, float(args_cli.stale_return_tau))
                alpha = 1.0 - np.exp(-sim.get_physics_dt() / tau)
                desired[0, upper_indices] += alpha * (
                    nominal[0, upper_indices] - desired[0, upper_indices]
                )

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
                balance.reset()
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

            desired = torch.max(
                torch.min(desired, robot.data.soft_joint_pos_limits[:, :, 1]),
                robot.data.soft_joint_pos_limits[:, :, 0],
            )
            if args_cli.fall_arrest:
                apply_fall_arrest(robot, pelvis_body_id)
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
                        body_errors.append(float(np.linalg.norm(actual_local - reference_local)))
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
                    "joint_tracking_rmse_rad": joint_rmse,
                    "body_tracking_mpjpe_m": body_mpjpe,
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
                }
                telemetry.setdefault("g1_skeleton", {})[
                    "actual_positions_m"
                ] = actual_positions_m
                telemetry["latency_breakdown_ms"] = latency
                telemetry["system_metrics"] = system_packet_metrics.snapshot()
                telemetry["schema"] = "zed_gmr_g1_23dof_isaac_telemetry/v1"
                telemetry_sock.sendto(
                    json.dumps(telemetry, separators=(",", ":")).encode(),
                    telemetry_destination,
                )
                pending_telemetry = None
            if now - last_print >= 1.0:
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
                    f"stance={args_cli.stance_mode} "
                    f"upper_state={upper_state} fall_arrest={'ON' if args_cli.fall_arrest else 'OFF'} "
                    f"cmd_delta={command_delta if packet_count else 0.0:.3f}rad "
                    f"actual_delta={actual_delta:.3f}rad "
                    f"tracking_err={tracking_error:.3f}rad "
                    f"max_cmd={max_command_delta:.3f}rad "
                    f"max_actual={max_actual_delta:.3f}rad "
                    f"resets={resets}"
                )
                last_print = now
            if args_cli.max_steps and step_count >= args_cli.max_steps:
                print(f"TEST_COMPLETE steps={step_count} packets={packet_count}")
                break
    finally:
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

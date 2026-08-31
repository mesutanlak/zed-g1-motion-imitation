"""Convert a Unitree 23-DOF CSV into an Isaac Lab reference-motion NPZ.

Unlike the upstream Unitree RL Lab converter bundled with this installation,
this variant selects the official G1 23-DOF asset and stores explicit joint and
body names.  FK is evaluated by Isaac itself, so reward targets and the training
robot use exactly the same USD frame definitions.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--input-file", type=Path, required=True)
parser.add_argument("--output-file", type=Path, required=True)
parser.add_argument("--metadata-file", type=Path)
parser.add_argument(
    "--usd-file",
    type=Path,
    default=Path(r"C:\g1il\cache\g1_23dof\g1_23dof_rev_1_0.usd"),
)
parser.add_argument("--fps", type=float, default=50.0)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass
from isaaclab.utils.math import axis_angle_from_quat, quat_conjugate, quat_mul
from unitree_rl_lab.assets.robots.unitree import UNITREE_G1_23DOF_CFG


JOINT_ORDER = [
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
]

ROBOT_CFG = UNITREE_G1_23DOF_CFG.copy()
ROBOT_CFG.prim_path = "{ENV_REGEX_NS}/Robot"
ROBOT_CFG.spawn.usd_path = str(args_cli.usd_file.resolve())


@configclass
class ReplaySceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    robot: ArticulationCfg = ROBOT_CFG


def normalized_wxyz(quaternions: np.ndarray) -> np.ndarray:
    result = np.asarray(quaternions, dtype=np.float64).copy()
    for index in range(1, len(result)):
        if np.dot(result[index - 1], result[index]) < 0.0:
            result[index] *= -1.0
    result /= np.maximum(np.linalg.norm(result, axis=1, keepdims=True), 1.0e-12)
    return result


def load_csv() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    motion = np.loadtxt(args_cli.input_file, delimiter=",", ndmin=2)
    if motion.shape[1] != 30 or len(motion) < 3 or not np.isfinite(motion).all():
        raise ValueError("input CSV must be finite Nx30: xyz + quaternion xyzw + 23 joints")
    root_pos = motion[:, :3]
    root_quat = normalized_wxyz(motion[:, 3:7][:, [3, 0, 1, 2]])
    return root_pos, root_quat, motion[:, 7:]


def angular_velocity(quat_wxyz: np.ndarray, dt: float, device: str) -> np.ndarray:
    quat = torch.as_tensor(quat_wxyz, dtype=torch.float32, device=device)
    if len(quat) == 3:
        relative = quat_mul(quat[1:], quat_conjugate(quat[:-1]))
        omega = axis_angle_from_quat(relative) / dt
        omega = torch.cat((omega[:1], omega), dim=0)
    else:
        relative = quat_mul(quat[2:], quat_conjugate(quat[:-2]))
        omega = axis_angle_from_quat(relative) / (2.0 * dt)
        omega = torch.cat((omega[:1], omega, omega[-1:]), dim=0)
    return omega.cpu().numpy()


def metadata(frame_count: int) -> dict[str, np.ndarray]:
    result = {
        "reference_confidence": np.ones(frame_count, dtype=np.float32),
        "phase": np.linspace(0.0, 1.0, frame_count, dtype=np.float32),
        "foot_contact": np.ones((frame_count, 2), dtype=np.float32),
        "source_timestamp_ns": np.arange(frame_count, dtype=np.int64) * int(round(1.0e9 / args_cli.fps)),
    }
    if args_cli.metadata_file:
        source = np.load(args_cli.metadata_file, allow_pickle=False)
        for key in tuple(result):
            if key in source.files:
                value = np.asarray(source[key])
                if len(value) != frame_count:
                    raise ValueError(f"metadata {key} has {len(value)} frames, expected {frame_count}")
                result[key] = value
    return result


def main() -> int:
    if not args_cli.usd_file.is_file():
        raise FileNotFoundError(f"G1 23-DOF USD not found: {args_cli.usd_file}")
    root_pos, root_quat, q_sdk = load_csv()
    frame_count = len(q_sdk)
    dt = 1.0 / args_cli.fps
    qd_sdk = np.gradient(q_sdk, dt, axis=0)
    root_lin_vel = np.gradient(root_pos, dt, axis=0)

    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt=dt)
    sim = SimulationContext(sim_cfg)
    scene = InteractiveScene(ReplaySceneCfg(num_envs=1, env_spacing=2.0))
    sim.reset()
    robot = scene["robot"]
    joint_ids = robot.find_joints(JOINT_ORDER, preserve_order=True)[0]
    if len(joint_ids) != 23:
        raise RuntimeError(f"USD exposes {len(joint_ids)} canonical joints instead of 23")
    root_ang_vel = angular_velocity(root_quat, dt, str(sim.device))

    log = {key: [] for key in ("joint_pos", "joint_vel", "body_pos_w", "body_quat_w", "body_lin_vel_w", "body_ang_vel_w")}
    for index in range(frame_count):
        root_state = robot.data.default_root_state.clone()
        root_state[:, :3] = torch.as_tensor(root_pos[index], dtype=torch.float32, device=sim.device)
        root_state[:, 3:7] = torch.as_tensor(root_quat[index], dtype=torch.float32, device=sim.device)
        root_state[:, 7:10] = torch.as_tensor(root_lin_vel[index], dtype=torch.float32, device=sim.device)
        root_state[:, 10:13] = torch.as_tensor(root_ang_vel[index], dtype=torch.float32, device=sim.device)
        robot.write_root_state_to_sim(root_state)

        joint_pos = robot.data.default_joint_pos.clone()
        joint_vel = robot.data.default_joint_vel.clone()
        joint_pos[:, joint_ids] = torch.as_tensor(q_sdk[index], dtype=torch.float32, device=sim.device)
        joint_vel[:, joint_ids] = torch.as_tensor(qd_sdk[index], dtype=torch.float32, device=sim.device)
        robot.write_joint_state_to_sim(joint_pos, joint_vel)
        scene.write_data_to_sim()
        sim.render()
        scene.update(dt)

        log["joint_pos"].append(robot.data.joint_pos[0].cpu().numpy().copy())
        log["joint_vel"].append(robot.data.joint_vel[0].cpu().numpy().copy())
        log["body_pos_w"].append(robot.data.body_pos_w[0].cpu().numpy().copy())
        log["body_quat_w"].append(robot.data.body_quat_w[0].cpu().numpy().copy())
        log["body_lin_vel_w"].append(robot.data.body_lin_vel_w[0].cpu().numpy().copy())
        log["body_ang_vel_w"].append(robot.data.body_ang_vel_w[0].cpu().numpy().copy())

    output = {key: np.stack(value) for key, value in log.items()}
    output.update(metadata(frame_count))
    output.update(
        {
            "schema": np.asarray("g1_23dof_reference_motion/v1"),
            "fps": np.asarray([args_cli.fps], dtype=np.float32),
            "joint_names": np.asarray(robot.joint_names),
            "body_names": np.asarray(robot.body_names),
            "source_csv": np.asarray(str(args_cli.input_file.resolve())),
        }
    )
    args_cli.output_file.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args_cli.output_file, **output)
    print(
        f"ISAAC_REFERENCE_NPZ_OK frames={frame_count} fps={args_cli.fps:g} "
        f"joints={len(robot.joint_names)} bodies={len(robot.body_names)} output={args_cli.output_file.resolve()}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        simulation_app.close()

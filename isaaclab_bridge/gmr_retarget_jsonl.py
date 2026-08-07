"""Offline ZED BODY_38 -> GMR -> G1 23/29-DOF conversion.

Run this in the dedicated ``gmr_zed`` environment.  It uses the upstream GMR
solver and robot model, while registering this project's BODY_38 IK config at
runtime so the upstream checkout remains unmodified.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

from body38_to_gmr import Body38ToGMR, GMR_BODY38_MAP
from g1_dof_projection import (
    AnatomicalElbowRegularizer,
    G1_23DOF_ORDER,
    constrain_gmr_to_23dof,
    named_joint_values,
    official_g1_23dof_xml,
    project_29_to_23,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path, help="zed_body38_*.jsonl")
    parser.add_argument("--output", type=Path, required=True, help="output .npz")
    parser.add_argument("--gmr-root", type=Path, required=True)
    parser.add_argument("--human-height", type=float, default=1.80)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--elbow-posture-cost", type=float, default=0.05)
    parser.add_argument(
        "--mode", choices=("upper_body", "whole_body"), default="upper_body"
    )
    return parser.parse_args()


def actuated_joint_names(model: mujoco.MjModel) -> list[str]:
    names: list[str] = []
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if not name:
            raise RuntimeError(f"actuator {actuator_id} has no named joint")
        names.append(name)
    if len(set(names)) != len(names):
        raise RuntimeError("GMR robot model contains duplicate actuated joints")
    return names


def qpos_for_joints(
    model: mujoco.MjModel, qpos: np.ndarray, joint_names: list[str]
) -> np.ndarray:
    values = []
    for name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise KeyError(name)
        values.append(float(qpos[int(model.jnt_qposadr[joint_id])]))
    return np.asarray(values, dtype=np.float64)


def solve_with_regularizer(retargeter, human_data: dict, regularizer) -> np.ndarray:
    """Match the live bridge's deterministic Mink solve."""
    import mink

    retargeter.update_targets(human_data, offset_to_ground=True)
    regularizer.update(human_data)
    tasks = [*retargeter.tasks1, regularizer.task]
    current_error = retargeter.error1()
    dt = retargeter.configuration.model.opt.timestep
    for _ in range(retargeter.max_iter + 1):
        velocity = mink.solve_ik(
            retargeter.configuration,
            tasks,
            dt,
            retargeter.solver,
            retargeter.damping,
            retargeter.ik_limits,
        )
        retargeter.configuration.integrate_inplace(velocity, dt)
        next_error = retargeter.error1()
        if current_error - next_error <= 0.001:
            break
        current_error = next_error
    return retargeter.configuration.data.qpos.copy()


def main() -> int:
    args = parse_args()
    gmr_root = args.gmr_root.resolve()
    if str(gmr_root) not in sys.path:
        sys.path.insert(0, str(gmr_root))

    from general_motion_retargeting import params

    # Keep offline conversion bit-for-bit aligned with the live 23-DOF bridge.
    config_path = Path(__file__).with_name("zed_body38_to_g1_23dof.json").resolve()
    robot_key = "unitree_g1_23dof"
    params.ROBOT_XML_DICT[robot_key] = official_g1_23dof_xml()
    params.IK_CONFIG_DICT.setdefault("zed_body38", {})[robot_key] = config_path
    from general_motion_retargeting.motion_retarget import GeneralMotionRetargeting

    retargeter = GeneralMotionRetargeting(
        src_human="zed_body38",
        tgt_robot=robot_key,
        actual_human_height=args.human_height,
        solver="daqp",
        damping=0.5,
        verbose=False,
        use_velocity_limit=False,
    )
    retargeter.max_iter = 20
    constrain_gmr_to_23dof(
        retargeter,
        lock_waist_yaw=args.mode == "upper_body",
        restrict_backward_arms=args.mode == "upper_body",
    )
    joint_order_29 = actuated_joint_names(retargeter.model)
    elbow_regularizer = AnatomicalElbowRegularizer(
        retargeter, cost=max(0.0, args.elbow_posture_cost)
    )
    adapter = Body38ToGMR(fixed_stance=args.mode == "upper_body")
    required = set(GMR_BODY38_MAP)

    timestamps: list[int] = []
    q29_rows: list[np.ndarray] = []
    q23_rows: list[np.ndarray] = []
    full_qpos_rows: list[np.ndarray] = []
    source_indices: list[int] = []
    skipped = 0

    with args.input.open("r", encoding="utf-8") as stream:
        for line_no, line in enumerate(stream):
            record = json.loads(line)
            if str(record.get("schema", "")).endswith("/metadata/v1"):
                continue
            if record.get("schema") == "zed_body38_3d_analysis/frame/v1":
                record = record.get("source", {})
            elif record.get("schema") == "zed_body38_rerun_analysis/frame/v1":
                record = record.get("source_packet", {})
            if record.get("schema") not in (
                "zed_body38_g1_reference/v1",
                "zed_body38_live/v1",
            ):
                continue
            adapted = adapter.adapt(record)
            if adapted is None or not required.issubset(adapted.human_data):
                skipped += 1
                continue

            qpos = solve_with_regularizer(
                retargeter, adapted.human_data, elbow_regularizer
            )
            q29 = qpos_for_joints(retargeter.model, qpos, joint_order_29)
            q23 = project_29_to_23(named_joint_values(joint_order_29, q29))
            timestamps.append(int(record["timestamp_ns"]))
            q29_rows.append(q29)
            q23_rows.append(q23)
            full_qpos_rows.append(np.asarray(qpos, dtype=np.float64).copy())
            source_indices.append(
                int(record.get("frame_index", record.get("sequence", line_no)))
            )
            if args.max_frames and len(q29_rows) >= args.max_frames:
                break

    if not q29_rows:
        raise RuntimeError("No complete BODY_38 frames could be retargeted")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output,
        schema="zed_body38_gmr_g1/v1",
        source_jsonl=str(args.input.resolve()),
        timestamp_ns=np.asarray(timestamps, dtype=np.int64),
        source_frame_index=np.asarray(source_indices, dtype=np.int64),
        joint_names_29=np.asarray(joint_order_29),
        q_g1_29dof=np.stack(q29_rows),
        q_g1_full_qpos=np.stack(full_qpos_rows),
        joint_names_23=np.asarray(G1_23DOF_ORDER),
        q_g1_23dof=np.stack(q23_rows),
    )
    print(
        f"retargeted={len(q29_rows)} skipped={skipped} "
        f"output={args.output.resolve()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

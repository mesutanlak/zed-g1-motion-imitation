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

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from motion_pipeline.collision_geometry import (
    project_configuration_along_safe_path,
    upper_body_capsule_report,
)
from motion_pipeline.safety import G1FeasibilityFilter, SafetyLevel

from body38_to_gmr import Body38ToGMR, GMR_BODY38_MAP
from g1_dof_projection import (
    AnatomicalElbowRegularizer,
    ArmPositionIKRefiner,
    SimpleArmDirectionIK,
    SimpleArmPositionIK,
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
    parser.add_argument(
        "--arm-ik", choices=("position", "direction", "direct", "none"),
        default="position",
        help="A/B replay: simple position/direction IK, legacy direct, or GMR only",
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


SAFETY_BODIES = (
    "pelvis",
    "torso_link",
    "left_shoulder_pitch_link",
    "left_elbow_link",
    "left_wrist_roll_rubber_hand",
    "right_shoulder_pitch_link",
    "right_elbow_link",
    "right_wrist_roll_rubber_hand",
)


def _close_kinematic_relatives(
    model: mujoco.MjModel, first: int, second: int, max_hops: int = 3
) -> bool:
    for child, possible_ancestor in ((first, second), (second, first)):
        current = int(child)
        for _ in range(max_hops):
            current = int(model.body_parentid[current])
            if current == possible_ancestor:
                return True
            if current == 0:
                break
    return False


def _forward_safety_geometry(
    model: mujoco.MjModel,
    base_qpos: np.ndarray,
    joint_values: np.ndarray,
) -> tuple[dict[str, list[float]], int]:
    data = mujoco.MjData(model)
    data.qpos[:] = np.asarray(base_qpos, dtype=np.float64)
    for name, value in zip(G1_23DOF_ORDER, joint_values):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id >= 0:
            data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)
    mujoco.mj_forward(model, data)
    positions = {}
    for name in SAFETY_BODIES:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id >= 0:
            positions[name] = data.xpos[body_id].astype(float).tolist()
    self_contacts = 0
    for contact in data.contact:
        first = int(model.geom_bodyid[int(contact.geom1)])
        second = int(model.geom_bodyid[int(contact.geom2)])
        if (
            first > 0
            and second > 0
            and first != second
            and not _close_kinematic_relatives(model, first, second)
        ):
            self_contacts += 1
    return positions, self_contacts


def _set_named_joint_values(
    model: mujoco.MjModel,
    qpos: np.ndarray,
    names: list[str] | tuple[str, ...],
    values: np.ndarray,
) -> np.ndarray:
    output = np.asarray(qpos, dtype=np.float64).copy()
    for name, value in zip(names, values):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id >= 0:
            output[int(model.jnt_qposadr[joint_id])] = float(value)
    return output


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
    if args.arm_ik == "position":
        arm_ik_refiner = SimpleArmPositionIK(
            retargeter, anchor_fixed_base=args.mode == "upper_body"
        )
    else:
        arm_ik_refiner = {
            "direction": SimpleArmDirectionIK,
            "direct": ArmPositionIKRefiner,
            "none": ArmPositionIKRefiner,
        }[args.arm_ik](retargeter)
    adapter = Body38ToGMR(fixed_stance=args.mode == "upper_body")
    required = set(GMR_BODY38_MAP)

    timestamps: list[int] = []
    q29_rows: list[np.ndarray] = []
    q23_rows: list[np.ndarray] = []
    full_qpos_rows: list[np.ndarray] = []
    source_indices: list[int] = []
    arm_ik_selected_source: list[list[str]] = []
    arm_ik_direction_error_deg: list[list[float]] = []
    arm_ik_position_rms_m: list[list[float]] = []
    safety_margin_m: list[float] = []
    safety_level: list[str] = []
    safety_reason_codes: list[list[str]] = []
    safety_safe_margin_m: list[float] = []
    safety_projection_alpha: list[float] = []
    skipped = 0
    feasibility = G1FeasibilityFilter(
        residual_warn=0.24 if args.mode == "upper_body" else 0.10,
        residual_severe=0.40 if args.mode == "upper_body" else 0.45,
        yellow_blend=0.82 if args.mode == "upper_body" else 0.55,
    )
    previous_timestamp_ns: int | None = None

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
            if args.arm_ik != "none":
                qpos = arm_ik_refiner.update(qpos, adapted.human_data)
            retargeter.configuration.update(qpos)
            raw_q29 = qpos_for_joints(retargeter.model, qpos, joint_order_29)
            raw_q23 = project_29_to_23(named_joint_values(joint_order_29, raw_q29))
            positions, self_contacts = _forward_safety_geometry(
                retargeter.model, qpos, raw_q23
            )
            collision = upper_body_capsule_report(positions)
            timestamp_ns = int(record["timestamp_ns"])
            dt = (
                1.0 / 50.0
                if previous_timestamp_ns is None
                else float(np.clip(
                    (timestamp_ns - previous_timestamp_ns) * 1.0e-9,
                    1.0 / 240.0,
                    0.10,
                ))
            )
            previous_safe_command = (
                feasibility.safe_q.copy()
                if feasibility.safe_q is not None
                else feasibility.nominal.copy()
            )
            projected = feasibility.update(
                raw_q23,
                dt,
                self_collision=self_contacts > 0,
                collision_margin_m=collision.minimum_margin_m,
                arm_collision_margins=collision.arm_minimum_margin_m,
            )
            def evaluate_body_barrier(joint_position):
                skeleton, contacts = _forward_safety_geometry(
                    retargeter.model, qpos, joint_position
                )
                return upper_body_capsule_report(skeleton), contacts

            barrier = project_configuration_along_safe_path(
                previous_safe_command,
                projected.safe_q,
                evaluate_body_barrier,
                clearance_m=0.0,
            )
            q23 = barrier.joint_position
            level = SafetyLevel(projected.level)
            reason_codes = list(projected.reasons)
            if barrier.applied:
                level = max(level, SafetyLevel.YELLOW)
                reason_codes.append("robot_body_barrier_projection")
                feasibility.safe_q = q23.copy()
                feasibility.velocity = (q23 - previous_safe_command) / dt
                feasibility.last_reliable = q23.copy()
            qpos = _set_named_joint_values(
                retargeter.model, qpos, G1_23DOF_ORDER, q23
            )
            retargeter.configuration.update(qpos)
            q29 = qpos_for_joints(retargeter.model, qpos, joint_order_29)
            timestamps.append(timestamp_ns)
            previous_timestamp_ns = timestamp_ns
            q29_rows.append(q29)
            q23_rows.append(q23)
            full_qpos_rows.append(np.asarray(qpos, dtype=np.float64).copy())
            safety_margin_m.append(float(collision.minimum_margin_m))
            safety_level.append(level.name)
            safety_reason_codes.append(list(dict.fromkeys(reason_codes)))
            safety_safe_margin_m.append(float(barrier.minimum_margin_m))
            safety_projection_alpha.append(float(barrier.alpha))
            arm_ik_selected_source.append([
                arm_ik_refiner.last_selected_source[side]
                if args.arm_ik != "none" else "GMR_ONLY"
                for side in ("left", "right")
            ])
            arm_ik_direction_error_deg.append([
                arm_ik_refiner.last_direction_error_deg[side][segment]
                if args.arm_ik != "none" else float("nan")
                for side in ("left", "right")
                for segment in ("upper", "forearm")
            ])
            arm_ik_position_rms_m.append([
                arm_ik_refiner.last_error_m[side]
                if args.arm_ik != "none" else float("nan")
                for side in ("left", "right")
            ])
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
        arm_ik_selected_source=np.asarray(arm_ik_selected_source),
        arm_ik_direction_error_deg=np.asarray(
            arm_ik_direction_error_deg, dtype=np.float64
        ),
        arm_ik_direction_columns=np.asarray((
            "left_upper", "left_forearm", "right_upper", "right_forearm",
        )),
        arm_ik_position_rms_m=np.asarray(
            arm_ik_position_rms_m, dtype=np.float64
        ),
        robot_body_barrier_margin_m=np.asarray(safety_margin_m, dtype=np.float64),
        robot_body_barrier_level=np.asarray(safety_level),
        robot_body_barrier_reason_codes=np.asarray(
            ["|".join(values) for values in safety_reason_codes]
        ),
        robot_body_barrier_safe_margin_m=np.asarray(
            safety_safe_margin_m, dtype=np.float64
        ),
        robot_body_barrier_projection_alpha=np.asarray(
            safety_projection_alpha, dtype=np.float64
        ),
    )
    print(
        f"retargeted={len(q29_rows)} skipped={skipped} "
        f"output={args.output.resolve()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

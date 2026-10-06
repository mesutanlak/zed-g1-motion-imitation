#!/usr/bin/env python3
"""Read G1 low state and compare it with live targets without publishing commands."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import time

import numpy as np


G1_29_INDEX = {
    "left_hip_pitch_joint": 0, "left_hip_roll_joint": 1,
    "left_hip_yaw_joint": 2, "left_knee_joint": 3,
    "left_ankle_pitch_joint": 4, "left_ankle_roll_joint": 5,
    "right_hip_pitch_joint": 6, "right_hip_roll_joint": 7,
    "right_hip_yaw_joint": 8, "right_knee_joint": 9,
    "right_ankle_pitch_joint": 10, "right_ankle_roll_joint": 11,
    "waist_yaw_joint": 12,
    "left_shoulder_pitch_joint": 15, "left_shoulder_roll_joint": 16,
    "left_shoulder_yaw_joint": 17, "left_elbow_joint": 18,
    "left_wrist_roll_joint": 19,
    "right_shoulder_pitch_joint": 22, "right_shoulder_roll_joint": 23,
    "right_shoulder_yaw_joint": 24, "right_elbow_joint": 25,
    "right_wrist_roll_joint": 26,
}
ARM_NAMES = tuple(
    name for name in G1_29_INDEX
    if "shoulder" in name or "elbow" in name or "wrist" in name
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--network-interface", required=True)
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=15055)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stale-after-ms", type=float, default=150.0)
    parser.add_argument("--log-hz", type=float, default=100.0)
    args = parser.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    ChannelFactoryInitialize(0, args.network_interface)
    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    subscriber.Init()
    target_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    target_socket.bind((args.listen_host, args.listen_port))
    target_socket.setblocking(False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    latest_target = None
    target_received_monotonic = -float("inf")
    samples = 0
    log_period_s = 1.0 / max(1.0, float(args.log_hz))
    next_log = time.monotonic()
    print("G1 SHADOW: sadece rt/lowstate okunur; hicbir DDS komut topic'i acilmaz.")
    try:
        with args.output.open("w", encoding="utf-8", buffering=1024 * 1024) as stream:
            while True:
                while True:
                    try:
                        payload, _ = target_socket.recvfrom(2_000_000)
                        packet = json.loads(payload)
                        if packet.get("schema") == "zed_gmr_g1_23dof_live/v1":
                            latest_target = packet
                            target_received_monotonic = time.monotonic()
                    except BlockingIOError:
                        break
                state = subscriber.Read()
                if state is None:
                    time.sleep(0.002)
                    continue
                now = time.monotonic()
                if now < next_log:
                    continue
                next_log = max(next_log + log_period_s, now)
                stale_ms = (now - target_received_monotonic) * 1000.0
                if latest_target is None:
                    time.sleep(0.002)
                    continue
                names = latest_target.get("joint_names") or []
                values = latest_target.get("safe_joint_position_rad") or []
                target_by_name = dict(zip(names, values))
                compared_names = [name for name in ARM_NAMES if name in target_by_name]
                target = np.asarray([target_by_name[name] for name in compared_names])
                actual = np.asarray([
                    state.motor_state[G1_29_INDEX[name]].q for name in compared_names
                ])
                velocity = np.asarray([
                    state.motor_state[G1_29_INDEX[name]].dq for name in compared_names
                ])
                error = actual - target
                record = {
                    "schema": "unitree_g1_shadow_validation/v1",
                    "timestamp_ns": time.time_ns(),
                    "target_sequence": latest_target.get("sequence"),
                    "target_age_ms": stale_ms,
                    "target_fresh": stale_ms <= args.stale_after_ms,
                    "joint_names": compared_names,
                    "target_q_rad": target.tolist(),
                    "actual_q_rad": actual.tolist(),
                    "actual_dq_rad_s": velocity.tolist(),
                    "error_rad": error.tolist(),
                    "rmse_rad": float(np.sqrt(np.mean(np.square(error)))),
                    "maximum_error_rad": float(np.max(np.abs(error))),
                    "imu_rpy_rad": list(state.imu_state.rpy),
                    "mode_machine": int(state.mode_machine),
                    "command_publishing_enabled": False,
                }
                stream.write(json.dumps(record, separators=(",", ":")) + "\n")
                samples += 1
                if samples % 250 == 0:
                    stream.flush()
                    print(
                        f"shadow samples={samples} target_age={stale_ms:.1f}ms "
                        f"rmse={record['rmse_rad']:.4f}rad max={record['maximum_error_rad']:.4f}rad"
                    )
                time.sleep(0.002)
    except KeyboardInterrupt:
        return 0
    finally:
        target_socket.close()
        subscriber.Close()


if __name__ == "__main__":
    raise SystemExit(main())

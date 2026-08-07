"""Count G1 self-contact body pairs from recorded GMR telemetry."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

import mujoco
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("telemetry_jsonl", type=Path)
    parser.add_argument("--g1-xml", type=Path, required=True)
    return parser.parse_args()


def nearby(model: mujoco.MjModel, first: int, second: int, hops: int = 3) -> bool:
    for child, ancestor in ((first, second), (second, first)):
        current = child
        for _ in range(hops):
            current = int(model.body_parentid[current])
            if current == ancestor:
                return True
            if current == 0:
                break
    return False


def benign_hand_rest(model: mujoco.MjModel, first: int, second: int) -> bool:
    names = {
        str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, first)),
        str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, second)),
    }
    return (
        any("wrist_roll_rubber_hand" in name for name in names)
        and any(
            name == "pelvis" or name == "torso_link" or "hip_" in name
            for name in names
        )
    )


def main() -> int:
    args = parse_args()
    model = mujoco.MjModel.from_xml_path(str(args.g1_xml))
    data = mujoco.MjData(model)
    counts: Counter[tuple[str, str]] = Counter()
    sequences: set[int] = set()
    frames_with_contact = 0
    evaluated = 0
    with args.telemetry_jsonl.open(encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            packet = record.get("telemetry_packet") or record
            sequence = int(packet.get("sequence", -1))
            if sequence in sequences:
                continue
            sequences.add(sequence)
            names = packet.get("joint_names") or []
            values = packet.get("raw_joint_position_rad") or []
            if not names or len(names) != len(values):
                continue
            data.qpos[:] = model.qpos0
            for name, value in zip(names, values):
                joint_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_JOINT, str(name)
                )
                if joint_id >= 0 and np.isfinite(float(value)):
                    data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)
            mujoco.mj_forward(model, data)
            frame_pairs: set[tuple[str, str]] = set()
            for contact in data.contact:
                first = int(model.geom_bodyid[int(contact.geom1)])
                second = int(model.geom_bodyid[int(contact.geom2)])
                if first <= 0 or second <= 0 or first == second:
                    continue
                if nearby(model, first, second):
                    continue
                if benign_hand_rest(model, first, second):
                    continue
                names_pair = tuple(sorted((
                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, first),
                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, second),
                )))
                frame_pairs.add(names_pair)
            if frame_pairs:
                frames_with_contact += 1
                counts.update(frame_pairs)
            evaluated += 1
    print(json.dumps({
        "evaluated_unique_sequences": evaluated,
        "frames_with_contact": frames_with_contact,
        "contact_ratio": frames_with_contact / max(1, evaluated),
        "body_pairs": [
            {"first": pair[0], "second": pair[1], "frames": count}
            for pair, count in counts.most_common()
        ],
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

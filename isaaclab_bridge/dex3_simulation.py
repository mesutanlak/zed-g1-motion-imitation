"""DDS-free Unitree Dex3 target handling for the local Isaac articulation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from hand_tracking.contracts import DEX3_JOINT_ORDER, validate_dex3_control


DEX3_API_ORDER = DEX3_JOINT_ORDER
DEX3_ASSET_JOINTS = {
    side: tuple(f"{side}_hand_{name}_joint" for name in DEX3_API_ORDER)
    for side in ("left", "right")
}
G1_29_NEUTRAL_ONLY_JOINTS = (
    "waist_roll_joint", "waist_pitch_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)


@dataclass(frozen=True)
class Dex3SimulationState:
    q_left: np.ndarray
    q_right: np.ndarray
    watchdog_left: str
    watchdog_right: str
    packet_age_ms: float | None
    source_timestamp_ns: int | None


class Dex3SimulationController:
    """Hold then fade compact targets without importing Unitree DDS."""

    def __init__(
        self,
        joint_names: Sequence[str],
        *,
        hold_s: float = 0.20,
        fade_s: float = 0.55,
    ) -> None:
        names = list(joint_names)
        missing = [
            name for side in ("left", "right")
            for name in DEX3_ASSET_JOINTS[side]
            if name not in names
        ]
        if missing:
            raise ValueError(f"Official Dex3 joints missing from articulation: {missing}")
        self.indices = {
            side: np.asarray([names.index(name) for name in DEX3_ASSET_JOINTS[side]], dtype=np.int64)
            for side in ("left", "right")
        }
        self.hold_s = max(0.0, float(hold_s))
        self.fade_s = max(1.0e-3, float(fade_s))
        self.reset()

    def reset(self) -> None:
        self._target = {side: np.zeros(7, dtype=np.float64) for side in ("left", "right")}
        self._received_s: float | None = None
        self._source_timestamp_ns: int | None = None
        self._source_watchdog = {side: "NO_PACKET" for side in ("left", "right")}

    def submit(self, value: Any, received_s: float) -> bool:
        valid, _reason = validate_dex3_control(value)
        if not valid:
            return False
        self._target["left"] = np.asarray(value["q_left"], dtype=np.float64).copy()
        self._target["right"] = np.asarray(value["q_right"], dtype=np.float64).copy()
        self._source_watchdog = {
            side: str(value.get(f"watchdog_{side}", "TRACKING"))
            for side in ("left", "right")
        }
        self._source_timestamp_ns = int(value["timestamp_ns"])
        self._received_s = float(received_s)
        return True

    def state(self, now_s: float) -> Dex3SimulationState:
        if self._received_s is None:
            return Dex3SimulationState(
                np.zeros(7), np.zeros(7), "NO_PACKET", "NO_PACKET", None, None
            )
        age_s = max(0.0, float(now_s) - self._received_s)
        if age_s <= self.hold_s:
            alpha = 0.0
            watchdog = dict(self._source_watchdog)
        else:
            alpha = float(np.clip((age_s - self.hold_s) / self.fade_s, 0.0, 1.0))
            watchdog = {
                side: "FADE_TO_NEUTRAL" if alpha < 1.0 else "NEUTRAL"
                for side in ("left", "right")
            }
        return Dex3SimulationState(
            self._target["left"] * (1.0 - alpha),
            self._target["right"] * (1.0 - alpha),
            watchdog["left"], watchdog["right"], age_s * 1000.0,
            self._source_timestamp_ns,
        )

    def write_targets(self, desired: Any, now_s: float) -> Dex3SimulationState:
        state = self.state(now_s)
        for side, values in (("left", state.q_left), ("right", state.q_right)):
            desired[0, self.indices[side].tolist()] = desired.new_tensor(values)
        return state


def assert_neutral_only_joints_present(joint_names: Sequence[str]) -> None:
    missing = [name for name in G1_29_NEUTRAL_ONLY_JOINTS if name not in joint_names]
    if missing:
        raise ValueError(f"G1-29 neutral-only joints missing: {missing}")

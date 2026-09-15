from __future__ import annotations

import numpy as np

from isaaclab_bridge.dex3_simulation import (
    DEX3_ASSET_JOINTS, Dex3SimulationController,
    G1_29_NEUTRAL_ONLY_JOINTS, assert_neutral_only_joints_present,
)


def joint_names() -> list[str]:
    return list(G1_29_NEUTRAL_ONLY_JOINTS) + list(DEX3_ASSET_JOINTS["left"]) + list(DEX3_ASSET_JOINTS["right"])


def packet() -> dict:
    return {
        "schema": "unitree_g1_dex3_control/v1",
        "timestamp_ns": 123,
        "joint_order_per_hand": [
            "thumb_0", "thumb_1", "thumb_2",
            "middle_0", "middle_1", "index_0", "index_1",
        ],
        "q_left": [0.1] * 7,
        "q_right": [0.2] * 7,
        "confidence_left": 0.8,
        "confidence_right": 0.8,
        "watchdog_left": "TRACKING",
        "watchdog_right": "TRACKING",
        "physical_robot_output_enabled": False,
    }


def test_dex3_simulation_holds_then_fades_without_dds() -> None:
    controller = Dex3SimulationController(joint_names(), hold_s=0.2, fade_s=0.5)
    assert controller.submit(packet(), 10.0)
    held = controller.state(10.1)
    np.testing.assert_allclose(held.q_left, 0.1)
    faded = controller.state(10.45)
    np.testing.assert_allclose(faded.q_left, 0.05)
    assert faded.watchdog_left == "FADE_TO_NEUTRAL"
    np.testing.assert_allclose(controller.state(11.0).q_right, 0.0)


def test_dex3_simulation_rejects_physical_output_and_checks_asset() -> None:
    controller = Dex3SimulationController(joint_names())
    unsafe = packet()
    unsafe["physical_robot_output_enabled"] = True
    assert not controller.submit(unsafe, 1.0)
    assert_neutral_only_joints_present(joint_names())

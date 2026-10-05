from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_official_sources_are_pinned_and_runtime_is_dds_free() -> None:
    lock = json.loads((ROOT / "config" / "unitree_official_sources.lock.json").read_text(encoding="utf-8"))
    assert all(len(item["commit"]) == 40 for item in lock["sources"].values())
    assert lock["runtime_safety"] == {
        "dds_enabled": False,
        "physical_robot_output_enabled": False,
    }
    live = (ROOT / "isaaclab_bridge" / "isaac_g1_23dof_live.py").read_text(encoding="utf-8")
    assert "G129_CFG_WITH_DEX3_BASE_FIX" in live
    assert "enable_dex3_dds" not in live
    assert "unitree_sdk2py" not in live
    bridge = (ROOT / "isaaclab_bridge" / "gmr_live_bridge.py").read_text(encoding="utf-8")
    launcher = (ROOT / "start_g1_isaaclab61_live_ubuntu.sh").read_text(encoding="utf-8")
    assert "G1_DEX3_OPEN_HAND_ENDPOINT_OFFSET_LOCAL_M" in bridge
    assert '--end-effector-profile "$asset_profile"' in launcher
    assert "REJECTED_END_EFFECTOR_PROFILE" in live

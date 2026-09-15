#!/usr/bin/env python3
"""Verify pinned Unitree Dex3 sources/assets without opening Isaac or DDS."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaaclab_bridge.dex3_simulation import DEX3_ASSET_JOINTS


def revision(path: Path) -> str:
    return subprocess.check_output(
        ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
    ).strip()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--install-root", type=Path, default=Path(r"C:\g1il"))
    args = parser.parse_args()
    lock = json.loads((ROOT / "config" / "unitree_official_sources.lock.json").read_text(encoding="utf-8"))
    repos = args.install_root / "repos"
    sim = repos / "unitree_sim_isaaclab"
    xr = repos / "xr_teleoperate"
    errors = []
    for name, path in (("unitree_sim_isaaclab", sim), ("xr_teleoperate", xr)):
        if not (path / ".git").exists():
            errors.append(f"missing repository: {path}")
            continue
        actual = revision(path)
        expected = lock["sources"][name]["commit"]
        if actual != expected:
            errors.append(f"{name} commit {actual} != {expected}")
    asset = sim / lock["sources"]["unitree_sim_isaaclab"]["asset"]
    if not asset.is_file() or asset.stat().st_size < 1024:
        errors.append(f"official Dex3 USD missing/incomplete: {asset}")
    config_path = xr / lock["sources"]["xr_teleoperate"]["required_config"]
    if config_path.is_file():
        config_text = config_path.read_text(encoding="utf-8")
        for side in ("left", "right"):
            if any(name not in config_text for name in DEX3_ASSET_JOINTS[side]):
                errors.append(f"official {side} Dex3 joint contract changed")
    else:
        errors.append(f"official retarget config missing: {config_path}")
    dex_python = args.install_root / "envs" / "dex3" / "Scripts" / "python.exe"
    if not dex_python.is_file():
        errors.append(f"isolated DexPilot Python missing: {dex_python}")
    if errors:
        print("UNITREE_DEX3_VERIFY_FAILED")
        for error in errors:
            print(f"- {error}")
        return 2
    print("UNITREE_DEX3_VERIFY_OK pinned_sources=2 asset=official dds=disabled")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

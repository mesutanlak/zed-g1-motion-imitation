#!/usr/bin/env python3
"""Generate a deterministic G1 23-DOF obstacle course with Unitree tools."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--unitree-mujoco-root",
        type=Path,
        default=Path.home() / "ros2_ws" / "src" / "unitree_mujoco",
    )
    args = parser.parse_args()
    root = args.unitree_mujoco_root.expanduser().resolve()
    sys.path.insert(0, str(root / "terrain_tool"))
    import terrain_generator as unitree_terrain

    g1_dir = root / "unitree_robots" / "g1"
    unitree_terrain.ROBOT = "g1"
    unitree_terrain.INPUT_SCENE_PATH = str(g1_dir / "scene_23dof.xml")
    unitree_terrain.OUTPUT_SCENE_PATH = str(
        g1_dir / "scene_obstacles_23dof.xml"
    )
    np.random.seed(23)
    terrain = unitree_terrain.TerrainGenerator()
    for x, y in ((2.0, -0.75), (2.8, 0.75), (3.6, -0.75), (4.4, 0.75)):
        terrain.AddGeometry(
            position=[x, y, 0.6], size=[0.36, 1.2], geo_type="cylinder"
        )
    terrain.AddBox(position=[2.4, 0.0, 0.06], size=[0.65, 0.18, 0.12])
    terrain.AddBox(position=[3.4, 0.0, 0.09], size=[0.55, 0.22, 0.18])
    terrain.AddBox(position=[4.5, 0.0, 0.12], size=[0.45, 0.25, 0.24])
    terrain.AddBox(position=[4.0, 2.6, 0.5], size=[6.0, 0.15, 1.0])
    terrain.AddBox(position=[4.0, -2.6, 0.5], size=[6.0, 0.15, 1.0])
    terrain.AddStairs(
        init_pos=[5.0, -1.8, 0.0],
        yaw=0.0,
        width=0.22,
        height=0.08,
        length=1.2,
        stair_nums=6,
    )
    terrain.AddBox(
        position=[6.0, 1.7, 0.18],
        euler=[0.0, -0.12, 0.0],
        size=[2.2, 1.2, 0.10],
    )
    terrain.AddRoughGround(
        init_pos=[6.4, -0.9, 0.0],
        nums=[8, 7],
        box_size=[0.32, 0.32, 0.10],
        separation=[0.31, 0.31],
        box_size_rand=[0.03, 0.03, 0.06],
        box_euler_rand=[0.06, 0.06, 0.05],
        separation_rand=[0.01, 0.01],
    )
    terrain.Save()
    print(unitree_terrain.OUTPUT_SCENE_PATH)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

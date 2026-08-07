"""Prepare the official Unitree G1 23-DOF URDF for RViz mesh loading."""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    source = Path(args.source).resolve()
    output = Path(args.output).resolve()
    if not source.is_file():
        raise SystemExit(f"Official G1 URDF not found: {source}")
    mesh_uri = source.parent.as_uri() + "/meshes/"
    text = source.read_text(encoding="utf-8")
    text = text.replace('filename="meshes/', f'filename="{mesh_uri}')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""User-facing entry point for this rig's fixed checkerboard validation."""

from pathlib import Path

from validate_charuco_svo_reference import main


if __name__ == "__main__":
    project = Path(__file__).resolve().parent.parent
    raise SystemExit(main(project / "config" / "fixed_reference_board.json"))

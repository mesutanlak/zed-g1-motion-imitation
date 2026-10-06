#!/usr/bin/env python3
"""Create a metric ChArUco board for independent camera/rig validation."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("charuco_7x5_a3.png"))
    parser.add_argument("--squares-x", type=int, default=7)
    parser.add_argument("--squares-y", type=int, default=5)
    parser.add_argument("--square-mm", type=float, default=55.0)
    parser.add_argument("--marker-mm", type=float, default=41.0)
    parser.add_argument("--dpi", type=int, default=300)
    args = parser.parse_args()
    if not 0.0 < args.marker_mm < args.square_mm:
        raise SystemExit("marker-mm, square-mm degerinden kucuk ve pozitif olmali")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    board = cv2.aruco.CharucoBoard(
        (args.squares_x, args.squares_y),
        args.square_mm / 1000.0,
        args.marker_mm / 1000.0,
        dictionary,
    )
    width_px = round(args.squares_x * args.square_mm / 25.4 * args.dpi)
    height_px = round(args.squares_y * args.square_mm / 25.4 * args.dpi)
    image = board.generateImage((width_px, height_px), marginSize=0, borderBits=1)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), image):
        raise SystemExit(f"Yazilamadi: {args.output}")
    print(
        f"ChArUco: {args.output.resolve()} | {args.squares_x}x{args.squares_y} "
        f"square={args.square_mm:.3f}mm marker={args.marker_mm:.3f}mm dpi={args.dpi}"
    )
    print("%100 olcekte yazdirin ve bir karenin gercek kenarini kumpasla olcun.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

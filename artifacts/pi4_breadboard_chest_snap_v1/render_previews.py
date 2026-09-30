"""Gercek STL geometrilerinden iki montaj onizlemesi uretir."""

from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
LOCAL_DEPS = ROOT.parent / "hw290_snapfit_v1" / "_deps"
if LOCAL_DEPS.exists():
    sys.path.insert(0, str(LOCAL_DEPS))

import numpy as np
import trimesh as tm
from PIL import Image, ImageDraw, ImageFont

import build_chest_carriers as cad


DARK = (58, 78, 90)
GREEN = (37, 125, 82)
SILVER = (183, 191, 196)
WHITE = (224, 227, 222)
ORANGE = (236, 126, 43)


def font(size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype("C:/Windows/Fonts/arial.ttf", size)


def render_meshes(
    image: np.ndarray,
    depth: np.ndarray,
    meshes: list[tuple[tm.Trimesh, tuple[int, int, int]]],
    rect: tuple[int, int, int, int],
    view: tuple[float, float, float],
) -> None:
    all_vertices = np.vstack([mesh.vertices for mesh, _ in meshes])
    target = (all_vertices.min(axis=0) + all_vertices.max(axis=0)) / 2
    direction = np.asarray(view, dtype=float)
    direction /= np.linalg.norm(direction)
    forward = -direction
    right = np.cross(forward, (0, 0, 1))
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)

    def project(vertices: np.ndarray) -> np.ndarray:
        centered = vertices - target
        return np.stack((centered @ right, -(centered @ up)), axis=-1)

    projected = project(all_vertices)
    low, high = projected.min(axis=0), projected.max(axis=0)
    center = (low + high) / 2
    x, y, width, height = rect
    scale = min((width - 50) / (high[0] - low[0]), (height - 50) / (high[1] - low[1]))
    offset = np.array((x + width / 2, y + height / 2))
    light = np.array((-0.25, -0.35, 0.90))
    light /= np.linalg.norm(light)

    for mesh, color in meshes:
        triangles = mesh.triangles
        points = (project(triangles.reshape(-1, 3)).reshape(-1, 3, 2) - center) * scale + offset
        z_values = ((triangles.reshape(-1, 3) - target) @ forward).reshape(-1, 3)
        shades = np.clip(0.68 + 0.32 * (mesh.face_normals @ light), 0.40, 1.0)
        for triangle, z_tri, shade in zip(points, z_values, shades):
            x0 = max(x, int(np.floor(triangle[:, 0].min())))
            x1 = min(x + width - 1, int(np.ceil(triangle[:, 0].max())))
            y0 = max(y, int(np.floor(triangle[:, 1].min())))
            y1 = min(y + height - 1, int(np.ceil(triangle[:, 1].max())))
            if x1 < x0 or y1 < y0:
                continue
            xa, ya = triangle[0]
            xb, yb = triangle[1]
            xc, yc = triangle[2]
            denominator = (yb - yc) * (xa - xc) + (xc - xb) * (ya - yc)
            if abs(denominator) < 1e-9:
                continue
            yy, xx = np.mgrid[y0 : y1 + 1, x0 : x1 + 1]
            wa = ((yb - yc) * (xx - xc) + (xc - xb) * (yy - yc)) / denominator
            wb = ((yc - ya) * (xx - xc) + (xa - xc) * (yy - yc)) / denominator
            wc = 1 - wa - wb
            zz = wa * z_tri[0] + wb * z_tri[1] + wc * z_tri[2]
            region = depth[y0 : y1 + 1, x0 : x1 + 1]
            mask = (wa >= 0) & (wb >= 0) & (wc >= 0) & (zz < region)
            region[mask] = zz[mask]
            image[y0 : y1 + 1, x0 : x1 + 1][mask] = np.clip(
                np.asarray(color) * shade, 0, 255
            )


def make_canvas() -> tuple[np.ndarray, np.ndarray]:
    canvas = np.array(Image.new("RGB", (1800, 1180), "#f5f7fa"))
    depth = np.full((1180, 1800), np.inf, dtype=np.float32)
    return canvas, depth


def label_and_save(
    pixels: np.ndarray,
    title: str,
    subtitles: list[tuple[int, int, str]],
    notes: list[str],
    filename: str,
) -> None:
    image = Image.fromarray(pixels)
    draw = ImageDraw.Draw(image)
    draw.text((45, 25), title, font=font(36), fill="#172e3a")
    for x, y, value in subtitles:
        draw.text((x, y), value, font=font(25), fill="#233e4b")
    for index, value in enumerate(notes):
        draw.text((45, 1020 + index * 34), value, font=font(22), fill="#526572")
    image.save(ROOT / filename)


def main() -> None:
    pi, pcb, ports = cad.pi_carrier()
    bread, bread_ref = cad.breadboard_carrier()

    pixels, depth = make_canvas()
    render_meshes(
        pixels,
        depth,
        [(pi, DARK)],
        (20, 135, 850, 800),
        (95, -115, 90),
    )
    render_meshes(
        pixels,
        depth,
        [(pi, DARK), (pcb, GREEN), *[(port, SILVER) for port in ports]],
        (900, 135, 870, 800),
        (95, -115, 90),
    )
    label_and_save(
        pixels,
        "Raspberry Pi 4 - acik, vidali ve kenar gecmeli gogus tabani",
        [(45, 90, "Bos taban: havalandirma, M2 yuvalari ve dort esnek tirnak"),
         (960, 90, "Montaj: portlar ve GPIO tamamen acik")],
        [
            "PCB: 85 x 56 mm | Delik araligi: 58 x 49 mm | Kart deligi: 2.7 mm",
            "117 x 64 x 11.95 mm | 4 x M2 vida + pul + somun | 4 kenar tirnagi",
            "Iki yanda 28 x 4.2 mm kayis/kelepce yuvasi; ust kapak yoktur.",
        ],
        "01_Raspberry_Pi4_onizleme.png",
    )

    pixels, depth = make_canvas()
    render_meshes(
        pixels,
        depth,
        [(bread, DARK)],
        (15, 145, 865, 790),
        (150, -185, 150),
    )
    render_meshes(
        pixels,
        depth,
        [(bread, DARK), (bread_ref, WHITE)],
        (900, 145, 875, 790),
        (150, -185, 150),
    )
    label_and_save(
        pixels,
        "160 x 52.5 x 9 mm breadboard - vidasiz gecmeli gogus tabani",
        [(45, 90, "Bos taban: alti esnek tirnak ve iki uc dayamasi"),
         (970, 90, "Montaj: breadboard ustten bastirilarak kilitlenir")],
        [
            "Tasiyici: 193 x 57.6 x 13.35 mm | Vida veya yapistirici gerekmez",
            "Her uzun kenarda 3 gecme tirnagi; iki uc dayamasi ileri-geri kaymayi keser.",
            "Iki yanda 28 x 4.2 mm kayis yuvasi ve dort ek plastik kelepce yuvasi vardir.",
        ],
        "02_Breadboard_onizleme.png",
    )


if __name__ == "__main__":
    main()

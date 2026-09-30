"""Gerçek STL geometrisinden destek gerektirmeyen basit önizleme üretir."""

from pathlib import Path
import sys

LOCAL_DEPS = Path(__file__).resolve().parent / "_deps"
LEGACY_DEPS = Path(r"C:\Users\mesut\AppData\Local\Temp\imu-cad-deps")
for dependency_path in (LEGACY_DEPS, LOCAL_DEPS):
    if dependency_path.exists():
        sys.path.insert(0, str(dependency_path))

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import build_hw290_snapfit as cad


ROOT = Path(__file__).resolve().parent
DARK = (61, 82, 95)
ORANGE = (235, 126, 45)
PCB = (25, 137, 170)
BLACK = (38, 43, 47)
WIRE = (219, 67, 55)


def canvas():
    image = Image.new("RGB", (1800, 1120), "#f5f7fa")
    return np.array(image), np.full((1120, 1800), np.inf, dtype=np.float32)


def render(pixels, depth, meshes, rect, view=(55, -80, 55)):
    all_vertices = np.vstack([mesh.vertices for mesh, _ in meshes])
    target = (all_vertices.min(0) + all_vertices.max(0)) / 2
    direction = np.asarray(view, dtype=float)
    direction /= np.linalg.norm(direction)
    forward = -direction
    right = np.cross(forward, [0, 0, 1])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)

    def project(vertices):
        vertices = vertices - target
        return np.stack([vertices @ right, -(vertices @ up)], axis=-1)

    projected = project(all_vertices)
    low, high = projected.min(0), projected.max(0)
    center = (low + high) / 2
    x, y, width, height = rect
    scale = min((width - 45) / (high[0] - low[0]), (height - 45) / (high[1] - low[1]))
    offset = np.array([x + width / 2, y + height / 2])
    light = np.array([-.25, -.4, .88])
    light /= np.linalg.norm(light)

    for mesh, color in meshes:
        triangles = mesh.triangles
        points = (project(triangles.reshape(-1, 3)).reshape(-1, 3, 2) - center) * scale + offset
        zs = ((triangles.reshape(-1, 3) - target) @ forward).reshape(-1, 3)
        shades = np.clip(.68 + .32 * (mesh.face_normals @ light), .40, 1)
        for triangle, zvalues, shade in zip(points, zs, shades):
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
            yy, xx = np.mgrid[y0:y1 + 1, x0:x1 + 1]
            wa = ((yb - yc) * (xx - xc) + (xc - xb) * (yy - yc)) / denominator
            wb = ((yc - ya) * (xx - xc) + (xa - xc) * (yy - yc)) / denominator
            wc = 1 - wa - wb
            zbuffer = wa * zvalues[0] + wb * zvalues[1] + wc * zvalues[2]
            region = depth[y0:y1 + 1, x0:x1 + 1]
            mask = (wa >= 0) & (wb >= 0) & (wc >= 0) & (zbuffer < region)
            region[mask] = zbuffer[mask]
            pixels[y0:y1 + 1, x0:x1 + 1][mask] = np.clip(np.array(color) * shade, 0, 255)


def moved(mesh, xyz):
    result = mesh.copy()
    result.apply_translation(xyz)
    return result


def font(size):
    return ImageFont.truetype("C:/Windows/Fonts/arial.ttf", size)


def main():
    body, cover, pcb, components, header, plugs = cad.build()
    pixels, depth = canvas()

    render(
        pixels,
        depth,
        [(body, DARK), (cover, ORANGE)],
        (15, 150, 850, 720),
        view=(60, -85, 65),
    )

    lifted_cover = moved(cover, (0, 0, 18))
    wires = [cad.box(x - .55, x + .55, 7.5, 22.0, 18.5, 19.6) for x in (-7, -4.5, -2, .5)]
    render(
        pixels,
        depth,
        [
            (body, DARK),
            (pcb, PCB),
            (components, BLACK),
            (header, BLACK),
            *[(plug, BLACK) for plug in plugs],
            *[(wire, WIRE) for wire in wires],
            (lifted_cover, ORANGE),
        ],
        (900, 150, 880, 720),
        view=(58, -88, 62),
    )

    image = Image.fromarray(pixels)
    draw = ImageDraw.Draw(image)
    draw.text((45, 25), "HW-290 vidasız geçme kasa", font=font(36), fill="#172e3a")
    draw.text((45, 92), "Kapalı: vida ve somun yok", font=font(26), fill="#233e4b")
    draw.text((950, 92), "Açık: arkaya kaydır, ortadaki tırnağa bastır", font=font(26), fill="#233e4b")
    notes = [
        "PCB: 21,60 × 17,20 × 2,25 mm  |  Arka kablo ağzı: 22 mm  |  20 mm cırt kulakları",
        "Kartı dört yan dudak üstten tutar; ön orta esnek tırnak kartı arka sabit dayamalara kilitler.",
        "Bir IMU için yalnızca 1 gövde + 1 geçme kapak basılır. Silikon, vida, somun ve ayrı çene kullanılmaz.",
    ]
    for index, note in enumerate(notes):
        draw.text((45, 985 + index * 34), note, font=font(23), fill="#526572")
    image.save(ROOT / "HW290_vidisiz_onizleme.png")


if __name__ == "__main__":
    main()

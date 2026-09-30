"""Render a simple assembly preview of the hinged IMU enclosure."""

from pathlib import Path
import sys

sys.path.insert(0, r"C:\Users\mesut\AppData\Local\Temp\imu-cad-deps")

import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
case = trimesh.load_mesh(ROOT / "IMU_case_menteseli.stl")
lid = trimesh.load_mesh(ROOT / "IMU_kapak_montaj_konumu.stl")

SIZE = (1800, 920)
im = Image.new("RGB", SIZE, "#f6f8fb")
pixels = np.asarray(im).copy()
depth_buffer = np.full((SIZE[1], SIZE[0]), np.inf, dtype=np.float32)
try:
    font = ImageFont.truetype(r"C:\Windows\Fonts\arial.ttf", 35)
    small = ImageFont.truetype(r"C:\Windows\Fonts\arial.ttf", 24)
except OSError:
    font = small = ImageFont.load_default()


def render(panel_x, panel_w, angle, title):
    moving = lid.copy()
    moving.apply_transform(
        trimesh.transformations.rotation_matrix(
            np.radians(-angle), (1, 0, 0), point=(0, 24.4, 5.8)
        )
    )
    meshes = [(case, np.array([80, 96, 107])), (moving, np.array([225, 119, 55]))]
    all_v = np.vstack([mesh.vertices for mesh, _ in meshes])
    target = (all_v.min(axis=0) + all_v.max(axis=0)) / 2
    eye = target + np.array([54.0, -67.0, 49.0])
    forward = target - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)

    def project(v):
        q = v - target
        return np.stack((q @ right, -(q @ up)), axis=-1)

    all_p = project(all_v)
    lo, hi = all_p.min(axis=0), all_p.max(axis=0)
    scale = min((panel_w - 130) / (hi[0] - lo[0]), 720 / (hi[1] - lo[1]))
    center = (lo + hi) / 2
    offset = np.array([panel_x + panel_w / 2, 500])

    light = np.array([0.35, -0.35, 0.87])
    light /= np.linalg.norm(light)
    for mesh, color in meshes:
        tri = mesh.triangles
        normals = mesh.face_normals
        pts = (project(tri.reshape(-1, 3)).reshape(-1, 3, 2) - center) * scale + offset
        depths = ((tri.reshape(-1, 3) - eye) @ forward).reshape(-1, 3)
        shade = np.clip(0.70 + 0.30 * (normals @ light), 0.45, 1.0)
        for t, d, s in zip(pts, depths, shade):
            x0 = max(panel_x, int(np.floor(t[:, 0].min())))
            x1 = min(panel_x + panel_w - 1, int(np.ceil(t[:, 0].max())))
            y0 = max(110, int(np.floor(t[:, 1].min())))
            y1 = min(825, int(np.ceil(t[:, 1].max())))
            if x1 < x0 or y1 < y0:
                continue
            xa, ya = t[0]
            xb, yb = t[1]
            xc, yc = t[2]
            denom = (yb - yc) * (xa - xc) + (xc - xb) * (ya - yc)
            if abs(denom) < 1e-8:
                continue
            yy, xx = np.mgrid[y0:y1 + 1, x0:x1 + 1]
            w0 = ((yb - yc) * (xx - xc) + (xc - xb) * (yy - yc)) / denom
            w1 = ((yc - ya) * (xx - xc) + (xa - xc) * (yy - yc)) / denom
            w2 = 1.0 - w0 - w1
            inside = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
            if not inside.any():
                continue
            z = w0 * d[0] + w1 * d[1] + w2 * d[2]
            sub_depth = depth_buffer[y0:y1 + 1, x0:x1 + 1]
            visible = inside & (z < sub_depth)
            if visible.any():
                sub_depth[visible] = z[visible]
                pixels[y0:y1 + 1, x0:x1 + 1][visible] = np.clip(color * s, 0, 255)


render(0, 900, 0, "Kapalı konum")
render(900, 900, 105, "105° açık konum")
im = Image.fromarray(pixels)
draw = ImageDraw.Draw(im)
draw.text((55, 42), "Kapalı konum", fill="#202a33", font=font)
draw.text((955, 42), "105° açık konum", fill="#202a33", font=font)
for panel_x in (0, 900):
    draw.text((panel_x + 55, 855), "Koyu: kasa    Turuncu: kapak", fill="#5c6975", font=small)
draw.line((900, 30, 900, 890), fill="#d9e0e7", width=2)
im.save(ROOT / "IMU_menteseli_onizleme.png")

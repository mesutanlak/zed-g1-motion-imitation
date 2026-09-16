"""Generate a ZED 2i adapter for the MakerWorld universal octagon socket.

Requires: trimesh, manifold3d, numpy, matplotlib.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import trimesh


OUT = Path(__file__).resolve().parent
PLATE_X = 80.0
PLATE_Y = 42.0
PLATE_H = 6.0
CORNER_R = 6.0
PEG_H = 5.8
CHAMFER_H = 0.7
SCREW_CLEARANCE_D = 6.8
COUNTERBORE_D = 12.0
COUNTERBORE_DEPTH = 4.8


def union(parts):
    return trimesh.boolean.union(parts, engine="manifold")


def difference(body, cutters):
    return trimesh.boolean.difference([body, *cutters], engine="manifold")


def rounded_plate():
    """Rounded 80 x 42 x 6 mm plate, bottom at z=PEG_H."""
    z = PEG_H + PLATE_H / 2
    parts = [
        trimesh.creation.box((PLATE_X - 2 * CORNER_R, PLATE_Y, PLATE_H)),
        trimesh.creation.box((PLATE_X, PLATE_Y - 2 * CORNER_R, PLATE_H)),
    ]
    for x in (-PLATE_X / 2 + CORNER_R, PLATE_X / 2 - CORNER_R):
        for y in (-PLATE_Y / 2 + CORNER_R, PLATE_Y / 2 - CORNER_R):
            c = trimesh.creation.cylinder(radius=CORNER_R, height=PLATE_H, sections=48)
            c.apply_translation((x, y, 0))
            parts.append(c)
    for p in parts:
        p.apply_translation((0, 0, z))
    return union(parts)


def octagon_peg(radius):
    """Chamfered regular-octagon male peg, bottom at z=0."""
    # Rotate by 22.5 degrees so the octagon has horizontal/vertical flats.
    angles = np.arange(8) * np.pi / 4 + np.pi / 8
    rings = [
        (radius - 0.65, 0.0),
        (radius, CHAMFER_H),
        (radius, PEG_H),
    ]
    vertices = []
    for r, z in rings:
        vertices.extend([(r * np.cos(a), r * np.sin(a), z) for a in angles])
    vertices.extend([(0, 0, 0), (0, 0, PEG_H)])
    bottom_center, top_center = 24, 25
    faces = []
    for ring in range(2):
        a0, b0 = ring * 8, (ring + 1) * 8
        for i in range(8):
            j = (i + 1) % 8
            faces.extend([(a0 + i, a0 + j, b0 + j), (a0 + i, b0 + j, b0 + i)])
    for i in range(8):
        j = (i + 1) % 8
        faces.append((bottom_center, j, i))
        faces.append((top_center, 16 + i, 16 + j))
    return trimesh.Trimesh(vertices=np.array(vertices), faces=np.array(faces), process=True)


def adapter(radius):
    body = union([rounded_plate(), octagon_peg(radius)])

    through = trimesh.creation.cylinder(
        radius=SCREW_CLEARANCE_D / 2, height=PLATE_H + PEG_H + 2, sections=64
    )
    through.apply_translation((0, 0, (PLATE_H + PEG_H) / 2))

    counterbore = trimesh.creation.cylinder(
        radius=COUNTERBORE_D / 2, height=COUNTERBORE_DEPTH + 0.2, sections=64
    )
    counterbore.apply_translation((0, 0, COUNTERBORE_DEPTH / 2 - 0.1))
    result = difference(body, [through, counterbore])
    result.remove_unreferenced_vertices()
    return result


def fit_test(radius):
    """Quick socket-fit coupon: peg plus a thin finger plate."""
    handle = trimesh.creation.box((44.0, 44.0, 2.4))
    handle.apply_translation((0, 0, PEG_H + 1.2))
    result = union([octagon_peg(radius), handle])
    result.remove_unreferenced_vertices()
    return result


def export_mesh(mesh, stem):
    mesh.export(OUT / f"{stem}.stl")
    # Geometry-only 3MF; Bambu Studio can assign the X1E profile on import.
    try:
        mesh.export(OUT / f"{stem}.3mf")
    except Exception as exc:
        print(f"3MF export skipped for {stem}: {exc}")


def validate(name, mesh):
    ext = mesh.extents
    if not mesh.is_watertight or not mesh.is_winding_consistent or mesh.volume <= 0:
        raise RuntimeError(f"{name} failed mesh validation")
    print(
        f"{name}: watertight={mesh.is_watertight}, "
        f"size={ext[0]:.2f} x {ext[1]:.2f} x {ext[2]:.2f} mm, "
        f"volume={mesh.volume:.1f} mm^3"
    )


def preview(mesh):
    fig = plt.figure(figsize=(11, 4), constrained_layout=True)
    views = [(28, -42, "İzometrik"), (90, -90, "Üst"), (-90, 90, "Alt")]
    for idx, (elev, azim, title) in enumerate(views, 1):
        ax = fig.add_subplot(1, 3, idx, projection="3d")
        # Plot triangles directly so preview generation needs no OpenGL renderer.
        tri = mesh.triangles
        poly = __import__("mpl_toolkits.mplot3d.art3d", fromlist=["Poly3DCollection"]).Poly3DCollection(
            tri, facecolor="#43a6c6", edgecolor="#15566b", linewidth=0.15, alpha=0.96
        )
        ax.add_collection3d(poly)
        bounds = mesh.bounds
        center = bounds.mean(axis=0)
        radius = max(mesh.extents) / 2
        ax.set_xlim(center[0] - radius, center[0] + radius)
        ax.set_ylim(center[1] - radius, center[1] + radius)
        ax.set_zlim(max(-2, center[2] - radius), center[2] + radius)
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title)
        ax.set_axis_off()
        ax.set_box_aspect((1, 1, 0.45))
    fig.suptitle("ZED 2i – evrensel sekizgen tripod adaptörü (standart tolerans)", fontsize=13)
    fig.savefig(OUT / "zed2i_adapter_preview.png", dpi=180, transparent=False)
    plt.close(fig)


def main():
    variants = {
        # MakerWorld drawing: nominal R18, 0.05 mm reduction per face.
        # R17.90 gives ~33.08 mm across flats; R17.75 gives ~32.80 mm.
        "standard": 17.90,
        "loose": 17.75,
    }
    built = {}
    for name, radius in variants.items():
        main_mesh = adapter(radius)
        coupon = fit_test(radius)
        validate(f"adapter_{name}", main_mesh)
        validate(f"fit_test_{name}", coupon)
        export_mesh(main_mesh, f"zed2i_octagon_adapter_{name}")
        export_mesh(coupon, f"octagon_fit_test_{name}")
        built[name] = main_mesh
    preview(built["standard"])


if __name__ == "__main__":
    main()

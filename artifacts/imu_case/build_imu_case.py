"""Build a hinged, snap-closed lid for the supplied IMU case STL.

Dimensions are millimetres. The original case is left untouched. The two
exported STL files share the same assembly coordinates; the lid can be flipped
onto its flat outer face in a slicer for printing.
"""

from pathlib import Path
import sys

DEPS = Path(r"C:\Users\mesut\AppData\Local\Temp\imu-cad-deps")
sys.path.insert(0, str(DEPS))

import numpy as np
import trimesh


SOURCE = Path(r"C:\Users\mesut\OneDrive\Masaüstü\IMU case.stl")
OUT = Path(__file__).resolve().parent


def box(x0, x1, y0, y1, z0, z1):
    mesh = trimesh.creation.box((x1 - x0, y1 - y0, z1 - z0))
    mesh.apply_translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    return mesh


def cylinder_x(x0, x1, y, z, radius):
    mesh = trimesh.creation.cylinder(radius, x1 - x0, sections=96)
    mesh.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, (0, 1, 0)))
    mesh.apply_translation(((x0 + x1) / 2, y, z))
    return mesh


def frustum_x(x0, x1, radius0, radius1, sections=96):
    """Closed circular frustum on the X axis, centred on Y=Z=0."""
    theta = np.linspace(0, 2 * np.pi, sections, endpoint=False)
    ring0 = np.column_stack(
        (np.full(sections, x0), radius0 * np.cos(theta), radius0 * np.sin(theta))
    )
    ring1 = np.column_stack(
        (np.full(sections, x1), radius1 * np.cos(theta), radius1 * np.sin(theta))
    )
    vertices = np.vstack((ring0, ring1))
    faces = []
    for i in range(1, sections - 1):
        faces.extend(((0, i + 1, i), (sections, sections + i, sections + i + 1)))
    for i in range(sections):
        j = (i + 1) % sections
        faces.extend(((i, j, sections + j), (i, sections + j, sections + i)))
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
    if mesh.volume < 0:
        mesh.invert()
    return mesh


def yz_prism(x0, x1, points):
    """Extrude a convex YZ polygon along X."""
    n = len(points)
    vertices = np.array(
        [(x, y, z) for x in (x0, x1) for y, z in points], dtype=float
    )
    faces = []
    for i in range(1, n - 1):
        faces.append((0, i + 1, i))
        faces.append((n, n + i, n + i + 1))
    for i in range(n):
        j = (i + 1) % n
        faces.extend(((i, j, n + j), (i, n + j, n + i)))
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
    if mesh.volume < 0:
        mesh.invert()
    return mesh


def union(*solids):
    return trimesh.boolean.union(list(solids), engine="manifold")


def difference(solid, *cutters):
    return trimesh.boolean.difference([solid, *cutters], engine="manifold")


base = trimesh.load_mesh(SOURCE, process=True)
assert base.is_watertight

# Five alternating hinge knuckles: three on the case, two on the lid.
# The axis is outside the existing rear wall and above its strap feature.
axis_y, axis_z = 24.4, 5.8
barrel_radius, pin_radius = 1.9, 1.05  # 3.8 mm OD, 2.1 mm nominal pin hole
case_knuckles = [(2.5, 9.0), (15.0, 20.0), (26.0, 32.5)]
lid_knuckles = [(9.35, 14.65), (20.35, 25.65)]

case_parts = [base]
for x0, x1 in case_knuckles:
    case_parts.append(cylinder_x(x0, x1, axis_y, axis_z, barrel_radius))
    case_parts.append(
        yz_prism(
            x0, x1,
            [(21.8, 3.5), (22.5, 3.5), (24.4, 4.8),
             (24.4, 5.8), (21.8, 5.0)],
        )
    )

# Two shallow catches sit clear of the front mounting feature.
catch_profile = [(-1.0, 2.1), (-1.85, 2.1), (-1.85, 2.55),
                 (-1.3, 2.95), (-1.0, 2.95)]
for x0, x1 in [(5.0, 7.9), (27.1, 30.0)]:
    case_parts.append(yz_prism(x0, x1, catch_profile))

case = union(*case_parts)
case = difference(case, cylinder_x(2.3, 32.7, axis_y, axis_z, pin_radius))

# Lid rests on the original rim. A shallow three-sided locating skirt fits
# within the 35 x 21.5 mm opening; short right corners leave the side slot free.
lid_plate = box(-1.0, 36.0, -1.0, 22.5, 5.05, 7.8)
for x0, x1 in [(2.2, 9.2), (14.8, 20.2), (25.8, 32.8)]:
    lid_plate = difference(lid_plate, box(x0, x1, 21.65, 27.0, 5.0, 8.0))

lid_parts = [lid_plate]
lid_parts.extend([
    box(0.35, 34.65, 0.35, 1.05, 4.0, 5.05),
    box(0.35, 34.65, 20.45, 21.15, 4.0, 5.05),
    box(0.35, 1.05, 1.05, 20.45, 4.0, 5.05),
    box(33.95, 34.65, 1.05, 2.5, 4.0, 5.05),
    box(33.95, 34.65, 19.0, 20.45, 4.0, 5.05),
])
for x0, x1 in lid_knuckles:
    lid_parts.append(cylinder_x(x0, x1, axis_y, axis_z, barrel_radius))
    lid_parts.append(
        yz_prism(
            x0, x1,
            [(22.2, 5.15), (22.5, 5.15), (24.4, 5.8),
             (24.4, 7.0), (22.2, 7.0)],
        )
    )

# Flexible latch arms. Their 0.3 mm engagement is intended for PETG.
for x0, x1 in [(4.7, 8.2), (26.8, 30.3)]:
    lid_parts.append(box(x0, x1, -2.7, -1.95, 1.2, 6.0))
    lid_parts.append(box(x0, x1, -2.7, -0.5, 5.7, 7.8))
    lid_parts.append(
        yz_prism(x0, x1, [(-1.95, 1.2), (-1.55, 1.9), (-1.95, 1.9)])
    )

lid = union(*lid_parts)
lid = difference(lid, cylinder_x(9.25, 25.75, axis_y, axis_z, pin_radius))

assert case.is_watertight and lid.is_watertight
assert case.volume > base.volume and lid.volume > 0

overlap = trimesh.boolean.intersection([case, lid], engine="manifold")
if overlap is not None and overlap.volume > 0.01:
    centers = overlap.triangles_center
    print("Overlap bounds:", overlap.bounds)
    print("Overlap face centers:", np.unique(np.round(centers, 1), axis=0)[:100])
    raise RuntimeError(f"Case/lid overlap: {overlap.volume:.3f} mm³")

case.export(OUT / "IMU_case_menteseli.stl")
lid.export(OUT / "IMU_kapak_montaj_konumu.stl")

# Same physical lid, rotated flat onto its outside face for support-light FDM
# slicing. Hinge and latch features face upward in this file.
lid_print = lid.copy()
lid_print.apply_transform(
    trimesh.transformations.rotation_matrix(np.pi, (1, 0, 0), point=(0, 0, 0))
)
lid_print.apply_translation((0, 0, -lid_print.bounds[0, 2]))
lid_print.export(OUT / "IMU_kapak_baski_konumu.stl")

# Printable D-section hinge pin: 1.85 mm shaft in the 2.1 mm hinge bore.
# A 3.6 mm head stops it at the left outer knuckle; the opposite end accepts
# a separate push-on cap. The underside flat gives the long, thin pin bed grip.
pin = union(
    cylinder_x(1.3, 2.5, 0, 0, 1.8),
    cylinder_x(2.45, 34.2, 0, 0, 0.925),
    frustum_x(34.15, 35.0, 0.925, 0.65),
)
pin = difference(pin, box(0, 36, -3, 3, -3, -0.7))
assert pin.is_watertight
pin_print = pin.copy()
pin_print.apply_translation(-pin_print.bounds[0])
pin_print.export(OUT / "IMU_mentese_pimi_baski.stl")

# The cap sits beyond the right case knuckle. Its blind 1.8 mm nominal bore
# provides a small press fit, subject to the printer's actual hole tolerance.
cap = difference(
    cylinder_x(32.7, 35.9, 0, 0, 1.9),
    cylinder_x(32.5, 35.1, 0, 0, 0.90),
    frustum_x(32.5, 33.2, 1.05, 0.90),
)
assert cap.is_watertight

pin_assembly = pin.copy()
pin_assembly.apply_translation((0, axis_y, axis_z))
cap_assembly = cap.copy()
cap_assembly.apply_translation((0, axis_y, axis_z))
for part_name, part in [("pin", pin_assembly), ("cap", cap_assembly)]:
    for solid_name, solid in [("case", case), ("lid", lid)]:
        collision = trimesh.boolean.intersection([part, solid], engine="manifold")
        if collision is not None and collision.volume > 0.01:
            raise RuntimeError(f"{part_name}/{solid_name} overlap: {collision.volume:.3f} mm³")

cap_print = cap.copy()
cap_print.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, (0, 1, 0)))
cap_print.apply_translation(-cap_print.bounds[0])
cap_print.export(OUT / "IMU_pim_uc_kapagi_baski.stl")

print("Case:", case.bounds, "volume", round(case.volume, 2))
print("Lid:", lid.bounds, "volume", round(lid.volume, 2))
print("Closed-position overlap:", 0 if overlap is None else overlap.volume)
print("Pin:", pin.bounds, "volume", round(pin.volume, 2))
print("Pin cap:", cap.bounds, "volume", round(cap.volume, 2))

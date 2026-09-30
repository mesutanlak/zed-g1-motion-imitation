"""Raspberry Pi 4 ve 160 x 52.5 x 9 mm breadboard icin giyilebilir tabanlar.

Birimler milimetredir. STL dosyalari baski yonunde, tabanlari Z=0 olacak
sekilde uretilir. Raspberry Pi olculeri RP-008343 resmi mekanik ciziminden:
PCB 85 x 56 mm, delik duzeni 58 x 49 mm, kart deligi 2.7 mm.
"""

from __future__ import annotations

from pathlib import Path
import json
import math
import sys


ROOT = Path(__file__).resolve().parent
LOCAL_DEPS = ROOT.parent / "hw290_snapfit_v1" / "_deps"
if LOCAL_DEPS.exists():
    sys.path.insert(0, str(LOCAL_DEPS))

import numpy as np
import trimesh as tm


PRINT = ROOT / "STL_BASKI"
REFERENCE = ROOT / "MONTAJ_REFERANS_BASILMAZ"
PRINT.mkdir(parents=True, exist_ok=True)
REFERENCE.mkdir(parents=True, exist_ok=True)

# Genel FDM degerleri.
BASE_T = 2.60
STRAP_SLOT_LENGTH = 28.0       # 25 mm cirta bosluk.
STRAP_SLOT_WIDTH = 4.2         # Kelepce veya ince kayis.
SNAP_VERTICAL_CLEARANCE = 0.25

# Raspberry Pi 4 resmi olculeri.
PI_X = 85.0
PI_Y = 56.0
PI_BOARD_T = 1.60
PI_STANDOFF_H = 6.00
PI_BOARD_Z = BASE_T + PI_STANDOFF_H
PI_TOP_Z = PI_BOARD_Z + PI_BOARD_T
PI_HOLES = [(x, y) for x in (3.5, 61.5) for y in (3.5, 52.5)]
M2_CLEARANCE_D = 2.30
M2_NUT_AF = 4.30
M2_NUT_DEPTH = 1.90

# Kullanicinin olctugu breadboard.
BREAD_X = 160.0
BREAD_Y = 52.5
BREAD_Z = 9.0
BREAD_SIDE_CLEARANCE = 0.25
BREAD_END_CLEARANCE = 0.30
BREAD_BOTTOM_Z = BASE_T
BREAD_TOP_Z = BREAD_BOTTOM_Z + BREAD_Z


def box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> tm.Trimesh:
    mesh = tm.creation.box((x1 - x0, y1 - y0, z1 - z0))
    mesh.apply_translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    return mesh


def cylinder(
    x: float, y: float, z0: float, z1: float, radius: float, sections: int = 48
) -> tm.Trimesh:
    mesh = tm.creation.cylinder(radius=radius, height=z1 - z0, sections=sections)
    mesh.apply_translation((x, y, (z0 + z1) / 2))
    return mesh


def union(*parts: tm.Trimesh) -> tm.Trimesh:
    return tm.boolean.union(list(parts), engine="manifold")


def difference(solid: tm.Trimesh, *cutters: tm.Trimesh) -> tm.Trimesh:
    return tm.boolean.difference([solid, *cutters], engine="manifold")


def rounded_box(
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    z0: float,
    z1: float,
    radius: float,
) -> tm.Trimesh:
    return union(
        box(x0 + radius, x1 - radius, y0, y1, z0, z1),
        box(x0, x1, y0 + radius, y1 - radius, z0, z1),
        *[
            cylinder(x, y, z0, z1, radius)
            for x in (x0 + radius, x1 - radius)
            for y in (y0 + radius, y1 - radius)
        ],
    )


def hex_prism(
    x: float, y: float, z0: float, z1: float, across_flats: float
) -> tm.Trimesh:
    mesh = cylinder(0, 0, z0, z1, across_flats / math.sqrt(3.0), sections=6)
    mesh.apply_transform(tm.transformations.rotation_matrix(math.radians(30), (0, 0, 1)))
    mesh.apply_translation((x, y, 0))
    return mesh


def x_wedge(
    x0: float,
    x1: float,
    outer_y: float,
    inner_y: float,
    lower_z: float,
    upper_z: float,
) -> tm.Trimesh:
    """X boyunca uzanan, baskida desteksiz tirnak rampasi."""
    yz = ((outer_y, lower_z), (inner_y, lower_z), (outer_y, upper_z))
    vertices = np.array(
        [(x, y, z) for x in (x0, x1) for y, z in yz], dtype=float
    )
    faces = np.array(
        [
            (0, 2, 1),
            (3, 4, 5),
            (0, 1, 4), (0, 4, 3),
            (1, 2, 5), (1, 5, 4),
            (2, 0, 3), (2, 3, 5),
        ],
        dtype=int,
    )
    mesh = tm.Trimesh(vertices=vertices, faces=faces, process=True)
    if mesh.volume < 0:
        mesh.invert()
    return mesh


def snap_clip_y(
    center_x: float,
    width_x: float,
    board_edge_y: float,
    direction: int,
    wall_outer_y: float,
    base_z: float,
    board_top_z: float,
    wall_t: float = 1.35,
    lip: float = 0.85,
) -> tm.Trimesh:
    """direction +1: alt kenardan +Y'ye; -1: ust kenardan -Y'ye tutar."""
    x0, x1 = center_x - width_x / 2, center_x + width_x / 2
    if direction > 0:
        wall = box(x0, x1, wall_outer_y, board_edge_y, base_z, board_top_z + 1.75)
        wedge = x_wedge(
            x0,
            x1,
            board_edge_y,
            board_edge_y + lip,
            board_top_z + SNAP_VERTICAL_CLEARANCE,
            board_top_z + 1.75,
        )
    else:
        wall = box(x0, x1, board_edge_y, wall_outer_y, base_z, board_top_z + 1.75)
        wedge = x_wedge(
            x0,
            x1,
            board_edge_y,
            board_edge_y - lip,
            board_top_z + SNAP_VERTICAL_CLEARANCE,
            board_top_z + 1.75,
        )
    return union(wall, wedge)


def pi_carrier() -> tuple[tm.Trimesh, tm.Trimesh, list[tm.Trimesh]]:
    # Taban ve iki yandaki 25 mm kayis kulagi.
    floor = rounded_box(-4.0, 89.0, -4.0, 60.0, 0, BASE_T, 4.0)
    left_ear = rounded_box(-16.0, -2.5, 9.0, 47.0, 0, 3.0, 3.0)
    right_ear = rounded_box(87.5, 101.0, 9.0, 47.0, 0, 3.0, 3.0)

    posts = [cylinder(x, y, BASE_T, PI_BOARD_Z, 3.8) for x, y in PI_HOLES]

    # Kart kenarindan tutan dort esnek tirnak. Portlar icin sag/sol kenarlar acik.
    clips: list[tm.Trimesh] = []
    for x in (3.5, 61.5):
        clips.append(
            snap_clip_y(
                x, 7.0, -0.25, +1, -2.0, BASE_T, PI_TOP_Z,
                wall_t=1.35, lip=0.80,
            )
        )
        clips.append(
            snap_clip_y(
                x, 7.0, PI_Y + 0.25, -1, PI_Y + 2.0, BASE_T, PI_TOP_Z,
                wall_t=1.35, lip=0.80,
            )
        )

    # Yalniz PCB alt seviyesine kadar cikan kose dayamalari X kaymasini keser.
    x_stops = [
        box(-1.65, -0.25, 7, 13, BASE_T, PI_BOARD_Z + 0.9),
        box(-1.65, -0.25, 43, 49, BASE_T, PI_BOARD_Z + 0.9),
        box(PI_X + 0.25, PI_X + 1.65, 7, 13, BASE_T, PI_BOARD_Z + 0.9),
        box(PI_X + 0.25, PI_X + 1.65, 43, 49, BASE_T, PI_BOARD_Z + 0.9),
    ]

    carrier = union(floor, left_ear, right_ear, *posts, *clips, *x_stops)

    cuts: list[tm.Trimesh] = [
        # Gogus kayisi / genis cirt yuvalari.
        box(-12.8, -8.6, 14.0, 42.0, -1, 5),
        box(93.6, 97.8, 14.0, 42.0, -1, 5),
        # PCB altinda hava ve agirlik azaltma pencereleri.
        rounded_box(10, 27, 11, 45, -1, BASE_T + 1, 2.0),
        rounded_box(31, 48, 11, 45, -1, BASE_T + 1, 2.0),
        rounded_box(67, 81, 11, 45, -1, BASE_T + 1, 2.0),
    ]
    for x, y in PI_HOLES:
        cuts.append(cylinder(x, y, -1, PI_BOARD_Z + 1, M2_CLEARANCE_D / 2))
        cuts.append(hex_prism(x, y, -0.1, M2_NUT_DEPTH, M2_NUT_AF))
    carrier = difference(carrier, *cuts)

    # Temsili elektronik: sadece onizleme/carpisma kontrolu.
    pcb = box(0, PI_X, 0, PI_Y, PI_BOARD_Z, PI_TOP_Z)
    pcb = difference(
        pcb,
        *[cylinder(x, y, PI_BOARD_Z - 0.2, PI_TOP_Z + 0.2, 1.35) for x, y in PI_HOLES],
    )
    ports = [
        box(70, 94, 2, 17, PI_TOP_Z, PI_TOP_Z + 15),
        box(70, 94, 20, 35, PI_TOP_Z, PI_TOP_Z + 15),
        box(66, 89, 39, 55, PI_TOP_Z, PI_TOP_Z + 14),
        box(8, 18, -3, 4, PI_TOP_Z, PI_TOP_Z + 4),
        box(7, 58, 48, 53, PI_TOP_Z, PI_TOP_Z + 8),
    ]
    return carrier, pcb, ports


def breadboard_carrier() -> tuple[tm.Trimesh, tm.Trimesh]:
    half_x = BREAD_X / 2
    half_y = BREAD_Y / 2
    inner_x = half_x + BREAD_END_CLEARANCE
    inner_y = half_y + BREAD_SIDE_CLEARANCE
    outer_x = inner_x + 2.3
    outer_y = inner_y + 2.3

    floor = rounded_box(-outer_x, outer_x, -outer_y, outer_y, 0, BASE_T, 3.0)
    left_ear = rounded_box(-96.5, -outer_x + 1.0, -19, 19, 0, 3.0, 3.0)
    right_ear = rounded_box(outer_x - 1.0, 96.5, -19, 19, 0, 3.0, 3.0)

    # Uc dayamalari ileri-geri kaymayi engeller; ust yuzeye cikmaz.
    end_stops = [
        box(-outer_x, -inner_x, -half_y + 4, half_y - 4, BASE_T, BREAD_BOTTOM_Z + 5.0),
        box(inner_x, outer_x, -half_y + 4, half_y - 4, BASE_T, BREAD_BOTTOM_Z + 5.0),
    ]

    clips: list[tm.Trimesh] = []
    for x in (-55.0, 0.0, 55.0):
        clips.append(
            snap_clip_y(
                x, 10.0, -inner_y, +1, -outer_y, BASE_T, BREAD_TOP_Z,
                wall_t=1.35, lip=0.90,
            )
        )
        clips.append(
            snap_clip_y(
                x, 10.0, inner_y, -1, outer_y, BASE_T, BREAD_TOP_Z,
                wall_t=1.35, lip=0.90,
            )
        )

    carrier = union(floor, left_ear, right_ear, *end_stops, *clips)
    carrier = difference(
        carrier,
        box(-92.8, -88.6, -14.0, 14.0, -1, 5),
        box(88.6, 92.8, -14.0, 14.0, -1, 5),
        # Dar kelepceler icin iki ek cift yuva.
        box(-74, -70.5, -outer_y + 3, -outer_y + 9, -1, 5),
        box(-74, -70.5, outer_y - 9, outer_y - 3, -1, 5),
        box(70.5, 74, -outer_y + 3, -outer_y + 9, -1, 5),
        box(70.5, 74, outer_y - 9, outer_y - 3, -1, 5),
    )
    reference = box(-half_x, half_x, -half_y, half_y, BREAD_BOTTOM_Z, BREAD_TOP_Z)
    return carrier, reference


def validate_and_export(name: str, mesh: tm.Trimesh) -> dict[str, object]:
    mesh = mesh.copy()
    mesh.remove_unreferenced_vertices()
    mesh.merge_vertices()
    if not mesh.is_watertight:
        raise RuntimeError(f"{name}: mesh watertight degil")
    if not mesh.is_winding_consistent:
        raise RuntimeError(f"{name}: winding tutarsiz")
    if mesh.volume <= 0:
        raise RuntimeError(f"{name}: hacim gecersiz")
    shells = mesh.split(only_watertight=False)
    if len(shells) != 1:
        raise RuntimeError(f"{name}: {len(shells)} ayri govde var")
    mesh.apply_translation((0, 0, -mesh.bounds[0, 2]))
    mesh.export(PRINT / f"{name}.stl")
    return {
        "file": f"{name}.stl",
        "bounds_mm": [round(float(v), 3) for v in mesh.extents],
        "volume_cm3": round(float(mesh.volume / 1000.0), 3),
        "triangles": int(len(mesh.faces)),
        "watertight": bool(mesh.is_watertight),
        "single_shell": len(mesh.split()) == 1,
    }


def main() -> None:
    pi, pi_pcb, pi_ports = pi_carrier()
    bread, bread_ref = breadboard_carrier()

    results = [
        validate_and_export("01_Raspberry_Pi4_acik_gecmeli_gogus_tabani_1_adet", pi),
        validate_and_export("02_Breadboard_160x52_5_gecmeli_gogus_tabani_1_adet", bread),
    ]
    pi.export(REFERENCE / "PI4_tabani_montaj.stl")
    pi_pcb.export(REFERENCE / "PI4_PCB_temsili.stl")
    bread.export(REFERENCE / "Breadboard_tabani_montaj.stl")
    bread_ref.export(REFERENCE / "Breadboard_temsili.stl")

    # Montaj hacimleri baski parcalarina girmemeli.
    pi_collision = tm.boolean.intersection([pi, pi_pcb], engine="manifold")
    bread_collision = tm.boolean.intersection([bread, bread_ref], engine="manifold")
    checks = {
        "pi_reference_collision_mm3": round(float(pi_collision.volume), 6),
        "breadboard_reference_collision_mm3": round(float(bread_collision.volume), 6),
        "pi_hole_grid_mm": [58.0, 49.0],
        "pi_board_mm": [PI_X, PI_Y, PI_BOARD_T],
        "breadboard_mm": [BREAD_X, BREAD_Y, BREAD_Z],
        "pi_snap_vertical_clearance_mm": SNAP_VERTICAL_CLEARANCE,
        "breadboard_side_clearance_each_mm": BREAD_SIDE_CLEARANCE,
        "breadboard_end_clearance_each_mm": BREAD_END_CLEARANCE,
    }
    if checks["pi_reference_collision_mm3"] > 0.01:
        raise RuntimeError(f"Pi montaj carpismasi: {checks['pi_reference_collision_mm3']}")
    if checks["breadboard_reference_collision_mm3"] > 0.01:
        raise RuntimeError(
            f"Breadboard montaj carpismasi: {checks['breadboard_reference_collision_mm3']}"
        )
    (ROOT / "mesh_validation.json").write_text(
        json.dumps({"meshes": results, "assembly_checks": checks}, indent=2),
        encoding="utf-8",
    )
    print(json.dumps({"meshes": results, "assembly_checks": checks}, indent=2))


if __name__ == "__main__":
    main()

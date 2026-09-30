"""HW-290 / GY-87 style board için vidasız giyilebilir kasa.

Kart ölçüsü: 21.60 x 17.20 x 2.25 mm.
Montaj: kablolar arka ağızdan geçirilir, kart arkaya kaydırılır ve
öndeki orta esnek tırnağa bastırılarak kilitlenir.

Gerekli paketler daha önceki CAD ortamındaki imu-cad-deps klasöründen
yüklenir. Birimler milimetredir.
"""

from pathlib import Path
import json
import math
import sys

LOCAL_DEPS = Path(__file__).resolve().parent / "_deps"
LEGACY_DEPS = Path(r"C:\Users\mesut\AppData\Local\Temp\imu-cad-deps")
for dependency_path in (LEGACY_DEPS, LOCAL_DEPS):
    if dependency_path.exists():
        sys.path.insert(0, str(dependency_path))

import numpy as np
import trimesh as tm


ROOT = Path(__file__).resolve().parent
PRINT = ROOT / "STL_BASKI"
REFERENCE = ROOT / "MONTAJ_REFERANS_BASILMAZ"
PRINT.mkdir(parents=True, exist_ok=True)
REFERENCE.mkdir(parents=True, exist_ok=True)

# Kullanıcının ölçtüğü gerçek kart ölçüleri.
PCB_X = 21.60
PCB_Y = 17.20
PCB_T = 2.25

# Kart bileşenleri aşağı bakar. PCB yalnızca boş kenar şeritlerinden taşınır.
PCB_Z = 7.00
PCB_TOP = PCB_Z + PCB_T

# Kullanıcının daha önce iyi oturduğunu söylediği ölçü mastarına yakın boşluk.
SIDE_CLEARANCE = 0.18
REAR_CLEARANCE = 0.12


def box(x0, x1, y0, y1, z0, z1):
    mesh = tm.creation.box((x1 - x0, y1 - y0, z1 - z0))
    mesh.apply_translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    return mesh


def cyl(x, y, z0, z1, r, sections=64):
    mesh = tm.creation.cylinder(radius=r, height=z1 - z0, sections=sections)
    mesh.apply_translation((x, y, (z0 + z1) / 2))
    return mesh


def union(*parts):
    return tm.boolean.union(list(parts), engine="manifold")


def difference(mesh, *cuts):
    return tm.boolean.difference([mesh, *cuts], engine="manifold")


def rounded(x0, x1, y0, y1, z0, z1, r=2.0):
    return union(
        box(x0 + r, x1 - r, y0, y1, z0, z1),
        box(x0, x1, y0 + r, y1 - r, z0, z1),
        *[
            cyl(x, y, z0, z1, r)
            for x in (x0 + r, x1 - r)
            for y in (y0 + r, y1 - r)
        ],
    )


def yz_prism(x0, x1, points):
    """YZ düzlemindeki dışbükey çokgeni X boyunca uzat."""
    n = len(points)
    vertices = np.array([(x, y, z) for x in (x0, x1) for y, z in points])
    faces = []
    for i in range(1, n - 1):
        faces.extend(((0, i + 1, i), (n, n + i, n + i + 1)))
    for i in range(n):
        j = (i + 1) % n
        faces.extend(((i, j, n + j), (i, n + j, n + i)))
    mesh = tm.Trimesh(vertices=vertices, faces=faces, process=True)
    if mesh.volume < 0:
        mesh.invert()
    return mesh


def moved(mesh, xyz=(0, 0, 0)):
    result = mesh.copy()
    result.apply_translation(xyz)
    return result


def printable(mesh):
    result = mesh.copy()
    result.apply_translation((0, 0, -result.bounds[0, 2]))
    return result


def build_body():
    # Ana taban ve kol cırtı için iki geniş kulak.
    floor = rounded(-16, 16, -14, 14.5, 0, 2.6, 2.4)
    ears = [
        rounded(-27, -15.3, -11, 11, 0, 3.2, 2.0),
        rounded(15.3, 27, -11, 11, 0, 3.2, 2.0),
    ]

    # Açık üstlü koruyucu çevre duvarı.
    shell = difference(
        rounded(-16, 16, -14, 14.5, 2.4, 13.0, 2.4),
        rounded(-13.45, 13.45, -11.35, 11.95, 2.3, 14.0, 1.2),
    )

    # PCB yalnızca iki uzun boş kenarından taşınır; orta alan bileşenlere açıktır.
    edge = PCB_X / 2
    ledges = [
        box(-11.35, -edge + 1.15, -8.55, 8.55, 2.5, PCB_Z),
        box(edge - 1.15, 11.35, -8.55, 8.55, 2.5, PCB_Z),
    ]

    # Sol/sağ kılavuzlar kartın gövdeye göre dönmesini engeller.
    inner_x = edge + SIDE_CLEARANCE
    guides = [
        box(-11.45, -inner_x, -7.4, 7.4, PCB_Z, PCB_TOP + 0.45),
        box(inner_x, 11.45, -7.4, 7.4, PCB_Z, PCB_TOP + 0.45),
    ]

    # BNO055 kasasındaki prensip: kart arkaya kayarken bu dört kısa dudak
    # kartın boş yan kenarlarını üstten kavrar. Arka header bölgesi açıktır.
    lip_under = PCB_TOP + 0.05
    lips = []
    for y0, y1 in ((-5.8, -2.0), (1.2, 5.0)):
        lips.extend(
            [
                box(-11.45, -edge + 0.45, y0, y1, lip_under, lip_under + 1.0),
                box(edge - 0.45, 11.45, y0, y1, lip_under, lip_under + 1.0),
            ]
        )

    # Arka sabit dayamalar headerin ortasını kapatmaz. Ön tırnak kartı bunlara iter.
    rear_y = PCB_Y / 2 + REAR_CLEARANCE
    rear_stops = [
        box(-11.1, -8.8, rear_y, rear_y + 1.15, PCB_Z, PCB_TOP + 0.15),
        box(8.8, 11.1, rear_y, rear_y + 1.15, PCB_Z, PCB_TOP + 0.15),
    ]
    # Dayamalar kartın altından yan taşıyıcılara bağlanır; bu parçalar PCB'ye
    # temas etmez, yalnızca STL'nin tek ve dayanıklı bir gövde olmasını sağlar.
    rear_supports = [
        box(-11.1, -8.8, 8.30, rear_y + 1.15, 2.5, PCB_Z),
        box(8.8, 11.1, 8.30, rear_y + 1.15, 2.5, PCB_Z),
    ]

    # Tek, geniş, esnek ön tırnak. Üst rampa kart bastırıldığında öne esner;
    # alt yüzey kart üstüne geçerek kartı aşağıda ve arkaya dayalı tutar.
    snap = yz_prism(
        -4.25,
        4.25,
        [
            (-9.55, 2.45),
            (-8.50, 2.45),
            # İç yüzey PCB'nin ön kenarına 0.10 mm yay baskısı uygular.
            (-8.50, PCB_TOP - 0.08),
            # Üst kanca kartın üzerinden geçip yukarı kalkmasını engeller.
            (-8.05, PCB_TOP - 0.08),
            (-8.05, PCB_TOP + 0.52),
            (-8.50, PCB_TOP + 1.40),
            (-9.55, PCB_TOP + 1.40),
        ],
    )

    body = union(
        floor,
        shell,
        *ears,
        *ledges,
        *guides,
        *lips,
        *rear_stops,
        *rear_supports,
        snap,
    )

    # 20 mm cırt geçişleri ve 22 mm geniş arka kablo ağzı.
    cuts = [
        box(-24.7, -18.0, -8.5, 8.5, -1, 5),
        box(18.0, 24.7, -8.5, 8.5, -1, 5),
        box(-11.0, 11.0, 11.65, 15.5, 2.45, 14.0),
    ]
    # Kablo bağıyla yük alma: jumper çekişi karta iletilmez.
    cuts.extend(
        [
            box(-9.0, -5.5, 10.8, 15.6, 3.4, 5.8),
            box(5.5, 9.0, 10.8, 15.6, 3.4, 5.8),
        ]
    )
    return difference(body, *cuts)


def build_cover():
    # Gövdenin dışına 0.25 mm yan boşlukla geçen vidasız koruyucu başlık.
    outer = rounded(-17.4, 17.4, -15.4, 16.0, 11.0, 29.0, 2.8)
    inner = rounded(-16.25, 16.25, -14.25, 14.75, 10.8, 26.7, 2.45)
    cover = difference(outer, inner)

    # Kablo demeti arka merkezden çıkar. Üstte 1.8 mm köprü kapağı tek parça tutar.
    cable_mouth = box(-11.5, 11.5, 13.6, 16.5, 10.5, 27.2)
    cover = difference(cover, cable_mouth)
    return cover


def references():
    pcb = box(-PCB_X / 2, PCB_X / 2, -PCB_Y / 2, PCB_Y / 2, PCB_Z, PCB_TOP)
    # Temsili alt bileşen hacmi; yalnızca görsel ve açıklık kontrolü içindir.
    components = box(-8.2, 8.2, -5.8, 5.8, 3.0, PCB_Z - 0.15)
    header = box(-9.7, 9.7, 6.0, 8.45, PCB_TOP, PCB_TOP + 2.8)
    plugs = [
        box(x - 1.05, x + 1.05, 6.0, 8.5, PCB_TOP + 2.8, 24.2)
        for x in np.linspace(-8.6, 8.6, 8)
    ]
    return pcb, components, header, plugs


def save_mesh(path, mesh):
    assert mesh.is_watertight, path.name
    assert mesh.is_winding_consistent, path.name
    assert mesh.volume > 0, path.name
    mesh.export(path)


def build():
    body = build_body()
    cover = build_cover()
    pcb, components, header, plugs = references()

    body_print = printable(body)
    cover_print = cover.copy()
    cover_print.apply_transform(
        tm.transformations.rotation_matrix(math.pi, (1, 0, 0), point=(0, 0, 0))
    )
    cover_print = printable(cover_print)

    save_mesh(PRINT / "01_HW290_vidisiz_gecme_govde_2_adet.stl", body_print)
    save_mesh(PRINT / "02_HW290_vidisiz_gecme_kapak_2_adet.stl", cover_print)
    save_mesh(REFERENCE / "HW290_govde_montaj.stl", body)
    save_mesh(REFERENCE / "HW290_kapak_montaj.stl", cover)

    # Gerçek PCB, bileşen hacmi ve kasa arasındaki kritik açıklık kontrolleri.
    def overlap(a, b):
        hit = tm.boolean.intersection([a, b], engine="manifold")
        return 0.0 if hit is None or len(hit.faces) == 0 else float(hit.volume)

    checks = {
        "components_body_overlap_mm3": overlap(components, body),
        "header_body_overlap_mm3": overlap(header, body),
        "cover_body_overlap_mm3": overlap(cover, body),
        # Ön tırnağın 0.08 mm kasıtlı baskı payı bu değere dahildir.
        "pcb_body_intentional_snap_overlap_mm3": overlap(pcb, body),
    }
    assert checks["components_body_overlap_mm3"] < 0.001, checks
    assert checks["header_body_overlap_mm3"] < 0.001, checks
    assert checks["cover_body_overlap_mm3"] < 0.001, checks

    stats = {
        "pcb_mm": [PCB_X, PCB_Y, PCB_T],
        "body_bounds_mm": body_print.extents.tolist(),
        "cover_bounds_mm": cover_print.extents.tolist(),
        "body_volume_cm3": body_print.volume / 1000,
        "cover_volume_cm3": cover_print.volume / 1000,
        "checks": checks,
    }
    (ROOT / "mesh_validation.json").write_text(
        json.dumps(stats, indent=2), encoding="utf-8"
    )
    print(json.dumps(stats, indent=2))
    return body, cover, pcb, components, header, plugs


if __name__ == "__main__":
    build()

"""Fusion 360 script: printable tilt head for four independent ZED 2i stands.

Run from Utilities > Add-Ins > Scripts and Add-Ins. Fusion API geometry uses cm;
all public dimensions below are deliberately specified in mm.
"""

import traceback

import adsk.core
import adsk.fusion


# --------------------------- USER PARAMETERS (mm) ---------------------------
CAMERA_LENGTH = 175.3
CAMERA_DEPTH = 43.1
CAMERA_HEIGHT = 30.3
CAMERA_MASS_G = 229

PLATE_LENGTH = 184.0
PLATE_DEPTH = 82.0
PLATE_THICKNESS = 12.0
CAMERA_SCREW_CLEARANCE_D = 6.8       # 1/4-20 clearance
CAMERA_SCREW_HEAD_D = 11.0
CAMERA_SCREW_COUNTERBORE_DEPTH = 7.0

YOKE_WALL = 8.0
YOKE_INNER_GAP = 187.0              # 1.5 mm print clearance per side
YOKE_DEPTH = 78.0
YOKE_BASE_THICKNESS = 9.0
PIVOT_HEIGHT = 62.0
PIVOT_CLEARANCE_D = 5.5              # M5 through-hole in yoke
PIVOT_INSERT_D = 6.4                 # tune for chosen M5 heat-set insert
PIVOT_INSERT_DEPTH = 9.0
STAND_INSERT_D = 9.5                 # tune for 1/4-20 heat-set insert

CABLE_DIAMETER = 6.4
CABLE_CLAMP_LENGTH = 38.0
CABLE_CLAMP_DEPTH = 16.0
CABLE_CLAMP_HEIGHT = 8.0
CABLE_CLAMP_SCREW_SPACING = 28.0
CABLE_CLAMP_SCREW_D = 3.4            # M3 clearance
CABLE_CLAMP_Y = -32.5                # 2.95 mm behind nominal camera envelope

SHOW_REFERENCE_CAMERA = True


def cm(mm):
    return mm / 10.0


def point(x, y, z):
    return adsk.core.Point3D.create(cm(x), cm(y), cm(z))


def vector(x, y, z):
    return adsk.core.Vector3D.create(x, y, z)


def temp_box(manager, x, y, z, sx, sy, sz):
    """Axis-aligned temporary box; x/y/z describe the lower-left-bottom."""
    center = point(x + sx / 2.0, y + sy / 2.0, z + sz / 2.0)
    obb = adsk.core.OrientedBoundingBox3D.create(
        center, vector(1, 0, 0), vector(0, 1, 0), cm(sx), cm(sy), cm(sz)
    )
    return manager.createBox(obb)


def temp_cylinder(manager, p1, p2, diameter):
    return manager.createCylinderOrCone(p1, cm(diameter / 2.0), p2, cm(diameter / 2.0))


def cut(manager, target, tool):
    ok = manager.booleanOperation(
        target, tool, adsk.fusion.BooleanTypes.DifferenceBooleanType
    )
    if not ok:
        raise RuntimeError("Boolean cut failed")
    return target


def join(manager, target, tool):
    ok = manager.booleanOperation(
        target, tool, adsk.fusion.BooleanTypes.UnionBooleanType
    )
    if not ok:
        raise RuntimeError("Boolean union failed")
    return target


def new_component(root, name):
    occurrence = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occurrence.component.name = name
    return occurrence.component


def add_body(component, temp_body, name):
    body = component.bRepBodies.add(temp_body)
    body.name = name
    return body


def build_camera_plate(root, manager):
    comp = new_component(root, "PRINT_01_Camera_Plate")
    x0 = -PLATE_LENGTH / 2.0
    y0 = -PLATE_DEPTH / 2.0
    z0 = PIVOT_HEIGHT - PLATE_THICKNESS / 2.0
    plate = temp_box(manager, x0, y0, z0, PLATE_LENGTH, PLATE_DEPTH, PLATE_THICKNESS)

    # Camera screw. A 1/4-20 x 3/8 in screw plus washer keeps engagement <7 mm.
    cut(manager, plate, temp_cylinder(
        manager, point(0, 0, z0 - 1), point(0, 0, z0 + PLATE_THICKNESS + 1),
        CAMERA_SCREW_CLEARANCE_D,
    ))
    cut(manager, plate, temp_cylinder(
        manager, point(0, 0, z0 - 1),
        point(0, 0, z0 + CAMERA_SCREW_COUNTERBORE_DEPTH), CAMERA_SCREW_HEAD_D,
    ))

    # Axial sockets for M5 heat-set inserts. The pivot bolt never clamps plastic
    # threads directly, which makes repeated angle changes durable.
    cut(manager, plate, temp_cylinder(
        manager,
        point(x0 - 1, 0, PIVOT_HEIGHT),
        point(x0 + PIVOT_INSERT_DEPTH, 0, PIVOT_HEIGHT),
        PIVOT_INSERT_D,
    ))
    cut(manager, plate, temp_cylinder(
        manager,
        point(-x0 + 1, 0, PIVOT_HEIGHT),
        point(-x0 - PIVOT_INSERT_DEPTH, 0, PIVOT_HEIGHT),
        PIVOT_INSERT_D,
    ))

    # Two M3 through-holes hold the removable cable saddle.
    for x in (-CABLE_CLAMP_SCREW_SPACING / 2.0, CABLE_CLAMP_SCREW_SPACING / 2.0):
        cut(manager, plate, temp_cylinder(
            manager, point(x, CABLE_CLAMP_Y, z0 - 1),
            point(x, CABLE_CLAMP_Y, z0 + PLATE_THICKNESS + 1),
            CABLE_CLAMP_SCREW_D,
        ))

    add_body(comp, plate, "Camera plate")
    return comp


def build_yoke(root, manager):
    comp = new_component(root, "PRINT_02_Tilt_Yoke")
    outer_length = YOKE_INNER_GAP + 2.0 * YOKE_WALL
    x0 = -outer_length / 2.0
    y0 = -YOKE_DEPTH / 2.0
    yoke = temp_box(
        manager, x0, y0, 0, outer_length, YOKE_DEPTH, YOKE_BASE_THICKNESS
    )
    arm_height = PIVOT_HEIGHT + 18.0
    left_arm = temp_box(manager, x0, y0, 0, YOKE_WALL, YOKE_DEPTH, arm_height)
    right_arm = temp_box(
        manager, x0 + outer_length - YOKE_WALL, y0, 0,
        YOKE_WALL, YOKE_DEPTH, arm_height,
    )
    join(manager, yoke, left_arm)
    join(manager, yoke, right_arm)
    # Square buttresses keep the layer lines at the arm/base junction from
    # becoming the only impact load path. They remain below the tilted plate.
    join(manager, yoke, temp_box(
        manager, x0 + YOKE_WALL, y0, 0, 10.0, YOKE_DEPTH, 18.0
    ))
    join(manager, yoke, temp_box(
        manager, x0 + outer_length - YOKE_WALL - 10.0, y0, 0,
        10.0, YOKE_DEPTH, 18.0,
    ))

    # M5 pivot clearance holes through both arms.
    cut(manager, yoke, temp_cylinder(
        manager, point(x0 - 1, 0, PIVOT_HEIGHT),
        point(x0 + YOKE_WALL + 1, 0, PIVOT_HEIGHT), PIVOT_CLEARANCE_D,
    ))
    cut(manager, yoke, temp_cylinder(
        manager, point(-x0 + 1, 0, PIVOT_HEIGHT),
        point(-x0 - YOKE_WALL - 1, 0, PIVOT_HEIGHT), PIVOT_CLEARANCE_D,
    ))

    # Blind/through pilot for a metal 1/4-20 insert that accepts a tripod screw.
    cut(manager, yoke, temp_cylinder(
        manager, point(0, 0, -1), point(0, 0, YOKE_BASE_THICKNESS + 1),
        STAND_INSERT_D,
    ))
    add_body(comp, yoke, "One-piece yoke")
    return comp


def build_cable_clamp(root, manager):
    comp = new_component(root, "PRINT_03_Cable_Saddle")
    z0 = PIVOT_HEIGHT + PLATE_THICKNESS / 2.0
    clamp = temp_box(
        manager,
        -CABLE_CLAMP_LENGTH / 2.0,
        CABLE_CLAMP_Y - CABLE_CLAMP_DEPTH / 2.0,
        z0,
        CABLE_CLAMP_LENGTH,
        CABLE_CLAMP_DEPTH,
        CABLE_CLAMP_HEIGHT,
    )

    # Semicircular channel along cable direction. Add 0.4 mm radius clearance.
    cut(manager, clamp, temp_cylinder(
        manager,
        point(0, CABLE_CLAMP_Y - CABLE_CLAMP_DEPTH / 2.0 - 1, z0),
        point(0, CABLE_CLAMP_Y + CABLE_CLAMP_DEPTH / 2.0 + 1, z0),
        CABLE_DIAMETER + 0.8,
    ))
    for x in (-CABLE_CLAMP_SCREW_SPACING / 2.0, CABLE_CLAMP_SCREW_SPACING / 2.0):
        cut(manager, clamp, temp_cylinder(
            manager, point(x, CABLE_CLAMP_Y, z0 - 1),
            point(x, CABLE_CLAMP_Y, z0 + CABLE_CLAMP_HEIGHT + 1),
            CABLE_CLAMP_SCREW_D,
        ))
    add_body(comp, clamp, "Cable saddle")
    return comp


def build_reference_camera(root, manager):
    if not SHOW_REFERENCE_CAMERA:
        return None
    comp = new_component(root, "REF_ZED2i_ENVELOPE_DO_NOT_PRINT")
    z0 = PIVOT_HEIGHT + PLATE_THICKNESS / 2.0
    camera = temp_box(
        manager, -CAMERA_LENGTH / 2.0, -CAMERA_DEPTH / 2.0, z0,
        CAMERA_LENGTH, CAMERA_DEPTH, CAMERA_HEIGHT,
    )
    body = add_body(comp, camera, "ZED 2i nominal envelope")
    body.opacity = 0.35
    return comp


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            raise RuntimeError("A Fusion Design document could not be created")
        design.designType = adsk.fusion.DesignTypes.DirectDesignType

        root = design.rootComponent
        manager = adsk.fusion.TemporaryBRepManager.get()
        build_camera_plate(root, manager)
        build_yoke(root, manager)
        build_cable_clamp(root, manager)
        build_reference_camera(root, manager)

        app.activeViewport.fit()
        ui.messageBox(
            "ZED 2i corner mount created.\n\n"
            "Export components beginning with PRINT_ as 3MF/STL. "
            "The REF component is only a clearance envelope."
        )
    except Exception:
        if ui:
            ui.messageBox("Script failed:\n{}".format(traceback.format_exc()))


def stop(context):
    pass

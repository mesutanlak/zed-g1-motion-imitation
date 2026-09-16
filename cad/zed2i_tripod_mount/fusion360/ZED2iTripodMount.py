"""Fusion 360 script: builds a native ZED 2i tripod adapter model."""

import adsk.core
import adsk.fusion
import math
import traceback


def mm(value):
    """Fusion API internal length unit is centimetres."""
    return value / 10.0


def add_user_parameter(design, name, expression, comment):
    existing = design.userParameters.itemByName(name)
    if existing:
        return existing
    units = design.unitsManager.defaultLengthUnits
    return design.userParameters.add(
        name,
        adsk.core.ValueInput.createByString(expression),
        units,
        comment,
    )


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        root = design.rootComponent
        root.name = "ZED 2i Tripod Mount"

        # Editable values shown in Modify > Change Parameters.
        add_user_parameter(design, "plateLength", "80 mm", "Adapter plate length")
        add_user_parameter(design, "plateWidth", "35 mm", "Adapter plate width")
        add_user_parameter(design, "plateThickness", "8 mm", "Adapter plate thickness")
        add_user_parameter(design, "cornerRadius", "4 mm", "Plan-view corner radius")
        add_user_parameter(design, "m3Spacing", "36 mm", "ZED 2i M3 hole spacing")
        add_user_parameter(design, "m3Clearance", "3.4 mm", "M3 clearance diameter")
        add_user_parameter(design, "m3HeadDiameter", "6.5 mm", "M3 socket-head recess")
        add_user_parameter(design, "m3HeadDepth", "3.2 mm", "M3 head recess depth")
        add_user_parameter(design, "tripodClearance", "7 mm", "1/4-20 screw clearance")
        add_user_parameter(design, "nutAcrossFlats", "11.3 mm", "1/4-20 nut pocket")
        add_user_parameter(design, "nutPocketDepth", "5.8 mm", "Nut pocket depth")

        sketches = root.sketches
        xy_plane = root.xYConstructionPlane
        base_sketch = sketches.add(xy_plane)
        base_sketch.name = "Rounded 80 x 35 mm Base"

        x = mm(40.0)
        y = mm(17.5)
        r = mm(4.0)
        lines = base_sketch.sketchCurves.sketchLines
        arcs = base_sketch.sketchCurves.sketchArcs
        p = adsk.core.Point3D.create

        lines.addByTwoPoints(p(-x + r, y, 0), p(x - r, y, 0))
        arcs.addByCenterStartSweep(p(x - r, y - r, 0), p(x - r, y, 0), -math.pi / 2)
        lines.addByTwoPoints(p(x, y - r, 0), p(x, -y + r, 0))
        arcs.addByCenterStartSweep(p(x - r, -y + r, 0), p(x, -y + r, 0), -math.pi / 2)
        lines.addByTwoPoints(p(x - r, -y, 0), p(-x + r, -y, 0))
        arcs.addByCenterStartSweep(p(-x + r, -y + r, 0), p(-x + r, -y, 0), -math.pi / 2)
        lines.addByTwoPoints(p(-x, -y + r, 0), p(-x, y - r, 0))
        arcs.addByCenterStartSweep(p(-x + r, y - r, 0), p(-x, y - r, 0), -math.pi / 2)

        extrudes = root.features.extrudeFeatures
        base_input = extrudes.createInput(
            base_sketch.profiles.item(0),
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
        )
        base_input.setDistanceExtent(False, adsk.core.ValueInput.createByString("plateThickness"))
        base = extrudes.add(base_input)
        base.name = "8 mm Adapter Plate"
        body = base.bodies.item(0)
        body.name = "ZED2i Tripod Adapter"

        top_face = max(
            (f for f in body.faces if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType),
            key=lambda f: f.pointOnFace.z,
        )
        bottom_face = min(
            (f for f in body.faces if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType),
            key=lambda f: f.pointOnFace.z,
        )

        # Through holes: two M3 clearances and the central tripod clearance.
        hole_sketch = sketches.add(top_face)
        hole_sketch.name = "M3 and Tripod Through Holes"
        circles = hole_sketch.sketchCurves.sketchCircles
        circles.addByCenterRadius(p(mm(-18), 0, 0), mm(1.7))
        circles.addByCenterRadius(p(mm(18), 0, 0), mm(1.7))
        circles.addByCenterRadius(p(0, 0, 0), mm(3.5))
        profiles = adsk.core.ObjectCollection.create()
        for i in range(hole_sketch.profiles.count):
            profiles.add(hole_sketch.profiles.item(i))
        through_input = extrudes.createInput(
            profiles,
            adsk.fusion.FeatureOperations.CutFeatureOperation,
        )
        through_input.setThroughAllExtent(adsk.fusion.ExtentDirections.NegativeExtentDirection)
        through_cut = extrudes.add(through_input)
        through_cut.name = "3.4 mm M3 + 7 mm Tripod Clearances"

        # Bottom counterbores let M3 socket-head screws sit flush.
        counter_sketch = sketches.add(bottom_face)
        counter_sketch.name = "M3 Head Counterbores"
        cc = counter_sketch.sketchCurves.sketchCircles
        cc.addByCenterRadius(p(mm(-18), 0, 0), mm(3.25))
        cc.addByCenterRadius(p(mm(18), 0, 0), mm(3.25))
        counter_profiles = adsk.core.ObjectCollection.create()
        for i in range(counter_sketch.profiles.count):
            counter_profiles.add(counter_sketch.profiles.item(i))
        counter_input = extrudes.createInput(
            counter_profiles,
            adsk.fusion.FeatureOperations.CutFeatureOperation,
        )
        counter_input.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString("m3HeadDepth")
            ),
            adsk.fusion.ExtentDirections.NegativeExtentDirection,
        )
        counter_cut = extrudes.add(counter_input)
        counter_cut.name = "6.5 mm M3 Head Recesses"

        # Top-loaded captive 1/4-20 hex nut pocket.
        nut_sketch = sketches.add(top_face)
        nut_sketch.name = "1/4-20 Captive Nut Pocket"
        polygon_lines = nut_sketch.sketchCurves.sketchLines
        radius = mm(11.3 / math.sqrt(3.0))
        pts = [
            p(radius * math.cos(math.radians(30 + i * 60)),
              radius * math.sin(math.radians(30 + i * 60)), 0)
            for i in range(6)
        ]
        for i in range(6):
            polygon_lines.addByTwoPoints(pts[i], pts[(i + 1) % 6])
        nut_input = extrudes.createInput(
            nut_sketch.profiles.item(0),
            adsk.fusion.FeatureOperations.CutFeatureOperation,
        )
        nut_input.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(
                adsk.core.ValueInput.createByString("nutPocketDepth")
            ),
            adsk.fusion.ExtentDirections.NegativeExtentDirection,
        )
        nut_cut = extrudes.add(nut_input)
        nut_cut.name = "11.3 mm AF Captive Nut Recess"

        ui.activeSelections.clear()
        app.activeViewport.fit()
        ui.messageBox(
            "ZED 2i tripod mount created.\n\n"
            "Use two M3x10 socket-head screws and one standard 1/4-20 UNC nut."
        )
    except Exception:
        if ui:
            ui.messageBox("Failed:\n{}".format(traceback.format_exc()))


def stop(context):
    pass

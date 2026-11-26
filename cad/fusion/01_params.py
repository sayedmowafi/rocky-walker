"""Rocky Walker / Fusion step 01: user parameters + Body/Leg components + leg skeleton.

Template: double-brace placeholders come from config/rocky.toml (rendered by tools/fusion_build.py).
Idempotent: re-running updates parameter expressions; the skeleton is built once.
Frame: Fusion world = robot body frame (origin at the body centre, at joint height, z up).
The Leg component's origin is on its coxa axis, +x radially outward (leg 0 on +x).
"""

import adsk.core
import adsk.fusion

COXA_DEG = {{limits.coxa_deg}}
FEMUR_DEG = {{limits.femur_deg}}
TIBIA_DEG = {{limits.tibia_deg}}

# name, expression, units, comment
PARAMS = [
    ("coxa_mm", "{{leg.coxa_mm}} mm", "mm", "config leg.coxa_mm: coxa axis -> femur axis"),
    ("femur_mm", "{{leg.femur_mm}} mm", "mm", "config leg.femur_mm: femur axis -> knee axis"),
    ("tibia_mm", "{{leg.tibia_mm}} mm", "mm", "config leg.tibia_mm: knee axis -> foot tip"),
    ("mount_radius_mm", "{{body.mount_radius_mm}} mm", "mm", "config body.mount_radius_mm"),
    ("body_height_mm", "{{gait.body_height_mm}} mm", "mm", "config gait.body_height_mm"),
    ("coxa_min_deg", f"{COXA_DEG[0]} deg", "deg", "config limits.coxa_deg[0]"),
    ("coxa_max_deg", f"{COXA_DEG[1]} deg", "deg", "config limits.coxa_deg[1]"),
    ("femur_min_deg", f"{FEMUR_DEG[0]} deg", "deg", "config limits.femur_deg[0]"),
    ("femur_max_deg", f"{FEMUR_DEG[1]} deg", "deg", "config limits.femur_deg[1]"),
    ("tibia_min_deg", f"{TIBIA_DEG[0]} deg", "deg", "config limits.tibia_deg[0]"),
    ("tibia_max_deg", f"{TIBIA_DEG[1]} deg", "deg", "config limits.tibia_deg[1]"),
    # MG996R (datasheet; confirm with calipers) and fit
    ("servo_l", "40.3 mm", "mm", "MG996R body length (calipers)"),
    ("servo_w", "19.9 mm", "mm", "MG996R body width (calipers)"),
    ("servo_h", "38.55 mm", "mm", "MG996R case height, base to top of case (STEP model)"),
    ("servo_ear_l", "54 mm", "mm", "MG996R length across mounting ears (calipers + STEP)"),
    ("servo_ear_hole_pitch", "49.4 mm", "mm", "ear hole spacing along the length (STEP model)"),
    ("servo_ear_hole_gap", "10.5 mm", "mm", "ear hole spacing across the width (STEP model)"),
    ("servo_ear_z", "30 mm", "mm", "ear underside height above the servo base (STEP model)"),
    ("servo_ear_t", "2.4 mm", "mm", "ear thickness (STEP model; calipers 2.5)"),
    ("servo_shaft_offset", "9.75 mm", "mm", "output shaft centre from the near end (STEP model)"),
    ("servo_shaft_h", "5.65 mm", "mm", "spline top above the case top (44.2 - 38.55, STEP model)"),
    ("fit_clearance", "0.3 mm", "mm", "added to pockets; tune with the fit coupon"),
    ("wall", "3 mm", "mm", "minimum printed wall"),
    ("m3_clear", "3.4 mm", "mm", "M3 clearance hole"),
    ("m3_nut_af", "5.6 mm", "mm", "M3 nut across flats (+clearance)"),
    ("m3_nut_t", "2.6 mm", "mm", "M3 nut thickness (+clearance)"),
    ("m4_clear", "4.4 mm", "mm", "M4 clearance hole (servo ear screws)"),
    ("m4_nut_af", "7.2 mm", "mm", "M4 nut across flats (+clearance)"),
    ("m4_nut_t", "3.4 mm", "mm", "M4 nut thickness (+clearance)"),
    ("m4_pilot", "3.4 mm", "mm", "M4 self-tapping pilot hole in printed plastic"),
    ("bearing_od", "10 mm", "mm", "623ZZ outer diameter"),
    ("bearing_id", "3 mm", "mm", "623ZZ bore"),
    ("bearing_w", "4 mm", "mm", "623ZZ width"),
    ("horn_d", "20 mm", "mm", "round aluminium 25T horn diameter"),
    ("horn_t", "2.5 mm", "mm", "horn plate thickness"),
    ("horn_bolt_circle", "14 mm", "mm", "horn M3 bolt-circle diameter"),
]


def run(_context: str):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    # New Fusion "Part" documents allow only one component; Rocky needs many.
    if design.designIntent == adsk.fusion.DesignIntentTypes.PartDesignIntentType:
        design.designIntent = adsk.fusion.DesignIntentTypes.HybridDesignIntentType
    root = design.rootComponent

    params = design.userParameters
    for name, expr, units, comment in PARAMS:
        p = params.itemByName(name)
        if p:
            p.expression = expr
            p.comment = comment
        else:
            params.add(name, adsk.core.ValueInput.createByString(expr), units, comment)

    def occurrence(name):
        for occ in root.occurrences:
            if occ.component.name == name:
                return occ, False
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occ.component.name = name
        return occ, True

    occurrence("Body")
    leg_occ, new_leg = occurrence("Leg")
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create(params.itemByName("mount_radius_mm").value, 0, 0)
    leg_occ.transform2 = m
    if design.snapshots.hasPendingSnapshot:
        design.snapshots.add()

    leg = leg_occ.component
    if not leg.constructionAxes.itemByName("axis_knee"):
        for name in ("axis_femur", "axis_knee"):
            if leg.constructionAxes.itemByName(name):
                leg.constructionAxes.itemByName(name).deleteMe()
        old = leg.sketches.itemByName("skeleton")
        if old:
            old.deleteMe()
        sk = leg.sketches.add(leg.xYConstructionPlane)
        sk.name = "skeleton"
        lines = sk.sketchCurves.sketchLines
        dims = sk.sketchDimensions
        cons = sk.geometricConstraints

        def axis_line(expr, name):
            x = design.unitsManager.evaluateExpression(expr, "mm")
            ln = lines.addByTwoPoints(
                adsk.core.Point3D.create(x, -1.5, 0), adsk.core.Point3D.create(x, 1.5, 0)
            )
            ln.isConstruction = True
            cons.addVertical(ln)
            d = dims.addDistanceDimension(
                sk.originPoint,
                ln.startSketchPoint,
                adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
                adsk.core.Point3D.create(x / 2, -3, 0),
            )
            d.parameter.expression = expr
            ai = leg.constructionAxes.createInput()
            ai.setByEdge(ln)
            ax = leg.constructionAxes.add(ai)
            ax.name = name

        axis_line("coxa_mm", "axis_femur")
        axis_line("coxa_mm + femur_mm", "axis_knee")

        tip_x = design.unitsManager.evaluateExpression("coxa_mm + femur_mm + tibia_mm", "mm")
        tip = sk.sketchPoints.add(adsk.core.Point3D.create(tip_x, 0, 0))
        d = dims.addDistanceDimension(
            sk.originPoint,
            tip,
            adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
            adsk.core.Point3D.create(tip_x / 2, 3, 0),
        )
        d.parameter.expression = "coxa_mm + femur_mm + tibia_mm"
        cons.addHorizontalPoints(sk.originPoint, tip)
        pi = leg.constructionPoints.createInput()
        pi.setByPoint(tip)
        leg.constructionPoints.add(pi).name = "foot_tip"

    print("params", params.count, "leg new" if new_leg else "leg updated",
          "axes", leg.constructionAxes.count, "points", leg.constructionPoints.count)

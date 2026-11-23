"""Rocky Walker / Fusion report: user parameters + leg skeleton positions, as JSON (mm/deg)."""

import json
import math

import adsk.core
import adsk.fusion


def run(_context: str):
    design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    params = {}
    for p in design.userParameters:
        if p.unit == "mm":
            params[p.name] = round(p.value * 10, 9)
        elif p.unit == "deg":
            params[p.name] = round(math.degrees(p.value), 9)
        else:
            params[p.name] = p.value
    leg_occ = next(o for o in design.rootComponent.occurrences if o.component.name == "Leg")
    leg = leg_occ.component

    def axis(name):
        return leg.constructionAxes.itemByName(name).geometry.origin

    femur, knee = axis("axis_femur"), axis("axis_knee")
    tip = leg.constructionPoints.itemByName("foot_tip").geometry
    frame = {
        "axis_femur_x": round(femur.x * 10, 6),
        "axis_knee_x": round(knee.x * 10, 6),
        "foot_tip_x": round(tip.x * 10, 6),
        "axes_z": [0.0, round(femur.z * 10, 6), round(knee.z * 10, 6), round(tip.z * 10, 6)],
        "leg_origin_x": round(leg_occ.transform2.translation.x * 10, 6),
    }
    print(json.dumps({"params": params, "leg_frame": frame}, indent=2, sort_keys=True))

"""Rocky Walker / Fusion report: body printed-part sizes, interference (body + leg) at the
standing pose and at the coxa limits, coxa-axis radius. Prints JSON. Moves joints; restores."""

import json
import math

import adsk.core
import adsk.fusion

STANCE_MM = {{gait.stance_radius_mm}}
MOUNT_MM = {{body.mount_radius_mm}}
HEIGHT_MM = {{gait.body_height_mm}}
MIN_VOL_MM3 = 1.0


def run(_context: str):
    design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = design.rootComponent
    P = {p.name: p.value for p in design.userParameters}
    occs = {o.component.name: o for o in root.occurrences}
    body_occ = occs["Body"]
    leg_occs = [o for o in root.occurrences if o.component.name == "Leg"]

    def all_bodies():
        out = []
        for top in [body_occ] + leg_occs:
            for occ in top.childOccurrences:
                for b in occ.bRepBodies:
                    out.append(b)
        return out

    def interference():
        coll = adsk.core.ObjectCollection.create()
        for b in all_bodies():
            coll.add(b)
        inp = design.createInterferenceInput(coll)
        inp.areCoincidentFacesIncluded = False
        out = []
        for r in design.analyzeInterference(inp):
            vol = r.interferenceBody.volume * 1000
            if vol > MIN_VOL_MM3:
                out.append([r.entityOne.name, r.entityTwo.name, round(vol, 1)])
        return out

    LC, LF, LT = P["coxa_mm"] * 10, P["femur_mm"] * 10, P["tibia_mm"] * 10
    r = (STANCE_MM - MOUNT_MM) - LC
    z = -HEIGHT_MM
    c3 = (r * r + z * z - LF * LF - LT * LT) / (2 * LF * LT)
    t3 = -math.acos(c3)
    t2 = math.atan2(z, r) - math.atan2(LT * math.sin(t3), LF + LT * math.cos(t3))
    flip = {"coxa": 1, "femur": -1, "knee": -1}

    def set_pose(coxa):
        for leg_occ in leg_occs:
            joints = {j.name: j.createForAssemblyContext(leg_occ) for j in leg_occ.component.asBuiltJoints}
            for name, a in (("coxa", coxa), ("femur", t2), ("knee", t3)):
                joints[name].jointMotion.rotationValue = flip[name] * a
        adsk.doEvents()

    report = {}
    set_pose(0.0)
    report["interference_neutral"] = interference()
    sweep = {}
    for label, val in (("min", P["coxa_min_deg"]), ("max", P["coxa_max_deg"])):
        set_pose(val)
        hits = interference()
        if hits:
            sweep[f"coxa_{label}"] = hits
    report["coxa_sweep"] = sweep
    for leg_occ in leg_occs:
        for j in leg_occ.component.asBuiltJoints:
            j.createForAssemblyContext(leg_occ).jointMotion.rotationValue = 0.0

    printed = {}
    for occ in body_occ.childOccurrences:
        for b in occ.bRepBodies:
            if b.name.startswith("BOUGHT_"):
                continue
            bb = b.boundingBox
            dims = [(bb.maxPoint.x - bb.minPoint.x) * 10, (bb.maxPoint.y - bb.minPoint.y) * 10,
                    (bb.maxPoint.z - bb.minPoint.z) * 10]
            printed[b.name] = {"bbox_mm": [round(d, 2) for d in dims],
                               "volume_mm3": round(b.volume * 1000, 1)}
    report["printed"] = printed
    t = leg_occs[0].transform2.translation
    report["coxa_axis_radius_mm"] = round(math.hypot(t.x, t.y) * 10, 4)
    chassis = next(o for o in body_occ.childOccurrences if o.component.name == "Chassis")
    zs = []
    for b in chassis.bRepBodies:
        if b.name.startswith("Chassis"):
            zs += [b.boundingBox.minPoint.z * 10, b.boundingBox.maxPoint.z * 10]
    report["chassis_height_mm"] = round(max(zs) - min(zs), 2)
    print(json.dumps(report, indent=2, sort_keys=True))

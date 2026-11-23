"""Rocky Walker / Fusion report: masses and centres of mass (printed parts at 0.6 x solid PLA,
bought parts at datasheet mass), per leg segment in its joint frame, plus five-leg interference
at the standing pose. Prints JSON. Moves joints; restores them to zero."""

import json
import math

import adsk.core
import adsk.fusion

PLA_G_CM3 = 1.24
INFILL_FACTOR = 0.6
MISC_KG = 0.060  # screws, nuts, wires, straps (not modelled)
BOUGHT_KG = {
    "BOUGHT_MG996R": 0.055, "BOUGHT_horn": 0.006, "BOUGHT_bearing": 0.002,
    "BOUGHT_pi5": 0.046, "BOUGHT_pca9685": 0.009, "BOUGHT_ubec": 0.025,
    "BOUGHT_buck": 0.015, "BOUGHT_lipo": 0.120, "BOUGHT_camera": 0.004,
}
STANCE_MM = {{gait.stance_radius_mm}}
MOUNT_MM = {{body.mount_radius_mm}}
HEIGHT_MM = {{gait.body_height_mm}}


def run(_context: str):
    design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = design.rootComponent
    P = {p.name: p.value for p in design.userParameters}
    legs = [o for o in root.occurrences if o.component.name == "Leg"]
    leg = legs[0].component
    body_occ = next(o for o in root.occurrences if o.component.name == "Body")
    joints = {j.name: j for j in leg.asBuiltJoints}
    for j in joints.values():
        j.jointMotion.rotationValue = 0.0
    adsk.doEvents()

    def mass_of(b):
        for prefix, kg in BOUGHT_KG.items():
            if b.name.startswith(prefix):
                return kg
        if b.name.startswith("BOUGHT_"):
            raise ValueError(f"no datasheet mass for {b.name}")
        return b.volume * PLA_G_CM3 * INFILL_FACTOR / 1000.0

    def com_of(b):
        c = b.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy).centerOfMass
        return (c.x * 10, c.y * 10, c.z * 10)

    LC, LF = P["coxa_mm"] * 10, P["femur_mm"] * 10
    origins = {"Coxa": (0, 0, 0), "Femur": (LC, 0, 0), "Tibia": (LC + LF, 0, 0)}
    segments, torso_kg, parts = {}, 0.0, {}
    for occ in leg.occurrences:
        name = occ.component.name
        kg, mom = 0.0, [0.0, 0.0, 0.0]
        for b in occ.bRepBodies:
            m = mass_of(b)
            c = com_of(b)
            parts[b.name] = round(m, 4)
            kg += m
            for i in range(3):
                mom[i] += m * c[i]
        if name in origins:
            o = origins[name]
            segments[name.lower()] = {
                "kg": round(kg, 4),
                "com_mm": [round(mom[i] / kg - o[i], 2) for i in range(3)],
            }
        else:  # the coxa servo rides in the body
            torso_kg += kg * len(legs)
    for occ in body_occ.childOccurrences:
        for b in occ.bRepBodies:
            m = mass_of(b)
            parts[b.name] = round(m, 4)
            torso_kg += m
    torso_kg += MISC_KG
    leg_kg = sum(s["kg"] for s in segments.values())
    total = torso_kg + leg_kg * len(legs)

    # five-leg interference at the standing pose
    LT = P["tibia_mm"] * 10
    r = (STANCE_MM - MOUNT_MM) - LC
    z = -HEIGHT_MM
    c3 = (r * r + z * z - LF * LF - LT * LT) / (2 * LF * LT)
    t3 = -math.acos(c3)
    t2 = math.atan2(z, r) - math.atan2(LT * math.sin(t3), LF + LT * math.cos(t3))
    def pose_all(values):
        # joints are per instance in an assembly: set them through each occurrence's proxy
        for occ in legs:
            for j in leg.asBuiltJoints:
                if j.name in values:
                    j.createForAssemblyContext(occ).jointMotion.rotationValue = values[j.name]
        adsk.doEvents()

    pose_all({"coxa": 0.0, "femur": -t2, "knee": -t3})
    coll = adsk.core.ObjectCollection.create()
    for top in [body_occ] + legs:
        for occ in top.childOccurrences:
            for b in occ.bRepBodies:
                coll.add(b)
    inp = design.createInterferenceInput(coll)
    inp.areCoincidentFacesIncluded = False
    hits = []
    for res in design.analyzeInterference(inp):
        vol = res.interferenceBody.volume * 1000
        if vol > 1.0:
            hits.append([res.entityOne.name, res.entityTwo.name, round(vol, 1)])
    pose_all({"coxa": 0.0, "femur": 0.0, "knee": 0.0})

    mounts = []
    for occ in legs:
        t = occ.transform2.translation
        mounts.append([round(math.hypot(t.x, t.y) * 10, 3), round(math.degrees(math.atan2(t.y, t.x)) % 360, 3),
                       round(t.z * 10, 3)])
    print(json.dumps({
        "legs": len(legs),
        "leg_mounts_r_deg_z": sorted(mounts, key=lambda m: m[1]),
        "total_kg": round(total, 4),
        "torso_kg": round(torso_kg, 4),
        "misc_kg": MISC_KG,
        "segments": segments,
        "parts_kg": parts,
        "interference_neutral": hits,
        "density": f"{PLA_G_CM3} g/cm3 x {INFILL_FACTOR}",
    }, indent=2, sort_keys=True))

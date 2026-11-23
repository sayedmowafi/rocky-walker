"""Rocky Walker / Fusion report: leg printed-part sizes, interference at zero + standing pose,
interference at each joint's limits, joint origins. Prints JSON. Moves joints (not read-only)
and restores them to zero afterwards."""

import json
import math

import adsk.core
import adsk.fusion

STANCE_MM = {{gait.stance_radius_mm}}
MOUNT_MM = {{body.mount_radius_mm}}
HEIGHT_MM = {{gait.body_height_mm}}
MIN_VOL_MM3 = 1.0


def params_snapshot(design):
    """All user parameters at report time (mm / deg), so a stale report can be detected."""
    out = {}
    for p in design.userParameters:
        if p.unit == "mm":
            out[p.name] = round(p.value * 10, 9)
        elif p.unit == "deg":
            out[p.name] = round(math.degrees(p.value), 9)
        else:
            out[p.name] = p.value
    return out


def run(_context: str):
    design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = design.rootComponent
    P = {p.name: p.value for p in design.userParameters}
    leg_occ = next(o for o in root.occurrences if o.component.name == "Leg")
    leg = leg_occ.component
    joints = {j.name: j for j in leg.asBuiltJoints}
    flip = {"coxa": 1, "femur": -1, "knee": -1}  # code angle -> Fusion joint angle

    def set_pose(code_angles):
        for name, a in code_angles.items():
            joints[name].jointMotion.rotationValue = flip[name] * a
        adsk.doEvents()

    def bodies():
        out = []
        for occ in leg_occ.childOccurrences:
            for b in occ.bRepBodies:
                out.append(b)
        return out

    def interference():
        coll = adsk.core.ObjectCollection.create()
        for b in bodies():
            coll.add(b)
        inp = design.createInterferenceInput(coll)
        inp.areCoincidentFacesIncluded = False
        res = design.analyzeInterference(inp)
        out = []
        for r in res:
            vol = r.interferenceBody.volume * 1000  # cm3 -> mm3
            if vol > MIN_VOL_MM3:
                out.append([r.entityOne.name, r.entityTwo.name, round(vol, 1)])
        return out

    # neutral stance via analytic IK (same as rocky.kinematics.leg)
    LC, LF, LT = P["coxa_mm"] * 10, P["femur_mm"] * 10, P["tibia_mm"] * 10
    r = (STANCE_MM - MOUNT_MM) - LC
    z = -HEIGHT_MM
    c3 = (r * r + z * z - LF * LF - LT * LT) / (2 * LF * LT)
    t3 = -math.acos(c3)
    t2 = math.atan2(z, r) - math.atan2(LT * math.sin(t3), LF + LT * math.cos(t3))
    stance = {"coxa": 0.0, "femur": t2, "knee": t3}
    zero = {"coxa": 0.0, "femur": 0.0, "knee": 0.0}

    report = {"stance_deg": {k: round(math.degrees(v), 2) for k, v in stance.items()}}
    set_pose(zero)
    report["interference_zero"] = interference()
    set_pose(stance)
    report["interference_neutral"] = interference()

    limits = {
        "coxa": ("coxa_min_deg", "coxa_max_deg"),
        "femur": ("femur_min_deg", "femur_max_deg"),
        "knee": ("tibia_min_deg", "tibia_max_deg"),
    }
    sweep = {}
    for name, (lo, hi) in limits.items():
        for label, val in (("min", P[lo]), ("max", P[hi])):
            pose = dict(stance)
            pose[name] = val
            set_pose(pose)
            hits = interference()
            if hits:
                sweep[f"{name}_{label}_{round(math.degrees(val))}"] = hits
    report["limit_sweep"] = sweep

    # largest interference-free range per joint, stepping 5 deg out from the stance pose
    clear = {}
    for name, (lo, hi) in limits.items():
        rng = []
        for bound, step in ((P[lo], -1), (P[hi], 1)):
            a = round(math.degrees(stance[name]) / 5) * 5
            last = a
            while (a + 5 * step) * step <= math.degrees(bound) * step + 1e-6:
                a += 5 * step
                pose = dict(stance)
                pose[name] = math.radians(a)
                set_pose(pose)
                if interference():
                    break
                last = a
            rng.append(last)
        clear[name] = rng
    report["clear_range_deg"] = clear

    set_pose(zero)
    printed = {}
    for occ in leg_occ.childOccurrences:
        for b in occ.bRepBodies:
            if b.name.startswith("BOUGHT_"):
                continue
            bb = b.boundingBox
            dims = [(bb.maxPoint.x - bb.minPoint.x) * 10, (bb.maxPoint.y - bb.minPoint.y) * 10,
                    (bb.maxPoint.z - bb.minPoint.z) * 10]
            printed[b.name] = {"bbox_mm": [round(d, 2) for d in dims],
                               "volume_mm3": round(b.volume * 1000, 1)}
    report["printed"] = printed
    # read the real Fusion joints (geometry origin in the Leg frame, rotation limits in code sign)
    jr = {}
    for name, j in joints.items():
        o = j.geometry.origin.copy()
        to_leg = leg_occ.transform2.copy()  # joint geometry is reported in world coordinates
        to_leg.invert()
        o.transformBy(to_leg)
        lim = j.jointMotion.rotationLimits
        lo, hi = math.degrees(lim.minimumValue), math.degrees(lim.maximumValue)
        if flip[name] < 0:
            lo, hi = -hi, -lo
        jr[name] = {"origin_mm": [round(o.x * 10, 6), round(o.y * 10, 6), round(o.z * 10, 6)],
                    "limits_deg": [round(lo, 6), round(hi, 6)]}
    report["joints"] = jr
    report["params_snapshot"] = params_snapshot(design)
    print(json.dumps(report, indent=2, sort_keys=True))

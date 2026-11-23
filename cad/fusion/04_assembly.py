"""Rocky Walker / Fusion step 04: place the five legs around the body (instances of the one Leg
component, so editing the leg edits all five). Leg i: rotated by 72*i deg about z, at the mount
radius. Idempotent: extra leg instances are recreated each run."""

import math

import adsk.core
import adsk.fusion


def run(_context: str):
    design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = design.rootComponent
    R = design.userParameters.itemByName("mount_radius_mm").value  # cm
    legs = [o for o in root.occurrences if o.component.name == "Leg"]
    leg_comp = legs[0].component
    for extra in legs[1:]:
        extra.deleteMe()
    for i in range(1, 5):
        a = 2 * math.pi * i / 5
        m = adsk.core.Matrix3D.create()
        m.setWithCoordinateSystem(
            adsk.core.Point3D.create(R * math.cos(a), R * math.sin(a), 0),
            adsk.core.Vector3D.create(math.cos(a), math.sin(a), 0),
            adsk.core.Vector3D.create(-math.sin(a), math.cos(a), 0),
            adsk.core.Vector3D.create(0, 0, 1),
        )
        occ = root.occurrences.addExistingComponent(leg_comp, m)
        occ.transform2 = m  # set explicitly: the add call's placement isn't reliable here
    if design.snapshots.hasPendingSnapshot:
        design.snapshots.add()
    # ground the body and every leg instance so posing joints moves only the leg parts
    for o in root.occurrences:
        if o.component.name in ("Body", "Leg"):
            o.isGrounded = True
    for o in root.occurrences:
        if o.component.name == "Leg":
            t = o.transform2.translation
            print(o.name, round(math.hypot(t.x, t.y) * 10, 3), round(math.degrees(math.atan2(t.y, t.x)), 3))

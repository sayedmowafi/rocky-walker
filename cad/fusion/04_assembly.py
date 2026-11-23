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
        m.setToRotation(a, adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
        t = adsk.core.Matrix3D.create()
        t.translation = adsk.core.Vector3D.create(R * math.cos(a), R * math.sin(a), 0)
        m.transformBy(t)
        root.occurrences.addExistingComponent(leg_comp, m)
    if design.snapshots.hasPendingSnapshot:
        design.snapshots.add()
    print("legs:", sum(1 for o in root.occurrences if o.component.name == "Leg"))

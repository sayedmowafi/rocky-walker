import adsk.core, adsk.fusion, os
OUT = r"{{repo}}/cad/stl"
def run(_c):
    d = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    em = d.exportManager
    root = d.rootComponent
    leg = next(o for o in root.occurrences if o.component.name == "Leg").component
    names = []
    def stl(body, name):
        o = em.createSTLExportOptions(body, os.path.join(OUT, name + ".stl"))
        o.isBinaryFormat = True
        o.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        em.execute(o); names.append(name)
    for occ in leg.occurrences:
        for b in occ.component.bRepBodies:
            if not b.name.startswith("BOUGHT_"):
                stl(b, b.name)
    fc = next(o for o in root.occurrences if o.component.name == "FitCoupon")
    for b in fc.component.bRepBodies:
        stl(b, "FitCoupon")
    print(names)

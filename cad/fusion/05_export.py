"""Rocky Walker / Fusion step 05: export the design.

  cad/rocky.f3d            Fusion archive (editable source of truth)
  cad/rocky.step           whole assembly, any CAD tool
  cad/stl/<Part>.stl       one per printed part (assembly orientation; lay flat in the slicer)
  cad/meshes/*.stl         every body (printed + bought) for Blender, at the zero pose, in its
                           component frame, with manifest.json giving each mesh's segment, colour
                           and the segment origin to subtract (so the mesh sits in its joint frame)
"""

import json
import os

import adsk.core
import adsk.fusion

REPO = r"{{repo}}"
COLORS = {"Rocky Blue": "blue", "Rocky White": "white", "Rocky Black": "black",
          "Rocky Grey": "grey", "Rocky Silver": "silver"}


def run(_context: str):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    em = design.exportManager
    P = {p.name: p.value for p in design.userParameters}
    cad = os.path.join(REPO, "cad")
    stl_dir, mesh_dir = os.path.join(cad, "stl"), os.path.join(cad, "meshes")
    os.makedirs(stl_dir, exist_ok=True)
    os.makedirs(mesh_dir, exist_ok=True)
    for d in (stl_dir, mesh_dir):  # drop meshes of parts that no longer exist
        for name in os.listdir(d):
            if name.endswith(".stl"):
                os.remove(os.path.join(d, name))

    legs = [o for o in root.occurrences if o.component.name == "Leg"]
    leg = legs[0].component
    for occ in legs:
        for j in leg.asBuiltJoints:
            j.createForAssemblyContext(occ).jointMotion.rotationValue = 0.0
    adsk.doEvents()

    em.execute(em.createFusionArchiveExportOptions(os.path.join(cad, "rocky.f3d")))
    em.execute(em.createSTEPExportOptions(os.path.join(cad, "rocky.step"), root))

    def stl(body, path, refinement):
        opt = em.createSTLExportOptions(body, path)
        opt.isBinaryFormat = True
        opt.meshRefinement = refinement
        em.execute(opt)

    HIGH = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    MED = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
    LC, LF = P["coxa_mm"] * 10, P["femur_mm"] * 10
    seg_of = {"CoxaServo": ("leg_static", [0, 0, 0]), "Coxa": ("coxa", [0, 0, 0]),
              "Femur": ("femur", [LC, 0, 0]), "Tibia": ("tibia", [LC + LF, 0, 0])}

    parts, printed = [], []
    # leg: native bodies of the Leg's sub-components (their space = the Leg frame at zero pose)
    for occ in leg.occurrences:
        segment, origin = seg_of[occ.component.name]
        for b in occ.component.bRepBodies:
            fname = f"{occ.component.name}__{b.name}.stl"
            stl(b, os.path.join(mesh_dir, fname), MED)
            parts.append({"file": fname, "segment": segment, "origin_mm": origin,
                          "color": COLORS.get(b.appearance.name, "grey")})
            if not b.name.startswith("BOUGHT_"):
                stl(b, os.path.join(stl_dir, f"{b.name}.stl"), HIGH)
                printed.append(b.name)
    body_occ = next(o for o in root.occurrences if o.component.name == "Body")
    for occ in body_occ.component.occurrences:
        for b in occ.component.bRepBodies:
            fname = f"{occ.component.name}__{b.name}.stl"
            stl(b, os.path.join(mesh_dir, fname), MED)
            parts.append({"file": fname, "segment": "torso", "origin_mm": [0, 0, 0],
                          "color": COLORS.get(b.appearance.name, "grey")})
            if not b.name.startswith("BOUGHT_"):
                stl(b, os.path.join(stl_dir, f"{b.name}.stl"), HIGH)
                printed.append(b.name)
    coupon = next(o for o in root.occurrences if o.component.name == "FitCoupon")
    for b in coupon.component.bRepBodies:
        stl(b, os.path.join(stl_dir, "FitCoupon.stl"), HIGH)
        printed.append("FitCoupon")

    with open(os.path.join(mesh_dir, "manifest.json"), "w") as f:
        json.dump({"units": "mm", "parts": parts}, f, indent=2)
    print(json.dumps({"printed": printed, "meshes": len(parts)}))

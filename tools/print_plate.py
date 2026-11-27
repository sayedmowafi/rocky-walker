"""Orient printable STLs on their largest flat face and pack them onto one build plate.

    uv run --with trimesh --with shapely --with networkx --with lxml python tools/print_plate.py \
        --bed 180 180 --out cad/stl/plates/test-leg cad/stl/FitCoupon.stl cad/stl/Coxa.stl ...

Writes one print-ready STL per part (flat face down, at the origin) and a 3MF with all parts
placed on the plate, plus a PNG of the layout.
"""

from __future__ import annotations

import argparse
import itertools
from pathlib import Path

import numpy as np
import trimesh

GAP = 6.0  # mm between parts and to the plate edge


def best_orientation(mesh: trimesh.Trimesh, down=None) -> trimesh.Trimesh:
    """Put the convex-hull facet with the most real contact area on the plate (z = 0), or the
    facet whose outward normal is ``down`` when the part was designed to print a specific way."""
    hull = mesh.convex_hull
    best = None
    for normal, origin in zip(hull.facets_normal, hull.facets_origin, strict=True):
        if down is not None and normal @ np.asarray(down, float) < 0.999:
            continue
        dist = (mesh.vertices - origin) @ normal
        tri_d = dist[mesh.faces]
        on_plane = (np.abs(tri_d) < 0.05).all(axis=1) & (mesh.face_normals @ normal > 0.999)
        contact = mesh.area_faces[on_plane].sum()
        if best is None or contact > best[0]:
            best = (contact, normal)
    if best is None:
        raise SystemExit(f"no flat face points {down}")
    m = mesh.copy()
    m.apply_transform(trimesh.geometry.align_vectors(best[1], [0, 0, -1]))
    # spin about z for the smallest footprint
    best_rot, best_area = None, None
    for deg in np.arange(0, 180, 1.0):
        r = trimesh.transformations.rotation_matrix(np.radians(deg), [0, 0, 1])
        v = trimesh.transform_points(m.vertices, r)
        ext = v.max(0) - v.min(0)
        if best_area is None or ext[0] * ext[1] < best_area:
            best_area, best_rot = ext[0] * ext[1], r
    m.apply_transform(best_rot)
    m.apply_translation(-m.bounds[0])
    return m, best[0]


def pack(sizes, bed):
    """Shelf-pack rectangles (w, h) with optional 90-degree turns; try all orders and turns."""
    W, H = bed
    best = None
    n = len(sizes)
    for order in itertools.permutations(range(n)):
        for turns in itertools.product((False, True), repeat=n):
            x = y = GAP
            shelf_h = 0.0
            pos = {}
            ok = True
            for i in order:
                w, h = sizes[i][::-1] if turns[i] else sizes[i]
                if x + w + GAP > W:
                    x, y = GAP, y + shelf_h + GAP
                    shelf_h = 0.0
                if x + w + GAP > W or y + h + GAP > H:
                    ok = False
                    break
                pos[i] = (x, y, turns[i])
                x += w + GAP
                shelf_h = max(shelf_h, h)
            if ok:
                used = y + shelf_h
                if best is None or used < best[0]:
                    best = (used, pos)
    return best[1] if best else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bed", nargs=2, type=float, default=(180, 180))
    ap.add_argument("--height", type=float, default=180)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument(
        "--down",
        action="append",
        default=[],
        metavar="PART=X,Y,Z",
        help="put this part's face with outward normal X,Y,Z on the plate",
    )
    ap.add_argument("stls", nargs="+", type=Path)
    a = ap.parse_args()
    downs = {k: [float(v) for v in xyz.split(",")] for k, xyz in (d.split("=") for d in a.down)}
    a.out.mkdir(parents=True, exist_ok=True)
    parts = []
    for p in a.stls:
        m, contact = best_orientation(trimesh.load_mesh(p), downs.get(p.stem))
        ext = m.extents
        print(
            f"{p.stem:10s} footprint {ext[0]:6.1f} x {ext[1]:6.1f}  height {ext[2]:6.1f}  "
            f"flat contact {contact:7.0f} mm2"
        )
        if ext[2] > a.height:
            raise SystemExit(f"{p.stem} is too tall for the printer")
        m.export(a.out / f"{p.stem}_print.stl")
        parts.append((p.stem, m))
    pos = pack([(m.extents[0], m.extents[1]) for _, m in parts], a.bed)
    if pos is None:
        raise SystemExit("the parts do not fit on one plate")
    scene = trimesh.Scene()
    for i, (name, m) in enumerate(parts):
        x, y, turned = pos[i]
        mm = m.copy()
        if turned:
            mm.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 0, 1]))
            mm.apply_translation(-mm.bounds[0])
        mm.apply_translation([x, y, 0])
        scene.add_geometry(mm, node_name=name, geom_name=name)
        b = mm.bounds
        print(f"  {name:10s} at x {b[0][0]:5.1f}-{b[1][0]:5.1f}  y {b[0][1]:5.1f}-{b[1][1]:5.1f}")
    scene.export(a.out / "plate.3mf")
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle

        fig, ax = plt.subplots(figsize=(6, 6))
        ax.add_patch(Rectangle((0, 0), *a.bed, fill=False, lw=2))
        for name, geom in scene.geometry.items():
            g = geom.copy()
            g.apply_transform(scene.graph[name][0])
            ax.triplot(g.vertices[:, 0], g.vertices[:, 1], g.faces, lw=0.2, color="tab:blue")
            c = g.bounds.mean(0)
            ax.text(c[0], c[1], name, ha="center", va="center", fontsize=10, weight="bold")
        ax.set_xlim(-5, a.bed[0] + 5)
        ax.set_ylim(-5, a.bed[1] + 5)
        ax.set_aspect("equal")
        ax.set_title(f"Build plate {a.bed[0]:.0f} x {a.bed[1]:.0f} mm (top view)")
        fig.savefig(a.out / "plate.png", dpi=120, bbox_inches="tight")
    except ImportError:
        pass
    print("wrote", a.out)


if __name__ == "__main__":
    main()

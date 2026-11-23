"""Rocky Walker / Fusion step 03: the body (chassis plates, deck + pillars, faceted carapace,
battery tray, electronics placeholders, camera mount).

Body frame (mm) = Fusion world: origin at the body centre at joint height, z up. Leg i's frame is
rotated by psi_i = 72*i deg and offset by mount_radius. Geometry is computed from the user
parameters; re-running rebuilds the Body component (it deletes its previous contents first).

Vertical stack (z, mm), derived from the leg design (coxa servo top at 16, ears underside at 2.1):
  battery tray ............ below the bottom plate, inside the legs' sweep
  bottom plate ............ under the coxa servo bases, idler bolts on the coxa axes
  top plate ............... under the coxa servo ears (ears screw into captive nuts)
  deck .................... above the coxa brackets' top arms; carries the Pi 5
  carapace ................ faceted dome over the deck; camera on a front facet
"""

import math

import adsk.core
import adsk.fusion

MM = 0.1


def run(_context: str):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    P = {p.name: p.value for p in design.userParameters}

    def mm(name):
        return P[name] / MM

    R = mm("mount_radius_mm")
    SL, SW, SH = mm("servo_l"), mm("servo_w"), mm("servo_h")
    EL, EP, EG = mm("servo_ear_l"), mm("servo_ear_hole_pitch"), mm("servo_ear_hole_gap")
    EZ, SO, SS = mm("servo_ear_z"), mm("servo_shaft_offset"), mm("servo_shaft_h")
    C, WALL = mm("fit_clearance"), mm("wall")
    M3, NAF, NT = mm("m3_clear"), mm("m3_nut_af"), mm("m3_nut_t")
    N = 5
    PSI = [2 * math.pi * i / N for i in range(N)]

    # must match 02_leg.py
    coxa_top = 21.0 - SS
    ear_z = coxa_top - SH + EZ  # coxa servo ear underside
    base_z = coxa_top - SH  # coxa servo base
    ear_cx = SL / 2 - SO

    top_z0, top_z1 = ear_z - WALL, ear_z
    bot_z1 = base_z - C
    bot_z0 = bot_z1 - WALL
    deck_z0 = 26.0
    deck_z1 = deck_z0 + WALL
    LOBE = 22.0  # plate lobe radius around each coxa axis (coxa bracket swings at >= 26.85)
    CORE = 46.0  # plate core radius
    DECK_R = 56.0

    tbm = adsk.fusion.TemporaryBRepManager.get()
    V = adsk.core.Vector3D.create
    Pt = adsk.core.Point3D.create
    UNION = adsk.fusion.BooleanTypes.UnionBooleanType
    CUT = adsk.fusion.BooleanTypes.DifferenceBooleanType
    INTER = adsk.fusion.BooleanTypes.IntersectionBooleanType

    def obox(center, xdir, ydir, lx, ly, lz):
        obb = adsk.core.OrientedBoundingBox3D.create(
            Pt(*[v * MM for v in center]), V(*xdir), V(*ydir), lx * MM, ly * MM, lz * MM
        )
        return tbm.createBox(obb)

    def box(x0, x1, y0, y1, z0, z1):
        return obox(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (1, 0, 0), (0, 1, 0),
                    abs(x1 - x0), abs(y1 - y0), abs(z1 - z0))

    def cyl(p0, p1, r):
        return tbm.createCylinderOrCone(Pt(*[v * MM for v in p0]), r * MM, Pt(*[v * MM for v in p1]), r * MM)

    def sphere(c, r):
        return tbm.createSphere(Pt(*[v * MM for v in c]), r * MM)

    def uni(*bodies):
        out = bodies[0]
        for b in bodies[1:]:
            tbm.booleanOperation(out, b, UNION)
        return out

    def cut(target, *tools):
        for t in tools:
            tbm.booleanOperation(target, t, CUT)
        return target

    def at_leg(body, i):
        """Move a body built in leg-i coordinates into the body frame."""
        m = adsk.core.Matrix3D.create()
        m.setToRotation(PSI[i], V(0, 0, 1), Pt(0, 0, 0))
        t = adsk.core.Matrix3D.create()
        t.translation = V(R * MM * math.cos(PSI[i]), R * MM * math.sin(PSI[i]), 0)
        m.transformBy(t)
        tbm.transform(body, m)
        return body

    def hexnut(cx, cy, z0, depth):
        out = None
        for k in range(3):
            a = math.radians(60 * k)
            s = obox((cx, cy, z0 + depth / 2), (math.cos(a), math.sin(a), 0),
                     (-math.sin(a), math.cos(a), 0), NAF, NAF * 3, depth)
            if out is None:
                out = s
            else:
                tbm.booleanOperation(out, s, INTER)
        return out

    # coxa servo body / ear holes in leg frame (servo: shaft +z on the axis, body toward -x)
    def servo_pocket(z0, z1):
        return box(-(SL - SO) - C, SO + C, -SW / 2 - C, SW / 2 + C, z0, z1)

    ear_holes = [(-(ear_cx + sx * EP / 2), sy * EG / 2) for sx in (-1, 1) for sy in (-1, 1)]

    def plate(z0, z1):
        p = cyl((0, 0, z0), (0, 0, z1), CORE)
        for i in range(N):
            lobe = uni(cyl((0, 0, z0), (0, 0, z1), LOBE), box(-30, 0, -LOBE, LOBE, z0, z1))
            tbm.booleanOperation(p, at_leg(lobe, i), UNION)
        return p

    pillar_angles = [p + math.pi / N for p in PSI]  # between the legs
    PILLAR_R = 44.0

    # ---------- top plate ----------
    top = plate(top_z0, top_z1)
    for i in range(N):
        cut(top, at_leg(servo_pocket(top_z0 - 1, top_z1 + 1), i))
        for hx, hy in ear_holes:
            cut(top, at_leg(cyl((hx, hy, top_z0 - 1), (hx, hy, top_z1 + 1), M3 / 2), i))
            cut(top, at_leg(hexnut(hx, hy, top_z0 - 0.01, NT), i))
    for a in pillar_angles:
        px, py = PILLAR_R * math.cos(a), PILLAR_R * math.sin(a)
        cut(top, cyl((px, py, top_z0 - 1), (px, py, top_z1 + 1), M3 / 2))
        cut(top, hexnut(px, py, top_z0 - 0.01, NT))

    # ---------- bottom plate ----------
    bottom = plate(bot_z0, bot_z1)
    for i in range(N):
        cut(bottom, at_leg(cyl((0, 0, bot_z0 - 1), (0, 0, bot_z1 + 1), M3 / 2), i))  # coxa idler
        cut(bottom, at_leg(hexnut(0, 0, bot_z1 - NT + 0.01, NT), i))
    # spacers between the plates (printed with the bottom plate), M3 through
    for a in pillar_angles:
        px, py = PILLAR_R * math.cos(a), PILLAR_R * math.sin(a)
        sp = cyl((px, py, bot_z1), (px, py, top_z0), 4.5)
        tbm.booleanOperation(bottom, sp, UNION)
        cut(bottom, cyl((px, py, bot_z0 - 1), (px, py, top_z0 + 1), M3 / 2))
    # battery strap slots
    for sx in (-1, 1):
        cut(bottom, obox((sx * 30 * math.cos(math.pi / N), sx * 30 * math.sin(math.pi / N),
                          (bot_z0 + bot_z1) / 2),
                         (-math.sin(math.pi / N), math.cos(math.pi / N), 0),
                         (math.cos(math.pi / N), math.sin(math.pi / N), 0), 22, 4, WALL + 2))

    # ---------- deck + pillars ----------
    deck = cyl((0, 0, deck_z0), (0, 0, deck_z1), DECK_R)
    for a in pillar_angles:
        px, py = PILLAR_R * math.cos(a), PILLAR_R * math.sin(a)
        tbm.booleanOperation(deck, cyl((px, py, top_z1), (px, py, deck_z0), 4.5), UNION)
        cut(deck, cyl((px, py, top_z1 - 1), (px, py, deck_z1 + 1), M3 / 2 - 0.4))  # self-tap
    # Pi 5 standoffs (M2.5, 58 x 49 pattern), board centred
    pi_holes = [(sx * 29, sy * 24.5) for sx in (-1, 1) for sy in (-1, 1)]
    for hx, hy in pi_holes:
        tbm.booleanOperation(deck, cyl((hx, hy, deck_z1), (hx, hy, deck_z1 + 5), 3), UNION)
        cut(deck, cyl((hx, hy, deck_z0 - 1), (hx, hy, deck_z1 + 6), 1.1))
    # cable pass-through
    cut(deck, box(-12, 12, -40, -30, deck_z0 - 1, deck_z1 + 1))

    # ---------- faceted carapace ----------
    DOME_R, DOME_WALL = 62.0, 2.4
    dome_c = (0, 0, deck_z1 - 14)
    outer = sphere(dome_c, DOME_R)
    inner = sphere(dome_c, DOME_R - DOME_WALL)
    keep_up = box(-100, 100, -100, 100, deck_z1, 200)
    tbm.booleanOperation(outer, keep_up, INTER)
    tbm.booleanOperation(inner, box(-100, 100, -100, 100, deck_z1 - 1, 200), INTER)
    # facets: cut by planes with deterministic, irregular normals (a rough, rock-like look)
    facets = []
    for k in range(17):
        az = math.radians(k * 137.5 % 360)  # golden-angle spread
        el = math.radians(18 + (k * 23) % 55)
        n = (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))
        d = DOME_R - 2.5 - (k * 7 % 5) * 0.6
        facets.append((n, d))
    facets.append(((0, 0, 1), DOME_R - 6))  # flat-ish top
    def facet_slab(n, d):
        c = tuple(dome_c[i] + n[i] * (d + 50) for i in range(3))
        # a 100 mm-thick slab whose inner face is the plane n.p = d
        x = (1, 0, 0) if abs(n[0]) < 0.9 else (0, 1, 0)
        xd = V(*x)
        nd = V(*n)
        u = nd.crossProduct(xd)
        u.normalize()
        w = nd.crossProduct(u)
        w.normalize()
        return obox(c, (u.x, u.y, u.z), (w.x, w.y, w.z), 300, 300, 100)

    for n, d in facets:
        tbm.booleanOperation(outer, facet_slab(n, d), CUT)
        tbm.booleanOperation(inner, facet_slab(n, d - DOME_WALL), CUT)  # same facets, offset in
    tbm.booleanOperation(outer, inner, CUT)
    shell = outer
    # rim screws into the deck edge
    rim_r = DECK_R - 9
    for a in pillar_angles:
        x, y = rim_r * math.cos(a), rim_r * math.sin(a)
        tbm.booleanOperation(shell, cyl((x, y, deck_z1), (x, y, deck_z1 + 3.5), 3.5), UNION)
        cut(shell, cyl((x, y, deck_z1 - 1), (x, y, deck_z1 + 4.5), 1.3))
        cut(deck, cyl((x, y, deck_z0 - 1), (x, y, deck_z1 + 1), M3 / 2))

    # camera: Camera Module 3 on a vertical tab at the deck's front edge between legs 0 and 1,
    # just outside the carapace, looking straight out (horizontal).
    cam_a = math.pi / N
    out_dir = (math.cos(cam_a), math.sin(cam_a), 0)
    side = (-math.sin(cam_a), math.cos(cam_a), 0)
    TAB_R0, TAB_T = 63.0, 3.0  # tab inner radius, thickness
    tab_h = 28.0
    tab_c = tuple(out_dir[i] * (TAB_R0 + TAB_T / 2) for i in range(2)) + (deck_z1 + tab_h / 2,)
    tab = obox(tab_c, out_dir, side, TAB_T, 28, tab_h)
    tongue = obox(tuple(out_dir[i] * (DECK_R + TAB_R0) / 2 for i in range(2)) + ((deck_z0 + deck_z1) / 2,),
                  out_dir, side, TAB_R0 - DECK_R + TAB_T + 2, 24, WALL)
    tbm.booleanOperation(deck, uni(tab, tongue), UNION)
    cam_c = tuple(out_dir[i] * (TAB_R0 + TAB_T) for i in range(2)) + (deck_z1 + 15,)
    up = (0, 0, 1)
    for sx, sy in ((-10.5, 6.25), (10.5, 6.25), (-10.5, -6.25), (10.5, -6.25)):
        p0 = tuple(cam_c[i] + side[i] * sx + up[i] * sy - out_dir[i] * (TAB_T + 1) for i in range(3))
        p1 = tuple(cam_c[i] + side[i] * sx + up[i] * sy + out_dir[i] * 1 for i in range(3))
        cut(deck, cyl(p0, p1, 1.1))
    camera = obox(tuple(cam_c[i] + out_dir[i] * 1.5 for i in range(3)), out_dir, side, 3, 25, 24)
    lens_p1 = tuple(cam_c[i] + out_dir[i] * 11.5 for i in range(3))
    tbm.booleanOperation(camera, cyl(cam_c, lens_p1, 4), UNION)

    # ---------- battery tray (under the bottom plate, long axis between legs 0 and 1) ----------
    bat_l, bat_w, bat_h = 106.0, 34.0, 18.0
    bat_dir = (math.cos(cam_a), math.sin(cam_a), 0)
    bat_side = (-math.sin(cam_a), math.cos(cam_a), 0)
    bat_top = bot_z0 - 1.0
    battery = obox((0, 0, bat_top - bat_h / 2), bat_dir, bat_side, bat_l, bat_w, bat_h)
    tray_floor = obox((0, 0, bat_top - bat_h - 1.5), bat_dir, bat_side, bat_l * 0.7, bat_w + 6, 3)
    # tray hangs from the bottom plate on two walls along the battery's long sides
    walls = [
        obox(tuple(bat_side[i] * s * (bat_w / 2 + 1.5 + C) for i in range(2)) + ((bot_z0 + bat_top - bat_h - 3) / 2,),
             bat_dir, bat_side, bat_l * 0.7, 3, bot_z0 - (bat_top - bat_h - 3))
        for s in (-1, 1)
    ]
    tray = uni(tray_floor, *walls)

    # ---------- electronics placeholders ----------
    pi5 = uni(box(-42.5, 42.5, -28, 28, deck_z1 + 5, deck_z1 + 6.6),
              box(-15, 15, -15, 15, deck_z1 + 6.6, deck_z1 + 16))
    pca = box(-31, 31, -12.5, 12.5, bot_z1, bot_z1 + 10)
    ubec = box(-22.5, 22.5, -12.5, 12.5, top_z0 - 10, top_z0)
    buck = box(-22.5, 22.5, -12.5, 12.5, top_z0 - 10, top_z0)

    def rotz(body, deg, dx=0.0, dy=0.0):
        m = adsk.core.Matrix3D.create()
        m.setToRotation(math.radians(deg), V(0, 0, 1), Pt(0, 0, 0))
        t = adsk.core.Matrix3D.create()
        t.translation = V(dx * MM, dy * MM, 0)
        m.transformBy(t)
        tbm.transform(body, m)
        return body

    rotz(pca, 36)
    rotz(ubec, 36, -14 * math.sin(cam_a) * 1, 14 * math.cos(cam_a))
    rotz(buck, 36, 14 * math.sin(cam_a), -14 * math.cos(cam_a))

    # ---------- write into Fusion ----------
    body_occ = next(o for o in root.occurrences if o.component.name == "Body")
    comp = body_occ.component
    for occ in list(comp.occurrences):
        occ.deleteMe()

    colors = {n: design.appearances.itemByName(n) for n in
              ("Rocky Blue", "Rocky White", "Rocky Black", "Rocky Grey", "Rocky Silver")}

    def add_component(name, bodies):
        occ = comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        c = occ.component
        c.name = name
        bf = c.features.baseFeatures.add()
        bf.startEdit()
        for bname, tb, color in bodies:
            b = c.bRepBodies.add(tb, bf)
            b.name = bname
            b.appearance = colors[color]
        bf.finishEdit()
        return occ

    add_component("Chassis", [("ChassisTop", top, "Rocky Black"), ("ChassisBottom", bottom, "Rocky Black"),
                              ("BatteryTray", tray, "Rocky Black")])
    add_component("Deck", [("Deck", deck, "Rocky Black")])
    add_component("Carapace", [("Carapace", shell, "Rocky White")])
    add_component("Electronics", [
        ("BOUGHT_pi5", pi5, "Rocky Grey"),
        ("BOUGHT_pca9685", pca, "Rocky Grey"),
        ("BOUGHT_ubec", ubec, "Rocky Grey"),
        ("BOUGHT_buck", buck, "Rocky Grey"),
        ("BOUGHT_lipo", battery, "Rocky Grey"),
        ("BOUGHT_camera", camera, "Rocky Grey"),
    ])
    print("body built:", [o.component.name for o in comp.occurrences])


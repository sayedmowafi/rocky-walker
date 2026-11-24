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

    def pbox(a, u0, u1, v0, v1, z0, z1):
        """Box in a local frame rotated by angle a about z (u radial, v tangential)."""
        ca, sa = math.cos(a), math.sin(a)
        uc, vc = (u0 + u1) / 2, (v0 + v1) / 2
        c = (uc * ca - vc * sa, uc * sa + vc * ca, (z0 + z1) / 2)
        return obox(c, (ca, sa, 0), (-sa, ca, 0), u1 - u0, v1 - v0, z1 - z0)

    def ppt(a, u, v):
        return (u * math.cos(a) - v * math.sin(a), u * math.sin(a) + v * math.cos(a))

    def pcyl(a, u, v, z0, z1, r):
        x, y = ppt(a, u, v)
        return cyl((x, y, z0), (x, y, z1), r)

    def plane_slab(n, d, thick=100, size=400):
        """Half-space slab whose inner face is the plane n.p = d (keeps n.p < d after a cut)."""
        c = tuple(n[i] * (d + thick / 2) for i in range(3))
        nd = V(*n)
        xd = V(1, 0, 0) if abs(n[0]) < 0.9 else V(0, 1, 0)
        u = nd.crossProduct(xd)
        u.normalize()
        w = nd.crossProduct(u)
        w.normalize()
        return obox(c, (u.x, u.y, u.z), (w.x, w.y, w.z), size, size, thick)

    SEC = [p + math.pi / N for p in PSI]  # sector mid-angles, between the legs
    CORE = 62.0  # plate core radius (reaches the side panels between the legs)
    SPACER_R = 26.0  # plate spacers, on the leg axes (inside the coxa servos' inner ends)
    DECK_PILLAR_R = 56.0  # deck pillars, on the sector midlines
    DECK_R = 65.5
    PANEL_R0, PANEL_R1 = 62.5, 65.0
    PANEL_HALF = math.radians(36 - 13.6)  # panels stop 13.6 deg short of each leg axis
    cam_a = math.pi / N  # camera / battery / belly reference direction (sector 0)

    def plate(z0, z1):
        p = cyl((0, 0, z0), (0, 0, z1), CORE)
        for i in range(N):
            lobe = uni(cyl((0, 0, z0), (0, 0, z1), LOBE), box(-30, 0, -LOBE, LOBE, z0, z1))
            tbm.booleanOperation(p, at_leg(lobe, i), UNION)
        return p

    # ---------- top plate ----------
    top = plate(top_z0, top_z1)
    for i in range(N):
        cut(top, at_leg(servo_pocket(top_z0 - 1, top_z1 + 1), i))
        for hx, hy in ear_holes:
            cut(top, at_leg(cyl((hx, hy, top_z0 - 1), (hx, hy, top_z1 + 1), M3 / 2), i))
            cut(top, at_leg(hexnut(hx, hy, top_z0 - 0.01, NT), i))
        cut(top, pcyl(PSI[i], SPACER_R, 0, top_z0 - 1, top_z1 + 1, M3 / 2))  # spacer screw
        cut(top, pcyl(SEC[i], DECK_PILLAR_R, 0, top_z0 - 1, top_z1 + 1, M3 / 2))  # deck pillar
        x, y = ppt(SEC[i], DECK_PILLAR_R, 0)
        cut(top, hexnut(x, y, top_z0 - 0.01, NT))
        for s_ in (-1, 1):  # side-panel tab holes
            x, y = ppt(SEC[i] + s_ * math.radians(12), 59.5, 0)
            cut(top, cyl((x, y, top_z0 - 1), (x, y, top_z1 + 1), M3 / 2))
            cut(top, hexnut(x, y, top_z0 - 0.01, NT))
        x, y = ppt(SEC[i], 40, 0)  # wire pass-through to the mezzanine
        cut(top, pbox(SEC[i], 34, 46, -6, 6, top_z0 - 1, top_z1 + 1))
    cut(top, cyl((0, 0, top_z0 - 1), (0, 0, top_z1 + 1), 8))  # central wire hole

    # ---------- bottom plate (+ spacers) ----------
    bottom = plate(bot_z0, bot_z1)
    for i in range(N):
        cut(bottom, at_leg(cyl((0, 0, bot_z0 - 1), (0, 0, bot_z1 + 1), M3 / 2), i))  # coxa idler
        cut(bottom, at_leg(hexnut(0, 0, bot_z1 - NT + 0.01, NT), i))
        tbm.booleanOperation(bottom, pcyl(PSI[i], SPACER_R, 0, bot_z1, top_z0, 4.5), UNION)
        cut(bottom, pcyl(PSI[i], SPACER_R, 0, bot_z0 - 1, top_z0 + 1, M3 / 2))
        x, y = ppt(PSI[i], SPACER_R, 0)
        cut(bottom, hexnut(x, y, bot_z0 - 0.01, NT))
        x, y = ppt(PSI[i], 35, 0)  # belly screws
        cut(bottom, cyl((x, y, bot_z0 - 1), (x, y, bot_z1 + 1), M3 / 2))
        cut(bottom, hexnut(x, y, bot_z1 - NT + 0.01, NT))
        for s_ in (-1, 1):  # side-panel tab holes
            x, y = ppt(SEC[i] + s_ * math.radians(12), 59.5, 0)
            cut(bottom, cyl((x, y, bot_z0 - 1), (x, y, bot_z1 + 1), M3 / 2))
            cut(bottom, hexnut(x, y, bot_z1 - NT + 0.01, NT))
        cut(bottom, pbox(SEC[i], 18, 24, -5, 5, bot_z0 - 1, bot_z1 + 1))  # belly wire slot
    cut(bottom, cyl((0, 0, bot_z0 - 1), (0, 0, bot_z1 + 1), 8))  # central wire hole

    # ---------- deck (+ pillars, Pi standoffs, ReSpeaker posts, camera tab) ----------
    deck = cyl((0, 0, deck_z0), (0, 0, deck_z1), DECK_R)
    for i in range(N):
        tbm.booleanOperation(deck, pcyl(SEC[i], DECK_PILLAR_R, 0, top_z1, deck_z0, 4.5), UNION)
        cut(deck, pcyl(SEC[i], DECK_PILLAR_R, 0, top_z1 - 1, deck_z1 + 1, M3 / 2))
    cut(deck, cyl((0, 0, deck_z0 - 1), (0, 0, deck_z1 + 1), 8))  # wires up from the belly
    # deck frame: x' along cam_a (toward the camera), y' perpendicular
    PI_X = -14.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = ppt(cam_a, PI_X + sx * 24.5, sy * 29)
            tbm.booleanOperation(deck, cyl((x, y, deck_z1), (x, y, deck_z1 + 5), 3), UNION)
            cut(deck, cyl((x, y, deck_z0 - 1), (x, y, deck_z1 + 6), 1.1))
    RS_Z = deck_z1 + 25.0  # ReSpeaker Lite board underside, above the Pi and its cooler
    for sy in (-1, 1):  # posts just outside the Pi, with ledges over the Pi's ports
        lo, hi = (45.5, 50.5) if sy > 0 else (-50.5, -45.5)
        llo, lhi = (42.0, 50.5) if sy > 0 else (-50.5, -42.0)
        post = uni(pbox(cam_a, PI_X - 5, PI_X + 5, lo, hi, deck_z1, RS_Z),
                   pbox(cam_a, PI_X - 8, PI_X + 8, llo, lhi, RS_Z - 2, RS_Z))
        tbm.booleanOperation(deck, post, UNION)
    # camera tab (inside the carapace, lens through the wall)
    CAM_Z = deck_z1 + 17.0
    tbm.booleanOperation(deck, pbox(cam_a, 47, 50, -14, 14, deck_z1, deck_z1 + 30), UNION)
    for sx, sy in ((-10.5, 6.25), (10.5, 6.25), (-10.5, -6.25), (10.5, -6.25)):
        x, y = ppt(cam_a, 46, sx)
        x2, y2 = ppt(cam_a, 51, sx)
        cut(deck, cyl((x, y, CAM_Z + sy), (x2, y2, CAM_Z + sy), 1.1))

    # ---------- carapace: faceted wall + dome, camera window, speaker grille, mic holes ----------
    CA, CC, CZ, CW = 65.5, 38.0, 56.0, 2.5  # outer horizontal / vertical radius, dome centre z, wall

    def capsule(a, c, grow=0.0):
        """Vertical wall of radius a up to CZ, capped by the sphere through the rim and CZ + c."""
        rs = (a * a + c * c) / (2 * c)
        cap = sphere((0, 0, CZ + c - rs), rs)
        tbm.booleanOperation(cap, box(-200, 200, -200, 200, CZ, CZ + c + 5), INTER)
        return uni(cap, cyl((0, 0, deck_z1 - grow), (0, 0, CZ + 0.01), a))

    RS_O = (CA * CA + CC * CC) / (2 * CC)
    Z0_O = CZ + CC - RS_O

    outer = capsule(CA, CC)
    inner = capsule(CA - CW, CC - CW, grow=1)
    facets = []
    for k in range(19):
        az = math.radians(k * 137.5 % 360)
        el = math.radians([4, 12, 22, 35, 50][k % 5])
        n = (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))
        if n[2] < (CZ - Z0_O) / RS_O:  # support point on the wall / rim
            h = CA * math.sqrt(1 - n[2] ** 2) + CZ * n[2]
        else:  # support point on the dome
            h = RS_O + Z0_O * n[2]
        depth = (1.0 + (k * 7 % 4) * 0.5) if el < math.radians(30) else (2.0 + (k * 7 % 5) * 0.4)
        facets.append((n, h - depth))
    facets.append(((0, 0, 1), CZ + CC - 5))
    for n, d in facets:
        tbm.booleanOperation(outer, plane_slab(n, d), CUT)
        tbm.booleanOperation(inner, plane_slab(n, d - CW), CUT)
    tbm.booleanOperation(outer, inner, CUT)
    shell = outer
    for i in range(N):  # rim screw bosses into the deck (20 deg off the deck pillars)
        a = SEC[i] + math.radians(20)
        tbm.booleanOperation(shell, pcyl(a, 60.0, 0, deck_z1, deck_z1 + 5, 3.5), UNION)
        tbm.booleanOperation(shell, pbox(a, 60, 64, -3.5, 3.5, deck_z1, deck_z1 + 5), UNION)
        cut(shell, pcyl(a, 60.0, 0, deck_z1 - 1, deck_z1 + 6, 1.3))
        cut(deck, pcyl(a, 60.0, 0, deck_z0 - 1, deck_z1 + 1, M3 / 2))
    x, y = ppt(cam_a, 50, 0)
    x2, y2 = ppt(cam_a, 75, 0)
    cut(shell, cyl((x, y, CAM_Z), (x2, y2, CAM_Z), 5.5))  # camera window
    SPK_U, SPK_V = 30.0, -19.0  # speaker centre in the deck frame
    for gu, gv in [(0, 0)] + [(7 * math.cos(math.radians(60 * j)), 7 * math.sin(math.radians(60 * j)))
                              for j in range(6)]:
        x, y = ppt(cam_a, SPK_U + gu, SPK_V + gv)
        cut(shell, cyl((x, y, deck_z1 + 20), (x, y, 120), 1.6))  # speaker grille
    for sy in (-1, 1):  # mic holes above the ReSpeaker's two mics (provisional positions)
        x, y = ppt(cam_a, PI_X, sy * 36)
        cut(shell, cyl((x, y, RS_Z), (x, y, 120), 1.5))

    # ---------- side panels (5, identical), between the legs ----------
    panels = []
    for i in range(N):
        a = SEC[i]
        wedge = cyl((0, 0, bot_z0 - 3), (0, 0, deck_z0), PANEL_R1 + 3)
        for s_ in (-1, 1):  # keep between a +- PANEL_HALF
            ang = a + s_ * PANEL_HALF
            nx, ny = -math.sin(ang) * s_, math.cos(ang) * s_
            tbm.booleanOperation(wedge, plane_slab((nx, ny, 0), 0), CUT)
        for k, off in enumerate((-15, 0, 15)):  # three flat facets outside
            ang = a + math.radians(off)
            tbm.booleanOperation(wedge, plane_slab((math.cos(ang), math.sin(ang), 0),
                                                   PANEL_R1 - (0.6 if off else 0.0)), CUT)
        cut(wedge, cyl((0, 0, bot_z0 - 4), (0, 0, deck_z0 + 1), PANEL_R0))
        for s_ in (-1, 1):  # inward tabs onto the top plate and under the bottom plate
            for z0, z1 in ((top_z1, top_z1 + 3), (bot_z0 - 3, bot_z0)):
                ang = a + s_ * math.radians(12)
                tab = pbox(ang, 56.5, PANEL_R0 + 0.5, -4, 4, z0, z1)
                cut(tab, pcyl(ang, 59.5, 0, z0 - 1, z1 + 1, M3 / 2))
                tbm.booleanOperation(wedge, tab, UNION)
        for i2 in range(N):  # notches where the plates' lobes pass through the panel ends
            for z0, z1 in ((top_z0, top_z1), (bot_z0, bot_z1)):
                notch = uni(cyl((0, 0, z0 - 0.3), (0, 0, z1 + 0.3), LOBE + 0.3),
                            box(-30.3, 0, -LOBE - 0.3, LOBE + 0.3, z0 - 0.3, z1 + 0.3))
                cut(wedge, at_leg(notch, i2))
        for zc in (-18, -10, 10):  # hidden vent slots
            cut(wedge, pbox(a, PANEL_R0 - 2, PANEL_R1 + 2, -12, 12, zc - 1.25, zc + 1.25))
        panels.append(wedge)

    # ---------- belly (power bay) + hatch ----------
    BU_R, BL_R, BW = 54.0, 70.0, 2.4  # upper ring / lower outer radius, wall
    B_TOP, B_MID, B_FLOOR = bot_z0, -37.0, -62.0  # belly top, step, floor top (hatch top)
    belly = uni(cyl((0, 0, B_MID), (0, 0, B_TOP), BU_R), cyl((0, 0, B_FLOOR), (0, 0, B_MID), BL_R))
    b_in = uni(cyl((0, 0, B_MID - 0.5), (0, 0, B_TOP + 1), BU_R - BW),
               cyl((0, 0, B_FLOOR - 1), (0, 0, B_MID), BL_R - BW))
    for k in range(9):  # rock facets on the lower wall
        az = math.radians(k * 137.5 % 360 + 11)
        n = (math.cos(az), math.sin(az), 0)
        depth = 1.0 + (k * 7 % 4) * 0.5
        tbm.booleanOperation(belly, plane_slab(n, BL_R - depth), CUT)
        tbm.booleanOperation(b_in, plane_slab(n, BL_R - depth - BW), CUT)
    tbm.booleanOperation(belly, b_in, CUT)
    for i in range(N):  # tabs up to the bottom plate screws
        ang = PSI[i]
        tab = uni(pbox(ang, 31, BU_R - 1, -5, 5, B_TOP - 3, B_TOP))
        cut(tab, pcyl(ang, 35, 0, B_TOP - 4, B_TOP + 1, M3 / 2))
        tbm.booleanOperation(belly, tab, UNION)
    hatch = cyl((0, 0, B_FLOOR - 3), (0, 0, B_FLOOR), BL_R - BW - 0.3)
    for k in range(9):
        az = math.radians(k * 137.5 % 360 + 11)
        n = (math.cos(az), math.sin(az), 0)
        tbm.booleanOperation(hatch, plane_slab(n, BL_R - 1.0 - (k * 7 % 4) * 0.5 - BW - 0.3), CUT)
    for deg in (20, 90, 160, 270):  # hatch screws into belly bosses
        a = cam_a + math.radians(deg)
        boss = pcyl(a, 62.5, 0, B_FLOOR, B_FLOOR + 8, 4.0)
        tbm.booleanOperation(belly, uni(boss, pbox(a, 62.5, BL_R - 1, -4, 4, B_FLOOR, B_FLOOR + 8)), UNION)
        cut(belly, pcyl(a, 62.5, 0, B_FLOOR - 1, B_FLOOR + 9, 1.3))
        cut(hatch, pcyl(a, 62.5, 0, B_FLOOR - 4, B_FLOOR + 1, M3 / 2))
    for sv in (-1, 1):  # battery strap slots in the hatch
        cut(hatch, pbox(cam_a, sv * 22 - 2, sv * 22 + 2, 2, 48, B_FLOOR - 4, B_FLOOR + 1))
    # switch (belly wall, 5 deg off the battery axis) and balance-charge port (opposite side)
    SW_A, SW_Z = cam_a + math.radians(5), B_FLOOR + 12.0
    x0, y0 = ppt(SW_A, BL_R - 6, 0)
    x1, y1 = ppt(SW_A, BL_R + 2, 0)
    cut(belly, cyl((x0, y0, SW_Z), (x1, y1, SW_Z), 9.7))
    PORT_A, PORT_Z = cam_a + math.radians(185), B_FLOOR + 10.0
    cut(belly, pbox(PORT_A, BL_R - 6, BL_R + 2, -6.5, 6.5, PORT_Z - 4, PORT_Z + 4))

    # ---------- electronics and other bought / hand-built parts ----------
    def deck_box(u0, u1, v0, v1, z0, z1):
        return pbox(cam_a, u0, u1, v0, v1, z0, z1)

    pz = deck_z1 + 5
    pi5 = uni(deck_box(PI_X - 28, PI_X + 28, -42.5, 42.5, pz, pz + 1.6),
              deck_box(PI_X - 26.5, PI_X + 26.5, 21.5, 44.5, pz + 1.6, pz + 17.6),  # USB + Ethernet
              deck_box(PI_X - 24, PI_X + 24, -30, 15, pz + 1.6, pz + 10.5))  # Active Cooler
    respeaker = deck_box(PI_X - 17.5, PI_X + 17.5, -45, 45, RS_Z, RS_Z + 8.6)
    speaker = deck_box(SPK_U - 15, SPK_U + 15, SPK_V - 21, SPK_V + 21, deck_z1, deck_z1 + 22)
    camera = deck_box(50, 53, -12.5, 12.5, CAM_Z - 12, CAM_Z + 12)
    x, y = ppt(cam_a, 53, 0)
    x2, y2 = ppt(cam_a, 61.5, 0)
    tbm.booleanOperation(camera, cyl((x, y, CAM_Z), (x2, y2, CAM_Z), 4), UNION)
    pca = uni(pbox(SEC[0], -12, 50, -12.7, 12.7, top_z1 + 4, top_z1 + 5.6),
              pbox(SEC[0], -3, 38, 5, 12.7, top_z1 + 5.6, top_z1 + 16.6))
    boards = [uni(pbox(SEC[i], 27, 57, -11, 11, bot_z1 + 3, bot_z1 + 4.6),
                  pbox(SEC[i], 29, 51, -8, 8, bot_z1 + 4.6, bot_z1 + 21.1)) for i in range(N)]
    lipo = deck_box(-36.5, 36.5, 8, 42, B_FLOOR, B_FLOOR + 18.5)
    ubecs = [deck_box(-44 + 9.5 * k, -44 + 9.5 * k + 8, -40, 2, B_FLOOR, B_FLOOR + 17) for k in range(5)]
    xl4005 = deck_box(4, 58, -29, -6, B_FLOOR, B_FLOOR + 18)
    ina219 = deck_box(10, 35, -26, -4, B_FLOOR + 20, B_FLOOR + 25)
    fuse = deck_box(4, 40, -48, -30, B_FLOOR, B_FLOOR + 18)
    x0, y0 = ppt(SW_A, BL_R - 32, 0)
    x1, y1 = ppt(SW_A, BL_R - 1, 0)
    switch = cyl((x0, y0, SW_Z), (x1, y1, SW_Z), 9.5)
    x1, y1 = ppt(SW_A, BL_R + 2, 0)
    xb, yb = ppt(SW_A, BL_R + 0.2, 0)
    tbm.booleanOperation(switch, cyl((xb, yb, SW_Z), (x1, y1, SW_Z), 11), UNION)
    port = pbox(PORT_A, BL_R - 16, BL_R - 1, -6, 6, PORT_Z - 3.5, PORT_Z + 3.5)

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

    add_component("Chassis", [("ChassisTop", top, "Rocky Black"), ("ChassisBottom", bottom, "Rocky Black")])
    add_component("Deck", [("Deck", deck, "Rocky Black")])
    add_component("Carapace", [("Carapace", shell, "Rocky White")])
    add_component("SidePanels", [(f"SidePanel_{i + 1}", panels[i], "Rocky White") for i in range(N)])
    add_component("Belly", [("Belly", belly, "Rocky White"), ("BellyHatch", hatch, "Rocky Black")])
    parts = [("BOUGHT_pi5", pi5), ("BOUGHT_respeaker", respeaker), ("BOUGHT_speaker", speaker),
             ("BOUGHT_camera", camera), ("BOUGHT_pca9685", pca), ("BOUGHT_lipo", lipo),
             ("BOUGHT_xl4005", xl4005), ("BOUGHT_ina219", ina219), ("BOUGHT_fuse", fuse),
             ("BOUGHT_switch", switch), ("BOUGHT_port", port)]
    parts += [(f"BOUGHT_ubec_{k + 1}", u) for k, u in enumerate(ubecs)]
    parts += [(f"BOUGHT_servo_board_{k + 1}", b) for k, b in enumerate(boards)]
    add_component("Electronics", [(n, b, "Rocky Grey") for n, b in parts])
    print("body built:", [o.component.name for o in comp.occurrences])

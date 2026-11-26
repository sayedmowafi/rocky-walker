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
    M4, NAF4, NT4, M4P = mm("m4_clear"), mm("m4_nut_af"), mm("m4_nut_t"), mm("m4_pilot")
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

    def hexnut(cx, cy, z0, depth, af=None):
        af = af or NAF
        out = None
        for k in range(3):
            a = math.radians(60 * k)
            s = obox((cx, cy, z0 + depth / 2), (math.cos(a), math.sin(a), 0),
                     (-math.sin(a), math.cos(a), 0), af, af * 3, depth)
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

    def plane_slab(n, d, thick=400, size=900):
        """Half-space slab whose inner face is the plane n.p = d (keeps n.p < d after a cut)."""
        c = tuple(n[i] * (d + thick / 2) for i in range(3))
        nd = V(*n)
        xd = V(1, 0, 0) if abs(n[0]) < 0.9 else V(0, 1, 0)
        u = nd.crossProduct(xd)
        u.normalize()
        w = nd.crossProduct(u)
        w.normalize()
        return obox(c, (u.x, u.y, u.z), (w.x, w.y, w.z), size, size, thick)

    # ---------- layout constants (mm) ----------
    Z_F0, Z_F1 = bot_z0, bot_z1  # floor (under the coxa servo bases)
    Z_D0, Z_D1 = top_z0, top_z1  # top plate (under the coxa servo ears)
    Z_EAVE = 26.0  # flank top = carapace underside (the coxa brackets' top arms end at 25)
    Z_CHEEK = 20.5  # hip-socket wall top (the top arms sweep over it from 21)
    TW = 2.5  # shell / flank wall
    CHEEK_R = 26.85 - 2.5  # hip socket radius (the coxa bracket's column swings at 26.85)
    FLANK = 100.0  # rock face between two legs: distance from the centre at the eave
    LEAN = math.tan(math.radians(8))  # the rock faces lean in toward the floor
    FLANK_T = 96.0  # chassis wall behind the rock face (at the eave)
    LEAN_T = 4.0 / (Z_EAVE - bot_z0)  # ... reaching 92 at the floor
    SLOT_R = 27.0  # around each coxa axis, above the hip socket: the top arm's sweep
    DISC_R = 87.0  # top plate radius (one piece, 174 mm)
    ROOF_R = FLANK + 1.0  # carapace pentagon: edge distance (a 1 mm lip over the flanks)
    TOP_Z = 45.0  # carapace flat top
    HS = math.radians(36)
    D45 = math.radians(45)
    SEAM = math.atan2(CHEEK_R * math.sin(D45), R + CHEEK_R * math.cos(D45))  # tub seam, 8.35 deg
    KERF = 0.15  # half the visible gap at every printed seam
    SEC = [p + math.pi / N for p in PSI]  # sector mid-angles, between the legs
    cam_a = SEC[0]  # camera looks out between legs 0 and 1
    top_a = cam_a + math.radians(144)  # top-level layout axis (the Pi's USB end points this way)

    def half(body, n, d):
        """Keep the part of `body` with n.p <= d (n need not be a unit vector)."""
        ln = math.sqrt(sum(c * c for c in n))
        tbm.booleanOperation(body, plane_slab(tuple(c / ln for c in n), d / ln), CUT)
        return body

    def sector(a0, a1, z0, z1, gap=0.0):
        w = box(-200, 200, -200, 200, z0, z1)
        half(w, (math.sin(a0), -math.cos(a0), 0), -gap)
        half(w, (-math.sin(a1), math.cos(a1), 0), -gap)
        return w

    def pent(rv, z0, z1):
        """Pentagonal prism with its corners at radius rv, pointing at the legs."""
        w = box(-rv - 5, rv + 5, -rv - 5, rv + 5, z0, z1)
        for k in range(N):
            half(w, (math.cos(SEC[k]), math.sin(SEC[k]), 0), rv * math.cos(HS))
        return w

    def hull(planes, z0, z1, inset=0.0):
        w = box(-160, 160, -160, 160, z0, z1)
        for n, d in planes:
            half(w, n, d - inset)
        return w

    def facet(az, r0, z0, slope_deg):
        """Plane through (r0 at azimuth az, z0) whose surface rises inward at slope_deg."""
        s = math.radians(slope_deg)
        n = (math.sin(s) * math.cos(az), math.sin(s) * math.sin(az), math.cos(s))
        return n, n[0] * r0 * math.cos(az) + n[1] * r0 * math.sin(az) + n[2] * z0

    def jit(k, amp):  # deterministic "random" in [-amp, amp]
        return amp * math.sin(k * 12.9898 + 4.1) * math.cos(k * 7.233 + 1.3)

    def hcyl(i, d, y, z0, z1, r):
        """Vertical cylinder at radial offset d, lateral offset y from leg i's coxa axis."""
        x, yy = ppt(PSI[i], R + d, y)
        return cyl((x, yy, z0), (x, yy, z1), r)

    def body_solid(inset, z0, z1, lean=LEAN_T):
        """The chassis outline as a solid: a pentagonal prism whose walls lean in toward the
        floor, with a round hip socket on each corner. `inset` shrinks it (inner wall face)."""
        out = box(-160, 160, -160, 160, z0, z1)
        for k in range(N):
            a = SEC[k]
            half(out, (math.cos(a), math.sin(a), -lean), FLANK_T - inset - lean * Z_EAVE)
        for i in range(N):
            out = uni(out, hcyl(i, 0, 0, z0, z1, CHEEK_R - inset))
        return out

    # ---------- chassis tub: floor + leaning flanks + hip sockets (split into 5 below) ----------
    tub = body_solid(0, Z_F0, Z_EAVE)
    cut(tub, body_solid(TW, Z_F1, Z_EAVE + 1))
    CAM_Z = Z_D1 + 14.0
    SW_Z, PORT_Z = -16.0, -14.0
    LUG_V, LUG_Z = 16.5, 21.5  # carapace screws: two per nose facet
    for i in range(N):
        cut(tub, hcyl(i, 0, 0, Z_CHEEK, Z_EAVE + 1, SLOT_R))  # the top arm sweeps through here
        cut(tub, hcyl(i, 0, 0, Z_F0 - 1, Z_F1 + 1, M3 / 2))  # coxa idler bolt
        cut(tub, at_leg(hexnut(0, 0, Z_F1 - NT + 0.01, NT), i))
        # pillar under the servo's outer ears (self-tapping servo screws)
        tbm.booleanOperation(
            tub, at_leg(box(SO + C + 0.2, CHEEK_R - TW + 0.5, -8.5, 8.5, Z_F1, Z_D1), i), UNION)
        for hx, hy in ear_holes:
            if hx > 0:
                cut(tub, at_leg(cyl((hx, hy, Z_D1 - 14), (hx, hy, Z_D1 + 1), M4P / 2), i))
        for s_ in (-1, 1):  # posts the top plate screws onto, either side of the servo
            tbm.booleanOperation(tub, hcyl(i, -20, s_ * 17, Z_F1, Z_D0, 4.0), UNION)
            cut(tub, hcyl(i, -20, s_ * 17, Z_D0 - 12, Z_D0 + 1, 1.3))
        # cable entry beside the servo, facing the coxa bracket's column
        ca, sa = math.cos(math.radians(36)), math.sin(math.radians(36))
        cut(tub, at_leg(obox((23 * ca, 23 * sa, -17), (ca, sa, 0), (-sa, ca, 0), 7, 9, 10), i))
        a = SEC[i]
        for v in (-8, 0, 8):  # floor vent slits (hidden underneath)
            cut(tub, pbox(a, 44, 53, v - 1.0, v + 1.0, Z_F0 - 1, Z_F1 + 1))
    # battery hatch opening, ledge and screw bosses
    cut(tub, pbox(cam_a, -42, 42, -23, 23, Z_F0 - 1, Z_F0 + 1.5))
    for s_ in (-1, 1):
        tbm.booleanOperation(tub, pcyl(cam_a, s_ * 40.5, 0, Z_F0 + 1.5, Z_F1 + 6, 3.4), UNION)
        cut(tub, pcyl(cam_a, s_ * 40.5, 0, Z_F0, Z_F1 + 5, 1.3))
    cut(tub, pbox(cam_a, -39, 39, -20, 20, Z_F0 - 1, Z_F1 + 7))
    cut(tub, pbox(SEC[2], 88, 104, -6.5, 6.5, PORT_Z - 4, PORT_Z + 4))  # balance-charge port

    # ---------- rock faces: five cladding panels over the chassis walls ----------
    def flank_hole(body, a, v, z, r):
        x0, y0 = ppt(a, 85, v)
        x1, y1 = ppt(a, 106, v)
        cut(body, cyl((x0, y0, z), (x1, y1, z), r))

    flank_hole(tub, cam_a, 0, CAM_Z, 5.5)  # camera window
    flank_hole(tub, SEC[3], 0, SW_Z, 9.7)  # power switch (in the dark base, under the rock)
    for i in range(N):
        for s_ in (-1, 1):  # carapace screws
            flank_hole(tub, SEC[i], s_ * LUG_V, LUG_Z, M3 / 2)
    panels = []
    for k in range(N):
        a = SEC[k]
        ca, sa = math.cos(a), math.sin(a)
        pn = pbox(a, 80, 110, -50, 50, -40, Z_EAVE)
        half(pn, (ca, sa, -LEAN), FLANK - LEAN * Z_EAVE)  # outer face
        half(pn, (-ca, -sa, LEAN_T), -(FLANK_T + 0.4 - LEAN_T * Z_EAVE))  # 0.4 off the wall
        v0, z_tip = jit(k + 71, 9), -9.0 + jit(k + 73, 2)
        tl = math.tan(math.radians(34 + jit(k + 79, 5)))
        tr = math.tan(math.radians(34 + jit(k + 83, 5)))
        half(pn, (tl * sa, -tl * ca, -1), -z_tip - tl * v0)  # lower edge, left
        half(pn, (-tr * sa, tr * ca, -1), -z_tip + tr * v0)  # lower edge, right
        half(pn, (0, 0, -1), jit(k + 89, 2))  # blunt tip, clear of the switch below
        for i in (k, (k + 1) % N):
            cut(pn, hcyl(i, 0, 0, -45, Z_EAVE + 1, CHEEK_R + 0.3))
            cut(pn, hcyl(i, 0, 0, Z_CHEEK, Z_EAVE + 1, SLOT_R))
        for s_ in (-1, 1):
            flank_hole(pn, a, s_ * LUG_V, LUG_Z, M3 / 2)
        panels.append(pn)
    flank_hole(panels[0], cam_a, 0, CAM_Z, 5.5)
    tubs = []
    for i in range(N):
        piece = tbm.copy(tub)
        wedge = sector(PSI[i] - SEAM, PSI[i] + 2 * HS - SEAM, Z_F0 - 5, Z_EAVE + 5, KERF)
        tbm.booleanOperation(piece, wedge, INTER)
        tubs.append(piece)

    hatch = uni(pbox(cam_a, -41.7, 41.7, -22.7, 22.7, Z_F0, Z_F0 + 1.5),
                pbox(cam_a, -38.7, 38.7, -19.7, 19.7, Z_F0, Z_F1))
    for s_ in (-1, 1):
        cut(hatch, pcyl(cam_a, s_ * 40.5, 0, Z_F0 - 1, Z_F1 + 1, M3 / 2))
        cut(hatch, pbox(cam_a, -10, 10, s_ * 18.5 - 0.9, s_ * 18.5 + 0.9, Z_F0 - 1, Z_F1 + 1))

    # ---------- top plate (one piece) + Pi standoffs, ReSpeaker posts, camera tab ----------
    def tbox(e0, e1, f0, f1, z0, z1):
        return pbox(top_a, e0, e1, f0, f1, z0, z1)

    disc = cyl((0, 0, Z_D0), (0, 0, Z_D1), DISC_R)
    for i in range(N):
        cut(disc, at_leg(box(-(SL - SO) - C, 60, -SW / 2 - C, SW / 2 + C, Z_D0 - 1, Z_D1 + 1), i))
        for hx, hy in ear_holes:
            if hx < 0:
                cut(disc, at_leg(cyl((hx, hy, Z_D0 - 1), (hx, hy, Z_D1 + 1), M4 / 2), i))
                cut(disc, at_leg(hexnut(hx, hy, Z_D0 - 0.01, NT4, NAF4), i))
        for s_ in (-1, 1):
            cut(disc, hcyl(i, -20, s_ * 17, Z_D0 - 1, Z_D1 + 1, M3 / 2))
        cut(disc, pbox(SEC[i], 30, 42, -6, 6, Z_D0 - 1, Z_D1 + 1))  # wires between the levels
    PI_F = -17.5  # Pi centre line; the ReSpeaker lies beside it
    pz = Z_D1 + 3.0
    for se in (-1, 1):
        for sf in (-1, 1):
            x, y = ppt(top_a, se * 29, PI_F + sf * 24.5)
            tbm.booleanOperation(disc, cyl((x, y, Z_D1), (x, y, pz), 3), UNION)
            cut(disc, cyl((x, y, Z_D0 - 1), (x, y, pz + 1), 1.1))
    RS_Z, RS_F0 = 27.0, 14.0  # ReSpeaker Lite underside (mics just under the roof), near edge
    for se in (-1, 1):
        post = tbox(se * 42.5 - 2.5, se * 42.5 + 2.5, RS_F0 + 10, RS_F0 + 25, Z_D1, RS_Z)
        tbm.booleanOperation(disc, post, UNION)
    tbm.booleanOperation(disc, pbox(cam_a, 83.5, 86.5, -14, 14, Z_D1, Z_D1 + 28), UNION)  # camera tab
    for sv, sz in ((-10.5, 6.25), (10.5, 6.25), (-10.5, -6.25), (10.5, -6.25)):
        x, y = ppt(cam_a, 82.5, sv)
        x2, y2 = ppt(cam_a, 87.5, sv)
        cut(disc, cyl((x, y, CAM_Z + sz), (x2, y2, CAM_Z + sz), 1.1))

    # ---------- carapace: a low faceted rock (pentagon, corners over the hips) ----------
    planes = [((0, 0, 1), TOP_Z)]
    for k in range(N):
        planes.append(facet(SEC[k], ROOF_R, Z_EAVE, 80))  # rim: a slab edge, tallest mid-side
        for j, s_ in enumerate((-1, 1)):  # two shoulder facets per side
            az = SEC[k] + s_ * math.radians(12 + jit(2 * k + j, 3))
            planes.append(facet(az, 105 + jit(2 * k + j + 13, 1.5), 30.0, 36 + jit(2 * k + j + 29, 7)))
        planes.append(facet(PSI[k] + math.radians(jit(k + 43, 8)), 119.0, 26.5,
                            25 + jit(k + 47, 6)))  # chipped corner over each hip
        planes.append(facet(SEC[k] + math.radians(20 + jit(k + 53, 15)), 78 + jit(k + 59, 4), 41.5,
                            6.5 + jit(k + 61, 1.5)))  # crown
    outer = hull(planes, Z_EAVE, TOP_Z + 5)
    cavity = hull(planes, Z_EAVE + TW, TOP_Z + 5, inset=TW)
    opening = body_solid(TW, Z_EAVE - 1, Z_EAVE + TW + 0.1, lean=0.0)
    for i in range(N):
        cut(opening, hcyl(i, 0, 0, Z_EAVE - 2, Z_EAVE + TW + 1, SLOT_R + TW))  # soffit over each hip
    cavity = uni(cavity, opening)
    shell = tbm.copy(outer)
    cut(shell, cavity)
    inside = body_solid(TW + 0.4, 19.0, Z_EAVE + 0.2)
    for i in range(N):  # baffle behind each hip: hides the inside from the top-arm slot
        arc = hcyl(i, 0, 0, 20.0, Z_EAVE + 0.2, SLOT_R + TW)
        cut(arc, hcyl(i, 0, 0, 19.0, Z_EAVE + 1, SLOT_R))
        tbm.booleanOperation(arc, tbm.copy(inside), INTER)
        tbm.booleanOperation(shell, arc, UNION)
    for k in range(N):  # lugs the flank screws bite into
        for s_ in (-1, 1):
            lug = pbox(SEC[k], 86.7, 92.6, s_ * LUG_V - 3.3, s_ * LUG_V + 3.3, 17.0, TOP_Z)
            tbm.booleanOperation(lug, tbm.copy(outer), INTER)
            tbm.booleanOperation(shell, lug, UNION)
            flank_hole(shell, SEC[k], s_ * LUG_V, LUG_Z, 1.3)
    # keystone: ledge for the cap, ring that ties the five wedges (screws hidden under the cap)
    CAP_R, OPEN_R, LEDGE_Z = 32.0, 21.0, TOP_Z - TW
    cut(shell, pent(CAP_R, LEDGE_Z, TOP_Z + 6))
    tbm.booleanOperation(shell, pent(CAP_R + 4, LEDGE_Z - TW, LEDGE_Z), UNION)
    cut(shell, pent(OPEN_R, LEDGE_Z - TW - 1, TOP_Z + 6))
    RING_Z0, RING_Z1 = LEDGE_Z - TW - 3.5, LEDGE_Z - TW
    ring = pent(CAP_R + 6, RING_Z0, RING_Z1)
    cut(ring, pent(OPEN_R - 4, RING_Z0 - 1, RING_Z1 + 1))
    cap = uni(pent(CAP_R - 0.3, LEDGE_Z, TOP_Z), pent(OPEN_R - 0.4, LEDGE_Z - TW, LEDGE_Z))
    crown = [((0, 0, 1), TOP_Z + 1.6)]
    for k in range(N):
        crown.append(facet(SEC[k] + math.radians(jit(k + 61, 6)), (CAP_R - 0.3) * math.cos(HS),
                           TOP_Z, 24 + jit(k + 67, 6)))
    tbm.booleanOperation(cap, hull(crown, TOP_Z - 0.01, TOP_Z + 3), UNION)
    for i in range(N):
        for s_ in (-1, 1):  # wedge-to-ring screws
            a = PSI[i] + s_ * math.radians(18)
            cut(shell, pcyl(a, 22.5, 0, LEDGE_Z - TW - 1, LEDGE_Z + 1, M3 / 2))
            cut(ring, pcyl(a, 22.5, 0, RING_Z0 - 1, RING_Z1 + 1, M3 / 2))
            x, y = ppt(a, 22.5, 0)
            cut(ring, hexnut(x, y, RING_Z0 - 0.01, NT))
        cut(shell, pcyl(PSI[i], 27.0, 0, LEDGE_Z - 2.1, LEDGE_Z + 1, 3.05))  # 6 x 2 magnets
        cut(cap, pcyl(PSI[i], 27.0, 0, LEDGE_Z - 1, LEDGE_Z + 2.1, 3.05))
    # speaker slits and mic holes
    SPK_A = top_a + math.radians(72)
    for v in (-12, -6, 0, 6, 12):
        cut(shell, pbox(SPK_A, 58, 80, v - 0.8, v + 0.8, 30, 60))
    for se in (-1, 1):
        x, y = ppt(top_a, se * 36, RS_F0 + 17.5)
        cut(shell, cyl((x, y, 30), (x, y, 60), 1.2))
    wedges = []
    for i in range(N):
        piece = tbm.copy(shell)
        tbm.booleanOperation(piece, sector(PSI[i] - HS, PSI[i] + HS, 10, TOP_Z + 10, KERF), INTER)
        wedges.append(piece)

    # ---------- electronics and other bought / hand-built parts ----------
    pi5 = uni(tbox(-42.5, 42.5, PI_F - 28, PI_F + 28, pz, pz + 1.6),
              tbox(21.5, 44.5, PI_F - 26.5, PI_F + 26.5, pz + 1.6, pz + 17.6),  # USB + Ethernet
              tbox(-30, 15, PI_F - 24, PI_F + 24, pz + 1.6, pz + 10.5))  # Active Cooler
    respeaker = tbox(-45, 45, RS_F0, RS_F0 + 35, RS_Z, RS_Z + 8.6)
    speaker = pbox(SPK_A, 54, 84, -21, 21, Z_D1, Z_D1 + 22)
    PCA_A = top_a - math.radians(72)
    pca = uni(pbox(PCA_A, 59.5, 84.9, -31, 31, Z_D1 + 4, Z_D1 + 5.6),
              pbox(PCA_A, 77.2, 84.9, -20.5, 20.5, Z_D1 + 5.6, Z_D1 + 16.6))
    camera = pbox(cam_a, 86.5, 89.5, -12.5, 12.5, CAM_Z - 12, CAM_Z + 12)
    x, y = ppt(cam_a, 89.5, 0)
    x2, y2 = ppt(cam_a, 98, 0)
    tbm.booleanOperation(camera, cyl((x, y, CAM_Z), (x2, y2, CAM_Z), 4), UNION)
    lipo = pbox(cam_a, -36.5, 36.5, -17, 17, Z_F1, Z_F1 + 18.5)
    boards = [uni(pbox(PSI[i], 46, 68, -15, 15, Z_F1 + 3, Z_F1 + 4.6),
                  pbox(PSI[i], 49, 65, -11, 11, Z_F1 + 4.6, Z_F1 + 21.1)) for i in range(N)]
    ubecs = [pbox(SEC[i], 55.5, 63.5, -21, 21, Z_F1, Z_F1 + 17) for i in range(N)]
    ubecs[3] = pbox(SEC[0], 66, 74, -21, 21, Z_F1, Z_F1 + 17)  # the switch has bay 3
    ina219 = pbox(SEC[2], 66, 88, -12.5, 12.5, Z_F1, Z_F1 + 5)  # under the charge port
    xl4005 = pbox(SEC[1], 66, 89, -27, 27, Z_F1, Z_F1 + 18)
    fuse = pbox(SEC[4], 66, 84, -18, 18, Z_F1, Z_F1 + 18)
    port = pbox(SEC[2], 77.5, 92.5, -6, 6, PORT_Z - 3.5, PORT_Z + 3.5)
    sw_out = FLANK_T - LEAN_T * (Z_EAVE - SW_Z)  # chassis wall at the switch height
    x0, y0 = ppt(SEC[3], sw_out - 31, 0)
    x1, y1 = ppt(SEC[3], sw_out + 1.0, 0)
    switch = cyl((x0, y0, SW_Z), (x1, y1, SW_Z), 9.5)
    x2, y2 = ppt(SEC[3], sw_out + 3.0, 0)
    tbm.booleanOperation(switch, cyl((x1, y1, SW_Z), (x2, y2, SW_Z), 11), UNION)

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

    add_component("Chassis", [(f"ChassisTub_{i + 1}", tubs[i], "Rocky Black") for i in range(N)]
                  + [("ChassisTop", disc, "Rocky Black"), ("BatteryHatch", hatch, "Rocky Black")])
    add_component("Carapace", [(f"Carapace_{i + 1}", wedges[i], "Rocky White") for i in range(N)]
                  + [("CarapaceRing", ring, "Rocky Black"), ("CarapaceCap", cap, "Rocky White")]
                  + [(f"RockFace_{k + 1}", panels[k], "Rocky White") for k in range(N)])
    parts = [("BOUGHT_pi5", pi5), ("BOUGHT_respeaker", respeaker), ("BOUGHT_speaker", speaker),
             ("BOUGHT_camera", camera), ("BOUGHT_pca9685", pca), ("BOUGHT_lipo", lipo),
             ("BOUGHT_xl4005", xl4005), ("BOUGHT_ina219", ina219), ("BOUGHT_fuse", fuse),
             ("BOUGHT_switch", switch), ("BOUGHT_port", port)]
    parts += [(f"BOUGHT_ubec_{k + 1}", u) for k, u in enumerate(ubecs)]
    parts += [(f"BOUGHT_servo_board_{k + 1}", b) for k, b in enumerate(boards)]
    add_component("Electronics", [(n, b, "Rocky Grey") for n, b in parts])
    print("body built:", [o.component.name for o in comp.occurrences])

"""Rocky Walker / Fusion step 02: the leg (coxa bracket, femur, tibia, bought-part placeholders,
revolute joints, bench mount).

Geometry is computed from the Fusion user parameters at run time; re-running this script
rebuilds the leg from the current parameter values (it deletes the previous leg parts first).
All coordinates are in the Leg frame (mm): origin on the coxa axis, +x radially outward, z up,
femur axis along y at x = coxa_mm, knee axis along y at x = coxa_mm + femur_mm.

Servo convention (MG996R placeholder, local frame): origin on the shaft axis at the servo's top
face, local Z = shaft direction, local X = along the body length from the shaft toward the far
end. Horn plate sits on the spline (Z from spline_h - horn_t to spline_h); the driven printed
arm starts at Z = spline_h. Each joint has an idler on the opposite side: an M3 bolt in the
servo-side part running in a 623 bearing pressed into the driven part.
"""

import adsk.core
import adsk.fusion

MM = 0.1  # Fusion API lengths are cm


def run(_context: str):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    P = {p.name: p.value for p in design.userParameters}

    def mm(name):
        return P[name] / MM

    LC, LF, LT = mm("coxa_mm"), mm("femur_mm"), mm("tibia_mm")
    SL, SW, SH = mm("servo_l"), mm("servo_w"), mm("servo_h")
    EL, EP, EG = mm("servo_ear_l"), mm("servo_ear_hole_pitch"), mm("servo_ear_hole_gap")
    EZ, ET, SO, SS = mm("servo_ear_z"), mm("servo_ear_t"), mm("servo_shaft_offset"), mm("servo_shaft_h")
    C, WALL = mm("fit_clearance"), mm("wall")
    LW, LH, LZ = mm("servo_lead_w"), mm("servo_lead_h"), mm("servo_lead_z")
    LOUT, LSTIFF = mm("servo_lead_out"), mm("servo_lead_stiff")
    M3, NAF, NT = mm("m3_clear"), mm("m3_nut_af"), mm("m3_nut_t")
    M4, NAF4, NT4 = mm("m4_clear"), mm("m4_nut_af"), mm("m4_nut_t")  # servo ear screws
    BOD, BID, BW = mm("bearing_od"), mm("bearing_id"), mm("bearing_w")
    HD, HT, HBC = mm("horn_d"), mm("horn_t"), mm("horn_bolt_circle")
    HSC = mm("horn_screw_clear")
    M3P = mm("m3_pilot")
    ARM = 4.0  # driven arm plate thickness (horn side)
    IDL = 5.0  # idler arm thickness (bearing side, 1 mm floor behind the bearing)
    GAP = 0.5  # running gap between parts
    # idler pivot: M3x8 screw, head in a recess in the idler arm clamping only the bearing inner
    # ring against two M3 washers on the cradle floor, threaded into that floor (no nut: the servo
    # base sits right behind it). Washer stack = idler gap, so arm and floor never rub; the floor
    # stays flat so cradles print floor-down without supports.
    IGAP = 1.0  # cradle floor to idler arm = 2 x 0.5 mm M3 washers
    M4P = mm("m4_pilot")  # servo ear screws self-tap into solid ear blocks
    HEAD_R = 3.3  # M3 socket/button head (5.5-5.7 mm) clearance; leaves a 1.7 mm bearing lip
    HUB = 13.0  # hub radius around joint axes

    tbm = adsk.fusion.TemporaryBRepManager.get()
    V = adsk.core.Vector3D.create
    Pt = adsk.core.Point3D.create
    UNION = adsk.fusion.BooleanTypes.UnionBooleanType
    CUT = adsk.fusion.BooleanTypes.DifferenceBooleanType
    INTER = adsk.fusion.BooleanTypes.IntersectionBooleanType

    # ---------- primitive helpers (mm in, temporary B-Rep out) ----------
    def box(x0, x1, y0, y1, z0, z1):
        c = Pt((x0 + x1) / 2 * MM, (y0 + y1) / 2 * MM, (z0 + z1) / 2 * MM)
        obb = adsk.core.OrientedBoundingBox3D.create(
            c, V(1, 0, 0), V(0, 1, 0), abs(x1 - x0) * MM, abs(y1 - y0) * MM, abs(z1 - z0) * MM
        )
        return tbm.createBox(obb)

    def cyl(p0, p1, r):
        return tbm.createCylinderOrCone(Pt(*[v * MM for v in p0]), r * MM, Pt(*[v * MM for v in p1]), r * MM)

    def cone(p0, r0, p1, r1):
        return tbm.createCylinderOrCone(Pt(*[v * MM for v in p0]), r0 * MM, Pt(*[v * MM for v in p1]), r1 * MM)

    def sphere(c, r):
        return tbm.createSphere(Pt(*[v * MM for v in c]), r * MM)

    def hexprism(center, axis, af, depth):
        """Hex nut pocket: axis in {'x','y','z'}, from center along +axis for depth."""
        import math

        big = af * 3
        out = None
        for k in range(3):
            a = math.radians(60 * k)
            ca, sa = math.cos(a), math.sin(a)
            # slab of width af across direction (ca, sa) in the plane normal to axis
            if axis == "y":
                lx, lz = V(ca, 0, sa), V(-sa, 0, ca)
                ly = V(0, 1, 0)
                c = Pt(center[0] * MM, (center[1] + depth / 2) * MM, center[2] * MM)
                obb = adsk.core.OrientedBoundingBox3D.create(c, lx, lz, af * MM, big * MM, depth * MM)
            elif axis == "z":
                lx, ly = V(ca, sa, 0), V(-sa, ca, 0)
                c = Pt(center[0] * MM, center[1] * MM, (center[2] + depth / 2) * MM)
                obb = adsk.core.OrientedBoundingBox3D.create(c, lx, ly, af * MM, big * MM, depth * MM)
            else:
                ly, lz = V(0, ca, sa), V(0, -sa, ca)
                c = Pt((center[0] + depth / 2) * MM, center[1] * MM, center[2] * MM)
                obb = adsk.core.OrientedBoundingBox3D.create(c, ly, lz, af * MM, big * MM, depth * MM)
            s = tbm.createBox(obb)
            if out is None:
                out = s
            else:
                tbm.booleanOperation(out, s, INTER)
        return out

    def uni(*bodies):
        out = bodies[0]
        for b in bodies[1:]:
            tbm.booleanOperation(out, b, UNION)
        return out

    def cut(target, *tools):
        for t in tools:
            tbm.booleanOperation(target, t, CUT)
        return target

    def frame_matrix(origin, zdir, xdir):
        z = V(*zdir)
        x = V(*xdir)
        y = z.crossProduct(x)
        m = adsk.core.Matrix3D.create()
        m.setWithCoordinateSystem(Pt(*[v * MM for v in origin]), x, y, z)
        return m

    def place(body, origin, zdir, xdir):
        tbm.transform(body, frame_matrix(origin, zdir, xdir))
        return body

    # ---------- bought-part placeholders (built in servo-local frame, then placed) ----------
    ear_cx = SL / 2 - SO  # ear/body centre along local X from the shaft

    def servo(origin, zdir, xdir):
        b = box(-SO, SL - SO, -SW / 2, SW / 2, -SH, 0)
        ears = box(ear_cx - EL / 2, ear_cx + EL / 2, -SW / 2, SW / 2, -SH + EZ, -SH + EZ + ET)
        spline = cyl((0, 0, 0), (0, 0, SS - HT), 3.0)
        # cable-exit block on the shaft-end face near the base, plus the straight stretch of
        # cable before it can bend (modelled solid so nothing printed may occupy that space)
        lead = box(-SO - LOUT - LSTIFF, -SO, -LW / 2, LW / 2, -SH + LZ - LH / 2, -SH + LZ + LH / 2)
        return place(uni(b, ears, spline, lead), origin, zdir, xdir)

    def horn(origin, zdir, xdir):
        return place(cyl((0, 0, SS - HT), (0, 0, SS), HD / 2), origin, zdir, xdir)

    def bearing(p0, p1):
        return cut(cyl(p0, p1, BOD / 2), cyl(p0, p1, BID / 2))

    # ear screw positions in servo-local frame (ear plane underside at Z = -SH + EZ)
    ear_holes = [(ear_cx + sx * EP / 2, sy * EG / 2) for sx in (-1, 1) for sy in (-1, 1)]

    def cradle(origin, zdir, xdir):
        """Printed servo cradle in servo-local frame: walls around the body, a flat floor under
        the servo base (idler screw pilot on the shaft axis), and solid ear blocks from the floor
        up to the ears, so nothing overhangs and it prints support-free floor-down or on a side.
        The M4 ear screws self-tap into pilots in the blocks. Open on the shaft side."""
        o = C + WALL
        floor_z0 = -SH - C - WALL
        body = box(-SO - o, SL - SO + o, -SW / 2 - o, SW / 2 + o, floor_z0, -SH + EZ)
        blocks = box(ear_cx - EL / 2 - 2, ear_cx + EL / 2 + 2, -SW / 2 - o, SW / 2 + o, floor_z0, -SH + EZ)
        c = uni(body, blocks)
        pocket = box(-SO - C, SL - SO + C, -SW / 2 - C, SW / 2 + C, -SH - C, 10)
        cut(c, pocket)
        # open-topped slot in the shaft-end wall for the cable-exit block and its straight cable
        cut(c, box(-SO - LOUT - LSTIFF - 1, -SO + 0.01, -LW / 2 - C, LW / 2 + C,
                   -SH + LZ - LH / 2 - C, 10))
        cut(c, cyl((0, 0, floor_z0 - 1), (0, 0, -SH + 1), M3P / 2))  # idler screw pilot
        for hx, hy in ear_holes:
            cut(c, cyl((hx, hy, -SH + EZ - 16), (hx, hy, -SH + EZ + 1), M4P / 2))
        return place(c, origin, zdir, xdir)

    def horn_holes(center_axis_p0, axis, length):
        """4 M3 holes on the horn bolt circle + a centre clearance hole, through a plate."""
        import math

        holes = []
        ax = {"y": (0, 1, 0), "z": (0, 0, 1)}[axis]
        for k in range(4):
            a = math.radians(45 + 90 * k)
            r = HBC / 2
            if axis == "y":
                off = (r * math.cos(a), 0, r * math.sin(a))
            else:
                off = (r * math.cos(a), r * math.sin(a), 0)
            p0 = tuple(center_axis_p0[i] + off[i] for i in range(3))
            p1 = tuple(p0[i] + ax[i] * length for i in range(3))
            holes.append(cyl(p0, p1, HSC / 2))
        p1 = tuple(center_axis_p0[i] + ax[i] * length for i in range(3))
        holes.append(cyl(center_axis_p0, p1, 3.0))  # centre-screw head clearance; must not merge with the bolt holes
        return holes

    # ---------- servo placements ----------
    y_top = SH / 2  # servos on the femur and knee axes are centred on y = 0
    # coxa servo: vertical, shaft +z on the coxa axis, body toward the body centre (-x)
    coxa_arm_z0 = 21.0  # underside of the coxa bracket's top arm = coxa horn top
    coxa_top = coxa_arm_z0 - SS
    coxa_servo_frame = ((0, 0, coxa_top), (0, 0, 1), (-1, 0, 0))
    femur_servo_frame = ((LC, y_top, 0), (0, 1, 0), (0, 0, -1))  # body hangs below the axis
    knee_x = LC + LF
    knee_servo_frame = ((knee_x, y_top, 0), (0, 1, 0), (-1, 0, 0))  # body toward the hip

    arm_y0 = y_top + SS  # horn-side arm inner face
    arm_y1 = arm_y0 + ARM
    floor_y = -SH / 2 - C - WALL  # cradle floor outer face (servo-side idler)
    idl_y1 = floor_y - IGAP
    idl_y0 = idl_y1 - IDL

    # ---------- coxa bracket ----------
    fc = cradle(*femur_servo_frame)
    # cradle extents in leg frame
    o = C + WALL
    cr_x0, cr_x1 = LC - SW / 2 - o, LC + SW / 2 + o
    top_z0, top_z1 = coxa_arm_z0, coxa_arm_z0 + ARM
    bot_z1 = coxa_top - SH - C - WALL - IGAP  # under the hip-yaw pocket floor + 2 washers
    bot_z0 = bot_z1 - IDL
    cr_y0, cr_y1 = floor_y, -SH / 2 + EZ  # cradle y range (floor .. ear plate top)
    def hub_web(z0, z1):
        """Solid web between the arm's hub and the cradle-floor plane, bounded by a 45-degree
        line tangent to the hub, so the bracket prints lying on its cradle floor with nothing
        overhanging. Lies in the arm's own layer (above/below the body)."""
        import math

        t = HUB / math.sqrt(2)  # 45-degree tangent point on the hub at (-t, -t)
        # everything above the 45-degree line (toward the hub), clipped to the floor plane
        web = place(box(-40, 80, 0, 60, z0, z1), (-t, -t, 0), (0, 0, 1), (t / HUB, -t / HUB, 0))
        tbm.booleanOperation(web, box(-t, cr_x0 + 0.5, cr_y0, 0, z0, z1), INTER)
        return web

    top_arm = uni(
        cyl((0, 0, top_z0), (0, 0, top_z1), HUB),
        hub_web(top_z0, top_z1),
        box(0, cr_x1, -HUB, HUB, top_z0, top_z1),
        box(cr_x0, cr_x1, cr_y0, cr_y1, top_z0, top_z1),
    )
    cut(top_arm, *horn_holes((0, 0, top_z0 - 1), "z", ARM + 2))
    bot_arm = uni(
        cyl((0, 0, bot_z0), (0, 0, bot_z1), HUB),
        hub_web(bot_z0, bot_z1),
        box(0, cr_x1, -HUB, cr_y1, bot_z0, bot_z1),  # stays below the femur servo's ears
        box(cr_x0, cr_x1, cr_y0, cr_y1, bot_z0, bot_z1),
    )
    cut(bot_arm, cyl((0, 0, bot_z1 - BW), (0, 0, bot_z1 + 1), BOD / 2))
    cut(bot_arm, cyl((0, 0, bot_z0 - 1), (0, 0, bot_z1), HEAD_R))  # pivot screw head recess
    column = box(cr_x0, cr_x0 + WALL, cr_y0, cr_y1, bot_z0, top_z1)
    # femur servo pocket must stay clear in the column/arms region
    coxa_bracket = uni(fc, top_arm, bot_arm, column)
    cut(coxa_bracket, box(cr_x0 + WALL, cr_x1 - WALL, cr_y0 + WALL, 60, -(SL - SO) - C, SO + C))
    # the femur servo's cable leaves its shaft end pointing up: slot it through the top arm so it
    # exits on top of the bracket and runs back toward the body along the arm
    lead_y = -SH / 2 + LZ
    cut(coxa_bracket, box(LC - LW / 2 - C, LC + LW / 2 + C, lead_y - LH / 2 - C, lead_y + LH / 2 + C,
                          SO, top_z1 + 1))

    # ---------- femur ----------
    def driven_arms(axis_x, length, half_w):
        """Horn-side arm (bolted to the horn) and idler-side arm (bearing) around a y-axis
        joint at (axis_x, 0), both extending +x for `length`."""
        horn_arm = uni(
            cyl((axis_x, arm_y0, 0), (axis_x, arm_y1, 0), HUB),
            box(axis_x, axis_x + length, arm_y0, arm_y1, -half_w, half_w),
        )
        cut(horn_arm, *horn_holes((axis_x, arm_y0 - 1, 0), "y", ARM + 2))
        idl_arm = uni(
            cyl((axis_x, idl_y0, 0), (axis_x, idl_y1, 0), HUB),
            box(axis_x, axis_x + length, idl_y0, idl_y1, -half_w, half_w),
        )
        cut(idl_arm, cyl((axis_x, idl_y1 - BW, 0), (axis_x, idl_y1 + 1, 0), BOD / 2))
        cut(idl_arm, cyl((axis_x, idl_y0 - 1, 0), (axis_x, idl_y1, 0), HEAD_R))  # screw head recess
        return horn_arm, idl_arm

    kc = cradle(*knee_servo_frame)
    kc_x0 = knee_x - (SL - SO) - o  # knee cradle's hip-side edge (servo body points to the hip)
    f_horn, f_idl = driven_arms(LC, kc_x0 - LC + 1, 10.0)
    bridge = box(kc_x0 - 10, kc_x0 + 1, idl_y0, arm_y1, -10, 10)
    femur = uni(f_horn, f_idl, bridge, kc)
    # clearance above the knee servo's hip-side ears (and screw-head access)
    ear_x0 = knee_x - (ear_cx + EL / 2)
    cut(femur, box(ear_x0 - GAP, kc_x0 + 1.5, -SH / 2 + EZ, 60, -SW / 2 - C - 0.5, SW / 2 + C + 0.5))
    # keep the knee servo pocket clear of the bridge
    cut(femur, box(knee_x - SL + SO - C, knee_x + SO + C, -SH / 2 - C, 60, -SW / 2 - C, SW / 2 + C))

    # ---------- tibia ----------
    t_horn, t_idl = driven_arms(knee_x, 38.0, 10.0)
    t_bridge = box(knee_x + 30, knee_x + 42, idl_y0, arm_y1, -10, 10)
    tip_r = 6.0
    CAP_T, CAP_FIT, CAP_LEN = 2.5, 0.15, 14.0  # TPU foot cap: wall, fit gap, sleeve length
    tip_x = knee_x + LT - tip_r - CAP_T  # printed tip sits CAP_T short; the cap restores LT
    spine = cone((knee_x + 36, 0, 0), 10.0, (tip_x, 0, 0), tip_r)
    foot = sphere((tip_x, 0, 0), tip_r)
    tibia = uni(t_horn, t_idl, t_bridge, spine, foot)

    def spine_r(x):  # radius of the tibia cone at x
        return 10.0 + (tip_r - 10.0) * (x - (knee_x + 36)) / (tip_x - (knee_x + 36))

    cap_x0 = tip_x - CAP_LEN
    cap = uni(sphere((tip_x, 0, 0), tip_r + CAP_T),
              cone((cap_x0, 0, 0), spine_r(cap_x0) + CAP_T, (tip_x, 0, 0), tip_r + CAP_T))
    cut(cap, sphere((tip_x, 0, 0), tip_r + CAP_FIT),
        cone((cap_x0 - 1, 0, 0), spine_r(cap_x0 - 1) + CAP_FIT, (tip_x, 0, 0), tip_r + CAP_FIT))
    for k in range(4):  # grip ribs on the sleeve
        x = tip_x - 3 - 3 * k
        groove = cyl((x - 0.4, 0, 0), (x + 0.4, 0, 0), spine_r(x) + CAP_T + 1)
        cut(groove, cyl((x - 0.5, 0, 0), (x + 0.5, 0, 0), spine_r(x) + CAP_T - 0.6))
        cut(cap, groove)

    # ---------- bench mount (stands in for the body on the bench) ----------
    # The hip-yaw servo's pocket exactly as the body will hold it, plus a tail that clamps to a
    # desk edge. The coxa's lower bearing pivots on an M3x8 + 2 washers into this pocket's floor,
    # so the leg hangs between the servo horn (top) and the pivot (bottom), never off the spline
    # alone. The tail stays behind x = -26 so nothing on the leg sweeps into it at +-45 deg yaw;
    # put the tail's front face on the desk edge so the leg hangs over the edge.
    TAIL_X0, TAIL_X1, TAIL_T = -95.0, -26.0, 8.0
    mount = cradle(*coxa_servo_frame)
    m_floor = coxa_top - SH - C - WALL
    m_rim = coxa_top - SH + EZ  # top of the pocket walls / ear blocks
    # the tail is an open-topped box up to the rim, so the mount prints upside down (rim on the
    # plate) with only short bridges: pocket floor and tail plate span ~20 mm between walls
    pocket_back = -(ear_cx + EL / 2 + 2)  # leg x of the pocket's back ear block
    tail = box(TAIL_X0, pocket_back + 0.01, -SW / 2 - o, SW / 2 + o, m_floor - TAIL_T, m_rim)
    cut(tail, box(TAIL_X0 + WALL, pocket_back, -SW / 2 - o + WALL, SW / 2 + o - WALL,
                  m_floor - TAIL_T + 5.0, m_rim + 1))  # 5 mm clamp plate, 3 mm walls
    # under the pocket's back end only a plate (the servo sits above it); stops short of the
    # leg's sweep
    tail = uni(tail, box(pocket_back, TAIL_X1, -SW / 2 - o, SW / 2 + o, m_floor - TAIL_T, m_floor + 0.01))
    mount = uni(mount, tail)
    for hx in (-50.0, -80.0):  # clamp / wood-screw holes through the tail
        cut(mount, cyl((hx, 0, m_floor - TAIL_T - 1), (hx, 0, m_floor + 50), 2.25))

    # ---------- write into Fusion ----------
    leg_occ = next(o for o in root.occurrences if o.component.name == "Leg")
    leg = leg_occ.component
    for j in list(leg.asBuiltJoints) + list(leg.joints):
        j.deleteMe()
    for occ in list(leg.occurrences):
        occ.deleteMe()
    for o2 in list(root.occurrences):
        if o2.component.name in ("FitCoupon", "BenchMount"):
            o2.deleteMe()

    lib = app.materialLibraries.itemByName("Fusion Appearance Library")
    base = None
    for i in range(lib.appearances.count):
        a = lib.appearances.item(i)
        if "Plastic" in a.name and "Glossy" in a.name:
            base = a
            break

    def appearance(name, rgb):
        a = design.appearances.itemByName(name)
        if a:
            return a
        a = design.appearances.addByCopy(base, name)
        for prop in a.appearanceProperties:
            if prop.objectType == adsk.core.ColorProperty.classType() and prop.id == "opaque_albedo":
                prop.value = adsk.core.Color.create(*rgb, 255)
        return a

    colors = {
        "blue": appearance("Rocky Blue", (30, 90, 200)),
        "white": appearance("Rocky White", (235, 235, 230)),
        "black": appearance("Rocky Black", (30, 30, 32)),
        "grey": appearance("Rocky Grey", (120, 120, 125)),
        "silver": appearance("Rocky Silver", (190, 190, 195)),
    }

    def add_component(parent, name, bodies):
        occ = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        comp = occ.component
        comp.name = name
        bf = comp.features.baseFeatures.add()
        bf.startEdit()
        for bname, tb, color in bodies:
            b = comp.bRepBodies.add(tb, bf)
            b.name = bname
            b.appearance = colors[color]
        bf.finishEdit()
        return occ

    servo_occ = add_component(
        leg, "CoxaServo", [("BOUGHT_MG996R_coxa", servo(*coxa_servo_frame), "grey")]
    )
    coxa_occ = add_component(
        leg,
        "Coxa",
        [
            ("Coxa", coxa_bracket, "black"),
            ("BOUGHT_MG996R_femur", servo(*femur_servo_frame), "grey"),
            ("BOUGHT_horn_coxa", horn(*coxa_servo_frame), "silver"),
            ("BOUGHT_bearing_coxa", bearing((0, 0, bot_z1 - BW), (0, 0, bot_z1)), "silver"),
        ],
    )
    femur_occ = add_component(
        leg,
        "Femur",
        [
            ("Femur", femur, "blue"),
            ("BOUGHT_MG996R_knee", servo(*knee_servo_frame), "grey"),
            ("BOUGHT_horn_femur", horn(*femur_servo_frame), "silver"),
            ("BOUGHT_bearing_femur", bearing((LC, idl_y1 - BW, 0), (LC, idl_y1, 0)), "silver"),
        ],
    )
    tibia_occ = add_component(
        leg,
        "Tibia",
        [
            ("Tibia", tibia, "blue"),
            ("FootCap", cap, "black"),
            ("BOUGHT_horn_knee", horn(*knee_servo_frame), "silver"),
            ("BOUGHT_bearing_knee", bearing((knee_x, idl_y1 - BW, 0), (knee_x, idl_y1, 0)), "silver"),
        ],
    )
    tbm.transform(mount, leg_occ.transform)  # built in leg coordinates; sit it where the leg is
    mount_occ = add_component(root, "BenchMount", [("BenchMount", mount, "white")])
    mount_occ.isLightBulbOn = False

    # ---------- joints (axes on the skeleton) ----------
    sk = leg.sketches.itemByName("joints")
    if sk:
        sk.deleteMe()
    sk = leg.sketches.add(leg.xYConstructionPlane)
    sk.name = "joints"
    pts = {
        "coxa": sk.sketchPoints.add(Pt(0, 0, 0)),
        "femur": sk.sketchPoints.add(Pt(LC * MM, 0, 0)),
        "knee": sk.sketchPoints.add(Pt(knee_x * MM, 0, 0)),
    }
    J = adsk.fusion.JointDirections

    def revolute(name, occ1, occ2, point, direction, lo_hi):
        ji = leg.asBuiltJoints.createInput(occ1, occ2, adsk.fusion.JointGeometry.createByPoint(point))
        ji.setAsRevoluteJointMotion(direction)
        j = leg.asBuiltJoints.add(ji)
        j.name = name
        lim = j.jointMotion.rotationLimits
        lim.isMinimumValueEnabled = True
        lim.isMaximumValueEnabled = True
        lim.minimumValue, lim.maximumValue = lo_hi
        return j

    import math

    def fusion_range(lo_param, hi_param, flip):
        lo, hi = P[lo_param], P[hi_param]
        return (-hi, -lo) if flip else (lo, hi)

    # coxa: +z, same sign as the code. femur/knee: code axis is -y, Fusion's is +y -> flip.
    revolute("coxa", coxa_occ, servo_occ, pts["coxa"], J.ZAxisJointDirection,
             fusion_range("coxa_min_deg", "coxa_max_deg", False))
    revolute("femur", femur_occ, coxa_occ, pts["femur"], J.YAxisJointDirection,
             fusion_range("femur_min_deg", "femur_max_deg", True))
    revolute("knee", tibia_occ, femur_occ, pts["knee"], J.YAxisJointDirection,
             fusion_range("tibia_min_deg", "tibia_max_deg", True))
    print("leg built:", [o.component.name for o in leg.occurrences], "joints", leg.asBuiltJoints.count,
          "deg", round(math.degrees(P["tibia_min_deg"]), 1))

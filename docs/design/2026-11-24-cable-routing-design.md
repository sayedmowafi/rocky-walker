# Cable Routing: Hidden Channels

- **Status:** approach chosen by the owner (option A, hidden channels), waiting for servo measurements
- **Date:** 2026-11-24
- **Applies to:** `cad/fusion/02_leg.py` (coxa bracket, femur), `cad/fusion/03_body.py` (chassis)

## Goal

No visible loose wiring. Every servo lead runs **inside** the printed parts and crosses each joint in a controlled loop, then reaches the servo distribution board in the middle of the body (see `docs/bom.md` §7). The anti-goal is the "spaghetti" look of open hexapods. Leads must never be pinched, stretched or rubbed by a moving joint anywhere in its range: coxa ±45°, femur ±40°, knee −135° to −20°.

## The three leads per leg

| Lead | Starts at | Crosses | Route |
|---|---|---|---|
| Coxa (hip-yaw) servo | Inside the chassis | No moving joint | Straight between the chassis plates to the distribution board |
| Femur (hip-lift) servo | Coxa bracket cradle | Coxa joint | Cradle → coxa-bracket channel → **coxa window** → between the plates |
| Knee servo | Knee cradle on the femur | Femur joint, then coxa joint | Femur channel → **femur drum** → coxa-bracket channel → coxa window → between the plates |

## Elements

1. **Channels.** A U-groove along each printed link (femur, coxa bracket), on the inside face, sized to the measured cable plus 0.5 mm. A snap-in printed cover strip, or rows of small retention tabs, keeps the cable seated. The cable is laid in, not threaded, so plugs never need to pass through.
2. **Femur drum (knee lead crossing the femur joint).** A round boss concentric with the femur axis, on the idler (bearing) side of the coxa bracket, with a covered groove around it. The cable wraps part-way round the drum. As the femur swings ±40°, the cable winds and unwinds on a fixed radius instead of stretching. The slack needed is about the drum radius × the joint range (≈ 14 mm × 1.4 rad ≈ 20 mm), which sits in the groove.
3. **Coxa window (both leg leads crossing the coxa joint).** A slot through the coxa bracket's vertical column. Beyond it, the cables cross a small open gap to the chassis edge and enter between the two chassis plates, beside the coxa servo. The ±45° yaw is taken up by a short service loop in that gap, kept below the top arm and above the bottom arm so nothing outside the leg can catch it.
4. **Inside the body.** The leads run between the plates, alongside each coxa servo, to the distribution board at the centre. Cable-tie anchor slots are added to the bottom plate, and the plate pillars double as routing posts.
5. **Extensions.** A lead that comes up short gets a 22 AWG extension soldered and heat-shrunk *inside* a channel. The joint is never placed in a drum or a service loop.

## Measurements needed (when the servos arrive)

Measure two servos, one Reland Sun and one Beffkkip, with calipers:

1. **Cable cross-section:** width × thickness of the 3-wire lead (typically about 3.5 × 1.3 mm).
2. **Lead exit:** which face it leaves the servo body from, and its distance from the base and from the end opposite the shaft.
3. **Lead length:** from the servo body to the start of the plug.
4. **Plug size:** length × width × thickness of the 3-pin plug housing.
5. **Horn bolt circle:** the hole spacing on the aluminium disc horn (the CAD currently assumes 14 mm).

These five numbers become Fusion parameters (`cable_w`, `cable_t`, `lead_exit_*`, `lead_len`, `horn_bolt_circle`).

## Checks after modelling

- **No collisions:** the existing leg and body interference sweeps still pass at every joint limit.
- **Cable paths stay clear:** each lead's path is modelled as a thin body and added to the sweep, so a moving part that cuts a cable path fails the report.
- **Lengths add up:** each lead's path length plus 20 mm slack is at most the measured lead length; otherwise an extension is listed in the BOM.
- **Printable:** parts still fit the 180 mm bed, and channels face up or have tabs, so they need no supports.

## Order of work

1. **Now:** print the **fit coupon only** (`cad/stl/FitCoupon.stl`). Leave the leg parts until the channels are in, so nothing gets printed twice.
2. **Servos arrive:** measure, send the numbers, and I add the channels, drum, window and body slots in Fusion. Then re-run the reports and re-export.
3. **Then print the bench leg** with routing built in.

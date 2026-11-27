# Measured parts (calipers, 2026-11-26)

Real measurements replace listing/datasheet sizes in the Fusion parameters.

## MG996R (Reland Sun)

| Dimension | Measured | Tower Pro datasheet | Was in CAD |
|---|---|---|---|
| Body length | **40.3 mm** | 40.3 (drawing) / 40.7 (text) | 40.7 |
| Body width | **19.9 mm** | 20 / 19.7 | 19.7 |
| Case height, base to top of case | **36.6 mm** | 36.6 | 42.9 (included the top boss) |
| Base to top of the metal output spline | **44.5 mm** | — (47.6 incl. horn) | 47.9 |
| Length across both ears | **54.0 mm** | 53.6 | 54.6 |
| Base to top of ears | ≈ 31 mm (rough) | 26.6 (unclear which face) | 29.0 underside |
| Ear thickness | **2.5 mm** | — | 2.5 |
| Ear holes | fit **M4** screws | — | M3 |
| Ear hole spacing, along / across | — | — | 49.5 / 10 |
| Shaft centre to nearest body end | — | — | 10.2 |

## Cross-check against a community STEP model (`mg996r v17.step`, read with OpenCascade)

| Dimension | STEP model | Used in CAD |
|---|---|---|
| Body L × W | 40.0 × 19.5 (rounded) | **40.3 × 19.9** (calipers: the pocket must fit the real part) |
| Case height, base to top of case | 38.55 | **38.55** |
| Base to top of spline | 44.2 | **44.2** (spline 5.65 above the case) |
| Across ears | 54.0 | **54.0** |
| Ear underside / top above base | 30.0 / 32.4 | **28.4** (real servo, see below) |
| Ear thickness | 2.4 | **2.4** |
| Ear holes | Ø 4.4, pitch 49.4 along, 10.5 across | **Ø 4.4 (M4), 49.4 × 10.5** |
| Shaft centre to nearest end | 9.75 | **9.75** |
| Output spline | 25T, Ø 6.1 | 25T |

Decision (owner, 2026-11-26): use the STEP model's values where they disagreed with rough caliper readings; keep the caliper body L × W.
Servo ear screws change from M3 to **M4** (holes are Ø 4.4), with M4 captive-nut pockets or M4 self-tapping pilots.

## Aluminium 25T disc horn

| Dimension | Caliper (tips in the holes = edge to edge) | Centre to centre | In CAD |
|---|---|---|---|
| Opposite holes | 11.3–11.4 mm | **13.8 mm** | bolt circle 13.8 |
| Neighbouring holes | 7.3 mm | 9.8 mm | 9.76 |
| Threaded hole size (solved) | — | ≈ 2.5 mm (M3 tap) | — |

A square pattern gives neighbour = across / √2 centre to centre. Edge readings subtract one hole
diameter from both: (13.8 − 2.5 = 11.3, 9.76 − 2.5 = 7.26). The first CAD pass took 11.3 as centre
to centre. The printed parts then measured 8.1 / 4.9 edge to edge (11.3 / 8.0 centre to centre,
3.2 mm holes), which matched the CAD but not the horn. Corrected 2026-11-27. Printed horn holes
are now 3.6 mm (PETG prints ~0.2 mm small), and the centre clearance hole stays 6 mm.

## Fit checks on printed parts (2026-11-27)

| Check | Measured | CAD before | Fix |
|---|---|---|---|
| Servo base to top of the aluminium horn disc (horn fitted) | **47.0 mm** | 44.2 (disc assumed flush with the spline top) | `horn_top_h` = 8.45 above the case top |
| Bench mount underside to horn disc top, servo hanging on its ears | **52 mm** | 50.3 | The servo hung 1.7 mm higher than modelled: its ears are lower on the body than the STEP model says. `servo_ear_z` 30.0 → **28.4** (agrees with the rough ≈ 31 to the ear top, minus 2.5 ear) |
| M4 ear screw into a 3.4 mm pilot (PLA) | too tight to drive by hand | — | `m4_pilot` → 3.7 mm |

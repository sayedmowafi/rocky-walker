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
| Ear underside / top above base | 30.0 / 32.4 | **30.0** |
| Ear thickness | 2.4 | **2.4** |
| Ear holes | Ø 4.4, pitch 49.4 along, 10.5 across | **Ø 4.4 (M4), 49.4 × 10.5** |
| Shaft centre to nearest end | 9.75 | **9.75** |
| Output spline | 25T, Ø 6.1 | 25T |

Decision (owner, 2026-11-26): use the STEP model's values where they disagreed with rough caliper readings; keep the caliper body L × W.
Servo ear screws change from M3 to **M4** (holes are Ø 4.4), with M4 captive-nut pockets or M4 self-tapping pilots.

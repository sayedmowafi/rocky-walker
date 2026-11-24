# What the CAD changed (M2)

Values the physical design forced to change in `config/rocky.toml`, and the re-checks that
followed (full test suite incl. the walking-envelope check, and the torque sweep).

| Key | Was | Now | Why |
|---|---|---|---|
| `limits.femur_deg` | [-60, 90] | [-40, 40] | Below -40° the knee servo swings into the hip (coxa) bracket and femur servo; above 40° the femur's bridge reaches the coxa bracket. |
| `limits.tibia_deg` | [-150, -20] | [-135, -20] | Below -135° the tibia folds into the femur and the femur servo. |

Re-check after the change: `uv run pytest` 76 passed (walking envelope OK for both gaits);
torque wave 73.4 %, star 90.2 % of the rule (unchanged — walking never used the removed range).
| `body.thickness_mm` | 40 | 32 | It's the sim's torso collision cylinder; set to the real chassis height (32.3 mm). |

**Other design changes from the body (not config values):**
- **Battery:** 2S **2200 mAh** shorty instead of 3000–5000 mAh. The reference pack is the Zeee 2S 2200 mAh 50C XT60 (73 × 34 × 18.5 mm, 98 g). A larger pack would reach the coxa brackets' bottom arms. It hangs in a strapped tray under the chassis, long axis between legs 0 and 1.
- **Camera:** on a vertical tab at the deck's front edge between legs 0 and 1, looking straight out, instead of on a carapace facet (a dome curves away underneath any flat board).
- **Carapace:** one printed piece (≈ 117 × 121 × 42 mm), not wedges. It fits the 180 mm bed whole.

## Mass (Task 5)

| | M1 estimate | Measured from CAD |
|---|---|---|
| Total | 2.000 kg | **1.694 kg** (incl. 60 g screws/wires allowance and the 98 g reference pack; printed parts at 0.6 × solid PLA) |
| Coxa segment | 0.080 kg | 0.0852 kg, COM 34.1 mm from the coxa axis (z offset −7.4 mm, ignored by the sim) |
| Femur segment | 0.090 kg | 0.0876 kg, COM 48.9 mm from the femur axis (the knee servo pulls it outward) |
| Tibia segment | 0.040 kg | 0.0322 kg, COM 29.6 mm from the knee axis |

The sim now uses each segment's measured mass at its measured centre of mass (thin-rod inertia).
Torque re-check: **wave 59.0 %**, **star 70.4 %**, dance routine 56 % of the rule (was 73.4 % / 90.2 % with the estimates).

## Fit pass (2026-11-24): every bought part modelled, body enclosed

All parts from `docs/bom.md` are now modelled at listing/datasheet sizes (**provisional until measured with calipers**) and included in the collision check, with all five legs swung to their hip limits. Result: **no collisions**; every printed part fits a 180 mm bed.

| Level | What lives there |
|---|---|
| Carapace (taller, faceted, z 29–94) | Pi 5 + Active Cooler on the deck; ReSpeaker Lite on posts above it (mic holes in the roof); speaker beside it (grille in the roof); Camera Module 3 behind a window in the wall |
| Mezzanine (top plate → deck) | PCA9685 |
| Basement (between the plates) | 5 servo boards, one per leg, which the servos plug into |
| Belly (new, with screw-on hatch) | 2S LiPo, 5 UBECs, XL4005, INA219, fuse holder; power switch and balance-charge port in the belly wall |
| Side panels (5, new) | Close the gaps between the legs; hidden vent slots; notched where the plates pass through |

Other changes:
- Plate core radius 46 → 62 mm; plate spacers moved onto the leg axes; deck pillars moved to the sector midlines.
- Deck radius 56 → 65.5 mm. The battery tray is gone; the battery sits on the belly hatch, strapped through two slots.
- **TPU foot caps:** the printed tibia tip is 2.5 mm shorter and the cap restores the full 100 mm, so the walking maths is unchanged.
- **Mass: 1.694 → 2.039 kg** (enclosure + all electronics). Torque re-check: **wave 73.0 %, star 87.4 %, dance 70 %** of the rule. Still within budget.

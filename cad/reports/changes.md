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
- **Battery:** 2S **2200 mAh** (≈ 106 × 34 × 18 mm, ≈ 120 g) instead of 3000–5000 mAh. A larger pack would reach the coxa brackets' bottom arms. It hangs in a strapped tray under the chassis, long axis between legs 0 and 1.
- **Camera:** on a vertical tab at the deck's front edge between legs 0 and 1, looking straight out, instead of on a carapace facet (a dome curves away underneath any flat board).
- **Carapace:** one printed piece (≈ 117 × 121 × 42 mm), not wedges. It fits the 180 mm bed whole.

## Mass (Task 5)

| | M1 estimate | Measured from CAD |
|---|---|---|
| Total | 2.000 kg | **1.718 kg** (incl. 60 g screws/wires allowance; printed parts at 0.6 × solid PLA) |
| Coxa segment | 0.080 kg | 0.0852 kg, COM 34.1 mm from the coxa axis (z offset −7.4 mm, ignored by the sim) |
| Femur segment | 0.090 kg | 0.0876 kg, COM 48.9 mm from the femur axis (the knee servo pulls it outward) |
| Tibia segment | 0.040 kg | 0.0322 kg, COM 29.6 mm from the knee axis |

The sim now uses each segment's measured mass at its measured centre of mass (thin-rod inertia).
Torque re-check: **wave 60.0 %**, **star 71.6 %** of the rule (was 73.4 % / 90.2 % with the estimates).

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

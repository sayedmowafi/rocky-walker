# What the CAD changed (M2)

Values the physical design forced to change in `config/rocky.toml`, and the re-checks that
followed (full test suite incl. the walking-envelope check, and the torque sweep).

| Key | Was | Now | Why |
|---|---|---|---|
| `limits.femur_deg` | [-60, 90] | [-40, 40] | Below -40° the knee servo swings into the hip (coxa) bracket and femur servo; above 40° the femur's bridge reaches the coxa bracket. |
| `limits.tibia_deg` | [-150, -20] | [-135, -20] | Below -135° the tibia folds into the femur and the femur servo. |

Re-check after the change: `uv run pytest` 76 passed (walking envelope OK for both gaits);
torque wave 73.4 %, star 90.2 % of the rule (unchanged — walking never used the removed range).

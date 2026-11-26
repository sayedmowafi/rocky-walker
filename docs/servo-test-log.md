# Servo bench test log

Test each servo on its own with `tools/servo_test_esp32/` (ESP32 + one UBEC at 6.0 V, 5 A fuse).
Label every servo with a number on masking tape before testing.

| # | Brand | Centres and holds quietly (c) | Smooth sweep, no grinding (s) | Hold-buzz at rest | Wide-sweep end stops (us) | Notes | Use for |
|---|---|---|---|---|---|---|---|
| 1 | Reland Sun (single pack) | ✅ | ✅ | silent | not tested | 1000/2000/1500 us OK, factory horn fitted | hip candidate |
| 2 | Reland Sun (single pack) | ✅ | ✅ | silent | not tested | OK | hip candidate |
| 3 | Reland Sun (single pack) | ✅ | ✅ | silent | not tested | OK | hip candidate |
| 4 | Reland Sun (single pack) | ✅ | ✅ | silent | not tested | OK | hip candidate |
| 5 | Reland Sun (single pack) | ✅ | ✅ | silent | not tested | OK | hip candidate |
| 6 | Reland Sun (single pack) | ✅ | ✅ | silent | not tested | OK | hip candidate |
| 7 | Reland Sun (single pack) | ✅ | ✅ | silent | not tested | OK | hip candidate |
| 8 | Reland Sun (single pack) | ✅ | ✅ | silent | not tested | OK | hip candidate |
| 9 | Reland Sun (single pack) | ✅ | ✅ | silent | not tested | OK | hip candidate |
| 10 | Reland Sun (single pack) | ✅ | ✅ | silent | not tested | OK | hip candidate |
| 11 | Beffkkip (2-pack) | ✅ | ❌ jams | silent | — | FAULTY: clicks in slow sweep; on fast jumps jams at the same spot, clicks, then recovers | RETURN, do not use |
| 12 | Beffkkip (2-pack) | ✅ | ❌ jams | silent | — | FAULTY: drives to 1000 us end then sticks, gears click; by hand it stops dead at one point (internal jam / damaged gear) | RETURN, do not use |
| 13 | Beffkkip (2-pack) | — | — | — | — | FAULTY out of the box: jams by hand at the same position as #11/#12 | RETURN, do not use |
| 14 | Beffkkip (2-pack) | — | — | — | — | FAULTY out of the box: jams by hand at the same position as #11/#12 | RETURN, do not use |
| 15 | Besomi (local shop, single) | ✅ | ✅ | silent | not tested | slow sweep + 6 full-range jumps clean | knee |

Best ten (smoothest, quietest) go to the hips (coxa, femur). The rest go to the knees.

## Result (2026-11-26)

- **Reland Sun (#1–#10): 10/10 good.** All smooth and silent at rest. These are the hip servos.
- **Beffkkip (#11–#14): 4/4 faulty.** Every one jams at the same position (#11 and #12 under power, #13 and #14 by hand, out of the box): a defective batch or design. All four returned.
- **Besomi #15: good** (slow sweep + full-range jumps clean). **11 good servos in total.**
- **Still needed:** 4 knee servos + 2 spares (Reland Sun, since all 10 were perfect).
- **Lesson for rebuilders:** before testing a new servo under power, turn the output shaft gently by hand through its range. A dead spot means a damaged gear, and powering it only makes it worse.

# Build log #1: Rocky walks (in simulation)

*2026-11-23 · Milestone M1*

Before printing anything, I wanted to know one thing: **can 15 cheap MG996R servos actually carry a five-legged, roughly 2 kg robot?** So Rocky's first steps happened in MuJoCo, a free physics simulator, driven by the exact same Python code that will drive the real servos later.

| Forward | Diagonal | Turn in place |
|---|---|---|
| ![forward](media/01-walk-forward.gif) | ![diagonal](media/01-walk-diagonal.gif) | ![turn](media/01-turn-in-place.gif) |

## What got built

- **One config file** ([`config/rocky.toml`](../../config/rocky.toml)) holds every dimension, limit and gait setting. The simulator model is *generated* from it, and so will the CAD be.
- **Leg inverse kinematics.** Each leg has 3 joints (hip yaw, hip lift, knee), solved analytically. It's tested by round-tripping every joint combination inside the joint limits.
- **Five legs, no front.** Legs sit at 72° steps around the body. Walking toward a leg, between two legs, sideways or at a random 197° heading all pass the same test.
- **Two gaits for five legs:**
  - **Wave:** one leg in the air at a time, so 4 feet are always down. The safe default.
  - **Star:** leg *i* starts its step at phase 2i/5, so legs lift in the order **0 → 3 → 1 → 4 → 2**. Two legs can be up at once, but never two neighbours. It's 33% faster.

  The usual hexapod "tripod" gait doesn't divide evenly across five legs, which is why Rocky needs its own.
- **Motion guards.** The command ramps at an acceleration limit, a joystick pushed too far is scaled down to the longest stride the legs can manage, joint limits are enforced, and joint speed is capped at what an MG996R can do. Letting go of the joystick mid-step lets the swinging feet finish and settle.
- **A sim CLI:** `uv run python -m rocky.sim walk --vx 0.03 --video out/walk.mp4`, and `... torque --gait star` for a torque report.

## The question: are MG996Rs strong enough?

My rule is that **sustained torque on any joint (0.5 s average) must stay at or below 0.49 N·m (5 kg·cm)**. That's about half of an MG996R's stall torque at 6 V, leaving room for cheap-clone variation.

In the sim, the servos are torque-limited to MG996R stall, so the sim can't cheat.

**First result, at the starting geometry (stance radius 180 mm):** it passes, but only just.

| Robot mass | Wave | Star |
|---|---|---|
| 2.0 kg (estimate) | 89% of the limit | 98% |
| 2.2 kg | 99% | 109% ❌ |
| 2.4 kg | 109% ❌ | 120% ❌ |

My mass estimate is 1.8–2.1 kg, and real builds always come out heavier, so 10% margin wasn't enough to print against. The hip-lift (femur) servo does almost all the work. Its load is the robot's weight times the horizontal distance from the hip joint to the foot.

**The fix: shrink before upgrading.** I brought the feet 15 mm closer to the body (stance radius 180 → **165 mm**). That cuts the femur lever from 70 mm to 55 mm.

| Robot mass | Wave (default) | Star, worst incl. start-up | Star, once walking |
|---|---|---|---|
| 2.0 kg | 73% | 90% | 78% |
| 2.2 kg | 82% | 101% ❌ | 87% |
| 2.4 kg | 90% | 112% ❌ | 95% |

The default wave gait now passes even if Rocky comes out 20% heavier than estimated. Star passes at the design mass. Once it's walking it stays inside the rule up to 2.4 kg, but **starting** in star from a standstill goes over at 2.2 kg, because the last leg drags a long way before its first step. If the real robot comes out heavy, the fix is to start walking in wave and switch to star once moving. Walking accuracy and tilt didn't change measurably (tilt stays under 1°).

*(These numbers include the fixes from the code review below, which made start-up more honest and a little harder on star.)*

## Frozen values (M1)

| | |
|---|---|
| Coxa / femur / tibia | **40 / 70 / 100 mm** |
| Body centre → hip (mount radius) | 70 mm |
| Stance radius (body centre → foot) | **165 mm** (≈ 33 cm foot-to-foot) |
| Body height (hip joints above ground) | 100 mm |
| Step height | 30 mm |
| Max stride | 60 mm |
| Gait period | 1.5 s (wave duty 0.8, star duty 0.6) |
| Top speed | wave 5 cm/s, star 6.7 cm/s |
| Acceleration limit | 0.15 m/s², 45°/s² (≈0.4 s to top speed) |

Torque at 80% of top speed, 2.0 kg, worst leg:

| Joint | Wave (sustained) | Star (sustained) |
|---|---|---|
| Coxa (hip yaw) | 0.054 N·m | 0.054 N·m |
| Femur (hip lift) | 0.36 N·m | 0.44 N·m (start-up) |
| Tibia (knee) | 0.19 N·m | 0.28 N·m |

## What didn't go perfectly

- **The star gait briefly hits stall torque at touchdown.** The sustained average is fine (79%), but the instantaneous peaks reach 1.08 N·m, the MG996R's stall limit, for a few milliseconds as feet land. Wave peaks at about 0.9 N·m. Wave stays the default, and softening touchdown is on the list for the real robot.
- **Rocky walks about 5% further than commanded.** Feet start from neutral rather than mid-stride, so the first steps add a little extra. It's harmless, but it will matter for follow mode later.
- **Floating-point maths nearly broke the gait.** `0.6 − 0.2` isn't exactly `0.4` in floating point, and at exact phase boundaries that briefly put two neighbouring legs in the air together. The tests caught it, and phases are now rounded at the boundaries.
- **The simulator is optimistic.** It has perfect servos with no gear slop or deadband, rigid parts and an even floor. The real MG996Rs will be worse, which is exactly why I wanted the margin.

## What the code review caught

Before calling M1 done, the whole branch got an independent review. It found real bugs that the walking tests didn't catch, because those tests only used sensible joystick inputs:

- **Holding the joystick past top speed wound up the command.** The speed ramp chased the raw joystick value (say 5 m/s) while the legs could only do 5 cm/s. After letting go, Rocky kept walking for **about 11 seconds**. The command is now limited to what the legs can do *before* it's ramped.
- **The star gait jerked one leg 30 mm up in a single tick** whenever it started from standing. That leg was already half-way through its step window, so it started its step half-way up. On real servos that's a 2,600°/s demand. It now waits for its next window.
- **One bad number (NaN) froze the controller forever.** Invalid commands are now rejected and counted, and the config refuses `nan`/`inf`.
- **The start-up check only looked at the standing pose.** Rocky now drives a scratch copy of itself at full speed in 20 directions, then reverses, and refuses to start if any joint would hit a limit. The check immediately caught that full-speed start-up and reversal pushed the hip-yaw joint 1° past its ±45° limit. **Halving the acceleration limit** (to 0.15 m/s²) fixed it.
- **Every safety guard now counts when it activates,** so the tests prove normal walking never needs the guards, not just that the guards exist.

## Next: M2, one leg on the bench

Print one leg, wire 3 MG996Rs through a PCA9685, write the servo calibration tool, and make the real foot trace a circle smoothly. That means designing the leg in build123d, from this same config file.

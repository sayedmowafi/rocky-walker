# Rocky Walker M0 + M1 (Repo Skeleton, Kinematics, Gait, Simulated Walking): Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stand up the open-source repo (M0), then get a five-legged Rocky walking in any direction and turning in place in MuJoCo, using the same kinematics and gait code that will later drive real servos. Then use the sim's torque log to freeze leg lengths (M1).

**Architecture:**
- One TOML config (mm and degrees, for humans) is loaded into SI-unit dataclasses that every module reads.
- Pure-numpy modules: analytic 3-DOF leg IK, body/mount geometry, phase-based gait patterns, and an incremental foot planner.
- A `MotionController` turns a body-velocity command into a `(5, 3)` joint-angle array each tick and writes it to a servo backend (dry-run now, MuJoCo for sim, PCA9685 in the next plan).
- The MuJoCo model is generated from the config.

**Tech Stack:** Python 3.13 (pinned via uv; `requires-python >= 3.11`), numpy, mujoco, pytest, ruff, uv. Optional: imageio + imageio-ffmpeg for sim video. GitHub Actions CI.

**Spec:** `docs/design/2026-11-23-rocky-walker-design.md`. Read §5–§9 before starting.

**Out of scope for this plan:**
- build123d CAD, PCA9685 backend, calibration tool (next plan: M2, bench leg)
- the separate prioritised motion process, IPC, command arbitration (M3/M5)
- phone UI, vision, voice

## Global Constraints

- Internal units are **SI** (metres, radians, seconds, newton-metres). Only `config/rocky.toml` uses mm/degrees/kg·cm, and key names carry the unit suffix (`_mm`, `_deg`, `_kgcm`, `_dps`).
- **5 legs** at yaw `ψᵢ = 2πi/5` (leg 0 on +x, counter-clockwise numbering), **3 DOF per leg** in the order coxa, femur, tibia. Joint arrays are always shape `(5, 3)`.
- Leg frame: origin on the coxa axis at the mount, x radially outward, z up. Body frame: origin at the body centre at joint height, z up. Every joint is at the same height as its mount.
- Joint conventions: coxa `θ1` about +z (0 = radially outward). Femur `θ2`, positive lifts the femur (0 = horizontal). Tibia `θ3`, relative to the femur, negative bends the knee down (standing ≈ −90°). Knee-up IK branch only.
- **Torque rule:** sustained (0.5 s moving average of |τ|) joint torque ≤ **0.49 N·m (5 kg·cm)** on every joint during the default gait.
- Motion loop: **target 100 Hz, minimum 50 Hz**.
- Gait legs swing with phase offset **φᵢ = 2i/5**. Swing start order is **0 → 3 → 1 → 4 → 2**. **Two adjacent legs are never in swing together. At least 3 feet are always down.** Wave = duty 0.8 (exactly one swing leg at a time); star = duty 0.6.
- When constraints collide: **shrink before upgrading** (spec §6.3 order: wave-only/stance → smaller robot → stronger servo last).
- Licences: **MIT** (software), **CERN-OHL-P v2** (hardware). The README carries the fan disclaimer (Task 1). There must be **no book or film art, text or assets** in the repo.
- Every public push needs the owner's explicit go-ahead. **Never create or push to a GitHub repo without asking.**

## Review Focus

1. **Joystick pushed past what the legs can reach** (for example `vx=5 m/s, wz=5 rad/s`): the command is scaled down to max stride. There are no IK failures and no joint-limit clamps. Test: Task 7 `test_extreme_command_stays_reachable`.
2. **Abrupt reversal** (full forward → full backward in one tick): no joint moves faster than the servo speed limit, and the command ramps at the accel limit. Test: Task 7 `test_reversal_respects_joint_speed`.
3. **Joystick released mid-step:** swinging feet finish their step, all feet settle on neutral, and stepping stops (phase freezes). Test: Task 6 `test_stop_mid_step_settles_to_neutral`.
4. **A rebuilder edits the config badly** (negative length, reversed limits, duty too low for the gait, stance out of reach): it fails at load or start-up with a `ConfigError` naming the offending key, not a crash deep in the sim. Tests: Task 2 `test_rejects_bad_values`, Task 7 `test_unreachable_stance_rejected`.
5. **Long continuous turning:** feet stay within half a stride of neutral, with no drift after 60 s. Test: Task 6 `test_continuous_turn_feet_bounded`.

---

## File map

```
pyproject.toml  .python-version  .gitignore  README.md  LICENSE  LICENSE-HARDWARE
.github/workflows/ci.yml
config/rocky.toml
docs/bom.md
docs/build-log/2026-11-23-00-hello-rocky.md
rocky/__init__.py
rocky/config.py                 config dataclasses + loader + validation
rocky/commands.py               BodyVelocity
rocky/kinematics/__init__.py
rocky/kinematics/leg.py         leg FK/IK
rocky/kinematics/body.py        mounts, frame transforms, neutral feet
rocky/gait/__init__.py
rocky/gait/patterns.py          GaitPattern, make_pattern
rocky/gait/generator.py         GaitGenerator (foot planner)
rocky/motion/__init__.py
rocky/motion/controller.py      MotionController, MotionStats
rocky/backends/__init__.py
rocky/backends/base.py          ServoBackend protocol, DryRunBackend
rocky/backends/mujoco_backend.py
rocky/sim/__init__.py
rocky/sim/model.py              build_mjcf, joint_names, load_model
rocky/sim/runner.py             simulate, SimResult
rocky/sim/__main__.py           CLI: python -m rocky.sim ...
tests/...                       mirrors the package; sim tests in tests/sim/
```

---

### Task 1: Repo skeleton (M0)

**Files:**
- Create: `pyproject.toml`, `.python-version`, `.gitignore`, `README.md`, `LICENSE`, `LICENSE-HARDWARE`, `.github/workflows/ci.yml`, `docs/bom.md`, `docs/build-log/2026-11-23-00-hello-rocky.md`, `rocky/__init__.py`
- Test: `tests/test_smoke.py`

**Interfaces:**
- Produces: importable package `rocky` with `rocky.__version__ == "0.1.0"`. Commands `uv run pytest`, `uv run ruff check .` and `uv run ruff format --check .`. Pytest marker `sim` registered.

- [ ] **Step 1:** `git init -b main` in the project root. Run `uv init --package --name rocky --python 3.13 --no-readme` (or write `pyproject.toml` by hand), then make the layout flat: the package lives at `rocky/`, not `src/rocky/`.
- [ ] **Step 2:** `pyproject.toml`:
  - `requires-python = ">=3.11"`, dependencies `numpy>=2`, `mujoco>=3.3`
  - optional `[project.optional-dependencies] video = ["imageio>=2.34", "imageio-ffmpeg>=0.5"]`
  - dev group `pytest>=8`, `ruff>=0.8`
  - `[tool.pytest.ini_options] markers = ["sim: runs the MuJoCo simulation (slower)"]`, `testpaths = ["tests"]`
  - `[tool.ruff] line-length = 100`, lint select `["E","F","I","UP","B","NPY"]`
- [ ] **Step 3:** Write the failing test `tests/test_smoke.py::test_package_imports`: `import rocky; assert rocky.__version__ == "0.1.0"`. Run `uv run pytest`. Expected: FAIL (no `__version__`).
- [ ] **Step 4:** In `rocky/__init__.py`, set `__version__ = "0.1.0"`. Run `uv run pytest`. Expected: 1 passed.
- [ ] **Step 5:** `README.md` sections:
  - title "Rocky Walker", a one-paragraph pitch (five-legged, radially symmetric, Pi 5 + PCA9685 + MG996R, cheap to rebuild)
  - **Status** ("M0: repo skeleton. Nothing walks yet.")
  - **Why** (open from day one, build log)
  - **Repo layout** (copy spec §10)
  - **Licences**
  - **Disclaimer**, with this exact text:
    > Rocky Walker is a fan-made project inspired by the character Rocky from Andy Weir's novel *Project Hail Mary*. It is not affiliated with or endorsed by Andy Weir, his publishers, or the film's producers. All designs, code and media in this repository are original.
- [ ] **Step 6:** Licence files:
  - `LICENSE`: standard MIT text, `Copyright (c) 2026 El Sayed Mowafi`.
  - `LICENSE-HARDWARE`: the official CERN-OHL-P v2 text from `https://ohwr.org/cern_ohl_p_v2.txt`, unmodified. It covers `cad/`, generated models and hardware docs. Say so in one line in the README.
- [ ] **Step 7:** `docs/bom.md`: the spec §7.5 table as a draft ("prices are estimates, not yet bought"), plus a "Buy now for M2" line: 4 × MG996R, 1 × PCA9685, the 6 V UBEC, a 2S LiPo + XT60, a fuse, and 4 aluminium horns. `docs/build-log/2026-11-23-00-hello-rocky.md`: what the project is, decisions so far (link the design doc), what's next (M1 sim).
- [ ] **Step 8:** `.github/workflows/ci.yml` on push/PR, `ubuntu-latest`: `astral-sh/setup-uv` → `uv sync` → `uv run ruff check .` → `uv run ruff format --check .` → `uv run pytest`. `.gitignore`: Python defaults, `.venv/`, `*.mp4`, `out/`, `*.stl`, `*.step`.
- [ ] **Step 9:** Run `uv run ruff check . && uv run ruff format --check . && uv run pytest`. Expected: clean, 1 passed.
- [ ] **Step 10:** Commit everything, including the existing `docs/design/` and `docs/plans/`:
  ```bash
  git add -A && git commit -m "chore: repo skeleton, licences, design doc, build log #0"
  ```
- [ ] **Step 11:** **Ask the owner** for the GitHub repo name and whether to publish now. Only on a yes: `gh repo create <name> --public --source . --push`, then confirm CI goes green.

---

### Task 2: Config file and loader

**Files:**
- Create: `config/rocky.toml`, `rocky/config.py`
- Test: `tests/test_config.py`

**Interfaces:**
- Produces (all frozen dataclasses, SI units):
  - `BodyParams(n_legs: int, mount_radius: float, thickness: float)`
  - `MassParams(total: float, coxa_link: float, femur_link: float, tibia_link: float)`
  - `LegGeometry(coxa: float, femur: float, tibia: float)`
  - `JointLimits(coxa: tuple[float, float], femur: tuple[float, float], tibia: tuple[float, float])`, with `as_array() -> np.ndarray` of shape `(3, 2)`
  - `ServoSpec(stall_torque: float, max_speed: float, torque_rule: float)`
  - `GaitParams(default: str, period: float, duty_factor: dict[str, float], body_height: float, stance_radius: float, step_height: float, max_stride: float)`
  - `MotionParams(loop_hz: float, max_linear_accel: float, max_angular_accel: float)`
  - `RobotConfig(body, mass, leg, limits, servo, gait, motion)`
  - `class ConfigError(ValueError)`
  - `DEFAULT_CONFIG_PATH: Path` (repo `config/rocky.toml`, resolved relative to the package)
  - `load_config(path: Path | str = DEFAULT_CONFIG_PATH) -> RobotConfig`
  - `parse_config(data: dict) -> RobotConfig`, for tests

- [ ] **Step 1:** Write `config/rocky.toml` with exactly these starting values (spec §6.2–6.3, MG996R at 6 V):
  ```toml
  [body]
  n_legs = 5
  mount_radius_mm = 70
  thickness_mm = 40

  [mass]
  total_kg = 2.0
  coxa_link_kg = 0.08    # femur servo + bracket
  femur_link_kg = 0.09   # tibia servo + femur
  tibia_link_kg = 0.04

  [leg]
  coxa_mm = 40
  femur_mm = 70
  tibia_mm = 100

  [limits]
  coxa_deg = [-45, 45]
  femur_deg = [-60, 90]
  tibia_deg = [-150, -20]

  [servo]
  stall_torque_kgcm = 11.0
  max_speed_dps = 400
  torque_rule_kgcm = 5.0

  [gait]
  default = "wave"
  period_s = 1.5
  body_height_mm = 100
  stance_radius_mm = 180
  step_height_mm = 30
  max_stride_mm = 60

  [gait.duty_factor]
  wave = 0.8
  star = 0.6

  [motion]
  loop_hz = 100
  max_linear_accel_mps2 = 0.3
  max_angular_accel_dps2 = 90
  ```
  Conversions: 1 kg·cm = 0.0980665 N·m. dps → rad/s.
- [ ] **Step 2:** Write the failing tests:
  - `test_loads_default_config_in_si`:
    - `cfg.leg.femur == pytest.approx(0.070)`
    - `cfg.limits.tibia == pytest.approx((radians(-150), radians(-20)))`
    - `cfg.servo.torque_rule == pytest.approx(0.490, abs=1e-3)`
    - `cfg.servo.max_speed == pytest.approx(radians(400))`
    - `cfg.gait.duty_factor["star"] == 0.6`
    - `cfg.motion.loop_hz == 100`
  - `test_rejects_bad_values`, parametrised over mutations of the default dict. Each expects `ConfigError` whose message contains the key:
    - `leg.femur_mm = -5` → "femur_mm"
    - `limits.coxa_deg = [45, -45]` → "coxa_deg"
    - `gait.duty_factor.wave = 1.2` → "wave"
    - `body.n_legs = 6` → "n_legs" (only 5 supported)
    - `motion.loop_hz = 30` → "loop_hz" (minimum 50)
    - a missing `[servo]` table → "servo"
- [ ] **Step 3:** Run `uv run pytest tests/test_config.py -v`. Expected: FAIL (module missing).
- [ ] **Step 4:** Implement `rocky/config.py`. Use `tomllib`. Validation rules:
  - all lengths, masses, torques and speeds > 0
  - each limit pair has min < max
  - each duty factor is in (0, 1)
  - `n_legs == 5`
  - `loop_hz >= 50`
  - `gait.default` is a key of `duty_factor`
  - sum of link masses × n_legs < total mass

  Gait-specific duty rules live in Task 5, not here.
- [ ] **Step 5:** Run `uv run pytest tests/test_config.py -v`. Expected: all pass.
- [ ] **Step 6:** Commit: `git commit -am "feat: robot config file and SI loader"` (add new files first).

---

### Task 3: Leg kinematics

**Files:**
- Create: `rocky/kinematics/__init__.py`, `rocky/kinematics/leg.py`
- Test: `tests/kinematics/test_leg.py`

**Interfaces:**
- Consumes: `LegGeometry`.
- Produces:
  - `class UnreachableError(ValueError)`
  - `leg_fk(angles: np.ndarray, geom: LegGeometry) -> np.ndarray`: `(3,)` angles to `(3,)` foot position in the leg frame
  - `leg_ik(foot: np.ndarray, geom: LegGeometry) -> np.ndarray`: `(3,)` to `(3,)`; raises `UnreachableError`

- [ ] **Step 1:** Write the failing tests (geometry 0.04 / 0.07 / 0.10):
  - `test_fk_straight_leg`: `leg_fk([0,0,0])` ≈ `(0.21, 0, 0)`
  - `test_fk_standing`: `leg_fk([0,0,-π/2])` ≈ `(0.11, 0, -0.10)`
  - `test_fk_coxa_yaw`: `leg_fk([π/2,0,-π/2])` ≈ `(0, 0.11, -0.10)`
  - `test_ik_fk_round_trip`: a grid of 9 × 9 × 9 angles inside the default limits; `leg_fk(leg_ik(leg_fk(a)))` equals `leg_fk(a)` to 1e-9, **and** `leg_ik(leg_fk(a))` equals `a` to 1e-9 (knee-up branch is unique within the limits)
  - `test_ik_unreachable_raises`: `leg_ik([0.5, 0, 0])` raises `UnreachableError`
- [ ] **Step 2:** Run `uv run pytest tests/kinematics/test_leg.py -v`. Expected: FAIL (import error).
- [ ] **Step 3:** Implement. Formulas (fixes the convention; the knee-up branch is `θ3 ≤ 0`):
  ```
  FK:  r = Lc + Lf·cos θ2 + Lt·cos(θ2+θ3);  z = Lf·sin θ2 + Lt·sin(θ2+θ3);  x = r·cos θ1;  y = r·sin θ1
  IK:  θ1 = atan2(y, x);  r = hypot(x, y) − Lc;  c3 = (r² + z² − Lf² − Lt²) / (2·Lf·Lt)
       |c3| > 1 → UnreachableError;  θ3 = −acos(c3)
       θ2 = atan2(z, r) − atan2(Lt·sin θ3, Lf + Lt·cos θ3)
  ```
- [ ] **Step 4:** Run the tests. Expected: all pass.
- [ ] **Step 5:** Commit: `feat: analytic 3-DOF leg kinematics`.

---

### Task 4: Body geometry

**Files:**
- Create: `rocky/kinematics/body.py`
- Test: `tests/kinematics/test_body.py`

**Interfaces:**
- Consumes: `RobotConfig`, `leg_ik`.
- Produces:
  - `LegMount(index: int, position: np.ndarray, yaw: float)`, frozen dataclass
  - `leg_mounts(body: BodyParams) -> list[LegMount]`: leg i at yaw `2πi/5`, position `R·(cos ψ, sin ψ, 0)`
  - `body_to_leg(p: np.ndarray, mount: LegMount) -> np.ndarray`
  - `leg_to_body(p: np.ndarray, mount: LegMount) -> np.ndarray`
  - `neutral_feet(cfg: RobotConfig) -> np.ndarray`: shape `(5, 3)`, row i = `(S cos ψᵢ, S sin ψᵢ, −H)`, with S = `gait.stance_radius` and H = `gait.body_height`

- [ ] **Step 1:** Write the failing tests:
  - `test_five_mounts_at_72_degrees`: yaws `[0, 72, 144, 216, 288]°`, all `‖position‖ == 0.07`
  - `test_frame_round_trip`: random points, `leg_to_body(body_to_leg(p, m), m) ≈ p` for every mount
  - `test_neutral_foot_in_leg_frame`: for every leg, `body_to_leg(neutral_feet(cfg)[i], mount_i) ≈ (0.11, 0, −0.10)`. With the default config this is exactly the standing pose `(0, 0, −90°)`.
  - `test_radial_symmetry`: rotating `neutral_feet` by +72° about z gives the same rows, shifted by one leg
- [ ] **Step 2:** Run. Expected: FAIL.
- [ ] **Step 3:** Implement with a z-rotation matrix helper `rot_z(angle) -> np.ndarray (3,3)` in the same module. Later tasks reuse it.
- [ ] **Step 4:** Run. Expected: all pass.
- [ ] **Step 5:** Commit: `feat: body mounts, frame transforms, neutral stance`.

---

### Task 5: Gait patterns

**Files:**
- Create: `rocky/gait/__init__.py`, `rocky/gait/patterns.py`
- Test: `tests/gait/test_patterns.py`

**Interfaces:**
- Produces:
  - `GaitPattern(name: str, offsets: tuple[float, ...], duty_factor: float)`, a frozen dataclass with:
    - property `swing_fraction = 1 − duty_factor`
    - `local_phase(leg: int, phase: float) -> float`, equal to `(phase − offsets[leg]) % 1`
    - `in_swing(leg: int, phase: float) -> bool`, equal to `local_phase < swing_fraction` (half-open)
  - `make_pattern(name: str, duty_factor: float, n_legs: int = 5) -> GaitPattern`:
    - offsets `2i/5` for both gaits
    - raises `ConfigError` for an unknown name, for `wave` with duty < 0.8, or for `star` with duty < 0.6. Those thresholds are exactly what keeps the invariants.

- [ ] **Step 1:** Write the failing tests:
  - `test_star_offsets`: `(0, .4, .8, .2, .6)`
  - `test_swing_start_order`: legs sorted by offset → `[0, 3, 1, 4, 2]`
  - `test_wave_one_leg_at_a_time`: 10 000 evenly spaced phases. Exactly one leg is in swing at every phase (count == 1).
  - `test_star_invariants`: at 10 000 phases, swing count ≤ 2, stance count ≥ 3, and never `in_swing(i) and in_swing((i+1) % 5)`.
  - `test_rejects_unsafe_duty`: `make_pattern("star", 0.5)` and `make_pattern("wave", 0.7)` raise `ConfigError`.
- [ ] **Step 2:** Run. Expected: FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run. Expected: all pass.
- [ ] **Step 5:** Commit: `feat: wave and star gait patterns with invariants`.

---

### Task 6: Gait generator (foot planner)

**Files:**
- Create: `rocky/commands.py`, `rocky/gait/generator.py`
- Test: `tests/gait/test_generator.py`

**Interfaces:**
- Consumes: `RobotConfig`, `GaitPattern`, `neutral_feet`, `rot_z`.
- Produces:
  - `BodyVelocity(vx: float = 0.0, vy: float = 0.0, wz: float = 0.0)`: frozen; m/s, m/s, rad/s in the body frame; `is_zero(eps: float = 1e-6) -> bool`
  - `GaitGenerator(cfg: RobotConfig, pattern: GaitPattern)` with:
    - `.feet -> np.ndarray` `(5, 3)` (copy)
    - `.phase -> float`
    - `.stance_time -> float` = `duty·period`
    - `.clamp(cmd: BodyVelocity) -> BodyVelocity`
    - `.step(dt: float, cmd: BodyVelocity) -> np.ndarray` (returns the new feet, a copy)

- [ ] **Step 1:** Write the failing tests (default config, wave unless stated; `dt = 0.01`; N = neutral):
  - `test_idle_holds_neutral`: zero command for 200 steps → feet == N exactly, every tick (the phase may keep running; legs skip their swings).
  - `test_stance_feet_move_against_command`: `vx=0.03`. After 3 s, for each leg in stance on both of two consecutive ticks: `Δfoot ≈ (−0.03·dt, 0, 0)` to 1e-9.
  - `test_swing_lifts_to_step_height`: over one full period with `vx=0.03`, every leg's max z ≈ `−H + 0.03` (±1 mm), and min z == `−H`.
  - `test_clamp_limits_stride`: `clamp(BodyVelocity(vx=10))` gives `vx·stance_time == pytest.approx(max_stride)`. With `wz` included, the largest per-leg stride equals `max_stride`, and direction is preserved (the ratio vx/wz is unchanged).
  - `test_feet_stay_within_half_stride`: `vx=10, vy=-3` for 5 s → every tick, every foot satisfies `‖xy − Nxy‖ ≤ max_stride/2 + 1e-3`.
  - `test_continuous_turn_feet_bounded` (Review Focus 5): `wz=10` for 60 s → the same bound holds every tick.
  - `test_stop_mid_step_settles_to_neutral` (Review Focus 3): `vx=0.04` for 2.37 s, then zero for 2 periods → feet ≈ N (2 mm in xy, exact z), and the feet are bit-identical across a further 100 ticks.
  - `test_resume_from_idle_is_continuous`: idle 1 s, then `vx=0.04` for 2 s → no foot moves more than `max(step_height·π·dt/(swing_fraction·period), |v|·dt) + 1e-9` in any tick (no jumps when walking starts mid-window).
- [ ] **Step 2:** Run. Expected: FAIL.
- [ ] **Step 3:** Implement. The algorithm is incremental, so command changes never make feet jump. Swing is a **per-leg decision taken only at the start of that leg's window**. A leg that's already at neutral while the command is zero skips its swing and stays down. A leg that missed its window start (for example walking resumed mid-window) stays in stance until its next window. Extra feet on the ground only adds stability, so the Global Constraints invariants still hold.
  ```
  state: phase = 0; swinging[i] = False; liftoff[i] = N_i; prev_local[i] = pattern.local_phase(i, 0)
  clamp: stride_i = |(v + ω×N_i)|·T_stance (xy). s = min(1, max_stride / max_i stride_i). Return cmd scaled by s.
  step(dt, cmd):
    cmd = clamp(cmd)
    phase = (phase + dt/period) % 1
    for each leg i:
      local = pattern.local_phase(i, phase)
      target_i = rot_z(wz·T_stance/2) @ N_i + (vx, vy, 0)·T_stance/2   (z = −H)
      window_started = local < prev_local[i]                     # wrapped past this leg's offset
      if window_started and not (cmd.is_zero() and |feet_i.xy − N_i.xy| ≤ 2 mm):
          swinging[i] = True; liftoff[i] = feet[i]
      if swinging[i] and local >= swing_fraction:                # window over: touch down on target
          feet[i] = target_i (z = −H); swinging[i] = False
      elif swinging[i]:
          u = local/swing_fraction
          xy = liftoff + (target_i − liftoff)·(3u² − 2u³);  z = −H + step_height·sin(πu)
      else:                                                      # stance
          feet[i] = rot_z(−wz·dt) @ feet[i] − (vx, vy, 0)·dt;  z = −H
      prev_local[i] = local
  ```
- [ ] **Step 4:** Run. Expected: all pass.
- [ ] **Step 5:** Commit: `feat: incremental foot planner with stride clamp and idle hold`.

---

### Task 7: Motion controller and servo backends

**Files:**
- Create: `rocky/motion/__init__.py`, `rocky/motion/controller.py`, `rocky/backends/__init__.py`, `rocky/backends/base.py`
- Test: `tests/motion/test_controller.py`, `tests/backends/test_base.py`

**Interfaces:**
- Consumes: everything above.
- Produces:
  - `MotionStats(ticks: int = 0, unreachable: int = 0, clamped: int = 0)`: mutable dataclass
  - `MotionController(cfg: RobotConfig, gait: str | None = None)`:
    - `gait=None` uses `cfg.gait.default`
    - raises `ConfigError` if any neutral foot is unreachable or its IK angles fall outside the limits
    - `.angles -> np.ndarray` `(5, 3)`, a copy; it starts as the IK of the neutral stance
    - `.command -> BodyVelocity`: the current rate-limited command
    - `.stats -> MotionStats`
    - `.tick(dt: float, target: BodyVelocity) -> np.ndarray` `(5, 3)`
  - `class ServoBackend(Protocol): def write(self, angles: np.ndarray) -> None`
  - `DryRunBackend`, with `.history: list[np.ndarray]` and `write` appending a copy

- [ ] **Step 1:** Write the failing tests (`dt = 0.01`):
  - `test_starts_in_neutral_stance`: angles == `[0, 0, −π/2]` for every leg, to 1e-9.
  - `test_idle_does_not_move`: 100 zero ticks → angles unchanged.
  - `test_command_ramps_at_accel_limit`: target `vx=0.04` from rest. After one tick `command.vx == pytest.approx(0.3·dt)`, and it reaches 0.04 after `ceil(0.04/(0.3·dt))` ticks. Same for `wz` with `radians(90)`.
  - `test_reversal_respects_joint_speed` (Review Focus 2): 3 s at `vx=+0.05`, then 3 s at `vx=−0.05`. Every tick, `max|Δangles| ≤ cfg.servo.max_speed·dt + 1e-12`.
  - `test_extreme_command_stays_reachable` (Review Focus 1): 10 s of `vx=5, vy=−5, wz=5` → `stats.unreachable == 0` and `stats.clamped == 0`. Every tick, all angles are within limits.
  - `test_unreachable_stance_rejected` (Review Focus 4): config with `stance_radius_mm = 400` → `ConfigError` mentioning `stance_radius_mm`.
  - `test_dry_run_records`: `DryRunBackend().write(a)` twice → `len(history) == 2`, stored as copies (mutating `a` doesn't change the history).
- [ ] **Step 2:** Run. Expected: FAIL.
- [ ] **Step 3:** Implement `tick`, in this order:
  1. Rate-limit `command` toward `target`: linear (vx, vy) as a vector, step norm ≤ `max_linear_accel·dt`; `wz` step ≤ `max_angular_accel·dt`.
  2. `feet = generator.step(dt, command)`.
  3. Per leg, `leg_ik(body_to_leg(...))`. On `UnreachableError`, keep that leg's previous angles and increment `unreachable`.
  4. Clip to limits; increment `clamped` per clipped joint.
  5. Clip the per-joint change to `±max_speed·dt`.
  6. Store, increment `ticks`, return a copy.
- [ ] **Step 4:** Run `uv run pytest -v`. Expected: all pass.
- [ ] **Step 5:** Commit: `feat: motion controller with rate, limit and speed guards; dry-run backend`.

---

### Task 8: MuJoCo model generated from config

**Files:**
- Create: `rocky/sim/__init__.py`, `rocky/sim/model.py`
- Test: `tests/sim/test_model.py` (mark the module `pytestmark = pytest.mark.sim`)

**Interfaces:**
- Consumes: `RobotConfig`, `leg_mounts`, `neutral_feet`, `leg_ik`.
- Produces:
  - `joint_names(n_legs: int = 5) -> list[str]`: `["leg0_coxa", "leg0_femur", "leg0_tibia", "leg1_coxa", …]`, row-major, matching `angles.reshape(-1)`
  - `FOOT_RADIUS = 0.010`
  - `build_mjcf(cfg: RobotConfig) -> str`
  - `load_model(cfg: RobotConfig) -> mujoco.MjModel`
  - keyframe `"stand"`: torso at `z = body_height + FOOT_RADIUS`, joints = neutral IK

- [ ] **Step 1:** Write the failing tests:
  - `test_model_compiles`: `nu == 15`, `njnt == 16` (free joint + 15), and actuator names == `joint_names()`.
  - `test_actuator_limits_match_config`: `actuator_forcerange` == `±cfg.servo.stall_torque`; `actuator_ctrlrange` rows == the config limits.
  - `test_stands_when_holding_neutral`: reset to `"stand"`, `ctrl` = neutral angles, 2 s → torso z within 10% of `body_height + FOOT_RADIUS`, tilt < 5°.
- [ ] **Step 2:** Run `uv run pytest tests/sim/test_model.py -v`. Expected: FAIL.
- [ ] **Step 3:** Implement `build_mjcf`. MJCF essentials (fixed by the conventions):
  - `timestep="0.002"`
  - floor plane `friction="1 0.005 0.0001"`
  - torso: free joint, cylinder of radius `mount_radius` and half-height `thickness/2`, mass `total − 5·(coxa+femur+tibia link)`
  - per leg: body at the mount with `euler="0 0 ψ"`
    - coxa hinge `axis="0 0 1"`
    - femur body at `pos="Lc 0 0"`, hinge `axis="0 -1 0"`
    - tibia body at `pos="Lf 0 0"`, hinge `axis="0 -1 0"`
    - links are capsules along +x with the link masses; foot sphere `FOOT_RADIUS` at `pos="Lt 0 0"`
  - only feet, torso and floor collide (`contype`/`conaffinity` 0 on link capsules)
  - `<position>` actuators: `kp=20`, `forcerange ±stall_torque`, `forcelimited="true"`, `ctrlrange` = limits
  - joint `damping="0.1" armature="0.002"`
  - a `trackcom` camera named `"track"` for video
  - The values `kp`, damping and armature are the builder's call if standing fails. Keep them as module constants.
- [ ] **Step 4:** Run. Expected: all pass.
- [ ] **Step 5:** Commit: `feat: MJCF model generated from config`.

---

### Task 9: Sim runner, MuJoCo backend, walking acceptance tests, CLI

**Files:**
- Create: `rocky/backends/mujoco_backend.py`, `rocky/sim/runner.py`, `rocky/sim/__main__.py`
- Test: `tests/sim/test_walking.py` (`pytestmark = pytest.mark.sim`)

**Interfaces:**
- Consumes: `MotionController`, `load_model`, `joint_names`, `BodyVelocity`.
- Produces:
  - `MujocoBackend(model: mujoco.MjModel, data: mujoco.MjData)`: `write(angles)` sets `data.ctrl[:] = angles.reshape(-1)`
  - `SimResult`, a dataclass with:
    - `times: np.ndarray` (T,)
    - `torso_pos` (T, 3)
    - `torso_yaw` (T,), unwrapped, in rad
    - `torso_tilt` (T,), the angle between the torso z axis and world z, in rad
    - `joint_torque` (T, 15), from `data.actuator_force`
    - `sustained_torque_peak(window_s: float = 0.5) -> np.ndarray` (15,): the max over time of the moving average of |τ|
  - `simulate(cfg, commands: Callable[[float], BodyVelocity], duration: float, gait: str | None = None, settle_s: float = 1.0, video_path: Path | None = None) -> SimResult`:
    - reset to `"stand"`
    - zero command during `settle_s`, then `commands(t − settle_s)`
    - controller ticks at `loop_hz` with `round(1/(loop_hz·timestep))` physics substeps per tick
    - record once per tick, after settle only
    - video (30 fps from the `"track"` camera via `mujoco.Renderer` + imageio) only if `video_path` is given; raise a clear error if the `video` extra isn't installed
  - CLI:
    - `python -m rocky.sim walk --gait {wave,star} --vx F --vy F --wz F --duration F [--video PATH] [--viewer]` prints displacement, yaw change, max tilt, min torso height, and a per-joint torque table versus `torque_rule` with a PASS/FAIL line. Exit code 1 on FAIL.
    - `python -m rocky.sim torque --gait G` is shorthand for walking at 80% of the max speed forward for 10 s plus the torque table.
    - `--viewer` uses `mujoco.viewer.launch_passive`. On macOS this must run as `uv run mjpython -m rocky.sim …`; the CLI help says so.
- Max speeds for tests: `v_max = max_stride / stance_time`, `w_max = max_stride / (stance_time · stance_radius)`. Tests use 80% of these, `duration = 10`.

- [ ] **Step 1:** Write the failing tests:
  - `test_walks_any_direction`, parametrised over heading `[0°, 36°, 90°, 197°]` (toward a leg, between legs, sideways, arbitrary):
    - displacement along the heading ≥ 0.7 × commanded distance
    - perpendicular drift ≤ 0.25 × along-heading displacement
    - `|yaw change| < 10°`
    - max tilt < 15°
    - min torso z > 0.5·H
  - `test_turns_in_place`: `wz = 0.8·w_max` → yaw change ≥ 0.7 × commanded, xy displacement < 0.05 m, max tilt < 15°.
  - `test_wave_gait_meets_torque_rule`: forward at `0.8·v_max` → `sustained_torque_peak().max() <= cfg.servo.torque_rule`.
  - `test_star_gait_meets_torque_rule`: the same with `gait="star"` (see Task 10 for what to do if it fails).
  - `test_mujoco_backend_writes_ctrl`: write a `(5, 3)` array → `data.ctrl` equals it flattened.
- [ ] **Step 2:** Run `uv run pytest -m sim -v`. Expected: FAIL (imports).
- [ ] **Step 3:** Implement the backend, the runner and the CLI.
- [ ] **Step 4:** Run `uv run pytest -v` (all tests, sim included). Expected: everything passes **except possibly the torque tests**. Those are what Task 10 tunes. If a walking test fails, debug with `--viewer` before touching the thresholds. The thresholds are acceptance criteria and must not be loosened.
- [ ] **Step 5:** Commit: `feat: MuJoCo runner, walking acceptance tests, sim CLI`.

---

### Task 10: Tune, freeze leg lengths, build log #1 (M1 done)

**Files:**
- Modify: `config/rocky.toml`, `docs/design/2026-11-23-rocky-walker-design.md` (§6.3 values), `tests/sim/test_walking.py` (only to mark the star test, as described below)
- Create: `docs/build-log/YYYY-MM-DD-01-walking-in-sim.md`

- [ ] **Step 1:** Run `uv run python -m rocky.sim torque --gait wave` and `uv run python -m rocky.sim torque --gait star`. Record the per-joint peaks.
- [ ] **Step 2:** If wave fails the rule, tune in the spec's give-way order. Rerun the full `uv run pytest -v` after each change.
  1. stance: `stance_radius_mm` inward and `body_height_mm`
  2. `femur_mm` shorter
  3. overall size down: all lengths and `mount_radius_mm`, and `total_kg` reduced to match the lighter build (document the estimate)

  Never loosen the torque rule or the walking thresholds. If no combination passes on MG996R values, stop. Write that up in the build log as a finding, and ask the owner before considering a stronger servo.
- [ ] **Step 3:** If star still fails after wave passes:
  - mark `test_star_gait_meets_torque_rule` with `@pytest.mark.xfail(strict=True, reason="star gait exceeds MG996R torque rule; see build log 01")`
  - leave `default = "wave"`
  - the star gait stays sim-only, as the spec allows
- [ ] **Step 4:** Record videos: `uv sync --extra video`, then `uv run python -m rocky.sim walk --gait wave --vx 0.03 --duration 10 --video out/walk.mp4`, plus a diagonal walk and a turn in place. Videos aren't committed (`.gitignore`). They're for the build-log post and a GitHub release.
- [ ] **Step 5:** Write the build log entry:
  - what was built
  - the frozen values table: leg lengths, stance, body height, period
  - torque peaks versus the rule for both gaits
  - what changed during tuning and why
  - what failed
  - next: M2, the bench leg
  - embed the GIFs/links

  Update spec §6.3 "Starting values" to "Frozen values (M1)".
- [ ] **Step 6:** Run `uv run ruff check . && uv run ruff format --check . && uv run pytest -v`. Expected: clean, all pass (star may be an xfail).
- [ ] **Step 7:** Commit and tag: `git commit -am "feat: M1 leg lengths frozen from sim torque log" && git tag m1`. Push only if the owner has published the repo (Task 1 Step 11) and agrees.

# M2 Fusion 360 Design + Blender Showcase: Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A complete, posable Rocky in Fusion 360 (built live through the Fusion MCP), with real masses fed back into the sim, then rendered in Blender walking and dancing, animated by the real motion code.

**Architecture:**
- **Fusion:** Python scripts in `cad/fusion/` run inside the owner's open Fusion document through the Fusion MCP `script` operation, via a small HTTP client in `tools/fusion_mcp.py`. Read-only scripts dump facts (parameters, joint origins, bounding boxes, interference, mass) to JSON files in `cad/reports/`. Pytest checks those reports against `config/rocky.toml`.
- **Moves:** a new `rocky.moves` package plays keyframe dance files through the same guards as walking.
- **Showcase:** `rocky.showcase` turns walking and moves into per-frame JSON (body world pose plus 15 joint angles). A headless Blender script builds the scene from Fusion meshes and keyframes it.

**Tech Stack:**
- Fusion 360 API (Python inside Fusion), Fusion MCP over HTTP JSON-RPC
- Blender 5.2 (`/Applications/Blender.app/Contents/MacOS/Blender --background --python`), EEVEE
- Python 3.13, numpy, mujoco, pytest, ruff, uv

**Spec:** `docs/design/2026-11-23-m2-fusion-blender-design.md`, and its parent `docs/design/2026-11-23-rocky-walker-design.md`.

## Global Constraints

- **Leg frame / joint conventions** are as in `rocky/kinematics` and must hold in CAD and Blender:
  - coxa axis vertical (+z), at `mount_radius` from the body centre, leg i at yaw `2πi/5`
  - femur axis parallel to leg-frame y, `coxa` out from the coxa axis
  - knee axis parallel to it, `femur` further out
  - foot **tip (contact point)** `tibia` from the knee
  - zero pose = leg straight out horizontally
  - femur and tibia rotate about **−y** (positive lifts)
- Every printable part fits **180 × 180 × 180 mm**, in PLA. The design never depends on multi-material printing.
- Fusion user parameters use the same names as the TOML keys (`coxa_mm`, `femur_mm`, `tibia_mm`, `mount_radius_mm`, ...), in mm or deg.
- **Physical fit wins over TOML values.** Any revised value is written back to `config/rocky.toml`, then: `uv run pytest` (which includes the walking-envelope check), the torque sweep, and a note for build log #2. Leg lengths change only if that re-check fails (shrink before upgrading).
- Printed mass = **0.6 × solid PLA** (1.24 g/cm³). Bought parts at datasheet mass: MG996R 55 g, Pi 5 46 g, PCA9685 9 g, 623ZZ bearing 2 g, Camera Module 3 4 g, 2S 3000 mAh LiPo 180 g, UBEC 25 g, buck converter 15 g.
- **The `.f3d` is the source of truth.** Scripts in `cad/fusion/` are a record and are not required to rebuild.
- Torque rule: sustained ≤ 0.49 N·m. Dance stability margin ≥ **15 mm**. Animation stance-foot drift < **1 mm**.
- CAD files committed in `cad/`. MP4s → GitHub release (ask the owner first). GIFs → `docs/build-log/media/`.
- **The owner's Fusion document is never closed, discarded or overwritten without asking.** Save to the Fusion cloud only with the owner's project choice.
- The Fusion MCP URL comes from env `FUSION_MCP_URL` (default `http://127.0.0.1:50463/mcp`). The port can change when Fusion restarts.

## Review Focus

1. **Fusion restarted mid-task, so the MCP port changed:** the client fails with a message naming `FUSION_MCP_URL`, not a hang. Test: Task 1 `test_unreachable_server_message`.
2. **The owner hand-edited a parameter in Fusion so it no longer matches the TOML:** the parameter check reports which key differs and by how much. Test: Task 2 `test_params_match_config`, which compares the exported report.
3. **A dance file with a keyframe outside the reachable or stable envelope** (for example a 60 mm body shift): it is rejected at load with the keyframe time and reason. Test: Task 8 `test_rejects_unstable_move`.
4. **A move that starts or ends away from neutral, chained into the routine:** a joint would jump between moves. Moves must start and end at neutral. Test: Task 8 `test_move_must_start_and_end_neutral`.
5. **The walking animation exported at a frame rate that doesn't divide the control rate** (30 fps against 100 Hz): frames are sampled from the controller's ticks without drift. Feet stay planted. Test: Task 9 `test_walk_feet_planted`.

---

## File map

```
tools/__init__.py
tools/fusion_mcp.py          MCP HTTP client + CLI: run a script file, print output
cad/fusion/                  Fusion API scripts (01_params.py ... ), one per build step
cad/fusion/report_*.py       read-only report scripts -> cad/reports/*.json
cad/reports/                 params.json, joints.json, bbox.json, interference.json, mass.json
cad/rocky.f3d  cad/rocky.step  cad/stl/*.stl  cad/meshes/*.stl + manifest.json
rocky/config.py              + COM fields in MassParams
rocky/sim/model.py           segment inertials with COM
rocky/kinematics/mass.py     centre of mass of the whole robot
rocky/moves/{__init__,move,player}.py
moves/*.toml                 sway, bob, twist, tilt-circle, leg-wave, routine
rocky/showcase/{__init__,animation,__main__}.py
showcase/build_scene.py      Blender script (runs inside Blender)
tests/cad/  tests/moves/  tests/showcase/  tests/tools/
```

---

### Task 1: Fusion MCP client

**Files:**
- Create: `tools/__init__.py`, `tools/fusion_mcp.py`
- Test: `tests/tools/test_fusion_mcp.py`

**Interfaces:**
- Produces:
  - `class FusionError(RuntimeError)`
  - `FusionClient(url: str | None = None, timeout: float = 300)`:
    - `url` defaults to env `FUSION_MCP_URL`, otherwise `http://127.0.0.1:50463/mcp`
    - `.run_script(source: str, read_only: bool = False) -> str` returns the printed output and raises `FusionError` on an error result
  - CLI `uv run python -m tools.fusion_mcp run <script.py> [--read-only]` prints the output; exit 1 on error.
  - Pytest marker `fusion` (tests needing a live Fusion): skipped automatically when `FusionClient().ping()` fails. Add `ping() -> bool`.

- [ ] **Step 1:** Failing tests:
  - `test_unreachable_server_message`: `FusionClient("http://127.0.0.1:9/mcp", timeout=2).run_script("...")` raises `FusionError` whose message contains `FUSION_MCP_URL`.
  - `test_parses_sse_and_json` with a stub HTTP server (`http.server` in a thread) that returns:
    - an SSE `data:` line wrapping `{"result":{"content":[{"type":"text","text":"{\"message\":\"hi\\n\",\"success\":true}"}]}}` → `run_script` returns `"hi\n"`
    - `"success": false` → raises `FusionError`
- [ ] **Step 2:** Run `uv run pytest tests/tools -v`. Expected: FAIL (import).
- [ ] **Step 3:** Implement with `urllib.request`:
  - `initialize`, then `notifications/initialized` (carry the `Mcp-Session-Id` header), then `tools/call` `fusion_mcp_execute` with `{"featureType":"script","object":{"script":source,"readOnly":read_only}}`
  - the tool text is JSON `{"message","success"}`
  - register the `fusion` marker in `pyproject.toml`; add a `tests/conftest.py` skip hook
- [ ] **Step 4:** Run the tests. Expected: PASS. Then a live check: `uv run python -m tools.fusion_mcp run /dev/stdin --read-only` with a version-printing script prints the Fusion version.
- [ ] **Step 5:** Commit: `feat: Fusion MCP client`.

---

### Task 2: Parameters + skeleton in Fusion

**Files:**
- Create: `cad/fusion/01_params.py`, `cad/fusion/report_params.py`, `cad/reports/params.json`, `tests/cad/test_params.py`, `tools/fusion_build.py`

**Interfaces:**
- `tools/fusion_build.py`: `render_script(path: Path, cfg_raw: dict) -> str` substitutes `{{key}}` placeholders (dotted TOML keys, for example `{{leg.coxa_mm}}`) in a Fusion script. CLI: `uv run python -m tools.fusion_build <script> [--read-only] [--out report.json]` renders it, runs it through `FusionClient`, and writes the output to `--out` if given.
- Fusion user parameters:
  - from the TOML: `coxa_mm`, `femur_mm`, `tibia_mm`, `mount_radius_mm`, `body_height_mm`, `coxa_min_deg`/`coxa_max_deg`, and likewise for femur and tibia
  - plus servo fit parameters from the MG996R datasheet: `servo_l = 40.7 mm`, `servo_w = 19.7 mm`, `servo_h = 42.9 mm`, `servo_ear_l = 54.6 mm`, `servo_ear_hole_pitch = 49.5 mm`, `servo_ear_hole_gap = 10 mm`, `servo_ear_z = 29 mm` (ear underside from the base), `servo_shaft_offset = 10.2 mm` (shaft centre from the near end), `fit_clearance = 0.3 mm`, `wall = 3 mm`, `m3_clear = 3.4 mm`, `m3_nut_af = 5.6 mm`, `bearing_od = 10 mm`, `bearing_id = 3 mm`, `bearing_w = 4 mm`
- Components: `Body`, `Leg` (`Leg` occurrence at the leg-0 mount, its origin on the coxa axis, +x radially outward). Construction axes named `axis_coxa`, `axis_femur`, `axis_knee` and a point `foot_tip` inside `Leg`.
- `report_params.py` prints JSON: `{"params": {name: value_in_param_units}, "leg_frame": {"axis_femur_x": mm, "axis_knee_x": mm, "foot_tip_x": mm, "axes_z": [mm, mm, mm]}}`.

- [ ] **Step 1:** Failing test `test_params_match_config`, which loads `cad/reports/params.json`:
  - every TOML-mirrored parameter equals the TOML value to 1e-6
  - `axis_femur_x == coxa_mm`, `axis_knee_x == coxa_mm + femur_mm`, `foot_tip_x == coxa + femur + tibia`
  - all `axes_z` equal (same height)
  - on mismatch, the message names the key and both values

  Run it. Expected: FAIL (report missing).
- [ ] **Step 2:** Write `01_params.py`: creates or updates the parameters (idempotent: update the expression if the parameter exists), the components, the construction axes and the foot point. Run it with `uv run python -m tools.fusion_build cad/fusion/01_params.py`. **Before the first run, check the active document is the owner's empty `Untitled`.** If it isn't empty, ask the owner.
- [ ] **Step 3:** Write `report_params.py` and run it read-only with `--out cad/reports/params.json`.
- [ ] **Step 4:** Run `uv run pytest tests/cad -v`. Expected: PASS.
- [ ] **Step 5:** Commit: `feat(cad): Fusion parameters and leg skeleton`.

---

### Task 3: Leg parts and joints

**Files:**
- Create: `cad/fusion/02_leg.py`, `cad/fusion/report_leg.py`, `cad/reports/leg.json`, `tests/cad/test_leg.py`

**Interfaces:**
- Inside `Leg`, sub-components:
  - `Coxa` (printed coxa bracket: holds the femur servo, bolts to the coxa servo horn, idler bearing under the coxa axis)
  - `Femur` (printed: driven by the femur servo spline, idler bearing on the opposite side, carries the knee servo pocket at the knee)
  - `Tibia` (printed: driven by the knee servo, idler at the knee, tapered to a rounded foot tip; the foot tip is the contact point)
  - placeholders `MG996R_coxa`, `MG996R_femur`, `MG996R_knee`, horns and bearings (each a separate body, named with the prefix `BOUGHT_`)
- Revolute joints `coxa`, `femur`, `knee` on the skeleton axes, with Fusion joint limits equal to the config's limits.
- `report_leg.py` prints JSON:
  - `printed`: `{name: {"bbox_mm": [x, y, z] in the best print orientation}}`, using the smallest axis-aligned bbox over the 3 principal orientations
  - `interference_neutral`: a list of interfering body pairs at the neutral stance pose (from `stand_angles(cfg)`) and at zero pose
  - `limit_sweep`: for each joint, any interference at its min and max angle (with the other joints at neutral)
  - `joint_origins_mm` in the leg frame

- [ ] **Step 1:** Failing `tests/cad/test_leg.py`:
  - `test_printable_parts_fit_bed`: every printed bbox ≤ 180 in every dimension
  - `test_no_interference_at_stance`: both interference lists are empty (bought–printed contact at mating faces is excluded by the report, which ignores touching faces and only counts volume > 1 mm³)
  - `test_joint_origins_match_config`: origins equal the config lengths
  - `test_limits_are_physical`: `limit_sweep` is empty
- [ ] **Step 2:** Run it. Expected: FAIL.
- [ ] **Step 3:** Model in `02_leg.py`. Design rules:
  - servo pockets = servo size + `fit_clearance`
  - M3 captive-nut slots for the ear screws
  - horn interface = 4 × M3 on the round aluminium horn's bolt circle (parameter `horn_bolt_circle = 14 mm`)
  - idler on the opposite side of each joint: an M3 shoulder screw through a 623 bearing pressed into the printed part
  - minimum wall `wall`
  - fillets where practical

  Also model a **fit coupon** (a separate component `FitCoupon`, not part of the leg): one servo pocket with an ear-screw captive-nut slot, one 623 bearing seat and one horn bolt pattern, ≤ 60 × 50 × 20 mm. Rebuilders test-print it to tune `fit_clearance`.

  Then run the report.
- [ ] **Step 4:** If `test_limits_are_physical` fails, **revise the limits** to the largest clear range in steps of 5°. Write them into both the Fusion parameters and `config/rocky.toml`, then rerun `uv run pytest` (which runs the walking-envelope check). Record the old → new values in `cad/reports/changes.md`. If the envelope check then fails, follow the give-way order and stop to ask the owner before changing leg lengths.
- [ ] **Step 5:** Run `uv run pytest -v`. Expected: all pass. Commit: `feat(cad): leg parts, joints and fit report`.

---

### Task 4: Body

**Files:**
- Create: `cad/fusion/03_body.py`, `cad/fusion/report_body.py`, `cad/reports/body.json`, `tests/cad/test_body.py`

**Interfaces:**
- `Body` sub-components:
  - `Chassis` (bottom and top plates sandwiching the 5 vertical coxa servos at the mount radius, at 72°; standoffs)
  - `Carapace` (faceted, rock-like shell split into identical printable wedges plus a cap)
  - `Mounts` (Pi 5 on M2.5 standoffs, 58 × 49 mm hole pattern; PCA9685; UBEC; buck converter; LiPo cradle with strap slots; Camera Module 3 mount on the carapace between legs 0 and 1, M2 holes 21 × 12.5 mm)
  - `BOUGHT_` placeholders for each electronics part
- The `Leg` occurrence at leg 0 sits on the coxa servo of slot 0.
- `report_body.py` prints the same shape as `report_leg.py` for the body's printed parts, plus `"chassis_height_mm"`.

- [ ] **Step 1:** Failing `tests/cad/test_body.py`: printed parts fit the bed, no interference, and `abs(chassis coxa-axis radius - mount_radius_mm) < 0.01`.
- [ ] **Step 2:** Run it. Expected: FAIL.
- [ ] **Step 3:** Model, then run the report. If the electronics don't fit at `mount_radius_mm = 70`, increase it in 5 mm steps (Global Constraints: write back, re-check, record in `changes.md`). Also set `body.thickness_mm` in the TOML to the chassis height (it is the sim collision cylinder).
- [ ] **Step 4:** Run `uv run pytest -v`. Expected: PASS. Commit: `feat(cad): body, carapace and electronics mounts`.

---

### Task 5: Assembly, mass report, COM in the sim

**Files:**
- Create: `cad/fusion/04_assembly.py`, `cad/fusion/report_mass.py`, `cad/reports/mass.json`, `tests/cad/test_assembly.py`
- Modify: `rocky/config.py` (`MassParams`), `config/rocky.toml` (`[mass]`), `rocky/sim/model.py`, `tests/test_config.py`, `tests/sim/test_model.py`

**Interfaces:**
- `MassParams` gains `coxa_com`, `femur_com` and `tibia_com` (metres along the segment's +x from its proximal joint axis). TOML keys `coxa_link_com_mm`, `femur_link_com_mm`, `tibia_link_com_mm` (required, ≥ 0 and ≤ the segment length; validated).
- `report_mass.py` prints JSON. Each printed body's mass is `0.6 × volume × 1.24 g/cm³`; each `BOUGHT_` body uses its datasheet mass by name.
  - `"torso_kg"` (body plus everything not on a leg segment)
  - `"segments"`: `{"coxa"|"femur"|"tibia": {"kg": float, "com_mm": [x, y, z] in that segment's joint frame}}`, where the segment includes the servo riding on it (the femur servo on the coxa, the knee servo on the femur)
  - `"total_kg"`
- In `rocky/sim/model.py`, each segment body gets `<inertial pos="com 0 0" mass="m" diaginertia="…">` (a thin rod about the COM: `m·L²/12` on y and z, `m·r²/2` on x with r = 0.01). Geoms become massless.

- [ ] **Step 1:** Failing tests:
  - `test_assembly.py::test_five_legs_no_interference` uses the assembly report at the neutral stance
  - `test_config.py::test_com_fields_loaded`: values in SI, and `femur_link_com_mm = 999` raises `ConfigError` naming the key
  - `test_model.py::test_segment_inertial_uses_com`: `model.body_ipos` of `leg0_femur` equals `(femur_com, 0, 0)`
- [ ] **Step 2:** Run them. Expected: FAIL.
- [ ] **Step 3:** Pattern the `Leg` occurrence 5 times about the body centre with joints (a Fusion circular pattern of occurrences, or 5 occurrences, each jointed to its coxa servo). Run the reports. Then update `[mass]` in the TOML from `mass.json`: `total_kg`, each link's mass and COM x (the y/z offsets are ignored and recorded in `changes.md` if > 5 mm). Implement the config fields and the MJCF inertials.
- [ ] **Step 4:** Run `uv run pytest -v`, then `uv run python -m rocky.sim torque --gait wave` and `--gait star`. Record the results in `cad/reports/changes.md`. If the rule fails, follow the give-way order and **stop and ask the owner** before changing leg lengths.
- [ ] **Step 5:** Commit: `feat(cad): full assembly, mass report, COM-accurate sim`.

---

### Task 6: Exports

**Files:**
- Create: `cad/fusion/05_export.py`, `cad/rocky.f3d`, `cad/rocky.step`, `cad/stl/*.stl`, `cad/meshes/*.stl`, `cad/meshes/manifest.json`, `tests/cad/test_exports.py`

**Interfaces:**
- `cad/stl/<Part>.stl`: one per printed part (including `FitCoupon.stl`), in its print orientation (largest flat face down), in mm.
- `cad/meshes/manifest.json`: `{"units": "mm", "parts": [{"file": "femur__Femur.stl", "segment": "torso"|"coxa"|"femur"|"tibia", "color": "blue"|"white"|"black"|"grey"}]}`.
  - Every body of every segment, printed and bought, is exported at the **zero pose**, in **its segment's joint frame**: origin on the segment's proximal joint axis at joint height, +x along the leg, z up. The torso frame is the body frame.
  - Colour defaults *(builder's call)*: chassis black, carapace white, femur/tibia blue, coxa black, servos grey.
- The `.f3d` export uses `Design.exportManager.createFusionArchiveExportOptions`; STEP uses `createSTEPExportOptions`.

- [ ] **Step 1:** Failing `test_exports.py`:
  - every printed part from the reports has an STL in `cad/stl/` whose bbox (parsed with numpy from binary STL) ≤ 180 on every axis
  - the manifest lists ≥ 1 mesh per segment, and every file exists
  - the tibia meshes' max x ≈ `tibia_mm` (±0.5), so the foot tip is at the segment length
  - `cad/rocky.step` and `cad/rocky.f3d` exist and are non-empty
- [ ] **Step 2:** Run. Expected: FAIL.
- [ ] **Step 3:** Write and run `05_export.py`, passing the repo's absolute `cad/` path in as a template value. Fusion writes the files directly.
- [ ] **Step 4:** Run the tests. Expected: PASS. Commit (with the binaries): `feat(cad): f3d, STEP, printable STLs and segment meshes`.

---

### Task 7: Centre of mass helper

**Files:**
- Create: `rocky/kinematics/pose.py`, `rocky/kinematics/mass.py`, `tests/kinematics/test_mass.py`

**Interfaces:**
- `segment_coms(cfg, angles: np.ndarray) -> np.ndarray`: shape `(5, 3, 3)`, the body-frame COM of each leg's coxa, femur and tibia segment (via partial FK along the chain)
- `center_of_mass(cfg, body_pose: BodyPose, angles: np.ndarray) -> np.ndarray`: the world-frame robot COM, where the torso COM is the body origin. `BodyPose` comes from Task 8, so define `BodyPose` here in `rocky/kinematics/pose.py` instead:
  - `BodyPose(x=0, y=0, z=0, roll=0, pitch=0, yaw=0)`, frozen; metres and radians
  - `.matrix() -> (4, 4)` (ZYX: yaw, pitch, roll)
  - this is an offset from the neutral body pose (body origin at `(0, 0, body_height)` in the world)
- `support_margin(points_xy: np.ndarray, com_xy: np.ndarray) -> float`: the signed distance from the COM to the convex-hull edge of the planted feet (positive = inside). Use the monotone-chain hull; for fewer than 3 points, return −inf.

- [ ] **Step 1:** Failing tests:
  - `test_com_neutral_is_centered`: at neutral, the COM xy ≈ (0, 0) (radial symmetry) to 1e-9
  - `test_com_moves_with_body_shift`: a body shift of x = 20 mm moves the COM x positive, by less than 20 mm (the feet stay put)
  - `test_support_margin_square`: a unit square with the COM at the centre gives 0.5; outside gives a negative value
  - `test_pose_matrix_round_trip`
- [ ] **Step 2:** Run. Expected: FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run. Expected: PASS. Commit: `feat: body pose, centre of mass and support margin`.

---

### Task 8: Dance moves

**Files:**
- Create: `rocky/moves/__init__.py`, `rocky/moves/move.py`, `rocky/moves/player.py`, `moves/{sway,bob,twist,tilt-circle,leg-wave,routine}.toml`, `tests/moves/test_moves.py`

**Interfaces:**
- `Keyframe(t: float, pose: BodyPose, feet: dict[int, np.ndarray])`: `feet` maps a leg index to a foot **offset** in metres (world frame) from its neutral position. A leg with offset z > 0.001 counts as lifted.
- `Move(name: str, keyframes: tuple[Keyframe, ...])`, with `.duration`.
- `load_move(path: Path) -> Move`. TOML format:
  ```toml
  name = "sway"
  [[key]]
  t = 0.0                     # seconds, strictly increasing, first = 0
  x_mm = 0  y_mm = 0  z_mm = 0  roll_deg = 0  pitch_deg = 0  yaw_deg = 0   # all optional, default 0
  # optional: [[key.foot]] leg = 1  dx_mm = 0  dy_mm = 0  dz_mm = 40
  ```
  For a routine: `name = "routine"` with `sequence = ["sway", "bob", ...]` resolved relative to the file. `load_move` concatenates the moves, shifting their times.
- `MovePlayer(cfg)`:
  - `.sample(move, t) -> tuple[BodyPose, dict[int, np.ndarray]]`: smoothstep easing between keyframes (each pose field and foot offset eased independently); held after the end
  - `.angles(pose, feet) -> np.ndarray` (5, 3): world feet = neutral world feet + offsets, transformed into the body frame by the inverse pose, then IK. Raises `UnreachableError`.
  - `.validate(move, dt=0.01) -> None`: raises `ConfigError("<move> at t=<t>s: <reason>")` if:
    - the first or last keyframe isn't neutral (zero pose, no offsets)
    - any sample is unreachable or outside the joint limits
    - any frame-to-frame joint change exceeds `max_speed·dt`
    - the support margin is < 0.015 m (planted feet = legs not lifted)
  - `.play(move, dt) -> Iterator[tuple[float, BodyPose, np.ndarray]]` yields `(t, pose, angles)`. The guards (limits, speed) are applied and counted in `.stats` (`MotionStats`).
- Moves (amplitudes are the builder's call, but each must pass `validate`):
  - `sway`: body x/y circle of 20 mm
  - `bob`: z ± 15 mm × 4
  - `twist`: yaw ± 15° × 3
  - `tilt-circle`: roll/pitch 8° circle
  - `leg-wave`: lift leg 0 by 40 mm and wave ±25 mm in y, body shifted away from leg 0 first so the COM stays inside the polygon of the other 4 feet
  - `routine`: all of them in sequence

- [ ] **Step 1:** Failing tests:
  - `test_all_moves_validate` (parametrised over the files in `moves/`)
  - `test_rejects_unstable_move` (Review Focus 3): an in-memory move with a 60 mm x shift while leg 0 is lifted → `ConfigError` matching `"t="` and `"support"`
  - `test_move_must_start_and_end_neutral` (Review Focus 4)
  - `test_play_needs_no_guards`: playing the routine gives `stats.clamped == stats.speed_limited == stats.unreachable == 0`
  - `test_sample_eases_between_keys`: at the midpoint the value is the midpoint, and at 25% it is `smoothstep(0.25)` of the way there
  - `test_routine_concatenates`: duration = the sum of the moves' durations
- [ ] **Step 2:** Run. Expected: FAIL.
- [ ] **Step 3:** Implement and write the move files.
- [ ] **Step 4:** Add `tests/sim/test_moves_torque.py` (marker `sim`): play the routine in MuJoCo by writing `play()` angles to `MujocoBackend` at `loop_hz`. The torso is free, so the body pose comes from physics. Assert `sustained_torque_peak().max() <= torque_rule` and max tilt < 20°. Add `simulate_angles(cfg, angle_stream, duration)` to `rocky/sim/runner.py` (sharing the recording code with `simulate`).
- [ ] **Step 5:** Run `uv run pytest -v`. Expected: PASS. Commit: `feat: dance moves with stability and torque validation`.

---

### Task 9: Animation export

**Files:**
- Create: `rocky/showcase/__init__.py`, `rocky/showcase/animation.py`, `rocky/showcase/__main__.py`, `tests/showcase/test_animation.py`

**Interfaces:**
- `Frame(t: float, pose_world: np.ndarray (4, 4), angles: np.ndarray (5, 3))`
- `walk_frames(cfg, commands: Callable[[float], BodyVelocity], duration: float, fps: int = 30, gait: str | None = None, settle_s: float = 1.0) -> list[Frame]`:
  - ticks a `MotionController` at `loop_hz`
  - integrates the body world pose from **`controller.command`** (the actual clamped and ramped command): yaw += wz·dt, xy += R(yaw)·(vx, vy)·dt, with z = body_height
  - emits a frame each time `t ≥ k/fps`, using the tick nearest that time (no interpolation, no drift)
- `move_frames(cfg, move, fps=30) -> list[Frame]`: pose_world = translate(0, 0, body_height) @ pose.matrix()
- `feet_world(cfg, frame) -> np.ndarray (5, 3)`
- `max_stance_drift(cfg, frames) -> float`: the max world xy displacement between consecutive frames of any foot with z < 0.5 mm in both frames
- `write_animation(frames, path)`: JSON `{"fps", "joint_names", "frames": [{"t", "pose": 16 floats row-major, "angles": 15 floats}]}`
- CLI: `uv run python -m rocky.showcase export <clip> <out.json>`, where clip ∈ `walk-forward` (vx = 0.8·v_max, 8 s), `walk-diagonal` (45°, 8 s), `turn` (0.8·w_max, 8 s), `dance` (`moves/routine.toml`).

- [ ] **Step 1:** Failing tests:
  - `test_walk_feet_planted` (Review Focus 5): `walk-forward` at 30 fps → `max_stance_drift < 0.001`; the same for `turn`
  - `test_frame_times_exact`: frame k has `t == round(k/fps, 9)` within one control tick
  - `test_dance_feet_planted`: same drift check for planted legs during the routine
  - `test_json_round_trip`
- [ ] **Step 2:** Run. Expected: FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Run. Expected: PASS. Commit: `feat: animation export from motion code`.

---

### Task 10: Blender scene and renders

**Files:**
- Create: `showcase/build_scene.py`, `showcase/README.md`, `tests/showcase/test_blender.py` (marker `blender`, skipped if the Blender binary is missing)

**Interfaces:**
- Run as: `/Applications/Blender.app/Contents/MacOS/Blender --background --factory-startup --python showcase/build_scene.py -- --meshes cad/meshes --config config/rocky.toml --anim out/anim/<clip>.json --out out/render/<clip>.mp4 [--still out/render/hero.png] [--frames N]`
- Inside Blender (no repo imports, because Blender has its own Python; read the TOML with `tomllib`):
  - mm → m
  - an object hierarchy of empties: `torso` → `leg{i}_coxa` (at the mount, yaw ψᵢ) → `leg{i}_femur` (at `(coxa, 0, 0)`) → `leg{i}_tibia` (at `(femur, 0, 0)`)
  - joint rotation: coxa about local Z by θ1; femur and tibia about local Y by **−θ2 / −θ3**, because the code's axes are −y
  - meshes from the manifest are imported once each and linked to all 5 legs (shared mesh data)
  - materials by colour; floor plane with a subtle grid; 3-point light; camera tracking the torso
  - keyframes: the torso's world matrix and the 15 joint rotations, every frame, with linear interpolation
  - render: EEVEE, 1280 × 720, 30 fps, FFmpeg H.264
- `test_blender.py::test_renders_two_frames`: running with `--frames 2` on a short dance export produces an MP4 > 10 kB, and the exit code is 0.

- [ ] **Step 1:** Write the failing test. Run it. Expected: FAIL (script missing).
- [ ] **Step 2:** Implement `build_scene.py`. Run the test. Expected: PASS.
- [ ] **Step 3:** Export all 4 clips (Task 9 CLI) and render them, plus a hero still. Extract a 3-frame strip from each render and **look at it** (Read the PNG) to check: legs attached at the right places, feet touching the floor, correct colours, legs bending in the right direction. Fix and re-render as needed.
- [ ] **Step 4:** Make GIFs (≈ 640 px wide, 15 fps, ≤ 1 MB each) into `docs/build-log/media/02-*.gif`, plus `02-hero.png`.
- [ ] **Step 5:** Commit: `feat: Blender showcase scene and renders`.

---

### Task 11: Build log #2, README, release

**Files:**
- Create: `docs/build-log/2026-MM-DD-02-fusion-and-blender.md`
- Modify: `README.md`, `docs/bom.md` (bearing count, M2.5 standoffs, strap), the parent spec (§6.2 mass table → measured)

- [ ] **Step 1:** Build log #2:
  - Fusion screenshots: export a viewport capture via a Fusion script (`app.activeViewport.saveAsImageFile`) into `docs/build-log/media/02-fusion-*.png`
  - Blender GIFs
  - the `changes.md` content (what the CAD forced to change and why)
  - measured mass versus the M1 estimate; the torque results
  - next: the bench leg
- [ ] **Step 2:** README: status M2 ✅, hero image, a `cad/` section on how to print (STL list, settings), and a `showcase/` section on how to render.
- [ ] **Step 3:** Run `uv run ruff check . && uv run ruff format --check . && uv run pytest -v`. Expected: all pass.
- [ ] **Step 4:** Commit: `docs: build log #2`, and tag `m2`. **Ask the owner** before pushing, and before creating a GitHub release `m2` with the MP4s attached.

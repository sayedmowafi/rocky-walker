# Rocky Walker: Design

- **Status:** reviewed, awaiting owner approval
- **Date:** 2026-11-23
- **Scope:** the whole robot (mechanics, electronics, software, milestones). Later features (follow mode, voice) are designed only as far as needed so the core doesn't block them.

---

## 1. Intent

A five-legged walking robot inspired by Rocky from *Project Hail Mary*. It is built as a hobby project and **open-sourced from day one**, with progress updates published on GitHub as the work happens.

The heart of the project is **locomotion**: making a radially symmetric, five-legged body walk smoothly and convincingly. Everything else (phone control, dances, camera follow mode, voice) is built on top of a solid walking core.

There are two reasons for the project:
1. **Showcase.** The owner's earlier projects were private. This one documents findings in public, including what failed.
2. **Rebuildability.** Rocky should be cheap and easy enough that other people can actually build one from the repo.

**Owner background.** The repo can assume a builder at this level. The owner is a computer engineering student who is strong in Python, C++ and TypeScript. They have used a Pi 5 with a camera and YOLO (NCNN), ESP32/Arduino over UART, servos, PID, and PLA-printed chassis. They are new to legged kinematics, gait design and code CAD.

## 2. Goals

1. **Walk smoothly on five legs, in any direction.** That means translating in any direction, turning in place, and combining the two. Rocky has no fixed "front". Heading is a software concept.
2. **Cheap and rebuildable:**
   - Common, globally available parts.
   - Parts cost, not counting the Pi, camera, audio and battery charger, of **about $200 or less** (§7).
   - Every printed part fits a **180 × 180 × 180 mm** bed, in PLA.
   - No special tools beyond a soldering iron, screwdrivers and a printer.
   - **Minimum supported controller: Raspberry Pi 5, 4 GB** *(Claude's call)*. Development happens on the owner's 16 GB board, so each process logs its memory use, and all processes running together (motion, vision, web with camera stream, voice) must stay under about 3 GB. That way the 4 GB claim can be checked without owning a 4 GB board.
3. **Editable, not just reprintable.** The CAD is a parametric Fusion 360 design whose key dimensions live in Fusion user parameters that mirror `config/rocky.toml`. The `.f3d`, STEP and STL exports are all published.
4. **Phone remote control:** a joystick for walking, buttons for poses and dance moves, mode switching and an e-stop. No app install is needed. It runs in the phone browser.
5. **Camera follow mode:** select a target and Rocky follows it. This is built as an extensible vision platform, so more vision behaviours can be added later.
6. **Voice:** a microphone and speaker so Rocky can "speak". Chord-like tones are the intended nod to the book.
7. **Development in public:** a build log with regular entries, plus assembly docs good enough for a stranger to follow.

## 3. Non-goals (v1)

- ROS 2 (can be wrapped around the Python modules later).
- SLAM, mapping or autonomous navigation beyond follow mode.
- Rough-terrain walking, stairs or dynamic gaits (trotting or running).
- Servo position feedback or force sensing (MG996Rs have neither).
- Custom PCBs. Perfboard or off-the-shelf boards are fine for v1.
- Battery-life optimisation.
- Being an exact replica of the book's or film's Rocky. It is *inspired by* the character.

## 4. Anti-goals (failure even if it "works")

- **It's a hexapod with a leg missing.** It has a front, it walks forward-only, or it's built around a front-facing layout. Radial symmetry is the identity of the project.
- **It needs expensive or hard-to-find parts:** a 16 GB Pi, smart servos, a big-bed printer, or custom-machined parts.
- **The only published files are STLs**, so nobody can edit the design.
- **Twitchy, jittery motion is accepted as "done".** Smoothness is a requirement. "Smooth" means **no visible twitching or jerking while moving or changing pose**. A slight audible buzz or a barely visible hold-wobble while standing still is accepted, because MG996R deadband makes that unavoidable at this price.
- **Unsafe power:** no fuse, no way to cut servo power quickly, or LiPo discharge left unmonitored.
- **The repo uses art, text or assets from the book or film.** The design is original and only inspired by the character.

## 5. Approaches considered

### 5.1 Servo control architecture (owner picked **A**)

| | Approach | Trade-off |
|---|---|---|
| **A** ✅ | **Pi 5 + PCA9685** (16-channel I²C PWM board). Everything runs in Python on the Pi. | Cheapest, fewest parts, one codebase, easiest to rebuild. The PCA9685 generates the pulses in hardware, so Linux scheduling jitter doesn't reach the servos. Risk: a loaded Pi (YOLO) could delay motion updates. Mitigated by a dedicated, prioritised motion process and a clean boundary (§8.2). |
| B | Pi 5 as "brain" + ESP32 as "spinal cord" running the gait loop and servos, with high-level commands over serial. | Deterministic timing and a possible camera-less ESP32-only build. Cost: two codebases, a protocol to design, and debugging across a boundary. |
| — | ROS 2 | Dropped: heavy setup for rebuilders and slows early progress. |

**Why A:** it is the cheapest and simplest to rebuild, and the PCA9685 already removes the main reason for a microcontroller (pulse jitter). The owner has many spare ESP32s. If motion timing on the Pi ever proves insufficient, the motion process can move to an ESP32 behind the same command interface. That is the fallback, not the plan.

### 5.2 CAD tool (owner picked **Fusion 360**, revised 2026-11-23)

| | Approach | Trade-off |
|---|---|---|
| build123d | Code CAD in Python, parts in the repo, driven by the shared config. | Parametric across five identical legs and interdependent dimensions. Readable git diffs. Rebuilders can change servo or leg sizes and regenerate everything. Claude can author and modify parts directly. Cost: a learning curve (the owner has used Tinkercad only), and organic shapes are harder. |
| **Fusion 360** ✅ | Visual parametric CAD with STEP and STL exports, driven through the Fusion MCP connection. | Precise, visual, and the owner's preferred central design. Claude builds it by script through the Fusion MCP, and the owner can see and edit it live. The source files are proprietary, so STEP is published alongside the `.f3d` for rebuilders using other CAD tools. |

**Decision (revised):** the owner first picked build123d on Claude's recommendation, then reversed it before any CAD existed. The central design is **Fusion 360**, with key dimensions as Fusion user parameters mirroring `config/rocky.toml`. **Blender** is used to showcase the assembled robot walking and dancing, animated by the real motion code (see `docs/design/2026-11-23-m2-fusion-blender-design.md`). The faceted, rock-like carapace style still stands.

### 5.3 Servo choice (owner's choice)

**MG996R or similar.** These are cheap (about $3–5 each in multipacks), available everywhere and strong enough for a robot of about 2 kg. Mounts are designed around the **standard servo footprint** (about 40 × 20 mm body, standard ear spacing), so a stronger drop-in servo of the same size (for example the DS3218, about $12) works without reprinting. That gives a cheap default and a better optional upgrade.

## 6. Mechanical design

### 6.1 Layout

- **Five legs** at **72°** intervals around a roughly pentagonal body. The body is the same from every leg's point of view.
- **3 DOF per leg (provisional):**
  - coxa: hip yaw, swings the leg around the body
  - femur: hip pitch, lifts the leg
  - tibia: knee pitch
- That is **15 servos**, which leaves 1 spare PCA9685 channel (reserved; a camera pan servo is a candidate).
- The servo count and DOF are **not final**. The owner explicitly asked not to lock them in. Simulation (§9, M1) is where the leg design gets confirmed. Anything other than 15 servos needs a second PCA9685 (about $4), which is acceptable.

### 6.2 Size and mass budget

| Item | Mass (approx.) |
|---|---|
| 15 × MG996R | 0.83 kg |
| Pi 5 + camera + cooling | 0.10 kg |
| 2S LiPo (about 3000–5000 mAh) | 0.20–0.30 kg |
| Regulators, PCA9685, wiring | 0.15 kg |
| Printed parts, bearings, screws | 0.5–0.7 kg |
| **Total** | **about 1.8–2.1 kg** |

**Measured from the M2 Fusion design:** 1.694 kg total (printed parts at 0.6 × solid PLA, a 98 g 2S 2200 mAh shorty pack, and a 60 g allowance for screws and wires). See `cad/reports/mass.json` and `cad/reports/changes.md`.

That puts the robot at about **35–45 cm across** with the legs in a standing pose.

### 6.3 Leg lengths and the torque rule

**Frozen values (M1).** See [build log #1](../build-log/2026-11-23-01-walking-in-sim.md). Leg lengths are unchanged from the starting values. Stance radius moved from 180 to **165 mm** to give torque margin. The default wave gait stays within the rule up to 2.4 kg in simulation. Star passes at the 2.0 kg design mass, but starting it from a standstill exceeds the rule from 2.2 kg.

| Segment | Length |
|---|---|
| coxa | about 40 mm |
| femur | about 70 mm |
| tibia | about 100 mm |

The hard constraint is the **torque rule:** during the default gait in simulation, **sustained joint torque must stay at or below about 0.5 N·m (about 5 kg·cm)** on every joint. That is roughly half of the MG996R's stall torque at 6 V, which leaves margin for clone variance and dynamics. Leg lengths, stance height and gait duty factor are tuned until this holds.

Rough check behind it: with 3 feet down, each carries about 6.5 N. At about 90 mm horizontal lever arm on the femur joint, that is about 0.6 N·m, which is over budget. With 4 feet down it is about 0.45 N·m. So:
- shorter femurs and feet placed closer to the hips reduce femur torque, but a narrower stance shrinks the support polygon, so stance width is a torque-versus-stability trade-off tuned in sim;
- gaits that keep 4 feet down are the safe default, and 3-feet-down gaits are the "fast mode" that must also pass.

**When constraints collide (owner's decision): shrink before upgrading.** If simulation or the bench leg shows MG996Rs can't meet the torque rule at the target size, give way in this order:
1. Use the 4-feet-down wave gait only (drop the faster gait), and lower or re-shape the stance.
2. Make Rocky smaller: shorter legs, a smaller body, lighter printed parts. The 35–45 cm size is a target, not a requirement.
3. Only as a last resort, make a stronger servo (for example the DS3218) the reference.

The budget and cheap-servo rebuildability come before size and speed. M1 is allowed to change size, leg lengths and gait to make this work. Not finding a combination that works on MG996Rs counts as a design failure to report in the build log, not a tuning detail.

### 6.4 Joint construction

- **Every joint is supported on both sides.** The servo spline drives one side, and an idler pivot on a cheap bearing (for example 623ZZ or similar, builder's call) supports the opposite side. MG996R output shafts handle side loads poorly, and unsupported joints are the classic failure point of cheap hexapods.
- Fasteners are **M3** screws with **captive nuts** (no heat-set inserts, because they need an extra tool and cost more). The only other fasteners allowed are the servo horn screws.
- Use metal (aluminium) servo horns. The plastic horns that come with MG996R clones strip.

### 6.5 Printing constraints

- Every part fits within 180 × 180 × 180 mm.
- PLA, 0.4 mm nozzle, 0.2 mm layers as the reference profile.
- Parts are oriented and designed to minimise supports. Part files note their print orientation and infill. Structural parts use at least 4 walls and about 30–40% infill (builder's call).
- The carapace is printed in segments that bolt together (for example five identical wedges plus a centre piece, builder's call).
- Colours: the owner has blue, white and black PLA in stock. The colour scheme is the owner's call, and the design must not depend on multi-material printing.

## 7. Electronics and power

### 7.1 Block diagram

```
2S LiPo (7.4 V) ── fuse ── main switch ──┬── servo UBEC 6 V (≥ 15 A total) ── servo power bus ── 15 × servo V+
                                         │                                         (NOT through PCA9685's terminal)
                                         ├── 5.1 V / 5 A buck ── Pi 5 (GPIO 5 V pins or USB-C)
                                         └── INA219/INA226 (battery voltage + current, I²C)

Pi 5 ── I²C ── PCA9685 ── signal lines ── 15 × servo signal
Pi 5 ── CSI ── camera
Pi 5 ── I²S ── MAX98357A amp ── speaker      (Pi 5 has no analogue audio jack)
Pi 5 ── USB or I²S ── microphone (USB mic or INMP441)
All grounds common.
```

### 7.2 Power rules

- **Servo current is the main power problem.** MG996R stall current is about 2.5 A each. Walking averages a few amps, with peaks of 10–15 A or more. Servo power goes through a dedicated bus (thick wire, a distribution strip). The PCA9685 board's V+ terminal and traces can't carry it.
- A 15–20 A rated UBEC (or two 10 A UBECs, each powering part of the legs) at **6 V**. The exact part is the builder's call, but it must be from common hobby stock.
- **The Pi needs its own 5.1 V regulator rated for 5 A.** If it can't supply 5 A, the Pi 5 limits USB current and warns about undervoltage.
- **Servo power must be switchable separately from Pi power,** with a physical switch and ideally a MOSFET or relay that the Pi controls. This is for safety (an instant e-stop) and for the startup problem in §8.6.
- **Battery monitoring:** the INA219/INA226 reads pack voltage and current. Software warns at low voltage and sits Rocky down and cuts servo power before the LiPo is over-discharged. The thresholds are the builder's call within normal LiPo safety practice.
- Fuse on the battery lead. XT60 connectors.

### 7.3 Audio

- Speaker via a **MAX98357A I²S amplifier** (about $4) and a small 3 W speaker. The Pi 5 has no 3.5 mm jack.
- Microphone: a USB mic (simplest) or an INMP441 I²S MEMS mic. This is the builder's call when voice work starts (M7).

### 7.4 Camera

- **Reference camera: Raspberry Pi Camera Module 3 (standard lens, IMX708, autofocus)** on CSI. The owner has one.
  - Field of view is about 66° horizontal × 41° vertical. The Wide variant (about 102° horizontal) uses the same board and would make follow mode more forgiving, so the mount must fit both.
  - The board is about 25 × 24 mm, with M2 mounting holes on the same pattern as Camera Module 2. Check against the official mechanical drawing before finalising the mount.
  - Uses the Pi 5's smaller 22-pin camera connector, so it needs the matching Pi 5 camera cable (listed in the parts list).
- **A USB webcam is supported as a fallback** for rebuilders who don't have a Pi camera. The vision code reads frames through a camera source interface with two implementations: Picamera2 (CSI) and OpenCV/V4L2 (USB). The CAD only provides a mount for the Camera Module 3. Webcam mounting is up to the rebuilder.
- v1 uses a **fixed mount** on the carapace top, pointing between two legs. That direction becomes the default "facing" for follow mode. Because Rocky walks in any direction, it can follow while keeping the camera on the target. A pan servo on the spare channel is an optional later upgrade.

### 7.5 Rough parts list and cost (USD, typical online multipack prices; verify when buying)

| Part | Qty | Est. |
|---|---|---|
| MG996R (or similar) | 15 + 2 spares | $55–75 |
| Aluminium servo horns | 15 | $10–15 |
| PCA9685 board | 1 | $3–6 |
| 2S LiPo 3000–5000 mAh, XT60 | 1 | $20–35 |
| Servo UBEC 6 V 15–20 A (or 2 × 10 A) | 1–2 | $10–20 |
| 5.1 V 5 A buck converter | 1 | $5–10 |
| INA219/INA226 | 1 | $2–4 |
| Fuse holder, switch(es), XT60s, wire, servo extensions | — | $10–15 |
| Bearings for idler pivots | about 15 | $5–8 |
| M3 screw and nut assortment | 1 | $8 |
| PLA | about 1–1.5 kg | $15–25 |
| Pi 5 camera cable (22-pin to 15-pin, about 200–300 mm) | 1 | $2–4 |
| **Subtotal** | | **about $145–220** |
| *Not counted:* Pi, camera, audio parts (about $10), LiPo balance charger | | |

The target is about $200 or less. The final parts list lives in `docs/bom.md` and is kept up to date with real links and prices as parts are bought.

## 8. Software design

### 8.1 Principles

- **One language: Python** (3.11+), on current 64-bit Raspberry Pi OS. Development happens on the owner's Mac against the simulator.
- **One source of truth.** `config/rocky.toml` (format is the builder's call) holds the geometry (leg mount radius and angles, segment lengths), servo specs, joint limits and per-servo calibration. **The CAD, the MuJoCo model and the kinematics all read it.** Changing the femur length changes the printed part, the sim and the IK together.
- **Same motion code in sim and on hardware.** Only the backend differs.

### 8.2 Process structure

```
┌───────────────┐  commands   ┌──────────────────────────────────────────┐
│ control (web) │────────────▶│ motion (separate process, prioritised)   │
│ phone UI      │             │  command arbiter → gait → IK → smoothing │
├───────────────┤             │  → joint limits → servo backend          │
│ vision        │────────────▶│      backends: PCA9685 | MuJoCo | dry-run│
│ follow mode   │             └──────────────────────────────────────────┘
├───────────────┤                       ▲ telemetry (pose, battery, state)
│ voice         │                       │
└───────────────┘◀──────────────────────┘
```

- **The motion process** runs a fixed-rate loop (**target 100 Hz, minimum 50 Hz**). It has real-time priority and is pinned to its own CPU core on the Pi 5 (4 cores). It is the only thing that talks to servos. It accepts a small set of **high-level commands**:
  - body velocity (vx, vy, ωz)
  - body pose (height, tilt, rotate, shift)
  - play move / dance by name
  - sit, stand, relax, e-stop
- **The command interface is the boundary** that allows motion to move to an ESP32 later (§5.1) without changing its clients.
- **Command arbitration:** e-stop beats the remote, which beats follow mode, which beats idle. Each source has a **deadman timeout** (about 0.5 s, builder's call). If a source stops sending, its command lapses and Rocky comes to a stop and stands.
- The IPC mechanism (ZeroMQ, Unix sockets, etc.) is the builder's call. It needs low latency, to work across processes and to survive a client crashing.
- **The vision process** must never be able to starve motion. It's a separate process, it is not on the motion core, and its frame rate can drop without affecting walking.

### 8.3 Kinematics and gait

- **IK:** analytic 3-DOF inverse kinematics per leg, plus the body-to-leg transforms for all five mounts. Tested by round-trip (FK(IK(p)) ≈ p) across the workspace and by joint-limit checks.
- **Gait generator:** phase-based. Each leg has a phase offset and a duty factor (the fraction of the cycle its foot is on the ground). Stance feet move opposite to the commanded body velocity. Swing feet follow a smooth lift-and-place curve (Bezier or similar).
- **Initial gaits:**
  - **Wave (default, "4 feet down")**, duty factor about 0.8. One leg swings at a time. It's the most stable and lowest-torque gait, and it looks deliberate.
  - **Star gait ("3 feet down")**, duty factor about 0.6. Leg *i* (numbered by physical position around the body) gets phase **φᵢ = 2i/5**, so physically adjacent legs are always 0.4 cycle apart, exactly one swing width. Legs therefore start their swings in the order **0 → 3 → 1 → 4 → 2**. Two *non-adjacent* legs can be in the air at the same time, but **two adjacent legs never are**. It's faster, but must pass the torque rule (§6.3).
- **Gait invariants are unit-tested:** never two adjacent legs in swing, at least 3 feet down at all times, and the centre of mass inside the support polygon (checked in sim).
- **Smoothness:** commanded velocities are rate-limited and eased. Servo targets are interpolated at the loop rate. Each joint has speed limits. Per-servo calibration (offset, direction, pulse-to-angle mapping) compensates for MG996R clone variance. A calibration tool is part of M2.

### 8.4 Moves and dances

- Dances are **keyframe sequences** in body-pose and/or leg space, stored as data files (not code) so they're easy to author and share.
- Body-pose moves (sway, bob, twist, wave a leg) are possible as soon as Rocky can stand. They don't need walking.

### 8.5 Phone control

- A web app served by the Pi (the backend framework is the builder's call, for example FastAPI). The phone opens it in a browser. WebSocket for low-latency commands and telemetry.
- UI: a walk joystick plus a turn control, pose/dance buttons, a mode switch (manual / follow), a battery readout and a **big e-stop**.
- Network: the Pi joins home Wi-Fi by default, with a fallback where the Pi runs its own hotspot for use away from home (builder's call on the details).
- No authentication in v1, because it's local-network only. Documented as such.

### 8.6 Startup and safety

- **MG996Rs jump at full speed to the first commanded position** when power arrives. Startup sequence:
  1. Rocky is placed in, or assumed to be in, the "sit" rest pose.
  2. The PCA9685 is set to output the sit pose.
  3. Only then is servo power switched on.
  4. Rocky stands up slowly.
- **Soft joint limits** in software, derived from CAD clearances in the config.
- **E-stop** (phone button or loss of all control) means sit and relax. A **physical switch** cuts servo power outright.
- **Low battery** means a warning, then sit, then cutting servo power.

### 8.7 Simulation

- **MuJoCo** (free, `pip install mujoco`), run on the Mac.
- The MJCF model is **generated from the config**, with optional visual meshes exported from the CAD.
- Servos are modelled as position actuators with a torque limit and a speed limit set to MG996R-like values, so the sim refuses things the real servos can't do.
- The sim logs joint torques to check the torque rule (§6.3), and records video clips for the build log.

### 8.8 Vision (M6, designed now only so nothing blocks it)

- Picamera2 capture, then a detector/tracker (YOLO via NCNN, which the owner has used on a Pi 5 at 15+ FPS), then a **behaviour** that turns target position and size into motion commands.
- "Select a target" means tapping a detected object in the phone UI, which shows the live camera stream.
- Behaviours are plugins with a common interface (frames in, motion commands out), so new vision features can be added without touching motion.

### 8.9 Voice (M7, designed now only so nothing blocks it)

- Output: Rocky "speaks" in **chords**. Text or intent is mapped to short musical chord phrases synthesised on the Pi, with an optional spoken translation like the laptop in the book.
- Input: the microphone for wake words or commands. The approach is decided at M7.

## 9. Milestones

Each milestone ends with a **build log entry** (text, photos and/or video) in `docs/build-log/`.

| # | Milestone | Done when |
|---|---|---|
| **M0** | **Repo skeleton** | README (what, why, status, fan disclaimer), licences, build log #0, config skeleton, Python project and tests running on the Mac. |
| **M1** | **Kinematics + sim** | IK and gait code with tests. MuJoCo Rocky generated from config walks (wave gait) in any direction and turns in place. Torque log confirms leg lengths. Leg lengths are frozen for printing. |
| **M2** | **One leg on the bench** | One printed leg with 3 MG996Rs, PCA9685 and the power chain. Calibration tool works. The foot tip traces a commanded shape (for example a circle) smoothly. The first public "real hardware" post. |
| **M3** | **Stand up** | All five legs and the carapace assembled. Safe startup, stand, sit, relax. Body-pose moves (tilt, twist, bob) on the real robot. Battery monitoring and e-stop work. |
| **M4** | **Walk** | Wave gait on the real robot: any direction plus turning, smooth (no visible jitter at rest or while walking). Star gait if it passes the torque rule. Runs on battery. |
| **M5** | **Phone control + dances** | Phone web UI drives walking and poses, dances play from data files, deadman timeout and e-stop are verified. |
| **M6** | **Follow mode** | Tap a target on the phone and Rocky follows it. The vision behaviour plugin interface is in place. |
| **M7** | **Voice** | Rocky speaks in chords. Microphone input for at least one interaction. |

M1 comes before M2 so leg lengths are validated before any printing. Hardware bring-up work that doesn't depend on the lengths (a single servo plus the PCA9685 and the power chain on the bench) can run in parallel with M1.

**Overall success:** Rocky walks across a room on battery, smoothly, controlled from a phone. Someone else could build one from the repo alone.

## 10. Repository layout (proposed)

```
README.md               what / why / status / photos / fan disclaimer
LICENSE                 software licence
LICENSE-HARDWARE        hardware licence (CAD + docs)
config/rocky.toml       single source of truth
cad/                    Fusion design (.f3d) + STEP + STL exports
rocky/                  Python package
  kinematics/  gait/  motion/  backends/  control/  vision/  voice/  moves/
sim/                    MJCF generation + sim scripts
tools/                  calibration, servo test, etc.
tests/
docs/
  design/               this document and later design notes
  build-log/            dated progress entries
  assembly/             build guide for rebuilders
  bom.md                parts list with links and prices
```

CAD files (`.f3d`, STEP, STLs) are committed in `cad/`: with Fusion they can't be regenerated from code (revised 2026-11-23, see the M2 design). Rendered videos go on **GitHub releases**. Tooling (uv/pip, ruff, pytest, CI on GitHub Actions running tests and CAD export) is the builder's call.

## 11. Licensing and the fan-project question

- **Decided:** **MIT** for software, and **CERN-OHL-P v2** (permissive) for hardware (CAD, generated models and hardware docs). Anyone may rebuild, remix or sell kits.
- *Project Hail Mary* and the character Rocky belong to Andy Weir and the rights holders. The README states that this is a **fan-made, unaffiliated** project. The repo contains **no art, text, audio or assets from the book or film**. The robot's design is original.

## 12. Decisions made by the owner

- Locomotion first. Other features come after.
- Raspberry Pi 5 (owner has the 16 GB model) as the controller, with the owner's existing camera.
- MG996R-class servos. About 3 per leg, but the count is not locked.
- Everything 3D printed in PLA (owner has blue, white and black in stock).
- Cheap enough for others to rebuild.
- Features: walking, phone remote with dance moves, extensible camera follow mode, mic + speaker voice.
- Architecture A (Pi 5 + PCA9685, Python).
- CAD in Fusion 360 (revised from build123d before any CAD existed); Blender for the walking/dancing showcase.
- Spare ESP32s are available for later use if needed.
- When constraints collide: shrink before upgrading (§6.3). "Smooth" allows a slight buzz at rest (§4).
- Licences: MIT (software) + CERN-OHL-P v2 (hardware).
- Camera: Raspberry Pi Camera Module 3 (standard), with a USB webcam as a supported fallback.
- Buying: about 4 MG996Rs now for the bench leg; the rest after M1 freezes leg lengths.

## 13. Left to the builder

These are marked "builder's call" above: exact part SKUs within the constraints, IPC mechanism, web framework, config file format, bearing type, infill and wall settings, carapace segmentation, deadman timeout value, network fallback details, tooling and CI, and release packaging. Leg lengths and gait parameters are tuned in M1 against the torque rule.

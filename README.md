# Rocky Walker

An open-source, five-legged walking robot inspired by Rocky from *Project Hail Mary*.

Rocky Walker has five legs spread evenly around a round, rock-like carapace, and **no front**. It's meant to walk in any direction and turn in place, like the character. The brain is a Raspberry Pi 5 driving 15 cheap MG996R servos through a PCA9685 board. Every structural part is 3D printed in PLA and fits a 180 × 180 mm print bed. The goal is a robot that other people can actually afford to rebuild: about $200 in parts, not counting the Pi.

## Status

**M2 done: Rocky has a real, printable design (Fusion 360) and walks and dances in Blender, driven by the real motion code.** See [build log #2](docs/build-log/2026-11-23-02-fusion-and-blender.md).

![Rocky in Blender](docs/build-log/media/02-hero.png)

Next up is the **bench leg**: one printed leg on real servos. Follow along in the [build log](docs/build-log/).

| Milestone | What | State |
|---|---|---|
| M0 | Repo skeleton | ✅ |
| M1 | Kinematics + simulated walking | ✅ |
| M2 | Fusion 360 design + Blender showcase | ✅ |
| M2b | One leg on the bench | 🚧 |
| M3 | Stand up | — |
| M4 | Walk | — |
| M5 | Phone control + dances | — |
| M6 | Camera follow mode | — |
| M7 | Voice (Rocky speaks in chords) | — |

## Why

This project is open from day one. It's a public record of building a legged robot from scratch: the design decisions, the maths, what worked and especially what didn't. Read the [design doc](docs/design/2026-11-23-rocky-walker-design.md) for the full reasoning.

## Repo layout

```
README.md               what / why / status / photos / fan disclaimer
LICENSE                 software licence (MIT)
LICENSE-HARDWARE        hardware licence (CERN-OHL-P v2)
config/rocky.toml       single source of truth: geometry, servos, gait
cad/                    Fusion 360 design (.f3d) + STEP + STLs; build scripts in cad/fusion/
moves/                  dance moves (TOML keyframes)
showcase/               Blender scene script
rocky/                  Python package: kinematics, gait, motion, backends, sim, ...
tools/                  Fusion MCP client + script runner (calibration tools come with the bench leg)
tests/
docs/
  design/               design documents
  plans/                implementation plans
  build-log/            dated progress entries
  bom.md                parts list
```

## Development

The project needs [uv](https://docs.astral.sh/uv/).

```bash
uv sync
uv run pytest                                              # 120+ tests: kinematics, sim, CAD reports, moves
uv run python -m rocky.sim walk --vx 0.03 --vy 0.02        # walk and print a report
uv run python -m rocky.sim torque --gait star              # torque budget check
uv sync --extra video && uv run python -m rocky.sim walk --wz 0.25 --video out/turn.mp4
uv run mjpython -m rocky.sim walk --vx 0.03 --viewer       # interactive 3D viewer (macOS)
```

## Build one

Everything for printing is in [`cad/`](cad/):

- `cad/stl/`: one STL per printed part (Coxa, Femur, Tibia ×5; ChassisTop, ChassisBottom, BatteryTray, Deck, Carapace ×1). They're exported in assembly orientation, so use your slicer's "lay on face" to put each part flat. PLA, 0.2 mm layers, at least 4 walls, 30–40% infill. Every part fits a 180 mm bed.
- `cad/stl/FitCoupon.stl`: **print this first** to check servo, bearing and nut fit (tune `fit_clearance`).
- `cad/rocky.f3d` is the editable Fusion 360 source; `cad/rocky.step` opens in any CAD tool.
- Parts list: [docs/bom.md](docs/bom.md).

## Watch it in Blender

```bash
uv run python -m rocky.showcase export all out/anim/
/Applications/Blender.app/Contents/MacOS/Blender --background --factory-startup \
  --python showcase/build_scene.py -- --meshes cad/meshes --config config/rocky.toml \
  --anim out/anim/dance.json --out out/render/dance.mp4
```

See [showcase/README.md](showcase/README.md).

## Licences

- **Software** (everything under `rocky/`, `tools/`, `tests/`): [MIT](LICENSE).
- **Hardware** (CAD under `cad/`, generated models and hardware documentation): [CERN-OHL-P v2](LICENSE-HARDWARE).

## Disclaimer

> Rocky Walker is a fan-made project inspired by the character Rocky from Andy Weir's novel *Project Hail Mary*. It is not affiliated with or endorsed by Andy Weir, his publishers, or the film's producers. All designs, code and media in this repository are original.

"""Command-line simulator.

Examples:
    uv run python -m rocky.sim walk --gait wave --vx 0.03 --duration 10
    uv run python -m rocky.sim walk --vx 0.03 --video out/walk.mp4   (needs --extra video)
    uv run python -m rocky.sim torque --gait star
    uv run mjpython -m rocky.sim walk --vx 0.03 --viewer                 (macOS needs mjpython)
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

from rocky.commands import BodyVelocity
from rocky.config import DEFAULT_CONFIG_PATH, RobotConfig, load_config
from rocky.sim.model import joint_names
from rocky.sim.runner import SimResult, simulate


def max_speeds(cfg: RobotConfig, gait: str) -> tuple[float, float]:
    """Fastest body speed (m/s) and turn rate (rad/s) the stride limit allows."""
    t_stance = cfg.gait.duty_factor[gait] * cfg.gait.period
    return (
        cfg.gait.max_stride / t_stance,
        cfg.gait.max_stride / (t_stance * cfg.gait.stance_radius),
    )


def report(cfg: RobotConfig, res: SimResult, cmd: BodyVelocity, duration: float) -> bool:
    d = res.torso_pos[-1] - res.torso_pos[0]
    print(
        f"commanded      vx={cmd.vx:.4f} vy={cmd.vy:.4f} m/s  wz={cmd.wz:.4f} rad/s"
        f"  for {duration:g} s"
    )
    print(f"displacement   x={d[0]:+.3f} y={d[1]:+.3f} m  (|xy| {np.hypot(d[0], d[1]):.3f} m)")
    print(f"yaw change     {math.degrees(res.torso_yaw[-1] - res.torso_yaw[0]):+.1f} deg")
    print(f"max tilt       {math.degrees(res.torso_tilt.max()):.1f} deg")
    print(f"min torso z    {res.torso_pos[:, 2].min() * 1000:.0f} mm")
    rule = cfg.servo.torque_rule
    peaks = res.sustained_torque_peak()
    print(f"\nsustained torque peak (0.5 s window), rule <= {rule:.3f} N.m:")
    for name, p in zip(joint_names(), peaks, strict=True):
        flag = "OK " if p <= rule else "OVER"
        print(f"  {name:<11} {p:.3f} N.m  {100 * p / rule:5.1f}%  {flag}")
    ok = bool(peaks.max() <= rule)
    print(
        f"\nTORQUE RULE: {'PASS' if ok else 'FAIL'} (worst {100 * peaks.max() / rule:.1f}% of rule)"
    )
    return ok


def run_viewer(cfg: RobotConfig, cmd: BodyVelocity, gait: str | None, duration: float) -> None:
    import mujoco.viewer

    from rocky.backends.mujoco_backend import MujocoBackend
    from rocky.motion.controller import MotionController
    from rocky.sim.model import load_model

    model = load_model(cfg)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("stand").id)
    ctl = MotionController(cfg, gait)
    backend = MujocoBackend(model, data)
    substeps = max(1, round(1 / (cfg.motion.loop_hz * model.opt.timestep)))
    dt = substeps * model.opt.timestep
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and data.time < duration + 1.0:
            start = time.perf_counter()
            backend.write(ctl.tick(dt, cmd if data.time >= 1.0 else BodyVelocity()))
            for _ in range(substeps):
                mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0.0, dt - (time.perf_counter() - start)))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m rocky.sim",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    sub = parser.add_subparsers(dest="mode", required=True)

    walk = sub.add_parser("walk", help="walk with a fixed command and report")
    walk.add_argument("--gait", choices=["wave", "star"], default=None)
    walk.add_argument("--vx", type=float, default=0.0, help="m/s")
    walk.add_argument("--vy", type=float, default=0.0, help="m/s")
    walk.add_argument("--wz", type=float, default=0.0, help="rad/s")
    walk.add_argument("--duration", type=float, default=10.0, help="seconds")
    walk.add_argument("--video", type=Path, default=None, help="write an mp4 (needs --extra video)")
    walk.add_argument(
        "--viewer",
        action="store_true",
        help="interactive viewer (on macOS run with: uv run mjpython -m rocky.sim ...)",
    )

    torque = sub.add_parser(
        "torque", help="walk forward at 80%% of max speed for 10 s; torque table"
    )
    torque.add_argument("--gait", choices=["wave", "star"], default=None)

    args = parser.parse_args(argv)
    cfg = load_config(args.config)
    gait = args.gait or cfg.gait.default

    if args.mode == "torque":
        cmd, duration, video = BodyVelocity(vx=0.8 * max_speeds(cfg, gait)[0]), 10.0, None
    else:
        cmd, duration, video = BodyVelocity(args.vx, args.vy, args.wz), args.duration, args.video
        if args.viewer:
            run_viewer(cfg, cmd, gait, duration)
            return 0

    print(f"gait: {gait}")
    res = simulate(cfg, lambda t: cmd, duration, gait=gait, video_path=video)
    ok = report(cfg, res, cmd, duration)
    if video:
        print(f"video: {video}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

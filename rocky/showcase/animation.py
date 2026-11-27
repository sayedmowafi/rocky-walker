"""Per-frame animation data driven by the real motion code.

A frame is the body's world pose (4x4; floor at z = 0) plus the 15 joint angles. Walking frames
come from ``MotionController``; the body moves through the world at exactly the command the
controller used, so stance feet stay planted. Dance frames come from ``MovePlayer``.
"""

from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from rocky.commands import BodyVelocity
from rocky.config import RobotConfig
from rocky.kinematics.body import leg_mounts, leg_to_body, rot_z
from rocky.kinematics.leg import leg_fk
from rocky.motion.controller import MotionController
from rocky.moves.move import Move, load_move
from rocky.moves.player import MovePlayer
from rocky.sim.model import joint_names

FPS = 30
PLANTED_Z = 0.0005
MOVES_DIR = Path(__file__).resolve().parents[2] / "moves"


@dataclass(frozen=True)
class Frame:
    t: float
    pose_world: np.ndarray  # (4, 4)
    angles: np.ndarray  # (5, 3)


def _frame_ticks(duration: float, loop_hz: float, fps: int) -> list[int]:
    """Control-tick index nearest to each frame time k/fps, for k = 0..duration*fps."""
    return [round(k * loop_hz / fps) for k in range(round(duration * fps) + 1)]


def walk_frames(
    cfg: RobotConfig,
    commands: Callable[[float], BodyVelocity],
    duration: float,
    fps: int = FPS,
    gait: str | None = None,
    settle_s: float = 1.0,
) -> list[Frame]:
    ctl = MotionController(cfg, gait)
    dt = 1.0 / cfg.motion.loop_hz
    total = settle_s + duration
    wanted = _frame_ticks(total, cfg.motion.loop_hz, fps)
    x = y = yaw = 0.0
    frames: list[Frame] = []

    def pose() -> np.ndarray:
        m = np.eye(4)
        m[:3, :3] = rot_z(yaw)
        m[:3, 3] = [x, y, cfg.gait.body_height]
        return m

    angles = ctl.angles
    k = 0
    for tick in range(wanted[-1] + 1):
        if tick > 0:
            t = tick * dt
            cmd = BodyVelocity() if t <= settle_s + 1e-12 else commands(t - settle_s)
            angles = ctl.tick(dt, cmd)
            c = ctl.command
            # the planner moves stance feet by Rz(-wz dt) p - v dt; this body update is its
            # exact inverse, so stance feet stay fixed in the world
            yaw += c.wz * dt
            step = rot_z(yaw)[:2, :2] @ np.array([c.vx, c.vy]) * dt
            x, y = x + step[0], y + step[1]
        while k < len(wanted) and wanted[k] == tick:
            frames.append(Frame(k / fps, pose(), angles.copy()))
            k += 1
    return frames


def move_frames(cfg: RobotConfig, move: Move, fps: int = FPS) -> list[Frame]:
    player = MovePlayer(cfg)
    dt = 1.0 / cfg.motion.loop_hz
    stream = list(player.play(move, dt))
    frames = []
    for k, tick in enumerate(_frame_ticks(move.duration, cfg.motion.loop_hz, fps)):
        _, pose, angles = stream[min(tick, len(stream) - 1)]
        frames.append(Frame(k / fps, player.body_to_world(pose), angles))
    return frames


def feet_world(cfg: RobotConfig, frame: Frame) -> np.ndarray:
    out = []
    for m in leg_mounts(cfg.body):
        p = leg_to_body(leg_fk(frame.angles[m.index], cfg.leg), m)
        out.append((frame.pose_world @ np.append(p, 1.0))[:3])
    return np.array(out)


def max_stance_drift(cfg: RobotConfig, frames: list[Frame]) -> float:
    """Largest xy slide of any foot that is on the floor in two consecutive frames."""
    worst = 0.0
    prev = feet_world(cfg, frames[0])
    for f in frames[1:]:
        cur = feet_world(cfg, f)
        planted = (prev[:, 2] < PLANTED_Z) & (cur[:, 2] < PLANTED_Z)
        if planted.any():
            worst = max(worst, float(np.linalg.norm((cur - prev)[planted, :2], axis=1).max()))
        prev = cur
    return worst


def write_animation(frames: list[Frame], path: Path | str, fps: int = FPS) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "fps": fps,
        "joint_names": joint_names(),
        "frames": [
            {
                "t": round(f.t, 9),
                "pose": f.pose_world.reshape(-1).tolist(),
                "angles": f.angles.reshape(-1).tolist(),
            }
            for f in frames
        ],
    }
    path.write_text(json.dumps(data))


def read_animation(path: Path | str) -> list[Frame]:
    data = json.loads(Path(path).read_text())
    return [
        Frame(f["t"], np.array(f["pose"]).reshape(4, 4), np.array(f["angles"]).reshape(5, 3))
        for f in data["frames"]
    ]


def _speeds(cfg: RobotConfig, gait: str) -> tuple[float, float]:
    t_stance = cfg.gait.duty_factor[gait] * cfg.gait.period
    return cfg.gait.max_stride / t_stance, cfg.gait.max_stride / (t_stance * cfg.gait.stance_radius)


def _walk(heading_deg: float):
    def clip(cfg: RobotConfig) -> list[Frame]:
        v = 0.8 * _speeds(cfg, cfg.gait.default)[0]
        h = np.radians(heading_deg)
        cmd = BodyVelocity(float(v * np.cos(h)), float(v * np.sin(h)), 0.0)
        return walk_frames(cfg, lambda t: cmd, 8.0)

    return clip


def _turn(cfg: RobotConfig) -> list[Frame]:
    w = 0.8 * _speeds(cfg, cfg.gait.default)[1]
    return walk_frames(cfg, lambda t: BodyVelocity(wz=w), 8.0)


def _dance(cfg: RobotConfig) -> list[Frame]:
    return move_frames(cfg, load_move(MOVES_DIR / "routine.toml"))


def _move(name: str):
    def clip(cfg: RobotConfig) -> list[Frame]:
        return move_frames(cfg, load_move(MOVES_DIR / f"{name}.toml"))

    return clip


CLIPS: dict[str, Callable[[RobotConfig], list[Frame]]] = {
    "walk-forward": _walk(0.0),
    "walk-diagonal": _walk(45.0),
    "turn": _turn,
    "dance": _dance,
    # five-leg moves (generated by tools/make_moves.py)
    "ring-ripple": _move("ring-ripple"),
    "star-step": _move("star-step"),
    "rock-mode": _move("rock-mode"),
    "rest-mode": _move("rest-mode"),
}

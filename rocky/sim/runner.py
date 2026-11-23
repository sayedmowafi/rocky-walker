"""Run the motion stack against the MuJoCo model and record what happened."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from rocky.backends.mujoco_backend import MujocoBackend
from rocky.commands import BodyVelocity
from rocky.config import RobotConfig
from rocky.motion.controller import MotionController
from rocky.sim.model import TIMESTEP, load_model

VIDEO_FPS = 30


@dataclass
class SimResult:
    times: np.ndarray
    torso_pos: np.ndarray
    torso_yaw: np.ndarray
    torso_tilt: np.ndarray
    joint_torque: np.ndarray

    def sustained_torque_peak(self, window_s: float = 0.5) -> np.ndarray:
        """Per joint: the max over time of the ``window_s`` moving average of |torque|."""
        dt = float(np.median(np.diff(self.times)))
        n = max(1, round(window_s / dt))
        kernel = np.ones(n) / n
        absolute = np.abs(self.joint_torque)
        return np.array(
            [
                np.convolve(absolute[:, j], kernel, mode="valid").max()
                for j in range(absolute.shape[1])
            ]
        )


def _yaw(quat: np.ndarray) -> float:
    w, x, y, z = quat
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def simulate(
    cfg: RobotConfig,
    commands: Callable[[float], BodyVelocity],
    duration: float,
    gait: str | None = None,
    settle_s: float = 1.0,
    video_path: Path | None = None,
) -> SimResult:
    """Simulate ``duration`` seconds of ``commands(t)`` after ``settle_s`` of standing still."""
    controller = MotionController(cfg, gait)
    dt = _tick_dt(cfg)

    def angles_at(k: int, t: float, in_settle: bool) -> np.ndarray:
        cmd = BodyVelocity() if in_settle else commands(t - settle_s)
        return controller.tick(dt, cmd)

    return _run(cfg, angles_at, round(duration / dt), settle_s, video_path)


def simulate_angles(
    cfg: RobotConfig,
    angles: Sequence[np.ndarray],
    settle_s: float = 1.0,
    video_path: Path | None = None,
) -> SimResult:
    """Play a precomputed joint-angle stream (one ``(5, 3)`` array per control tick, at
    ``loop_hz``) on the free-standing model, after ``settle_s`` holding the first frame."""

    def angles_at(k: int, t: float, in_settle: bool) -> np.ndarray:
        return angles[0] if in_settle else angles[k]

    return _run(cfg, angles_at, len(angles), settle_s, video_path)


def _tick_dt(cfg: RobotConfig) -> float:
    substeps = max(1, round(1 / (cfg.motion.loop_hz * TIMESTEP)))
    return substeps * TIMESTEP


def _run(
    cfg: RobotConfig,
    angles_at: Callable[[int, float, bool], np.ndarray],
    n_active: int,
    settle_s: float,
    video_path: Path | None,
) -> SimResult:
    model = load_model(cfg)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("stand").id)
    backend = MujocoBackend(model, data)
    torso = model.body("torso").id

    substeps = max(1, round(1 / (cfg.motion.loop_hz * model.opt.timestep)))
    dt = substeps * model.opt.timestep
    n_settle = round(settle_s / dt)

    renderer, frames, next_frame = None, [], 0.0
    if video_path is not None:
        try:
            import imageio.v2  # noqa: F401
        except ImportError as e:
            raise RuntimeError("video needs the 'video' extra: uv sync --extra video") from e
        renderer = mujoco.Renderer(model, height=480, width=640)

    times, pos, yaw, tilt, torque = [], [], [], [], []
    for k in range(n_settle + n_active):
        t = k * dt
        in_settle = k < n_settle
        backend.write(angles_at(k - n_settle, t, in_settle))
        for _ in range(substeps):
            mujoco.mj_step(model, data)
        if renderer is not None and data.time >= next_frame:
            renderer.update_scene(data, camera="track")
            frames.append(renderer.render())
            next_frame += 1 / VIDEO_FPS
        if not in_settle:
            times.append(t + dt - settle_s)
            pos.append(data.qpos[:3].copy())
            yaw.append(_yaw(data.qpos[3:7]))
            up = data.xmat[torso].reshape(3, 3)[:, 2]
            tilt.append(float(np.arccos(np.clip(up[2], -1.0, 1.0))))
            torque.append(data.actuator_force.copy())

    if renderer is not None:
        import imageio.v2 as imageio

        Path(video_path).parent.mkdir(parents=True, exist_ok=True)
        imageio.mimwrite(video_path, frames, fps=VIDEO_FPS)
        renderer.close()

    return SimResult(
        times=np.array(times),
        torso_pos=np.array(pos),
        torso_yaw=np.unwrap(np.array(yaw)),
        torso_tilt=np.array(tilt),
        joint_torque=np.array(torque),
    )

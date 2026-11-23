"""Play and validate moves: body pose above planted feet, solved with the leg IK and the same
joint-limit and servo-speed guards as walking."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import replace

import numpy as np

from rocky.config import ConfigError, RobotConfig
from rocky.kinematics.body import body_to_leg, leg_mounts, neutral_feet
from rocky.kinematics.leg import UnreachableError, leg_ik
from rocky.kinematics.mass import center_of_mass, support_margin
from rocky.kinematics.pose import BodyPose
from rocky.motion.controller import MotionStats
from rocky.moves.move import Move

#: minimum distance (m) the COM must stay inside the planted feet's polygon
STABILITY_MARGIN = 0.015
#: a foot lifted more than this (m) is not planted
LIFT_EPS = 0.001
_FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")


def _smoothstep(u: float) -> float:
    return u * u * (3.0 - 2.0 * u)


class MovePlayer:
    def __init__(self, cfg: RobotConfig):
        self._cfg = cfg
        self._mounts = leg_mounts(cfg.body)
        self._limits = cfg.limits.as_array()
        h = np.array([0.0, 0.0, cfg.gait.body_height])
        self._feet_world = neutral_feet(cfg) + h  # floor at z = 0
        self.stats = MotionStats()

    def body_to_world(self, pose: BodyPose) -> np.ndarray:
        world = np.eye(4)
        world[2, 3] = self._cfg.gait.body_height
        return world @ pose.matrix()

    def sample(self, move: Move, t: float) -> tuple[BodyPose, dict[int, np.ndarray]]:
        keys = move.keyframes
        if t <= keys[0].t:
            return keys[0].pose, dict(keys[0].feet)
        if t >= keys[-1].t:
            return keys[-1].pose, dict(keys[-1].feet)
        k = next(i for i in range(len(keys) - 1) if keys[i].t <= t <= keys[i + 1].t)
        a, b = keys[k], keys[k + 1]
        u = _smoothstep((t - a.t) / (b.t - a.t))
        pose = BodyPose(
            **{f: (1 - u) * getattr(a.pose, f) + u * getattr(b.pose, f) for f in _FIELDS}
        )
        feet = {}
        for leg in set(a.feet) | set(b.feet):
            fa = a.feet.get(leg, np.zeros(3))
            fb = b.feet.get(leg, np.zeros(3))
            feet[leg] = (1 - u) * fa + u * fb
        return pose, feet

    def feet_world(self, feet: dict[int, np.ndarray]) -> np.ndarray:
        out = self._feet_world.copy()
        for leg, off in feet.items():
            out[leg] += off
        return out

    def angles(self, pose: BodyPose, feet: dict[int, np.ndarray]) -> np.ndarray:
        inv = np.linalg.inv(self.body_to_world(pose))
        out = []
        for m, foot in zip(self._mounts, self.feet_world(feet), strict=True):
            out.append(leg_ik(body_to_leg((inv @ np.append(foot, 1.0))[:3], m), self._cfg.leg))
        return np.array(out)

    def validate(self, move: Move, dt: float = 0.01) -> None:
        """Raise ConfigError naming the move, time and reason if any sample is unsafe."""
        if not (move.keyframes[0].is_neutral() and move.keyframes[-1].is_neutral()):
            raise ConfigError(f"{move.name}: must start and end at the neutral pose")
        max_step = self._cfg.servo.max_speed * dt
        prev = None
        for k in range(round(move.duration / dt) + 1):
            t = k * dt
            pose, feet = self.sample(move, t)
            try:
                a = self.angles(pose, feet)
            except UnreachableError as e:
                raise ConfigError(f"{move.name} at t={t:.2f}s: unreachable ({e})") from e
            if ((a < self._limits[:, 0]) | (a > self._limits[:, 1])).any():
                raise ConfigError(f"{move.name} at t={t:.2f}s: outside joint limits")
            if prev is not None and np.abs(a - prev).max() > max_step + 1e-9:
                raise ConfigError(f"{move.name} at t={t:.2f}s: faster than the servo speed limit")
            prev = a
            planted = [i for i in range(5) if feet.get(i, np.zeros(3))[2] <= LIFT_EPS]
            com = center_of_mass(self._cfg, pose, a)
            margin = support_margin(self.feet_world(feet)[planted, :2], com[:2])
            if margin < STABILITY_MARGIN:
                raise ConfigError(
                    f"{move.name} at t={t:.2f}s: centre of mass {margin * 1000:.1f} mm inside the "
                    f"support polygon (need {STABILITY_MARGIN * 1000:.0f} mm)"
                )

    def play(self, move: Move, dt: float) -> Iterator[tuple[float, BodyPose, np.ndarray]]:
        """Yield (t, pose, guarded joint angles) every dt, starting from the neutral stance."""
        current = self.angles(BodyPose(), {})
        max_step = self._cfg.servo.max_speed * dt
        for k in range(round(move.duration / dt) + 1):
            t = k * dt
            pose, feet = self.sample(move, t)
            try:
                target = self.angles(pose, feet)
            except UnreachableError:
                self.stats.unreachable += 1
                target = current
            clipped = np.clip(target, self._limits[:, 0], self._limits[:, 1])
            self.stats.clamped += int(np.count_nonzero(clipped != target))
            delta = clipped - current
            self.stats.speed_limited += int(np.count_nonzero(np.abs(delta) > max_step))
            current = current + np.clip(delta, -max_step, max_step)
            self.stats = replace(self.stats, ticks=self.stats.ticks + 1)
            yield t, pose, current.copy()

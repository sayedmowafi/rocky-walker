"""Motion controller: body-velocity command in, guarded joint angles out, once per tick.

Each tick: rate-limit the command, plan feet, solve IK per leg, clip to joint
limits, clip to servo speed. It never raises during walking: an unreachable leg
holds its previous angles and is counted in ``stats``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from rocky.commands import BodyVelocity
from rocky.config import ConfigError, RobotConfig
from rocky.gait.generator import GaitGenerator
from rocky.gait.patterns import make_pattern
from rocky.kinematics.body import body_to_leg, leg_mounts
from rocky.kinematics.leg import UnreachableError, leg_ik


@dataclass
class MotionStats:
    ticks: int = 0
    unreachable: int = 0
    clamped: int = 0


class MotionController:
    def __init__(self, cfg: RobotConfig, gait: str | None = None):
        gait = gait or cfg.gait.default
        if gait not in cfg.gait.duty_factor:
            raise ConfigError(f"gait {gait!r} has no entry in [gait.duty_factor]")
        self._cfg = cfg
        self._gen = GaitGenerator(cfg, make_pattern(gait, cfg.gait.duty_factor[gait]))
        self._mounts = leg_mounts(cfg.body)
        self._limits = cfg.limits.as_array()
        self._command = BodyVelocity()
        self.stats = MotionStats()

        angles = []
        for m, foot in zip(self._mounts, self._gen.feet, strict=True):
            try:
                a = leg_ik(body_to_leg(foot, m), cfg.leg)
            except UnreachableError as e:
                raise ConfigError(
                    "neutral stance is out of reach: check gait.stance_radius_mm, "
                    "gait.body_height_mm and the [leg] lengths"
                ) from e
            if ((a < self._limits[:, 0]) | (a > self._limits[:, 1])).any():
                raise ConfigError(
                    "neutral stance violates joint limits: check gait.stance_radius_mm, "
                    "gait.body_height_mm and [limits]"
                )
            angles.append(a)
        self._angles = np.array(angles)

    @property
    def angles(self) -> np.ndarray:
        return self._angles.copy()

    @property
    def command(self) -> BodyVelocity:
        return self._command

    def _ramp(self, dt: float, target: BodyVelocity) -> BodyVelocity:
        cur = np.array([self._command.vx, self._command.vy])
        delta = np.array([target.vx, target.vy]) - cur
        max_lin = self._cfg.motion.max_linear_accel * dt
        norm = np.linalg.norm(delta)
        if norm > max_lin:
            delta *= max_lin / norm
        max_ang = self._cfg.motion.max_angular_accel * dt
        dw = float(np.clip(target.wz - self._command.wz, -max_ang, max_ang))
        vx, vy = cur + delta
        return BodyVelocity(float(vx), float(vy), self._command.wz + dw)

    def tick(self, dt: float, target: BodyVelocity) -> np.ndarray:
        self._command = self._ramp(dt, target)
        feet = self._gen.step(dt, self._command)

        new = self._angles.copy()
        for m, foot in zip(self._mounts, feet, strict=True):
            try:
                new[m.index] = leg_ik(body_to_leg(foot, m), self._cfg.leg)
            except UnreachableError:
                self.stats.unreachable += 1

        clipped = np.clip(new, self._limits[:, 0], self._limits[:, 1])
        self.stats.clamped += int(np.count_nonzero(clipped != new))

        max_step = self._cfg.servo.max_speed * dt
        self._angles = self._angles + np.clip(clipped - self._angles, -max_step, max_step)
        self.stats.ticks += 1
        return self.angles

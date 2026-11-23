"""Incremental foot planner: turns a body-velocity command into foot targets each tick.

Stance feet move against the commanded body motion. Each leg decides at the start
of its own swing window whether to step; a leg already at neutral while the
command is zero skips its swing. When everything has settled, the planner idles
(phase frozen). Walking resumes from phase 0, so leg 0 lifts first.
"""

from __future__ import annotations

import numpy as np

from rocky.commands import BodyVelocity
from rocky.config import RobotConfig
from rocky.gait.patterns import GaitPattern
from rocky.kinematics.body import neutral_feet, rot_z

#: Feet closer than this (xy) to neutral count as "at neutral" for idling.
SETTLE_TOL = 0.002


def _smoothstep(u: float) -> float:
    return u * u * (3.0 - 2.0 * u)


class GaitGenerator:
    def __init__(self, cfg: RobotConfig, pattern: GaitPattern):
        self._pattern = pattern
        self._period = cfg.gait.period
        self._height = cfg.gait.body_height
        self._step_height = cfg.gait.step_height
        self._max_stride = cfg.gait.max_stride
        self._neutral = neutral_feet(cfg)
        self._n = len(self._neutral)
        self._feet = self._neutral.copy()
        self._liftoff = self._neutral.copy()
        self._swinging = [False] * self._n
        self._idle = True
        self._reset_phase()

    @property
    def feet(self) -> np.ndarray:
        return self._feet.copy()

    @property
    def phase(self) -> float:
        return self._phase

    @property
    def swinging(self) -> tuple[bool, ...]:
        return tuple(self._swinging)

    @property
    def stance_time(self) -> float:
        return self._pattern.duty_factor * self._period

    def _reset_phase(self) -> None:
        self._phase = 0.0
        # Sentinel above any local phase: the leg whose window opens at phase 0 starts at once.
        self._prev_local = [1.0] * self._n

    def clamp(self, cmd: BodyVelocity) -> BodyVelocity:
        """Scale ``cmd`` down so no leg's stride exceeds ``max_stride``."""
        v = np.array([cmd.vx, cmd.vy])
        n = self._neutral
        foot_v = v + cmd.wz * np.column_stack([-n[:, 1], n[:, 0]])
        stride = np.linalg.norm(foot_v, axis=1).max() * self.stance_time
        if stride <= self._max_stride:
            return cmd
        s = self._max_stride / stride
        return BodyVelocity(cmd.vx * s, cmd.vy * s, cmd.wz * s)

    def _settled(self) -> bool:
        dist = np.linalg.norm(self._feet[:, :2] - self._neutral[:, :2], axis=1)
        return not any(self._swinging) and bool((dist <= SETTLE_TOL).all())

    def step(self, dt: float, cmd: BodyVelocity) -> np.ndarray:
        cmd = self.clamp(cmd)
        if cmd.is_zero() and self._settled():
            self._idle = True
            return self.feet
        if self._idle:
            self._idle = False
            self._reset_phase()

        self._phase = (self._phase + dt / self._period) % 1.0
        sf = self._pattern.swing_fraction
        t_st = self.stance_time
        v = np.array([cmd.vx, cmd.vy, 0.0])
        stance_rot = rot_z(-cmd.wz * dt)
        target_rot = rot_z(cmd.wz * t_st / 2)

        for i in range(self._n):
            local = self._pattern.local_phase(i, self._phase)
            target = target_rot @ self._neutral[i] + v * t_st / 2
            target[2] = -self._height
            window_started = local < self._prev_local[i] and local < sf
            if window_started and not self._swinging[i]:
                at_neutral = np.linalg.norm(self._feet[i, :2] - self._neutral[i, :2]) <= SETTLE_TOL
                if not (cmd.is_zero() and at_neutral):
                    self._swinging[i] = True
                    self._liftoff[i] = self._feet[i]
            if self._swinging[i] and local >= sf:
                self._feet[i] = target
                self._swinging[i] = False
            elif self._swinging[i]:
                u = local / sf
                xy = self._liftoff[i, :2] + (target[:2] - self._liftoff[i, :2]) * _smoothstep(u)
                z = -self._height + self._step_height * np.sin(np.pi * u)
                self._feet[i] = [xy[0], xy[1], z]
            else:
                foot = stance_rot @ self._feet[i] - v * dt
                foot[2] = -self._height
                self._feet[i] = foot
            self._prev_local[i] = local
        return self.feet

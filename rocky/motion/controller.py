"""Motion controller: body-velocity command in, guarded joint angles out, once per tick.

Each tick: reject non-finite commands, clamp the command to the longest stride the
legs can take, rate-limit it, plan feet, solve IK per leg, clip to joint limits,
clip to servo speed. It never raises during walking: an unreachable leg holds its
previous angles. Every guard that engages is counted in ``stats``, so tests can
assert normal walking never relies on the guards.

At construction, the controller checks that the neutral stance and the whole
walking envelope the planner can command stay reachable and within joint limits.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from rocky.commands import BodyVelocity
from rocky.config import ConfigError, RobotConfig
from rocky.gait.generator import GaitGenerator
from rocky.gait.patterns import make_pattern
from rocky.kinematics.body import body_to_leg, leg_mounts
from rocky.kinematics.leg import UnreachableError, leg_ik

#: Headings (deg) probed by the start-up envelope check: toward legs, between legs, and more.
_PROBE_HEADINGS = range(0, 360, 18)
_PROBE_DT = 0.02
#: Envelope checks already passed, keyed by (repr(config), gait): construction stays cheap.
_ENVELOPE_OK: set[tuple[str, str]] = set()


@dataclass
class MotionStats:
    ticks: int = 0
    unreachable: int = 0
    clamped: int = 0
    speed_limited: int = 0
    rejected: int = 0


class MotionController:
    def __init__(self, cfg: RobotConfig, gait: str | None = None, *, _validate: bool = True):
        gait = gait or cfg.gait.default
        if gait not in cfg.gait.duty_factor:
            raise ConfigError(f"gait {gait!r} has no entry in [gait.duty_factor]")
        self._cfg = cfg
        pattern = make_pattern(gait, cfg.gait.duty_factor[gait])
        self._gen = GaitGenerator(cfg, pattern)
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
            if not self._within_limits(a):
                raise ConfigError(
                    "neutral stance violates joint limits: check gait.stance_radius_mm, "
                    "gait.body_height_mm and [limits]"
                )
            angles.append(a)
        self._angles = np.array(angles)
        if _validate:
            self._check_envelope(gait)

    def _within_limits(self, a: np.ndarray) -> bool:
        return bool(((a >= self._limits[:, 0]) & (a <= self._limits[:, 1])).all())

    def _check_envelope(self, gait: str) -> None:
        """Drive scratch controllers at full stride in many directions, starting from rest
        and then reversing at full speed. That covers the start-up and reversal transients.
        Every foot must stay reachable and within joint limits, with no guard engaging."""
        key = (repr(self._cfg), gait)
        if key in _ENVELOPE_OK:
            return
        big = 1e3  # far past the stride clamp: the controller scales it to max stride
        probes = [
            BodyVelocity(big * math.cos(h), big * math.sin(h), wz)
            for h in map(math.radians, _PROBE_HEADINGS)
            for wz in (0.0, big, -big)
        ] + [BodyVelocity(wz=big), BodyVelocity(wz=-big)]
        steps = round(1.5 * self._cfg.gait.period / _PROBE_DT)
        for cmd in probes:
            scratch = MotionController(self._cfg, gait, _validate=False)
            reverse = BodyVelocity(-cmd.vx, -cmd.vy, -cmd.wz)
            for target in (cmd, reverse):
                for _ in range(steps):
                    scratch.tick(_PROBE_DT, target)
            if scratch.stats.unreachable or scratch.stats.clamped:
                raise ConfigError(
                    f"walking envelope of the {gait!r} gait leaves the leg's reach or joint "
                    "limits: reduce gait.max_stride_mm or gait.step_height_mm, widen [limits], "
                    "or lower motion.max_linear_accel_mps2"
                )
        _ENVELOPE_OK.add(key)

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
        if not all(math.isfinite(v) for v in (target.vx, target.vy, target.wz)):
            self.stats.rejected += 1
            target = BodyVelocity()
        # Clamp before ramping, so the ramp never winds up past what the legs can do.
        self._command = self._ramp(dt, self._gen.clamp(target))
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
        delta = clipped - self._angles
        self.stats.speed_limited += int(np.count_nonzero(np.abs(delta) > max_step))
        self._angles = self._angles + np.clip(delta, -max_step, max_step)
        self.stats.ticks += 1
        return self.angles

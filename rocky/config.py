"""Robot configuration: loads ``config/rocky.toml`` (human units) into SI-unit dataclasses.

Every other module reads geometry, limits and gait parameters from here, so the
config file is the single source of truth for CAD, simulation and kinematics.
"""

from __future__ import annotations

import math
import tomllib
from dataclasses import dataclass
from math import radians
from pathlib import Path
from typing import Any

import numpy as np

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "rocky.toml"

KGCM_TO_NM = 0.0980665
SUPPORTED_N_LEGS = 5
MIN_LOOP_HZ = 50.0


class ConfigError(ValueError):
    """Raised when the robot config is missing a value or a value is invalid."""


@dataclass(frozen=True)
class BodyParams:
    n_legs: int
    mount_radius: float
    thickness: float


@dataclass(frozen=True)
class MassParams:
    total: float
    coxa_link: float
    femur_link: float
    tibia_link: float


@dataclass(frozen=True)
class LegGeometry:
    coxa: float
    femur: float
    tibia: float


@dataclass(frozen=True)
class JointLimits:
    coxa: tuple[float, float]
    femur: tuple[float, float]
    tibia: tuple[float, float]

    def as_array(self) -> np.ndarray:
        """Limits as a ``(3, 2)`` array of ``(min, max)`` rows: coxa, femur, tibia."""
        return np.array([self.coxa, self.femur, self.tibia], dtype=float)


@dataclass(frozen=True)
class ServoSpec:
    stall_torque: float
    max_speed: float
    torque_rule: float


@dataclass(frozen=True)
class GaitParams:
    default: str
    period: float
    duty_factor: dict[str, float]
    body_height: float
    stance_radius: float
    step_height: float
    max_stride: float


@dataclass(frozen=True)
class MotionParams:
    loop_hz: float
    max_linear_accel: float
    max_angular_accel: float


@dataclass(frozen=True)
class RobotConfig:
    body: BodyParams
    mass: MassParams
    leg: LegGeometry
    limits: JointLimits
    servo: ServoSpec
    gait: GaitParams
    motion: MotionParams


def load_config(path: Path | str = DEFAULT_CONFIG_PATH) -> RobotConfig:
    """Read a TOML config file and return it in SI units."""
    with open(path, "rb") as f:
        return parse_config(tomllib.load(f))


def parse_config(data: dict[str, Any]) -> RobotConfig:
    """Validate a parsed TOML dict and convert it to SI units."""

    def table(name: str) -> dict[str, Any]:
        t = data.get(name)
        if not isinstance(t, dict):
            raise ConfigError(f"missing [{name}] table")
        return t

    def num(t: dict[str, Any], section: str, key: str, *, positive: bool = True) -> float:
        if key not in t:
            raise ConfigError(f"missing {section}.{key}")
        v = t[key]
        if isinstance(v, bool) or not isinstance(v, int | float):
            raise ConfigError(f"{section}.{key} must be a number, got {v!r}")
        if not math.isfinite(v):
            raise ConfigError(f"{section}.{key} must be finite, got {v}")
        if positive and v <= 0:
            raise ConfigError(f"{section}.{key} must be > 0, got {v}")
        return float(v)

    def limit(t: dict[str, Any], key: str) -> tuple[float, float]:
        v = t.get(key)
        if not (isinstance(v, list) and len(v) == 2):
            raise ConfigError(f"limits.{key} must be a [min, max] pair")
        lo, hi = float(v[0]), float(v[1])
        if not (math.isfinite(lo) and math.isfinite(hi)):
            raise ConfigError(f"limits.{key} must be finite, got {v}")
        if lo >= hi:
            raise ConfigError(f"limits.{key} must have min < max, got {v}")
        return radians(lo), radians(hi)

    b = table("body")
    n_legs = b.get("n_legs")
    if n_legs != SUPPORTED_N_LEGS:
        raise ConfigError(f"body.n_legs must be {SUPPORTED_N_LEGS}, got {n_legs!r}")
    body = BodyParams(
        n_legs=SUPPORTED_N_LEGS,
        mount_radius=num(b, "body", "mount_radius_mm") / 1000,
        thickness=num(b, "body", "thickness_mm") / 1000,
    )

    m = table("mass")
    mass = MassParams(
        total=num(m, "mass", "total_kg"),
        coxa_link=num(m, "mass", "coxa_link_kg"),
        femur_link=num(m, "mass", "femur_link_kg"),
        tibia_link=num(m, "mass", "tibia_link_kg"),
    )
    if n_legs * (mass.coxa_link + mass.femur_link + mass.tibia_link) >= mass.total:
        raise ConfigError("mass.total_kg must exceed the summed leg link masses")

    lg = table("leg")
    leg = LegGeometry(
        coxa=num(lg, "leg", "coxa_mm") / 1000,
        femur=num(lg, "leg", "femur_mm") / 1000,
        tibia=num(lg, "leg", "tibia_mm") / 1000,
    )

    lim = table("limits")
    limits = JointLimits(
        coxa=limit(lim, "coxa_deg"),
        femur=limit(lim, "femur_deg"),
        tibia=limit(lim, "tibia_deg"),
    )

    s = table("servo")
    servo = ServoSpec(
        stall_torque=num(s, "servo", "stall_torque_kgcm") * KGCM_TO_NM,
        max_speed=radians(num(s, "servo", "max_speed_dps")),
        torque_rule=num(s, "servo", "torque_rule_kgcm") * KGCM_TO_NM,
    )

    g = table("gait")
    duty_raw = g.get("duty_factor")
    if not isinstance(duty_raw, dict) or not duty_raw:
        raise ConfigError("missing [gait.duty_factor] table")
    duty: dict[str, float] = {}
    for name, v in duty_raw.items():
        if isinstance(v, bool) or not isinstance(v, int | float) or not 0 < v < 1:
            raise ConfigError(f"gait.duty_factor.{name} must be in (0, 1), got {v!r}")
        duty[name] = float(v)
    default = g.get("default")
    if default not in duty:
        raise ConfigError(f"gait.default {default!r} must be one of {sorted(duty)}")
    gait = GaitParams(
        default=default,
        period=num(g, "gait", "period_s"),
        duty_factor=duty,
        body_height=num(g, "gait", "body_height_mm") / 1000,
        stance_radius=num(g, "gait", "stance_radius_mm") / 1000,
        step_height=num(g, "gait", "step_height_mm") / 1000,
        max_stride=num(g, "gait", "max_stride_mm") / 1000,
    )

    mo = table("motion")
    loop_hz = num(mo, "motion", "loop_hz")
    if loop_hz < MIN_LOOP_HZ:
        raise ConfigError(f"motion.loop_hz must be >= {MIN_LOOP_HZ:g}, got {loop_hz:g}")
    motion = MotionParams(
        loop_hz=loop_hz,
        max_linear_accel=num(mo, "motion", "max_linear_accel_mps2"),
        max_angular_accel=radians(num(mo, "motion", "max_angular_accel_dps2")),
    )

    return RobotConfig(
        body=body, mass=mass, leg=leg, limits=limits, servo=servo, gait=gait, motion=motion
    )

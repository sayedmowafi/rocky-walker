"""Move files: keyframes of body pose and foot offsets (TOML, human units)."""

from __future__ import annotations

import math
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from rocky.config import ConfigError
from rocky.kinematics.pose import BodyPose


@dataclass(frozen=True)
class Keyframe:
    t: float
    pose: BodyPose
    #: leg index -> foot offset (m, world frame) from its neutral position
    feet: dict[int, np.ndarray] = field(default_factory=dict)

    def is_neutral(self) -> bool:
        return self.pose == BodyPose() and all(not np.any(v) for v in self.feet.values())


@dataclass(frozen=True)
class Move:
    name: str
    keyframes: tuple[Keyframe, ...]

    @property
    def duration(self) -> float:
        return self.keyframes[-1].t


def _key(raw: dict, name: str) -> Keyframe:
    def f(k: str) -> float:
        v = raw.get(k, 0.0)
        if isinstance(v, bool) or not isinstance(v, int | float) or not math.isfinite(v):
            raise ConfigError(f"move {name}: {k} must be a finite number")
        return float(v)

    pose = BodyPose(
        x=f("x_mm") / 1000,
        y=f("y_mm") / 1000,
        z=f("z_mm") / 1000,
        roll=math.radians(f("roll_deg")),
        pitch=math.radians(f("pitch_deg")),
        yaw=math.radians(f("yaw_deg")),
    )
    feet = {}
    for foot in raw.get("foot", []):
        leg = foot.get("leg")
        if not isinstance(leg, int) or not 0 <= leg < 5:
            raise ConfigError(f"move {name}: foot.leg must be 0..4")
        feet[leg] = np.array([foot.get(k, 0.0) for k in ("dx_mm", "dy_mm", "dz_mm")], float) / 1000
    return Keyframe(f("t"), pose, feet)


def load_move(path: Path | str) -> Move:
    """Load a move file, or a routine (``sequence = [...]`` of sibling move files)."""
    path = Path(path)
    with open(path, "rb") as fh:
        raw = tomllib.load(fh)
    name = raw.get("name", path.stem)
    if "sequence" in raw:
        keys: list[Keyframe] = []
        offset = 0.0
        for part in raw["sequence"]:
            move = load_move(path.parent / f"{part}.toml")
            if not (move.keyframes[0].is_neutral() and move.keyframes[-1].is_neutral()):
                raise ConfigError(f"{name}: part {part} must start and end at the neutral pose")
            for i, k in enumerate(move.keyframes):
                if keys and i == 0:
                    continue  # previous move already ends at this (neutral) instant
                keys.append(Keyframe(k.t + offset, k.pose, k.feet))
            offset += move.duration
        return Move(name, tuple(keys))
    keys = [_key(k, name) for k in raw.get("key", [])]
    if len(keys) < 2 or keys[0].t != 0.0:
        raise ConfigError(f"move {name}: needs at least two keys, the first at t = 0")
    for a, b in zip(keys, keys[1:], strict=False):
        if b.t <= a.t:
            raise ConfigError(
                f"move {name}: key times must be strictly increasing ({a.t} -> {b.t})"
            )
    return Move(name, tuple(keys))

"""Phase-based gait patterns for five legs.

Leg i (numbered by position around the body) gets phase offset (2i mod 5)/5, so
physically adjacent legs are always 0.4 of a cycle apart and legs start their
swings in the order 0 -> 3 -> 1 -> 4 -> 2.

- wave: duty >= 0.8, exactly one leg in the air at a time (4 feet down).
- star: duty >= 0.6, at most two non-adjacent legs in the air (3 feet down).
"""

from __future__ import annotations

from dataclasses import dataclass

from rocky.config import ConfigError

#: Minimum duty factor per gait: the threshold that keeps each gait's invariants.
MIN_DUTY = {"wave": 0.8, "star": 0.6}


@dataclass(frozen=True)
class GaitPattern:
    name: str
    offsets: tuple[float, ...]
    duty_factor: float

    @property
    def swing_fraction(self) -> float:
        return round(1.0 - self.duty_factor, 12)

    def local_phase(self, leg: int, phase: float) -> float:
        """Where ``leg`` is in its own cycle; its swing window is ``[0, swing_fraction)``."""
        # Rounding keeps window boundaries exact despite float error (e.g. 0.6 - 0.2).
        local = round((phase - self.offsets[leg]) % 1.0, 12)
        return 0.0 if local >= 1.0 else local

    def in_swing(self, leg: int, phase: float) -> bool:
        return self.local_phase(leg, phase) < self.swing_fraction


def make_pattern(name: str, duty_factor: float, n_legs: int = 5) -> GaitPattern:
    """Build a named gait pattern, refusing duty factors that would break its invariants."""
    if name not in MIN_DUTY:
        raise ConfigError(f"unknown gait {name!r}; expected one of {sorted(MIN_DUTY)}")
    if n_legs != 5:
        raise ConfigError(f"gait patterns support 5 legs, got {n_legs}")
    if not MIN_DUTY[name] - 1e-12 <= duty_factor < 1.0:
        raise ConfigError(
            f"gait.duty_factor.{name} must be in [{MIN_DUTY[name]}, 1), got {duty_factor}"
        )
    offsets = tuple(((2 * i) % n_legs) / n_legs for i in range(n_legs))
    return GaitPattern(name=name, offsets=offsets, duty_factor=duty_factor)

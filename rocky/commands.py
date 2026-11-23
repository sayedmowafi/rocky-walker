"""Commands shared by the motion stack and its clients (phone, vision, sim)."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class BodyVelocity:
    """Desired body velocity in the body frame: m/s, m/s, rad/s."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def is_zero(self, eps: float = 1e-6) -> bool:
        return abs(self.vx) < eps and abs(self.vy) < eps and abs(self.wz) < eps

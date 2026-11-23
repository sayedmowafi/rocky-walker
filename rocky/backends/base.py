"""Servo backends: where the motion controller's joint angles go."""

from __future__ import annotations

from typing import Protocol

import numpy as np


class ServoBackend(Protocol):
    def write(self, angles: np.ndarray) -> None:
        """Send joint angles, shape ``(n_legs, 3)`` in radians."""
        ...


class DryRunBackend:
    """Records every write; useful for tests and for running without hardware."""

    def __init__(self) -> None:
        self.history: list[np.ndarray] = []

    def write(self, angles: np.ndarray) -> None:
        self.history.append(np.array(angles, dtype=float, copy=True))

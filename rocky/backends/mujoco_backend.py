"""Servo backend that drives the MuJoCo model's position actuators."""

from __future__ import annotations

import mujoco
import numpy as np


class MujocoBackend:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self._model = model
        self._data = data

    def write(self, angles: np.ndarray) -> None:
        self._data.ctrl[:] = np.asarray(angles, dtype=float).reshape(-1)

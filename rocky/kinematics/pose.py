"""Body pose: an offset of the body from its neutral standing pose."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class BodyPose:
    """Translation (m) and roll/pitch/yaw (rad, applied Z-Y-X) relative to neutral."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def matrix(self) -> np.ndarray:
        """4x4 homogeneous transform: R = Rz(yaw) @ Ry(pitch) @ Rx(roll), then translate."""
        cr, sr = np.cos(self.roll), np.sin(self.roll)
        cp, sp = np.cos(self.pitch), np.sin(self.pitch)
        cy, sy = np.cos(self.yaw), np.sin(self.yaw)
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        m = np.eye(4)
        m[:3, :3] = rz @ ry @ rx
        m[:3, 3] = [self.x, self.y, self.z]
        return m

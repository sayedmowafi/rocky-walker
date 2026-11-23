"""Analytic 3-DOF leg kinematics (coxa yaw, femur pitch, tibia pitch), knee-up branch.

Leg frame: origin on the coxa axis at the mount, x radially outward, z up.
Angles: coxa about +z (0 = outward); femur positive lifts (0 = horizontal);
tibia relative to femur, negative bends the knee down (standing is about -90 deg).
"""

from __future__ import annotations

import numpy as np

from rocky.config import LegGeometry


class UnreachableError(ValueError):
    """The requested foot position is outside the leg's workspace."""


def leg_fk(angles: np.ndarray, geom: LegGeometry) -> np.ndarray:
    """Foot position in the leg frame for joint angles ``(coxa, femur, tibia)``."""
    t1, t2, t3 = angles
    r = geom.coxa + geom.femur * np.cos(t2) + geom.tibia * np.cos(t2 + t3)
    z = geom.femur * np.sin(t2) + geom.tibia * np.sin(t2 + t3)
    return np.array([r * np.cos(t1), r * np.sin(t1), z])


def leg_ik(foot: np.ndarray, geom: LegGeometry) -> np.ndarray:
    """Joint angles ``(coxa, femur, tibia)`` that put the foot at ``foot`` (leg frame)."""
    x, y, z = foot
    t1 = np.arctan2(y, x)
    r = np.hypot(x, y) - geom.coxa
    lf, lt = geom.femur, geom.tibia
    c3 = (r * r + z * z - lf * lf - lt * lt) / (2 * lf * lt)
    if abs(c3) > 1.0:
        raise UnreachableError(f"foot {np.round(foot, 4).tolist()} is out of reach")
    t3 = -np.arccos(c3)
    t2 = np.arctan2(z, r) - np.arctan2(lt * np.sin(t3), lf + lt * np.cos(t3))
    return np.array([t1, t2, t3])

"""Body geometry: leg mounts around the body, body<->leg frame transforms, neutral stance.

Body frame: origin at the body centre at joint height, z up. Leg i is mounted at
yaw 2*pi*i/n (leg 0 on +x, numbered counter-clockwise).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from rocky.config import BodyParams, RobotConfig


def rot_z(angle: float) -> np.ndarray:
    """3x3 rotation matrix about +z."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


@dataclass(frozen=True)
class LegMount:
    index: int
    position: np.ndarray
    yaw: float


def leg_mounts(body: BodyParams) -> list[LegMount]:
    """The leg mounts, evenly spaced around the body."""
    mounts = []
    for i in range(body.n_legs):
        yaw = 2 * np.pi * i / body.n_legs
        pos = body.mount_radius * np.array([np.cos(yaw), np.sin(yaw), 0.0])
        mounts.append(LegMount(index=i, position=pos, yaw=yaw))
    return mounts


def body_to_leg(p: np.ndarray, mount: LegMount) -> np.ndarray:
    """Express a body-frame point in the given leg's frame."""
    return rot_z(-mount.yaw) @ (np.asarray(p, dtype=float) - mount.position)


def leg_to_body(p: np.ndarray, mount: LegMount) -> np.ndarray:
    """Express a leg-frame point in the body frame."""
    return rot_z(mount.yaw) @ np.asarray(p, dtype=float) + mount.position


def neutral_feet(cfg: RobotConfig) -> np.ndarray:
    """Neutral standing foot positions, shape ``(n_legs, 3)``, in the body frame."""
    s, h = cfg.gait.stance_radius, cfg.gait.body_height
    return np.array([[s * np.cos(m.yaw), s * np.sin(m.yaw), -h] for m in leg_mounts(cfg.body)])

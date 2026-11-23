"""Centre of mass of the whole robot, and how far it sits inside the planted feet."""

from __future__ import annotations

import numpy as np

from rocky.config import RobotConfig
from rocky.kinematics.body import leg_mounts, leg_to_body
from rocky.kinematics.pose import BodyPose


def segment_coms(cfg: RobotConfig, angles: np.ndarray) -> np.ndarray:
    """Body-frame COM of every leg segment, shape (n_legs, 3 [coxa, femur, tibia], 3)."""
    leg, m = cfg.leg, cfg.mass
    out = np.zeros((cfg.body.n_legs, 3, 3))
    for mount in leg_mounts(cfg.body):
        t1, t2, t3 = angles[mount.index]
        radial = (
            m.coxa_com,
            leg.coxa + m.femur_com * np.cos(t2),
            leg.coxa + leg.femur * np.cos(t2) + m.tibia_com * np.cos(t2 + t3),
        )
        height = (
            0.0,
            m.femur_com * np.sin(t2),
            leg.femur * np.sin(t2) + m.tibia_com * np.sin(t2 + t3),
        )
        for k in range(3):
            p = np.array([radial[k] * np.cos(t1), radial[k] * np.sin(t1), height[k]])
            out[mount.index, k] = leg_to_body(p, mount)
    return out


def center_of_mass(cfg: RobotConfig, pose: BodyPose, angles: np.ndarray) -> np.ndarray:
    """World-frame COM. The world frame has the floor at z = 0 and the neutral body origin at
    (0, 0, body_height); the torso's own COM is taken at the body origin."""
    m = cfg.mass
    seg_mass = np.array([m.coxa_link, m.femur_link, m.tibia_link])
    coms = segment_coms(cfg, angles)
    # the torso's mass sits at the body origin, so it adds nothing to the moment
    body_com = (coms * seg_mass[None, :, None]).sum(axis=(0, 1)) / m.total
    world = np.eye(4)
    world[2, 3] = cfg.gait.body_height
    t = world @ pose.matrix()
    return (t @ np.append(body_com, 1.0))[:3]


def _hull(points: np.ndarray) -> np.ndarray:
    pts = sorted(map(tuple, points))
    if len(pts) < 3:
        return np.array(pts)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])  # counter-clockwise


def support_margin(points_xy: np.ndarray, com_xy: np.ndarray) -> float:
    """Signed distance from the COM to the edge of the support polygon (convex hull of the
    planted feet): positive inside, negative outside, -inf with fewer than 3 feet."""
    if len(points_xy) < 3:
        return -np.inf
    hull = _hull(np.asarray(points_xy, float))
    if len(hull) < 3:
        return -np.inf
    c = np.asarray(com_xy, float)
    inside = True
    best = np.inf
    for a, b in zip(hull, np.roll(hull, -1, axis=0), strict=True):
        edge = b - a
        to_c = c - a
        if edge[0] * to_c[1] - edge[1] * to_c[0] < 0:
            inside = False
        t = np.clip(to_c @ edge / (edge @ edge), 0.0, 1.0)
        best = min(best, float(np.linalg.norm(c - (a + t * edge))))
    return best if inside else -best
